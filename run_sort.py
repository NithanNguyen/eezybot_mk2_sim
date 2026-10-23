"""run_sort.py — Mo phong chu trinh phan loai mau nap chai (model/mk2_sort.xml).

Chu trinh (de bai, 2026-10-23), lap lai cho tung nap:
  1. Tay may o Home (servo 90/90/90), gripper mo; nguoi dat 1 nap len tram cam bien.
  2. ESP32 doc kenh C moi POLL_MS; khi C khac C_empty (co nap) thi cho settle_ms (300 ms >= 200 ms)
     roi lay trung binh navg = 5 lan doc RGBC (data/color/model.json). Mo phong cam bien: color_sim.py.
  3. Phan loai: tam lop gan nhat trong mat phang (r, g); d > d_max -> vut bo + "# ERR UNKNOWN_COLOR";
     |C - C_empty| < C_margin -> khong co nap, tiep tuc cho.
  4. Den diem phia tren diem gap (cao hon PRE_GRASP_LIFT), ha thang xuong, dong kep, nhac len.
  5. Den diem ma day nap cach mat tren hop dich (hoac mat ban o vung vut bo) DROP_ABOVE = 50 mm, mo kep.
  6. Ve Home, cho nap tiep theo.
Quy dao: giua cac tram = double-S dong bo 3 khop (Ruckig, khong gian khop, cac khop xong cung luc);
ha/nhac = duong thang dung, double-S theo do dai cung (Ruckig). Lenh moi 20 ms -> IK -> servo (giu 20 ms).
Kep nap bang tiep xuc + ma sat that giua tam kep cua 2 ngon va nap (sort_scene.py).

Vi du:
    python run_sort.py                 # 9 nap (3 mau x 3), thu tu ngau nhien seed 0 -> results/sort/
    python run_sort.py --view          # xem truc tiep (cua so MuJoCo)
    python run_sort.py --foreign 2     # them 2 vat la (xanh duong) de thu nhanh "vut bo"
    python run_sort.py --seed 3
"""
import argparse, csv, math, os, time
import numpy as np
import matplotlib
import mujoco
import params as P
import mk2_kinematics as K
import sim_common as S
import trajectories as TR
import color_sim as CS
import sort_scene as SS

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=0, help="thu tu nap va mau do duoc rut tu test.csv")
ap.add_argument("--foreign", type=int, default=0, choices=range(SS.FOREIGN_CAPS + 1),
                help="so vat la (nhan 'other' trong test.csv) xen vao hang cho, toi da %d" % SS.FOREIGN_CAPS)
ap.add_argument("--view", action="store_true", help="mo cua so MuJoCo xem truc tiep (thoi gian thuc)")
ap.add_argument("--out", default=os.path.join(S.HERE, "results", "sort"))
args = ap.parse_args()
if not args.view:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

os.makedirs(args.out, exist_ok=True)
K.set_tool("cap")
DT = P.CMD_PERIOD
MODEL = CS.load_model()
LOG_LINES = []


def say(s):
    print(s); LOG_LINES.append(s)


# ======================================================================= diem va tu the
CAP_BELOW_TOOL = P.CAP_GRASP_Z - P.SHELF_HEIGHT            # day nap thap hon diem "cap" khi kep (16 mm)
st = SS.station()
P_GRASP = np.array([st[0], st[1], P.CAP_GRASP_Z])
P_PRE = P_GRASP + [0, 0, P.PRE_GRASP_LIFT]
Z_DROP_BIN = P.BIN_HEIGHT + P.DROP_ABOVE + CAP_BELOW_TOOL
Z_DROP_REJ = 0.0 + P.DROP_ABOVE + CAP_BELOW_TOOL
DROPS = {c: np.array([*xy, Z_DROP_BIN]) for c, xy in SS.bin_targets().items()}
rj = SS.reject_target()
DROPS["reject"] = np.array([rj[0], rj[1], Z_DROP_REJ])
P_REJ_PRE = np.array([rj[0], rj[1], max(Z_DROP_REJ, P_PRE[2])])  # toi vung vut bo o do cao an toan roi ha
Q_HOME = (0.0, math.pi / 2, 0.0)


def margin_deg(q):
    """Khoang cach nho nhat (do) toi cac gioi han khop / tam lenh (mk2_kinematics.within_limits)."""
    th1, th2, phi = (math.degrees(v) for v in q)
    phlo, phhi = K.phi_limits()
    m = [th1 - P.THETA1_LIM[0], P.THETA1_LIM[1] - th1, th2 - P.THETA2_LIM[0], P.THETA2_LIM[1] - th2,
         phi - phlo, phhi - phi, (phi - th2) - P.REL_LIM[0], P.REL_LIM[1] - (phi - th2),
         (th2 - 90) - P.CTRL_SHOULDER_DEG[0], P.CTRL_SHOULDER_DEG[1] - (th2 - 90),
         P.BASE_GEAR_RATIO * th1 - P.CTRL_BASE_DEG[0], P.CTRL_BASE_DEG[1] - P.BASE_GEAR_RATIO * th1]
    return min(m)


say(K.tool_warning().replace("Dau cong tac: cap", "Dau cong tac: tam nap (cap)"))
say("Tu the (diem 'cap' = tam nap khi kep, mm) -> goc servo (do), du tru toi gioi han gan nhat:")
POSES = {"tram: diem kep": P_GRASP, "tram: phia tren": P_PRE}
for c in P.BIN_ORDER:
    POSES[f"ngan {P.BIN_NUMBER[c]} ({c}): tha"] = DROPS[c]
POSES["vut bo: phia tren"] = P_REJ_PRE
POSES["vut bo: tha"] = DROPS["reject"]
Q = {}
for nm, p in POSES.items():
    try:
        q = K.ik(p)
    except K.IKError as e:
        raise SystemExit(f"Khong voi toi '{nm}' {np.round(p, 1)}: {e}. Kiem tra bo tri trong params.py muc 8.")
    Q[nm] = q
    say(f"  {nm:22s} p = {np.round(p, 1)}  servo = {np.round(K.to_servo(q), 2)}  du tru = {margin_deg(q):.2f} do")


# ======================================================================= quy dao
JV, JA, JJ = (v / P.TRAJ_R_REF for v in (P.TRAJ_V, P.TRAJ_A, P.TRAJ_J))   # rad/s, rad/s^2, rad/s^3


def joint_move(q0, q1):
    """Double-S dong bo 3 khop (Ruckig, dong bo thoi gian): nghi -> nghi, lay mau DT."""
    from ruckig import InputParameter, OutputParameter, Result, Ruckig
    otg = Ruckig(3, DT); inp, out = InputParameter(3), OutputParameter(3)
    inp.current_position = list(q0); inp.current_velocity = [0.0] * 3; inp.current_acceleration = [0.0] * 3
    inp.target_position = list(q1); inp.target_velocity = [0.0] * 3; inp.target_acceleration = [0.0] * 3
    inp.max_velocity = [JV] * 3; inp.max_acceleration = [JA] * 3; inp.max_jerk = [JJ] * 3
    qs = []; res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out); qs.append(list(out.new_position)); out.pass_to_input(inp)
    if res != Result.Finished:
        raise RuntimeError(f"Ruckig loi: {res}")
    qs = np.array(qs)
    for q in qs:
        ok, msg = K.within_limits(q)
        if not ok:
            raise RuntimeError(f"quy dao khop vuot gioi han: {msg}")
    return qs


def line_move(p0, p1):
    """Duong thang p0 -> p1, double-S theo do dai cung (Ruckig), IK moi DT."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = np.linalg.norm(p1 - p0)
    s = TR.ruckig_1d(L, P.TRAJ_V, P.TRAJ_A, P.TRAJ_J, DT)
    return np.array([K.ik(p0 + (p1 - p0) * (u / L)) for u in s])


# kiem tra khe ho tren quy dao lenh (FK giai tich): vat chan = (ten, ham "diem xy nam tren vat", do cao mat tren)
def _in_bin(xy, pad):
    c, a = SS.bin_frame(); Lu, Lv, _ = SS.bin_outer()
    u = (xy[0] - c[0]) * math.cos(a) + (xy[1] - c[1]) * math.sin(a)
    v = -(xy[0] - c[0]) * math.sin(a) + (xy[1] - c[1]) * math.cos(a)
    return abs(u) <= Lu / 2 + pad and abs(v) <= Lv / 2 + pad


def _in_shelf(xy, pad):
    return abs(xy[0] - st[0]) <= P.SHELF_SIZE / 2 + pad and abs(xy[1] - st[1]) <= P.SHELF_SIZE / 2 + pad


def clearance(qs, below, pad, cap_on_station):
    """Khe nho nhat (mm) tu diem thap nhat (= diem 'cap' - below) toi mat tren tung vat chan khi diem do nam
    tren vat (mo rong pad mm) va toi mat ban. pad: nua be rong vat duoc mang (nap: ban kinh nap; kep rong:
    ~ nua khoang mo ngon + be day ngon). Tra ve {ten vat: khe nho nhat}."""
    obst = [("hop dich", _in_bin, P.BIN_HEIGHT),
            ("ke" + (" + nap" if cap_on_station else ""), _in_shelf, P.SHELF_HEIGHT + (P.CAP_HEIGHT if cap_on_station else 0))]
    best = {"mat ban": 1e9}
    for q in qs:
        p = K.fk(*q); z = p[2] - below
        best["mat ban"] = min(best["mat ban"], z)
        for nm, inside, top in obst:
            if inside(p[:2], pad):
                best[nm] = min(best.get(nm, 1e9), z - top)
    return best


# ======================================================================= mo phong
m, d = S.load_model(sort=True)
N_SUB = int(round(DT / m.opt.timestep))
J_GRIP = m.joint("gripper").qposadr[0]
SITE_TOOL = m.site("tool").id
CAP_NAMES = SS.cap_names()
CAP_COLOR = {n: n.split("_")[1] for n in CAP_NAMES}
CAP_BODY = {n: m.body(n).id for n in CAP_NAMES}
CAP_GEOMS = {n: {i for i in range(m.ngeom) if m.geom_bodyid[i] == CAP_BODY[n] and m.geom_contype[i] == SS.CAP_T}
             for n in CAP_NAMES}
PAD_GEOMS = {i for i in range(m.ngeom) if m.geom(i).name.startswith("pad_")}
ENV_GEOMS = {i for i in range(m.ngeom) if m.geom_contype[i] == SS.ENV_T}
IS_PAD = np.zeros(m.ngeom, bool); IS_PAD[list(PAD_GEOMS)] = True          # tra nhanh trong vong mj_step
IS_ENV = np.zeros(m.ngeom, bool); IS_ENV[list(ENV_GEOMS)] = True
GRIP_OPEN = math.radians(P.GRIP_OPEN_DEG) * P.GRIPPER_GEAR_RATIO
GRIP_HOLD = math.radians(P.GRIP_HOLD_DEG) * P.GRIPPER_GEAR_RATIO

viewer = None
if args.view:
    import mujoco.viewer
    viewer = mujoco.viewer.launch_passive(m, d)
    viewer.cam.lookat[:] = [0.22, -0.02, 0.06]; viewer.cam.distance = 0.75
    viewer.cam.azimuth = 205; viewer.cam.elevation = -35

state = dict(q=np.array(Q_HOME), grip=0.0, phase="khoi dong", cap="", n_pad_env=0, t0=time.time())
LOG = []


def cap_pos(n):
    return d.xpos[CAP_BODY[n]] * 1000


def step_cmd(q, grip):
    state["q"] = np.asarray(q); state["grip"] = grip
    d.ctrl[:3] = S.q_to_ctrl(q); d.ctrl[3] = grip
    pe = False
    for _ in range(N_SUB):
        mujoco.mj_step(m, d)
        if not pe and d.ncon:
            g1, g2 = d.contact.geom1, d.contact.geom2
            pe = bool(np.any((IS_PAD[g1] & IS_ENV[g2]) | (IS_PAD[g2] & IS_ENV[g1])))
    state["n_pad_env"] += int(pe)
    tool = d.site_xpos[SITE_TOOL] * 1000
    cp = cap_pos(state["cap"]) if state["cap"] else np.full(3, np.nan)
    LOG.append([d.time, state["phase"], state["cap"], *K.to_servo(q), math.degrees(grip),
                *np.degrees(S.actual_q(m, d)), math.degrees(d.qpos[J_GRIP]), *tool, *cp, int(pe)])
    if viewer is not None:
        viewer.sync()
        lag = d.time - (time.time() - state["t0"])
        if lag > 0: time.sleep(lag)
        if not viewer.is_running(): raise SystemExit("Da dong cua so viewer")


def hold(seconds, phase=None):
    if phase: state["phase"] = phase
    for _ in range(max(1, int(round(seconds / DT)))):
        step_cmd(state["q"], state["grip"])


def run_q(qs, phase):
    state["phase"] = phase
    for q in qs:
        step_cmd(q, state["grip"])


def set_grip(rad, phase, wait):
    state["phase"] = phase; state["grip"] = rad
    hold(wait)


def cap_free_joint(n):
    return m.joint(m.body_jntadr[CAP_BODY[n]]).qposadr[0], m.joint(m.body_jntadr[CAP_BODY[n]]).dofadr[0]


def place_on_station(n):
    """Nguoi dat nap (up) len tram: tam nap trung tam lo, day nap cach mica 0.2 mm, dung yen."""
    qa, va = cap_free_joint(n)
    d.qpos[qa:qa + 3] = [st[0] / 1000, st[1] / 1000, (P.SHELF_HEIGHT + P.CAP_HEIGHT / 2 + 0.2) / 1000]
    d.qpos[qa + 3:qa + 7] = [1, 0, 0, 0]
    d.qvel[va:va + 6] = 0
    mujoco.mj_forward(m, d)


def station_object():
    """Vat dang nam yen tren tram: nhan mau ('green'|'red'|'white'|'other') hoac 'empty'."""
    for n in CAP_NAMES:
        p = cap_pos(n)
        if (np.hypot(p[0] - st[0], p[1] - st[1]) <= P.STATION_TOL and
                abs(p[2] - P.CAP_HEIGHT / 2 - P.SHELF_HEIGHT) <= 1.0):
            return CAP_COLOR[n]
    return "empty"


def where(p):
    """Vi tri cuoi cua nap: 'ngan 1 (red)' ... | 'vut bo' | 'ngoai'."""
    c, a = SS.bin_frame()
    u = (p[0] - c[0]) * math.cos(a) + (p[1] - c[1]) * math.sin(a)
    v = -(p[0] - c[0]) * math.sin(a) + (p[1] - c[1]) * math.cos(a)
    for i, col in enumerate(P.BIN_ORDER):
        if abs(u - (i - 1) * SS.bin_pitch()) <= P.BIN_INNER[0] / 2 and abs(v) <= P.BIN_INNER[1] / 2 and p[2] < P.BIN_HEIGHT:
            return col
    r = SS.reject_target(); a = math.radians(P.REJECT_ANGLE_DEG)
    u = (p[0] - r[0]) * math.cos(a) + (p[1] - r[1]) * math.sin(a)
    v = -(p[0] - r[0]) * math.sin(a) + (p[1] - r[1]) * math.cos(a)
    if abs(u) <= P.REJECT_SIZE / 2 and abs(v) <= P.REJECT_SIZE / 2 and p[2] < 20:
        return "reject"
    return "ngoai"


def rel_to_tool(n):
    """Vi tri tam nap trong he dau cong tac (site tool, he co tay luon nam ngang), mm."""
    R = d.site_xmat[SITE_TOOL].reshape(3, 3)
    return R.T @ (cap_pos(n) - d.site_xpos[SITE_TOOL] * 1000)


def pad_force(n):
    """Tong luc phap tuyen (N) giua cac tam kep va nap n."""
    g = CAP_GEOMS[n]; f6 = np.zeros(6); tot = 0.0
    for i in range(d.ncon):
        c = d.contact[i]
        if (c.geom1 in g and c.geom2 in PAD_GEOMS) or (c.geom2 in g and c.geom1 in PAD_GEOMS):
            mujoco.mj_contactForce(m, d, i, f6); tot += f6[0]
    return tot


# ----------------------------------------------------------------------- hang cho nap
rng = np.random.default_rng(args.seed)
queue = [n for n in CAP_NAMES if CAP_COLOR[n] != "other"]
queue = [queue[i] for i in rng.permutation(len(queue))]
for k in range(args.foreign):
    queue.insert(int(rng.integers(0, len(queue) + 1)), f"cap_other_{k}")
sensor = CS.SensorSim(seed=args.seed)
say(f"Thu tu dat nap (seed {args.seed}): " + ", ".join(CAP_COLOR[n] for n in queue))
say(f"Gioi han khop (quy doi tu {P.TRAJ_V:g} mm/s, {P.TRAJ_A:g} mm/s^2, {P.TRAJ_J:g} mm/s^3 qua r = {P.TRAJ_R_REF:g} mm): "
    f"{JV:.3f} rad/s, {JA:.2f} rad/s^2, {JJ:.1f} rad/s^3")
meas_s = (MODEL["settle_ms"] + (MODEL["navg"] - 1) * P.READ_INTERVAL_MS) / 1000.0

# ----------------------------------------------------------------------- khoi dong: Home, mo kep
state["phase"] = "khoi dong"
hold(0.5)
set_grip(GRIP_OPEN, "mo kep", P.GRIP_MOVE_S)
results = []
plan_clear = []
sv = P.MG996R[P.SUPPLY]; w_servo = math.radians(60) / sv["s_per_60deg"]
servo_w_max = np.zeros(3)


def track(qs):
    global servo_w_max
    w = np.abs(TR.derivs(np.vstack([state["q"], qs]))[0][1:]) * np.array([P.BASE_GEAR_RATIO, 1, 1])
    servo_w_max = np.maximum(servo_w_max, w.max(0))


def wait_for_cap(next_cap):
    """Tay o Home. Nguoi dat nap sau CAP_PLACE_DELAY_S. ESP32: doc C moi POLL_MS; co nap -> do day du."""
    t_place = d.time + P.CAP_PLACE_DELAY_S if next_cap else None
    placed_at = None
    state["phase"] = "cho nap"
    while True:
        if next_cap and placed_at is None and d.time >= t_place:
            state["cap"] = next_cap; place_on_station(next_cap); placed_at = d.time
        lab = station_object()
        c_read = sensor.measure(lab, consume=False)["C"] if lab != "empty" else sensor.measure("empty")["C"]
        if abs(c_read - MODEL["c_empty"]) >= MODEL["c_margin"]:
            t_det = d.time
            hold(meas_s, "do mau")                                   # settle_ms + (navg - 1) * 120 ms
            lab = station_object()
            row = sensor.measure(lab)
            res = CS.classify(MODEL, row["R"], row["G"], row["B"], row["C"], row["sat"])
            say(f"[t={d.time:7.2f} s] " + CS.cls_line(res, row))
            if res["label"] == "unknown":
                say(f"[t={d.time:7.2f} s] " + CS.err_line(res, MODEL, row))
            if res["label"] != "no_cap":
                return res, row, lab, placed_at, t_det
        elif next_cap is None:
            return None
        hold(P.POLL_MS / 1000.0, "cho nap")


# ----------------------------------------------------------------------- vong phan loai
count = {}
for k, n in enumerate(queue + [None]):
    got = wait_for_cap(n)
    if got is None:
        say(f"[t={d.time:7.2f} s] Tram trong, het nap -> dung.")
        break
    res, row, lab, t_place, t_det = got
    n = state["cap"]
    dest = res["label"] if res["label"] in CS.CLASSES else "reject"
    count[dest] = count.get(dest, 0) + 1
    say(f"[t={d.time:7.2f} s] # SORT {'REJECT' if dest == 'reject' else dest.upper()} (lan {count[dest]})  "
        f"[nap {k + 1}: {n}, dong test.csv {row['row']}]")
    t_dec = d.time
    rec = dict(i=k + 1, cap=n, color=CAP_COLOR[n], csv_row=row["row"], R=row["R"], G=row["G"], B=row["B"], C=row["C"],
               label=res["label"], nearest=res["nearest"], dist=res["dist"], r=res["r"], g=res["g"], dest=dest,
               t_place=t_place, t_detect=t_det, t_decision=t_dec)

    # 4. gap
    q_pre, q_grasp = Q["tram: phia tren"], Q["tram: diem kep"]
    seg = joint_move(state["q"], q_pre); plan_clear.append(("Home -> tren tram (kep rong)", clearance(seg, 7.5, 40, True)))
    track(seg); run_q(seg, "toi tram")
    seg = line_move(P_PRE, P_GRASP); track(seg); run_q(seg, "ha xuong")
    rec["z_tool_grasp"] = d.site_xpos[SITE_TOOL][2] * 1000
    hold(0.1)
    set_grip(GRIP_HOLD, "dong kep", P.GRIP_MOVE_S)
    rec["grip_deg"] = math.degrees(d.qpos[J_GRIP]); rec["F_pad_N"] = pad_force(n)
    r0 = rel_to_tool(n)
    seg = line_move(P_GRASP, P_PRE); track(seg); run_q(seg, "nhac len")
    rec["lift_mm"] = cap_pos(n)[2] - (P.SHELF_HEIGHT + P.CAP_HEIGHT / 2)

    # 5. tha
    slip = [0.0]
    def run_carry(qs, phase):
        state["phase"] = phase
        for q in qs:
            step_cmd(q, state["grip"]); slip.append(np.linalg.norm(rel_to_tool(n) - r0))
    if dest == "reject":
        seg = joint_move(state["q"], Q["vut bo: phia tren"])
        plan_clear.append(("tram -> tren vung vut bo (mang nap)", clearance(seg, CAP_BELOW_TOOL, P.CAP_DIAMETER / 2, False)))
        track(seg); run_carry(seg, "toi vung vut bo")
        seg = line_move(P_REJ_PRE, DROPS["reject"]); track(seg); run_carry(seg, "ha xuong vung vut bo")
    else:
        seg = joint_move(state["q"], Q[f"ngan {P.BIN_NUMBER[dest]} ({dest}): tha"])
        plan_clear.append((f"tram -> ngan {P.BIN_NUMBER[dest]} (mang nap)", clearance(seg, CAP_BELOW_TOOL, P.CAP_DIAMETER / 2, False)))
        track(seg); run_carry(seg, f"toi ngan {P.BIN_NUMBER[dest]}")
    rec["slip_max_mm"] = max(slip)
    rec["cap_bottom_drop"] = cap_pos(n)[2] - P.CAP_HEIGHT / 2
    set_grip(GRIP_OPEN, "mo kep", P.DROP_WAIT_S)

    # 6. ve Home
    if dest == "reject":
        seg = line_move(DROPS["reject"], P_REJ_PRE); track(seg); run_q(seg, "nhac len")
    seg = joint_move(state["q"], Q_HOME); plan_clear.append(("ve Home (kep rong)", clearance(seg, 7.5, 40, False)))
    track(seg); run_q(seg, "ve Home")
    rec["t_home"] = d.time
    rec["final"] = where(cap_pos(n)); rec["final_pos"] = np.round(cap_pos(n), 1)
    expect = CAP_COLOR[n] if CAP_COLOR[n] in CS.CLASSES else "reject"
    rec["ok"] = rec["final"] == dest
    say(f"           gap: z diem kep (thuc) = {rec['z_tool_grasp']:.1f} mm, kep {rec['grip_deg']:.2f} do, "
        f"luc kep {rec['F_pad_N']:.2f} N, nhac nap {rec['lift_mm']:.1f} mm, truot max {rec['slip_max_mm']:.2f} mm; "
        f"day nap luc tha {rec['cap_bottom_drop']:.1f} mm; nap nam o: {rec['final']} -> {'DUNG' if rec['ok'] else 'SAI'} dich"
        + ("" if dest == expect else f" (mau that {CAP_COLOR[n]}, bo phan loai cho ra {res['label']})"))
    results.append(rec)
    state["cap"] = ""

hold(1.0, "ket thuc")
# vi tri cuoi cung (sau khi moi nap da nam yen)
for rec in results:
    rec["final"] = where(cap_pos(rec["cap"])); rec["final_pos"] = np.round(cap_pos(rec["cap"]), 1)
    rec["ok"] = rec["final"] == rec["dest"]
if viewer is not None:
    viewer.close()

# ======================================================================= tong ket
say("")
say(f"Toc do servo lon nhat = {servo_w_max.round(2)} rad/s (MG996R khong tai {w_servo:.2f} rad/s @{P.SUPPLY}) -> "
    f"{'OK' if servo_w_max.max() < w_servo else 'VUOT'}")
cl = {}
for nm, dd in plan_clear:
    for ob, z in dd.items():
        cl.setdefault(nm, {}); cl[nm][ob] = min(cl[nm].get(ob, 1e9), z)
say("Khe ho nho nhat tren quy dao LENH (FK; diem thap nhat = day nap khi mang nap, mep duoi ngon khi kep rong):")
for nm, dd in cl.items():
    say(f"  {nm:38s} " + ", ".join(f"{ob} {z:.1f} mm" for ob, z in dd.items()))
say(f"So chu ky lenh co tam kep cham ke/hop/ban: {state['n_pad_env']}")
cyc = [r["t_home"] - r["t_place"] for r in results]
say(f"Thoi gian 1 chu ky (dat nap -> tay ve Home): TB {np.mean(cyc):.2f} s, min {np.min(cyc):.2f}, max {np.max(cyc):.2f}")
say(f"Thoi gian tu luc dat nap toi luc ra quyet dinh: TB {np.mean([r['t_decision'] - r['t_place'] for r in results]):.2f} s")

ROWS = ["green", "red", "white", "other"]
COLS = ["red", "green", "white", "reject", "ngoai"]
COL_VI = {"red": "ngăn 1\n(đỏ)", "green": "ngăn 2\n(xanh lá)", "white": "ngăn 3\n(trắng)", "reject": "vứt bỏ", "ngoai": "ngoài"}
ROW_VI = {"green": "xanh lá", "red": "đỏ", "white": "trắng", "other": "vật lạ"}
M = np.zeros((len(ROWS), len(COLS)), int)
for r in results:
    M[ROWS.index(r["color"]), COLS.index(r["final"])] += 1
rows_used = [i for i, rr in enumerate(ROWS) if M[i].sum() > 0]
say("Ma tran: mau that (hang) x noi nap nam sau chu trinh (cot: ngan 1 do, ngan 2 xanh, ngan 3 trang, vut bo, ngoai)")
for i in rows_used:
    say(f"  {ROWS[i]:6s} {M[i]}")
n_ok_dest = sum(r["ok"] for r in results)
n_ok_color = sum((r["final"] == r["color"]) or (r["color"] == "other" and r["final"] == "reject") for r in results)
say(f"Nap toi dung dich da chon (co hoc gap-tha): {n_ok_dest}/{len(results)}; "
    f"nap toi dung noi theo mau that (toan chu trinh): {n_ok_color}/{len(results)}")

# ----------------------------------------------------------------------- ghi file
keys = ["i", "cap", "color", "csv_row", "R", "G", "B", "C", "label", "nearest", "dist", "r", "g", "dest", "final",
        "ok", "t_place", "t_detect", "t_decision", "t_home", "z_tool_grasp", "grip_deg", "F_pad_N", "lift_mm",
        "slip_max_mm", "cap_bottom_drop", "final_pos"]
with open(os.path.join(args.out, "caps.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(keys)
    for r in results:
        w.writerow([(f"{r[k]:.4f}" if isinstance(r[k], float) else (" ".join(map(str, r[k])) if k == "final_pos" else r[k])) for k in keys])
with open(os.path.join(args.out, "log.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "pha", "nap", "servo0_deg", "servo1_deg", "servo2_deg", "kep_lenh_deg", "th1_deg", "th2_deg",
                "phi_deg", "kep_thuc_deg", "tool_x", "tool_y", "tool_z", "nap_x", "nap_y", "nap_z", "kep_cham_moi_truong"])
    for row in LOG:
        w.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in row])
with open(os.path.join(args.out, "summary.txt"), "w") as f:
    f.write("\n".join(LOG_LINES) + "\n")

fig, ax = plt.subplots(figsize=(6.4, 0.9 + 0.75 * len(rows_used)))
Mu = M[rows_used]
ax.imshow(Mu, cmap="Blues", vmin=0, vmax=max(1, Mu.max()))
for i in range(Mu.shape[0]):
    for j in range(Mu.shape[1]):
        ax.text(j, i, str(Mu[i, j]), ha="center", va="center", color="white" if Mu[i, j] > Mu.max() / 2 else "black")
ax.set_xticks(range(len(COLS))); ax.set_xticklabels([COL_VI[c] for c in COLS])
ax.set_yticks(range(len(rows_used))); ax.set_yticklabels([ROW_VI[ROWS[i]] for i in rows_used])
ax.set_xlabel("Nơi nắp nằm sau chu trình (mô phỏng)"); ax.set_ylabel("Màu thật")
ax.set_title(f"Phân loại {len(results)} nắp — seed {args.seed}")
fig.tight_layout(); fig.savefig(os.path.join(args.out, "sort_matrix.png"), dpi=130); plt.close(fig)

L = np.array([r[3:7] + r[10:11] for r in LOG], float)
t = np.array([r[0] for r in LOG])
fig, axs = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True)
for j, nm in enumerate(("servo đế", "servo vai", "servo khuỷu")):
    axs[0].plot(t, L[:, j], label=nm)
axs[0].set_ylabel("lệnh servo (độ)"); axs[0].legend(loc="upper right", fontsize=8); axs[0].grid(alpha=0.3)
axs[1].plot(t, L[:, 3], label="lệnh kẹp"); axs[1].plot(t, L[:, 4], label="góc kẹp thực")
axs[1].set_ylabel("servo kẹp (độ)"); axs[1].set_xlabel("t (s)"); axs[1].legend(loc="upper right", fontsize=8); axs[1].grid(alpha=0.3)
for r in results:
    for a_ in axs:
        a_.axvline(r["t_place"], color="0.6", lw=0.6, ls=":")
fig.tight_layout(); fig.savefig(os.path.join(args.out, "sort_timeline.png"), dpi=130); plt.close(fig)

try:
    from PIL import Image
    rd = mujoco.Renderer(m, 720, 960)
    for nm, (look, az, el, dist) in {"top": ([0.25, 0.0, 0.0], 90, -89.9, 0.62),
                                     "persp": ([0.24, -0.03, 0.04], 205, -40, 0.6)}.items():
        cam = mujoco.MjvCamera(); cam.lookat[:] = look; cam.azimuth = az; cam.elevation = el; cam.distance = dist
        rd.update_scene(d, cam); Image.fromarray(rd.render()).save(os.path.join(args.out, f"scene_end_{nm}.png"))
    rd.close()
except Exception as e:                                       # khong co OpenGL (vd. SSH khong man hinh)
    print(f"(khong chup duoc anh canh: {type(e).__name__})")
print(f"\nDa luu summary.txt, caps.csv, log.csv, sort_matrix.png, sort_timeline.png vao {args.out}")
