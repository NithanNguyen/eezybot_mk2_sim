"""run_sim.py — Mo phong thi nghiem ve ngoi sao + duong tron (de cuong, muc
'Thuc nghiem danh gia'): so sanh M0 (noi suy tuyen tinh) va M2 (double-S).

Kep giu but D10 thang dung, to giay A4 day 1 mm nam tren mat ban. Nét muc (rong 1 mm)
chi duoc ghi khi MuJoCo bao co tiep xuc giua ngoi but va giay.

Vi du:
    python run_sim.py                         # chay ca M0, M2, luu do thi + CSV + anh vao results/
    python run_sim.py --view                  # xem truc tiep trong cua so MuJoCo (thay vet muc)
    python run_sim.py --R 30 --vmax 150 --amax 1500 --jmax 30000

Chuoi xu ly giong firmware: p(t) -> IK -> goc servo (do) -> [giu 20 ms] -> servo.
Moi lan ve: ha but thang dung xuong diem dau (1 s) -> dung 0.3 s -> ve (M0 hoac M2)
-> dung 0.3 s -> nhac but len PEN_LIFT mm (0.5 s). Chi so chi tinh tren phan ve.
"""
import argparse, csv, math, os
import numpy as np
import matplotlib
import mujoco
import params as P
import mk2_kinematics as K
import sim_common as S
import trajectories as TR

ap = argparse.ArgumentParser()
ap.add_argument("--cx", type=float, default=P.PAPER_CENTER[0], help="tam hinh, x (mm)")
ap.add_argument("--cy", type=float, default=P.PAPER_CENTER[1], help="tam hinh, y (mm)")
ap.add_argument("--z", type=float, default=P.PAPER_THICKNESS, help="do cao LENH cua ngoi but khi ve (mm), mac dinh = mat giay")
ap.add_argument("--R", type=float, default=40.0, help="ban kinh duong tron (mm)")
ap.add_argument("--vmax", type=float, default=150.0, help="mm/s   [GIA-DINH]")
ap.add_argument("--amax", type=float, default=1500.0, help="mm/s^2 [GIA-DINH]")
ap.add_argument("--jmax", type=float, default=30000.0, help="mm/s^3 [GIA-DINH]")
ap.add_argument("--view", action="store_true", help="mo cua so MuJoCo xem truc tiep")
args = ap.parse_args()
if not args.view:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.join(S.HERE, "results"); os.makedirs(OUT, exist_ok=True)
print(K.tool_warning())

# hinh phai nam tren giay (cach mep >= 5 mm)
hx, hy = P.PAPER_SIZE[0] / 2 - 5, P.PAPER_SIZE[1] / 2 - 5
if abs(args.cx - P.PAPER_CENTER[0]) + args.R > hx or abs(args.cy - P.PAPER_CENTER[1]) + args.R > hy:
    raise SystemExit(f"Hinh ve (tam {args.cx},{args.cy}, R={args.R}) nam ngoai to giay "
                     f"(tam {P.PAPER_CENTER}, {P.PAPER_SIZE[0]} x {P.PAPER_SIZE[1]} mm). Sua params.PAPER_CENTER roi build lai.")

# ---------------------------------------------------------------- quy dao lenh
segs, verts = TR.star_circle_path(args.cx, args.cy, args.R, args.z)
p2, sid2 = TR.m2_path(segs, args.vmax, args.amax, args.jmax)
T = len(p2) * P.CMD_PERIOD
p0, sid0 = TR.m0_path(segs, T)
L = sum(TR.seg_length(s) for s in segs)
print(f"Duong di L = {L:.1f} mm (ly thuyet 13.55R = {13.55*args.R:.1f}); T(M2) = {T:.2f} s; "
      f"v(M0) = L/T = {L/T:.1f} mm/s")

lift = np.array([0, 0, P.PEN_LIFT])
start = segs[0][1]
down = TR.line_path(start + lift, start, 1.0)
n_dwell = int(round(0.3 / P.CMD_PERIOD))

runs = {}
sv = P.MG996R[P.SUPPLY]; w_servo = math.radians(60) / sv["s_per_60deg"]
for name, pts, sid in [("M0", p0, sid0), ("M2", p2, sid2)]:
    end = pts[-1]
    up = TR.line_path(end, end + lift, 0.5)
    full = np.vstack([down, [start] * n_dwell, pts, [end] * n_dwell, up])
    phase = np.r_[np.zeros(len(down) + n_dwell), np.ones(len(pts)), 2 * np.ones(n_dwell + len(up))].astype(int)
    q_full = TR.to_joint(full)                                  # IK moi 20 ms
    servo = np.array([K.to_servo(qq) for qq in q_full])         # lenh servo (do)
    q = q_full[phase == 1]
    qd, qdd, qddd = TR.derivs(q)
    w = np.abs(TR.derivs(q_full)[0]) * np.array([P.BASE_GEAR_RATIO, 1, 1])
    print(f"[{name}] toc do servo lon nhat = {w.max(0).round(2)} rad/s "
          f"(MG996R khong tai {w_servo:.2f} rad/s @{P.SUPPLY}) -> "
          f"{'OK' if w.max() < w_servo else 'VUOT gioi han, can gian thoi gian'}")
    runs[name] = dict(pts=full, phase=phase, sid=sid, q=q, servo=servo, qd=qd, qdd=qdd, qddd=qddd,
                      i0=len(down) + n_dwell)

# ---------------------------------------------------------------- mo phong MuJoCo
m, d = S.load_model()
viewer = None
if args.view:
    import mujoco.viewer
    viewer = mujoco.viewer.launch_passive(m, d)
    with viewer.lock():                                         # camera nhin vao to giay
        viewer.cam.lookat[:] = [args.cx / 1000 - 0.05, args.cy / 1000, 0.06]
        viewer.cam.distance = 0.55; viewer.cam.azimuth = -150; viewer.cam.elevation = -32
for name, R_ in runs.items():
    mujoco.mj_resetData(m, d)
    d.ctrl[:] = 0.0                                             # servo kep = 0: kep but
    ctrl = np.array([S.servo_to_ctrl(s) for s in R_["servo"]])
    S.settle(m, d, ctrl[0], seconds=2.0)                        # dua but toi tren diem dau
    d.time = 0.0
    ink = S.Ink(m); drawer = S.InkDrawer(S.INK_RGBA[name])
    if viewer is not None:
        with viewer.lock():
            viewer.user_scn.ngeom = 0                           # to giay moi cho moi lan ve
    R_["log"] = S.run_commands(m, d, ctrl, viewer=viewer, realtime=args.view, ink=ink, drawer=drawer)
    R_["ink"] = ink
    lg = R_["log"]; ph = R_["phase"][lg["k"]]
    draw = ph == 1
    cover = lg["ink"][draw].mean() * 100
    fn = lg["fn"][draw & lg["ink"]]
    R_["cover"] = cover
    print(f"[{name}] max vi pham rang buoc vong = {lg['viol'].max():.3f} mm, "
          f"max |mo-men servo| = {np.abs(lg['frc']).max(0).round(3)} Nm")
    print(f"[{name}] ngoi cham giay {cover:.1f}% thoi gian ve; so net muc = {len(ink.strokes)}; "
          f"luc ep ngoi TB/max = {fn.mean() if fn.size else 0:.2f}/{fn.max() if fn.size else 0:.2f} N")
    # anh chup cuoi lan ve (can OpenGL; bo qua neu khong co)
    try:
        r = mujoco.Renderer(m, 720, 960)
        cam = mujoco.MjvCamera(); cam.lookat[:] = [args.cx / 1000, args.cy / 1000, 0.02]
        cam.distance = 0.42; cam.azimuth = -150; cam.elevation = -38
        r.update_scene(d, cam)
        S.InkDrawer(S.INK_RGBA[name]).draw(r.scene, ink, append=False)
        from PIL import Image
        Image.fromarray(r.render()).save(os.path.join(OUT, f"ink_{name}.png"))
        r.close()
    except Exception as e:                                      # noqa: BLE001
        print(f"  (khong chup duoc anh ink_{name}.png: {e})")
if viewer is not None:
    viewer.close()


# ---------------------------------------------------------------- chi so nhu tren giay (tren vet MUC)
def metrics(R_):
    pts = np.array([p for st in R_["ink"].strokes for p in st])
    ph = R_["phase"][pts[:, 2].astype(int)]
    pts = pts[ph == 1]
    xy = pts[:, :2]
    sid = R_["sid"][pts[:, 2].astype(int) - R_["i0"]]

    def fit_line(i):
        """Duong thang khop voi phan giua (15-85%) cua net ve canh i (nhu ke thuoc tren giay)."""
        p0_, p1_ = segs[i][1][:2], segs[i][2][:2]
        e = (p1_ - p0_) / np.linalg.norm(p1_ - p0_)
        pts_ = xy[sid == i]; pr = (pts_ - p0_) @ e; ln = np.linalg.norm(p1_ - p0_)
        pts_ = pts_[(pr > 0.15 * ln) & (pr < 0.85 * ln)]
        if len(pts_) < 3: return None, None
        c = pts_.mean(0); _, _, vt = np.linalg.svd(pts_ - c)
        dvec = vt[0] if vt[0] @ e > 0 else -vt[0]
        return c, dvec

    over, corner = [], []
    for i in range(5):                     # mui sao i = dinh verts[2i], canh vao = 2i-1, canh ra = 2i
        c1, u = fit_line((2 * i - 1) % 10); c2, w2 = fit_line(2 * i)
        if c1 is None or c2 is None:
            over.append(np.nan); corner.append(np.nan); continue
        A_ = np.c_[u, -w2]; s_ = np.linalg.solve(A_, c2 - c1)
        Vd = c1 + s_[0] * u                 # giao diem 2 canh VE THAT keo dai
        near = (np.linalg.norm(xy - Vd, axis=1) < 0.4 * args.R) & np.isin(sid, [(2 * i - 1) % 10, 2 * i])
        over.append(float(((xy[near] - Vd) @ u).max()) if near.any() else np.nan)
        corner.append(float(np.linalg.norm(xy[near] - Vd, axis=1).min()) if near.any() else np.nan)
    straight = []
    for i in range(10):
        p0_, p1_ = segs[i][1][:2], segs[i][2][:2]
        e = (p1_ - p0_) / np.linalg.norm(p1_ - p0_); nrm = np.array([-e[1], e[0]])
        sel = sid == i
        proj = (xy[sel] - p0_) @ e; ln = np.linalg.norm(p1_ - p0_)
        mid = (proj > 0.15 * ln) & (proj < 0.85 * ln)
        straight.append(np.abs((xy[sel][mid] - p0_) @ nrm).max() if mid.any() else np.nan)
    circ = xy[sid == 10] - np.array([args.cx, args.cy])
    D = []
    if len(circ) > 10:
        ang = np.arctan2(circ[:, 1], circ[:, 0]); rad = np.hypot(circ[:, 0], circ[:, 1])
        for a in np.radians([0, 45, 90, 135]):
            r1 = rad[np.argmin(np.abs(np.angle(np.exp(1j * (ang - a)))))]
            r2 = rad[np.argmin(np.abs(np.angle(np.exp(1j * (ang - a - np.pi)))))]
            D.append(r1 + r2)
    return dict(overshoot=np.array(over), corner=np.array(corner), straight=np.array(straight),
                roundness=(max(D) - min(D)) if D else np.nan)


rows = []
for name, R_ in runs.items():
    mt = metrics(R_); R_["metrics"] = mt
    rows.append((name, np.nanmean(mt["overshoot"]), np.nanmax(mt["overshoot"]), np.nanmean(mt["corner"]),
                 mt["roundness"], np.nanmax(mt["straight"]), R_["cover"]))
print("\nChi so du doan tu mo phong (cung T), do tren VET MUC: 5 mui sao / duong tron / 10 canh:")
print(f"{'PP':4s} {'vot lo TB':>10s} {'vot lo max':>11s} {'sai lech dinh TB':>17s} {'do tron':>8s} {'lech thang max':>15s} {'co muc %':>9s}   (mm)")
for r in rows:
    print(f"{r[0]:4s} {r[1]:10.2f} {r[2]:11.2f} {r[3]:17.2f} {r[4]:8.2f} {r[5]:15.2f} {r[6]:9.1f}")
print("  vot lo < 0: net ve cat goc, khong cham giao diem 2 canh keo dai")

# ---------------------------------------------------------------- luu CSV
for name, R_ in runs.items():
    with open(os.path.join(OUT, f"log_{name}.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "phase", "cmd_x", "cmd_y", "cmd_z", "servo0_deg", "servo1_deg", "servo2_deg",
                    "tool_x", "tool_y", "tool_z", "th1_deg", "th2_deg", "phi_deg", "tau0", "tau1", "tau2",
                    "ink", "fn_N"])
        lg = R_["log"]
        for i in range(len(lg["t"])):
            k = lg["k"][i]
            w.writerow([f"{lg['t'][i]:.3f}", R_["phase"][k], *np.round(R_["pts"][k], 3), *np.round(R_["servo"][k], 3),
                        *np.round(lg["tool"][i], 3), *np.round(np.degrees(lg["q"][i]), 3), *np.round(lg["frc"][i], 4),
                        int(lg["ink"][i]), round(float(lg["fn"][i]), 4)])

# ---------------------------------------------------------------- do thi
t = np.arange(len(runs["M2"]["q"])) * P.CMD_PERIOD
fig, axs = plt.subplots(3, 3, figsize=(13, 8), sharex=True)
names = ["khop de th1", "vai th2", "khuyu phi"]
for j in range(3):
    for r, (lab, unit) in enumerate([("qd", "deg/s"), ("qdd", "deg/s^2"), ("qddd", "deg/s^3")]):
        for name, c in [("M0", "tab:red"), ("M2", "tab:blue")]:
            axs[r, j].plot(t, np.degrees(runs[name][lab][:, j]), c, lw=0.8, label=name)
        axs[r, j].set_ylabel(f"{lab} [{unit}]")
    axs[0, j].set_title(names[j]); axs[2, j].set_xlabel("t [s]")
axs[0, 0].legend()
fig.suptitle("Bang chung 1: van toc / gia toc / jerk cua LENH goc khop (sai phan 20 ms, phan ve)")
fig.tight_layout(); fig.savefig(os.path.join(OUT, "cmd_derivatives.png"), dpi=120)

ideal = runs["M2"]["pts"][runs["M2"]["phase"] == 1]
fig, axs = plt.subplots(1, 3, figsize=(15, 5))
for ax, name, c in [(axs[0], "M0", "tab:red"), (axs[1], "M2", "tab:blue")]:
    ax.plot(ideal[:, 0], ideal[:, 1], "k-", lw=0.6, label="ly tuong")
    for i, st in enumerate(runs[name]["ink"].strokes):
        st = np.array(st)
        ax.plot(st[:, 0], st[:, 1], color=c, lw=1.0, label="vet muc" if i == 0 else None)
    ax.set_aspect("equal"); ax.set_title(f"{name}: vet muc (net {P.INK_WIDTH:g} mm, chi khi ngoi cham giay)")
    ax.legend(fontsize=8); ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]")
V = np.array(verts[0])
for name, c in [("M0", "tab:red"), ("M2", "tab:blue")]:
    for i, st in enumerate(runs[name]["ink"].strokes):
        st = np.array(st)
        axs[2].plot(st[:, 0], st[:, 1], color=c, lw=P.INK_WIDTH * 2.2, alpha=0.6, label=name if i == 0 else None,
                    solid_capstyle="round")
axs[2].plot(*np.array([verts[9], verts[0], verts[1]]).T, "k--", lw=0.8, label="ly tuong")
axs[2].set_xlim(V[0] - 12, V[0] + 6); axs[2].set_ylim(V[1] - 9, V[1] + 9); axs[2].set_aspect("equal")
axs[2].set_title("Phong to mui sao (x lon nhat)"); axs[2].legend(fontsize=8)
fig.tight_layout(); fig.savefig(os.path.join(OUT, "pen_trace.png"), dpi=120)
print(f"\nDa luu do thi, log va anh vao {OUT}")
