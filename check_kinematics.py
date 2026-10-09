"""check_kinematics.py — Kiem chung dong hoc (muc 'Mo phong' trong de cuong):
  1. FK(IK(p)) - p tren luoi diem trong vung lam viec (cong thuc giai tich)
  2. FK giai tich == vi tri dau but trong MuJoCo co vong kin (khong trong luc)
     -> chung minh lop anh xa goc servo / hinh binh hanh dung
  3. Do vong do trong luc (servo P huu han) o vai tu the
  4. Ve vung lam viec (r, z) va tim kich thuoc hinh ve kha thi
[2] va [3] tat va cham ngoi-giay de so sanh dong hoc thuan (but co the xuyen giay o vai tu the).
Chay: python check_kinematics.py
"""
import math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import params as P
import mk2_kinematics as K
import sim_common as S
import trajectories as TR

OUT = os.path.join(S.HERE, "results"); os.makedirs(OUT, exist_ok=True)
rng = np.random.default_rng(0)
print(K.tool_warning())


def random_q(n):
    out = []
    while len(out) < n:
        q = (math.radians(rng.uniform(*P.THETA1_LIM)), math.radians(rng.uniform(*P.THETA2_LIM)), 0.0)
        rel = math.radians(rng.uniform(*P.REL_LIM))
        q = (q[0], q[1], q[1] + rel)
        if K.within_limits(q)[0]:
            out.append(q)
    return out


# ---- 1. FK(IK(p)) ------------------------------------------------------------
qs = random_q(5000)
err = []
for q in qs:
    p = K.fk(*q)
    q2 = K.ik(p, check_limits=False)
    err.append(np.linalg.norm(K.fk(*q2) - p))
    err.append(np.linalg.norm(np.array(q2) - np.array(q)) * 1000)   # mrad
err = np.array(err).reshape(-1, 2)
print(f"[1] FK(IK(p)) tren {len(qs)} diem: max |FK(IK(p))-p| = {err[:,0].max():.2e} mm, "
      f"max |IK(FK(q))-q| = {err[:,1].max():.2e} mrad")

# ---- 2. MuJoCo vong kin vs FK giai tich --------------------------------------
m, d = S.load_model(gravity=False, contact=False)
e_mj, viol = [], []
for q in random_q(40):
    S.settle(m, d, S.q_to_ctrl(q), seconds=1.5)
    e_mj.append(np.linalg.norm(d.site("tool").xpos * 1000 - K.fk(*q)))
    viol.append(S.loop_violation_mm(d))
print(f"[2] MuJoCo (vong kin, g=0) vs FK giai tich, 40 tu the: max sai lech = {max(e_mj):.3f} mm, "
      f"max vi pham rang buoc vong = {max(viol):.4f} mm")

# ---- 3. Vong do trong luc ----------------------------------------------------
mg, dg = S.load_model(gravity=True, contact=False)
print("[3] Do vong do trong luc (servo P huu han, params.SERVO_ERR_SAT_DEG):")
for qd in [(0, 90, 0), (0, 60, -30), (0, 45, -60), (0, 110, 0)]:
    q = tuple(math.radians(v) for v in qd)
    S.settle(mg, dg, S.q_to_ctrl(q), seconds=2.0)
    qa = S.actual_q(mg, dg)
    sag = np.linalg.norm(dg.site("tool").xpos * 1000 - K.fk(*q))
    print(f"    q={qd}: lech goc (deg) = {np.round(np.degrees(qa - q), 2)}, lech dau but = {sag:.2f} mm, "
          f"mo-men servo (Nm) = {np.round(dg.actuator_force[:3], 3)}")

# ---- 4. Vung lam viec + hinh ve kha thi -------------------------------------
fig, ax = plt.subplots(figsize=(6, 5))
pts = np.array([K.fk(*q) for q in random_q(20000)])
r = np.hypot(pts[:, 0], pts[:, 1])
ax.scatter(r, pts[:, 2], s=1, alpha=0.3)
ax.set_xlabel("r = sqrt(x^2+y^2) [mm]"); ax.set_ylabel("z [mm]")
ax.set_title("Vung lam viec dau but (mat phang canh tay)"); ax.axhline(0, color="k", lw=0.8)
ax.set_aspect("equal"); fig.tight_layout(); fig.savefig(os.path.join(OUT, "workspace_rz.png"), dpi=130)


def feasible(cx, R, z, margin_deg=3.0):
    segs, _ = TR.star_circle_path(cx, 0.0, R, z)
    for s in segs:
        for u in np.linspace(0, TR.seg_length(s), 40):
            try:
                q = K.ik(TR.seg_point(s, u))
            except K.IKError:
                return False
            # cach bien gioi han goc it nhat margin_deg
            th1, th2, phi = (math.degrees(v) for v in q)
            if (th2 - P.THETA2_LIM[0] < margin_deg or P.THETA2_LIM[1] - th2 < margin_deg or
                    (phi - th2) - P.REL_LIM[0] < margin_deg or P.REL_LIM[1] - (phi - th2) < margin_deg or
                    phi - P.PHI_LIM[0] < margin_deg or P.PHI_LIM[1] - phi < margin_deg):
                return False
    return True


best = []
for z in [P.PAPER_THICKNESS] + list(range(10, 101, 10)):
    bestR = (0, None)
    for cx in range(150, 331, 5):
        lo, hi = 0.0, 120.0
        if not feasible(cx, 5.0, z): continue
        for _ in range(14):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if feasible(cx, mid, z) else (lo, mid)
        if lo > bestR[0]: bestR = (lo, cx)
    if bestR[1] is not None:
        best.append((z, bestR[1], bestR[0]))
print("[4] Hinh ve lon nhat (sao + tron ban kinh R) theo do cao mat giay z (mm), cach gioi han >= 3 deg:")
for z, cx, R in best:
    tag = "   <- mat giay (params.PAPER_THICKNESS)" if z == P.PAPER_THICKNESS else ""
    print(f"    z_giay = {z:5.1f} mm: tam x = {cx:3d} mm, R_max = {R:5.1f} mm{tag}")
print(f"Da luu {os.path.join(OUT, 'workspace_rz.png')}")
