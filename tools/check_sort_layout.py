"""tools/check_sort_layout.py — Tinh lai cac so lieu [TINH] cua bai phan loai (params.py muc 8).

Chi dung dong hoc giai tich (mk2_kinematics.py) va hinh hoc gripper (gripper.py); khong can MuJoCo.
    python tools/check_sort_layout.py
In ra:
  [1] nap D30 trong gripper: goc servo vua cham nap, khe khi mo, khe toi cac chi tiet khong co va cham;
  [2] lop ngon nao om than nap o do cao kep CAP_GRASP_Z;
  [3] dai ban kinh voi toi (diem "cap") theo do cao, voi tam lenh khuyu cu (-57.3) va hien tai;
  [4] tung tu the cua bo tri: goc servo va du tru toi gioi han gan nhat.
"""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import params as P
import mk2_kinematics as K
import gripper as GR
import sort_scene as SS

print(f"[1] Nap D{P.CAP_DIAMETER:g}, tam cach truc banh rang GRIP_CAP_GY = {P.GRIP_CAP_GY:g} mm")
a_c = GR.cap_grip_deg()
print(f"    goc servo kep vua cham nap = {a_c:.2f} do; tam nap so voi W = {np.round(GR.cap_grip_w(), 2)} mm")
for ph in (25, 30, 35, P.GRIP_OPEN_DEG, 45):
    print(f"    mo {ph:4.1f} do: ngon cach nap {GR.cap_clearance(ph):5.2f} mm, mut ngon vuot mep truoc nap "
          f"{GR.finger_G(ph, -1)[:, 1].max() - P.GRIP_CAP_GY - P.CAP_DIAMETER / 2:5.1f} mm")
cl = GR.cap_link_clearance(a_c)
print("    khe toi chi tiet KHONG co va cham (luc kep): " + ", ".join(f"{k} {v:.1f}" for k, v in sorted(cl.items(), key=lambda kv: kv[1])[:4]) + " mm")

print(f"\n[2] Do cao kep CAP_GRASP_Z = {P.CAP_GRASP_Z:g} mm (diem 'cap' o giua chieu day ngon {P.GRIP_THICKNESS:g} mm)")
z_bot = P.CAP_GRASP_Z - P.GRIP_THICKNESS / 2
cap_lo, cap_hi = P.SHELF_HEIGHT, P.SHELF_HEIGHT + P.CAP_HEIGHT
print(f"    nap tren ke: z = {cap_lo:g} .. {cap_hi:g} mm; mep duoi ngon z = {z_bot:g} mm (cach mat mica {z_bot - P.SHELF_HEIGHT:.1f} mm)")
for nm, (a, b) in zip(("tam duoi", "lop long (thut 1.5 mm)", "tam tren"), GR.Z_FIN):
    lo, hi = z_bot + a, z_bot + b
    ov = max(0.0, min(hi, cap_hi) - max(lo, cap_lo))
    print(f"    {nm:22s} z = {lo:5.1f} .. {hi:5.1f} mm -> chong len than nap {ov:.1f} mm")

K.set_tool("cap")


def band(z, elbow_lo):
    old = P.CTRL_ELBOW_DEG; P.CTRL_ELBOW_DEG = (elbow_lo, old[1]); rs = []
    for r in np.arange(150, 380.01, 0.5):
        try:
            K.ik((r, 0, z)); rs.append(r)
        except K.IKError:
            pass
    P.CTRL_ELBOW_DEG = old
    return (min(rs), max(rs)) if rs else None


z_bin = P.BIN_HEIGHT + P.DROP_ABOVE + (P.CAP_GRASP_Z - P.SHELF_HEIGHT)
z_rej = P.DROP_ABOVE + (P.CAP_GRASP_Z - P.SHELF_HEIGHT)
print(f"\n[3] Dai ban kinh voi toi cua diem 'cap' (mm) theo do cao z, tam lenh khuyu cu (-57.3) / hien tai {P.CTRL_ELBOW_DEG[0]}:")
for z, nm in ((P.CAP_GRASP_Z, "kep nap tren ke"), (P.CAP_GRASP_Z + P.PRE_GRASP_LIFT, "phia tren diem kep"),
              (z_rej, "tha vung vut bo"), (z_bin, "tha vao hop")):
    print(f"    z = {z:5.1f} ({nm:18s}): cu {band(z, -57.3)}  |  hien tai {band(z, P.CTRL_ELBOW_DEG[0])}")


def margin(q):
    th1, th2, phi = (math.degrees(v) for v in q)
    lo, hi = K.phi_limits()
    return min(th1 - P.THETA1_LIM[0], P.THETA1_LIM[1] - th1, th2 - P.THETA2_LIM[0], P.THETA2_LIM[1] - th2,
               phi - lo, hi - phi, phi - th2 - P.REL_LIM[0], P.REL_LIM[1] - (phi - th2))


print("\n[4] Tu the cua bo tri (he the gioi, mm) -> goc servo (do), du tru toi gioi han gan nhat (do):")
st = SS.station()
pts = {"kep nap tai tram": (st[0], st[1], P.CAP_GRASP_Z), "phia tren tram": (st[0], st[1], P.CAP_GRASP_Z + P.PRE_GRASP_LIFT)}
for c, xy in SS.bin_targets().items():
    pts[f"tha ngan {P.BIN_NUMBER[c]} ({c})"] = (xy[0], xy[1], z_bin)
rj = SS.reject_target()
pts["tha vung vut bo"] = (rj[0], rj[1], z_rej)
for nm, p in pts.items():
    q = K.ik(p)
    print(f"    {nm:22s} r = {math.hypot(p[0], p[1]):6.1f}, goc {math.degrees(math.atan2(p[1], p[0])):6.1f}, z = {p[2]:5.1f}"
          f"  servo = {np.round(K.to_servo(q), 1)}  du tru = {margin(q):.2f}")
Lu, Lv, H = SS.bin_outer()
print(f"    hop dich: ngoai {Lu:g} x {Lv:g} x {H:g} mm, tam r = {P.BIN_CENTER_R:g} mm, goc {P.BIN_ANGLE_DEG:g} do")
