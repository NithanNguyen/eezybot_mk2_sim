"""
mk2_kinematics.py — Dong hoc thuan/nguoc giai tich + lop anh xa goc servo
cho EEZYbotARM Mk2. Viet bang numpy thuan, khong phu thuoc MuJoCo, de port
sang C/C++ cho ESP32 (cung cong thuc).

Bien khop dong hoc (rad):
  th1 : goc khop de quanh truc dung (0 = huong +x)
  th2 : goc TUYET DOI cua canh tay chinh O->E so voi phuong ngang
  phi : goc TUYET DOI cua tay ngang E->W so voi phuong ngang
Vi co cau hinh binh hanh: servo khuyu (qua tay quay 003_varm) dat truc tiep
goc phi, KHONG phai goc tuong doi DH (phi - th2). Day la "lop anh xa" trong
de cuong. Neu ban can bang DH chuan: q3_DH = phi - th2.

Dau cong tac giu huong co dinh (hinh binh hanh thu 2 va 3), nen vi tri
p = (x, y, z) chi phu thuoc (th1, th2, phi).

Dau cong tac (params.TOOL_OFFSETS) chon bang set_tool():
  "gripper" (mac dinh): TCP gap vat — giua 2 dem mui ngon luc dong.
  "pen": ngoi but — chi dung khi chay voi co --pen (model/mk2_pen.xml).
Dau cong tac lech khoi mat phang canh tay mot doan nho DY = TOOL_OFFSET_Y (mat doi xung gripper
khong trung mat phang khop). FK/IK duoi day tinh ca do lech nay.
"""
import math
import numpy as np
import params as P

L2 = P.L_UPPER
L3 = P.L_FORE
H = P.H_SHOULDER
DX, DZ = P.TOOL_OFFSETS["gripper"]
DY = P.TOOL_OFFSET_Y
OX = P.SHOULDER_OFFSET_X
TOOL = "gripper"


class IKError(ValueError):
    pass


def set_tool(name):
    """Chon dau cong tac cho FK/IK: "gripper" (TCP gap vat, mac dinh) hoac "pen" (ngoi but)."""
    global DX, DZ, TOOL
    if name not in P.TOOL_OFFSETS:
        raise ValueError(f"dau cong tac '{name}' khong co trong params.TOOL_OFFSETS {list(P.TOOL_OFFSETS)}")
    DX, DZ = P.TOOL_OFFSETS[name]
    TOOL = name


# ----------------------------------------------------------------- FK
def fk(th1, th2, phi):
    """Vi tri dau cong tac (mm)."""
    r = OX + L2 * math.cos(th2) + L3 * math.cos(phi) + DX
    z = H + L2 * math.sin(th2) + L3 * math.sin(phi) + DZ
    c1, s1 = math.cos(th1), math.sin(th1)
    return np.array([r * c1 - DY * s1, r * s1 + DY * c1, z])


def wrist(th1, th2, phi):
    """Vi tri tam co tay W (mm), khong co offset dau cong tac."""
    r = OX + L2 * math.cos(th2) + L3 * math.cos(phi)
    z = H + L2 * math.sin(th2) + L3 * math.sin(phi)
    return np.array([r * math.cos(th1), r * math.sin(th1), z])


# ----------------------------------------------------------------- IK
def ik(p, check_limits=True):
    """IK giai tich. p (mm) -> (th1, th2, phi) rad. Chon nghiem khuyu-tren
    (E nam tren duong O-W), dung voi cau hinh that cua Mk2."""
    x, y, z = p
    rho2 = x * x + y * y
    if rho2 <= DY * DY:
        raise IKError(f"diem qua gan truc de: {p}")
    rr = math.sqrt(rho2 - DY * DY)              # khoang cach trong mat phang canh tay
    th1 = math.atan2(y, x) - math.atan2(DY, rr)
    r = rr - DX - OX
    zz = z - H - DZ
    d2 = r * r + zz * zz
    c = (d2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if c > 1.0 or c < -1.0:
        raise IKError(f"diem ngoai tam voi: {p}")
    rel = -math.acos(c)                      # phi - th2 < 0: khuyu tren
    th2 = math.atan2(zz, r) - math.atan2(L3 * math.sin(rel), L2 + L3 * math.cos(rel))
    phi = th2 + rel
    q = (th1, th2, phi)
    if check_limits:
        ok, msg = within_limits(q)
        if not ok:
            raise IKError(msg + f" tai p={np.round(p, 1)}")
    return q


def phi_limits():
    """Gioi han goc phi hieu dung (do) = giao cua PHI_LIM (va cham STL) va CTRL_ELBOW_DEG (tam lenh
    servo khuyu trong model): IK khong duoc tra ve lenh ma actuator se cat."""
    return max(P.PHI_LIM[0], P.CTRL_ELBOW_DEG[0]), min(P.PHI_LIM[1], P.CTRL_ELBOW_DEG[1])


def within_limits(q):
    th1, th2, phi = (math.degrees(v) for v in q)
    rel = phi - th2
    if not (P.THETA1_LIM[0] <= th1 <= P.THETA1_LIM[1]):
        return False, f"th1={th1:.1f} ngoai {P.THETA1_LIM}"
    if not (P.THETA2_LIM[0] <= th2 <= P.THETA2_LIM[1]):
        return False, f"th2={th2:.1f} ngoai {P.THETA2_LIM}"
    if not (P.REL_LIM[0] <= rel <= P.REL_LIM[1]):
        return False, f"phi-th2={rel:.1f} ngoai {P.REL_LIM}"
    if not (P.PHI_LIM[0] <= phi <= P.PHI_LIM[1]):
        return False, f"phi={phi:.1f} ngoai {P.PHI_LIM}"
    # tam lenh actuator trong model (sim_common.q_to_ctrl): base = gear * th1, vai = th2 - 90, khuyu = phi
    for v, lim, nm in ((P.BASE_GEAR_RATIO * th1, P.CTRL_BASE_DEG, "CTRL_BASE_DEG"),
                       (th2 - 90.0, P.CTRL_SHOULDER_DEG, "CTRL_SHOULDER_DEG"),
                       (phi, P.CTRL_ELBOW_DEG, "CTRL_ELBOW_DEG")):
        if not (lim[0] <= v <= lim[1]):
            return False, f"lenh {nm}={v:.1f} ngoai {lim}"
    s = to_servo(q)
    for i, v in enumerate(s):
        if not (P.SERVO_RANGE[0] <= v <= P.SERVO_RANGE[1]):
            return False, f"servo{i}={v:.1f} ngoai {P.SERVO_RANGE}"
    return True, ""


# ----------------------------------------------------------------- Jacobian
def jacobian(th1, th2, phi):
    """J = dp/dq (mm/rad), q = (th1, th2, phi)."""
    r = OX + L2 * math.cos(th2) + L3 * math.cos(phi) + DX
    c1, s1 = math.cos(th1), math.sin(th1)
    dr2, dr3 = -L2 * math.sin(th2), -L3 * math.sin(phi)
    dz2, dz3 = L2 * math.cos(th2), L3 * math.cos(phi)
    return np.array([[-r * s1 - DY * c1, dr2 * c1, dr3 * c1],
                     [r * c1 - DY * s1, dr2 * s1, dr3 * s1],
                     [0.0, dz2, dz3]])


# ----------------------------------------------------------------- servo map
def to_servo(q):
    """(th1, th2, phi) rad -> goc lenh servo (do), theo params.SERVO_*."""
    th1, th2, phi = (math.degrees(v) for v in q)
    o, s, g = P.SERVO_OFFSET, P.SERVO_SIGN, P.BASE_GEAR_RATIO
    return np.array([o[0] + s[0] * g * th1,
                     o[1] + s[1] * (th2 - 90.0),
                     o[2] + s[2] * phi])


def from_servo(sv):
    o, s, g = P.SERVO_OFFSET, P.SERVO_SIGN, P.BASE_GEAR_RATIO
    th1 = (sv[0] - o[0]) / (s[0] * g)
    th2 = 90.0 + (sv[1] - o[1]) / s[1]
    phi = (sv[2] - o[2]) / s[2]
    return tuple(math.radians(v) for v in (th1, th2, phi))


# ----------------------------------------------------------------- linkage points
def tri_angles():
    """Goc (rad) cua 2 canh tam giac khuyu tai trang thai lap rap.
    BA song song O->F (hinh binh hanh O-F-A-E); BC suy ra tu hinh dang 007."""
    ax, az = P.TRI_A[0] - P.TRI_B[0], P.TRI_A[1] - P.TRI_B[1]
    cx, cz = P.TRI_C[0] - P.TRI_B[0], P.TRI_C[1] - P.TRI_B[1]
    ang_ba_local = math.atan2(az, ax)
    ang_bc_local = math.atan2(cz, cx)
    ang_f = math.atan2(P.F_REL_O[1], P.F_REL_O[0])
    rot = ang_f - ang_ba_local
    return ang_f, ang_bc_local + rot


def linkage_points(th2, phi):
    """Toa do (r, z) mm cua cac diem khop trong mat phang canh tay."""
    O = np.array([OX, H])
    E = O + L2 * np.array([math.cos(th2), math.sin(th2)])
    u = np.array([math.cos(phi), math.sin(phi)])
    W = E + L3 * u
    T = E - P.L_TAIL * u
    V = O - P.L_CRANK * u
    F = O + np.array(P.F_REL_O)
    a_ba, a_bc = tri_angles()
    A = E + P.TRI_ARM * np.array([math.cos(a_ba), math.sin(a_ba)])
    C = E + P.TRI_ARM * np.array([math.cos(a_bc), math.sin(a_bc)])
    Q = W + (C - E)
    return dict(O=O, E=E, W=W, T=T, V=V, F=F, A=A, C=C, Q=Q)


def tool_warning():
    return (f"Dau cong tac: {TOOL} {(round(DX, 2), round(DY, 2), round(DZ, 2))} mm so voi W. "
            "CANH BAO: PEN_LENGTH, TOOL_MASS_G, GRIP_CENTER_Y va gioi han khop trong params.py la gia tri tam "
            "[GIA-DINH]. Hay do tren tay may that roi sua lai.")
