"""
build_model.py — Dung mo hinh MuJoCo (MJCF) cua EEZYbotARM Mk2 tu cac file STL goc,
gom ca kep (gripper), but ve, to giay A4 va hop bao servo.

Cach dung:
    python build_model.py --stl-dir <thu muc chua EBAmk2_*.STL>

Ket qua:
    model/meshes/*.stl   (STL nhi phan, da dat vao he toa do cua tung body)
    model/mk2.xml        (MJCF)
    model/mass_report.txt

Nguyen tac lap rap (kiem chung bang tools/check_assembly.py):
  * Moi luoi hien thi la file STL goc, khong sua.
  * Trong mat phang canh tay: 2 tam lo truc cua chi tiet dat trung 2 diem khop tinh tu params.py.
  * Theo truc khop (truc y the gioi): moi chi tiet co chieu lap FLIP (+1/-1) va vi tri Y0
    (= toa do y the gioi cua mat toa do 0 doc truc lo trong file STL). Bo gia tri trong
    bang ARM la nghiem cua bai toan xep chong: chi tiet chung mot truc nam sat nhau,
    khong chi tiet nao lan vao nhau o moi tu the trong gioi han khop.
  * Kep 014 lap vao luoi duoi en cua 009; 2 ngon (015 phai, 017 trai) doi xung guong,
    an khop nhau bang banh rang 12 rang; 018 cam 2 chot vao ngon trai; 016 tren truc SG90.
  * 3 vong hinh binh hanh dong bang <equality><connect>; banh rang kep va banh rang de
    dong bang <equality><joint>.
"""
import argparse, math, os, sys
import numpy as np
import trimesh
from scipy.optimize import brentq

import params as P
import mk2_kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "model")
MESH_OUT = os.path.join(OUT, "meshes")
MM = 1e-3

# --------------------------------------------------------------------------
# Tam lo truc do tren STL (mm, he rieng cua tung file). Nguon: tools/holes.py
# axis: truc lo; h: toa do 2 truc con lai (theo thu tu x,y,z bo axis)
# --------------------------------------------------------------------------
HOLES = {
    "EBAmk2_002_mainarm":        dict(axis=2, h={"O": (16.16, 16.16), "E": (16.16, 151.16)}),
    "EBAmk2_003_varm":           dict(axis=2, h={"O": (15.00, 15.00), "V": (72.00, 15.00)}),
    "EBAmk2_004_link135":        dict(axis=2, h={"V": (6.56, 6.57), "T": (6.57, 141.57)}),
    "EBAmk2_005_link135angled":  dict(axis=2, h={"F": (20.20, 7.57), "A": (20.20, 142.58)}),
    "EBAmk2_006_horarm__":       dict(axis=0, h={"W": (6.51, 17.66), "E": (152.96, 30.47), "T": (209.74, 35.44)}),
    # tam giac doi xung: lap kieu FLIP=+1 thi lo "A" cua co cau la lo TRI_C cua file STL
    "EBAmk2_007_trialink":       dict(axis=2, h={"A": P.TRI_C, "E": P.TRI_B, "C": P.TRI_A}),
    "EBAmk2_008_link147_new":    dict(axis=2, h={"C": (6.56, 6.57), "Q": (6.57, 153.57)}),
    # W la lo tai vau hep 12 mm (vua khe 12 mm cua 006), Q la lo tai vau rong 22 mm.
    # Lap dung chieu nay thi luoi duoi en cua 009 huong xuong de gan kep.
    "EBAmk2_009_trialinkfront":  dict(axis=1, h={"W": (6.57, 39.68), "Q": (34.45, 6.95)}),
}

# Xep chong theo truc khop: (lo 1, lo 2, FLIP, Y0 mm). Nguon: tools/check_assembly.py
ARM = {
    "EBAmk2_002_mainarm":       ("O", "E", -1, 11.72),
    "EBAmk2_003_varm":          ("O", "V", +1, 11.49),
    "EBAmk2_004_link135":       ("V", "T", +1, 17.15),
    "EBAmk2_005_link135angled": ("F", "A", -1, -12.95),
    "EBAmk2_006_horarm__":      ("E", "W", -1, 20.51),
    "EBAmk2_007_trialink":      ("E", "A", +1, -28.76),
    "EBAmk2_008_link147_new":   ("C", "Q", -1, -12.95),
    "EBAmk2_009_trialinkfront": ("W", "Q", +1, -13.05),
}

# --------------------------------------------------------------------------
# Kep, trong he toa do cua 014_claw_base (mm). Nguon: tools/holes.py + tools/check_assembly.py
# --------------------------------------------------------------------------
# 014 -> 009: khop duoi en (ranh chu T cua 014 om dau luoi 009; mat day ranh cham dau luoi)
CLAW_IN_009_R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)
CLAW_IN_009_T = np.array([-3.33, 0.90, 6.69])
PIV_DX, PIV_SX = np.array([20.56, 3.58]), np.array([20.56, 21.67])   # lo truc ngon tren 014
LP_DX, LP_SX = np.array([47.50, 10.51]), np.array([48.25, 11.12])     # lo truc trong file ngon
Z_DX, Z_SX = 0.25 - 3.41, 0.25 - 3.35          # mat tren moay-o ngon cham mat duoi 014
MIRROR_DEG = 14.38                              # 017 = guong cua 015 xoay 14.38 do quanh lo truc
CLOSED_DEG = 180.0                              # goc 015 khi 2 ma kep cham nhau
LP_018 = np.array([12.78, 7.54]); D018_DEG = 14.0   # 018 cam chot vao ngon trai
Z_018 = Z_SX + 0.05 - 5.03                       # mat moay-o 018 cham mat duoi ngon trai
SG90_WIN = (21.56, 44.56, 6.38, 18.88)          # cua so lap SG90 tren 014 (x0, x1, y0, y1)
R016, R018 = 7.5, 12.0                           # ban kinh vong chia (m = 1.5; 10 va 16 rang)
LP_016 = np.array([8.77, 9.09]); D016_DEG = 26.0   # pha rang 016 an khop 018 o goc kep but


def sg90_shaft():
    """Truc SG90 tren duong tam cua so, cach truc ngon trai R016 + R018 (an khop 016-018)."""
    yc = (SG90_WIN[2] + SG90_WIN[3]) / 2
    dy = PIV_SX[1] - yc
    return np.array([PIV_SX[0] + math.sqrt((R016 + R018) ** 2 - dy ** 2), yc])


def Tz(angle_deg, lp, piv, z):
    """Quay chi tiet quanh truc z (do), dua diem lp (he rieng) toi piv (he kep), cao do z."""
    a = math.radians(angle_deg); R = np.eye(3)
    R[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.r_[piv, z] - R @ np.r_[lp, 0.0]
    return T


_CACHE = {}


def load(stl_dir, name):
    p = os.path.join(stl_dir, name + ".STL")
    if p not in _CACHE:
        if not os.path.exists(p):
            sys.exit(f"Khong tim thay {p}. Kiem tra --stl-dir (thu muc chua cac file EBAmk2_*.STL).")
        _CACHE[p] = trimesh.load(p, force="mesh")
    return _CACHE[p]


def hole3(name, key):
    """Tam lo -> diem 3D (mm) trong he rieng, toa do doc truc = 0."""
    d = HOLES[name]; v = np.zeros(3)
    others = [i for i in range(3) if i != d["axis"]]
    v[others[0]], v[others[1]] = d["h"][key]
    return v


def place_arm(name, pts):
    """T (4x4, mm): he rieng -> the gioi. 2 lo -> 2 diem khop (mat phang x-z); truc lo -> FLIP*y;
    mat toa do 0 doc truc lo -> y = Y0."""
    k1, k2, flip, y0 = ARM[name]
    ax = HOLES[name]["axis"]; a = np.zeros(3); a[ax] = 1.0
    h1, h2 = hole3(name, k1), hole3(name, k2)
    P1, P2 = pts[k1], pts[k2]
    vl, vw = h2 - h1, P2 - P1
    if abs(np.linalg.norm(vl) - np.linalg.norm(vw)) > 0.3:
        raise RuntimeError(f"{name}: khoang cach lo {np.linalg.norm(vl):.2f} != {np.linalg.norm(vw):.2f}")
    e1l = vl / np.linalg.norm(vl); e2l = flip * a; e3l = np.cross(e1l, e2l)
    e1w = vw / np.linalg.norm(vw); e2w = np.array([0, 1.0, 0]); e3w = np.cross(e1w, e2w)
    R = np.c_[e1w, e2w, e3w] @ np.c_[e1l, e2l, e3l].T
    t = P1 - R @ h1; t[1] = y0
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T


def mass_props(mesh_mm, name):
    """Khoi luong (kg), tam (m), ten-xo quan tinh quanh tam (kg m^2) trong he cua luoi."""
    m = mesh_mm.copy()
    note = "luoi kin"
    if not m.is_watertight:
        trimesh.repair.fill_holes(m); trimesh.repair.fix_normals(m)
        note = "luoi da va lo"
    if not m.is_watertight:
        # luoi van con ho: lap day theo tung lat cat z (quy tac chan-le), o luoi 0.3 mm
        from matplotlib.path import Path
        pitch = 0.3; lo, hi = m.bounds
        xs = np.arange(lo[0] + pitch / 2, hi[0], pitch); ys = np.arange(lo[1] + pitch / 2, hi[1], pitch)
        X, Y = np.meshgrid(xs, ys, indexing="ij"); XY = np.c_[X.ravel(), Y.ravel()]
        pts = []
        for z in np.arange(lo[2] + pitch / 2, hi[2], pitch):
            s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            if s is None: continue
            c = sum(Path(d_[:, :2]).contains_points(XY).astype(int) for d_ in s.discrete if len(d_) > 3)
            sel = (np.asarray(c) % 2 == 1)
            if np.any(sel): pts.append(np.c_[XY[sel], np.full(sel.sum(), z)])
        pts = np.vstack(pts)
        vol = len(pts) * pitch ** 3
        com = pts.mean(0)
        d = pts - com
        Iunit = (np.eye(3) * (d ** 2).sum(1).sum() - d.T @ d) / len(pts)
        note = "lat cat 0.3mm"
    else:
        vol = abs(m.volume); com = m.center_mass
        Iunit = m.moment_inertia / m.mass if m.mass != 0 else np.zeros((3, 3))
        if m.volume < 0: Iunit = -Iunit
    mass = vol * (MM ** 3) * P.PLA_DENSITY * P.FILL_FACTOR
    if name in P.MEASURED_MASS_G:
        mass = P.MEASURED_MASS_G[name] / 1000.0
        note += ", khoi luong CAN THUC TE"
    I = mass * Iunit * (MM ** 2)
    return mass, com * MM, I, vol, note


def combine(items):
    """items: list of (mass, com(3), I_about_com(3x3)) trong cung he -> tong."""
    M = sum(i[0] for i in items)
    c = sum(i[0] * np.asarray(i[1]) for i in items) / M
    I = np.zeros((3, 3))
    for m, ci, Ii in items:
        d = np.asarray(ci) - c
        I += Ii + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, c, I


def box_inertia(m, sx, sy, sz):
    return m / 12.0 * np.diag([sy * sy + sz * sz, sx * sx + sz * sz, sx * sx + sy * sy])


def fmt(v):
    return " ".join(f"{x:.6g}" for x in np.ravel(v))


def grip_angle(dx_mesh):
    """Goc ngon 015 (do) de 2 ma kep vua cham but D = PEN_DIAMETER, tam but nam giua mat kep.
    Tra ve (goc, tam but (x, y) trong he kep)."""
    s = dx_mesh.section(plane_origin=[0, 0, 6.0], plane_normal=[0, 0, 1])   # giua chieu cao ma kep
    L = [d[:, :2] for d in s.discrete if d[:, 0].max() < 14][0]
    Pj = np.vstack([a + (b - a) * np.linspace(0, 1, 40, endpoint=False)[:, None] for a, b in zip(L[:-1], L[1:])])
    yc = (PIV_DX[1] + PIV_SX[1]) / 2
    def placed(a):
        a = math.radians(a); R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        return (Pj - LP_DX) @ R.T + PIV_DX
    Q = placed(CLOSED_DEG); face = Q[Q[:, 1] > Q[:, 1].max() - 0.6]
    xp = (face[:, 0].min() + face[:, 0].max()) / 2
    f = lambda a: np.min(np.hypot(*(placed(a) - [xp, yc]).T)) - P.PEN_DIAMETER / 2
    return brentq(f, CLOSED_DEG - 40, CLOSED_DEG), np.array([xp, yc])


def assemble(stl_dir, th2_deg=90.0, phi_deg=0.0, verbose=False):
    """Dat moi chi tiet STL o tu the (theta1 = 0, theta2, phi). Tra ve dict:
    meshes {ten: (luoi mm, T he rieng -> the gioi mm)} cho cac chi tiet chuyen dong,
    static {...} cho de co dinh, va cac dai luong phu (diem khop, he kep, goc kep...)."""
    # ---------------- diem khop o tu the home (th2 = 90 do, phi = 0), mat phang x-z, mm
    lp = K.linkage_points(math.radians(th2_deg), math.radians(phi_deg))
    W3 = {k: np.array([v[0], 0.0, v[1]]) for k, v in lp.items()}
    O, E, Wp, T, V, F, A, C, Q = (W3[k] for k in "O E W T V F A C Q".split())

    meshes = {}   # name -> (mesh mm, T he rieng -> the gioi mm)
    for name in ARM:
        meshes[name] = (load(stl_dir, name), place_arm(name, W3))

    # kiem tra lo thu 3 (khong dung de dat chi tiet)
    for name, key, target in [("EBAmk2_007_trialink", "C", C), ("EBAmk2_006_horarm__", "T", T)]:
        m, Tm = meshes[name]
        p = Tm[:3, :3] @ hole3(name, key) + Tm[:3, 3]
        err = np.linalg.norm((p - target)[[0, 2]])
        if verbose: print(f"  kiem tra lo {key} cua {name}: lech {err:.2f} mm")
        if err > 1.0:
            raise RuntimeError("Hinh hoc khong khop — kiem tra lai HOLES/params")

    # ---------------- de quay 001 + gearmast 011 (quay theo khop de)
    base_bottom = P.H_SHOULDER - 43.00
    R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])           # rieng y->x, x->-y, z->z
    Tm = np.eye(4); Tm[:3, :3] = R; Tm[:3, 3] = np.array([0, 0, base_bottom]) - R @ np.array([40.40, 46.43, 0])
    meshes["EBAmk2_001_base"] = (load(stl_dir, "EBAmk2_001_base"), Tm)
    for key, target in [("O", O), ("F", F)]:
        p = Tm[:3, :3] @ np.array([0, *{"O": (46.43, 43.00), "F": (8.46, 63.19)}[key]]) + Tm[:3, 3]
        if verbose: print(f"  kiem tra lo {key} cua 001_base: lech {np.linalg.norm((p - target)[[0, 2]]):.2f} mm")
    gm = "EBAmk2_011_gearmast"
    R = np.diag([1.0, -1.0, -1.0])
    Tm = np.eye(4); Tm[:3, :3] = R; Tm[:3, 3] = np.array([0, 0, base_bottom]) - R @ np.array([39.34, 40.40, 0])
    meshes[gm] = (load(stl_dir, gm), Tm)

    # ---------------- de co dinh: 012, 013, 019, gearservo 010 (an khop gearmast, m = 1.5)
    static = {}
    Tm = np.eye(4); Tm[:3, 3] = -np.array([98.98, 45.21, 0])        # truc dung tai (98.98, 45.21)
    static["EBAmk2_012_mainbase"] = (load(stl_dir, "EBAmk2_012_mainbase"), Tm)
    Tm = np.eye(4); Tm[:3, 3] = np.array([0, 0, P.LOWER_BASE_Z]) - np.array([45.45, 41.21, 0])
    static["EBAmk2_013_lower_base"] = (load(stl_dir, "EBAmk2_013_lower_base"), Tm)
    Tm = np.eye(4); Tm[:3, :3] = DRIVE_COVER_R; Tm[:3, 3] = DRIVE_COVER_T
    static["EBAmk2_019_drive_cover"] = (load(stl_dir, "EBAmk2_019_drive_cover"), Tm)
    gear_cd = 1.5 * (50 + 25) / 2                                   # khoang cach truc 010-011
    gs_axis = np.array([-gear_cd, 0.0])
    gs_z0 = base_bottom - 7.0 + 1.0                                 # giua dai rang gearmast (7 mm)
    gs_T = np.eye(4); gs_T[:3, 3] = np.array([gs_axis[0], gs_axis[1], gs_z0]) - np.array([20.44, 20.35, 0.0])

    # ---------------- kep: 014 theo 009; ngon o goc kep but
    T009 = meshes["EBAmk2_009_trialinkfront"][1]
    Tc = np.eye(4); Tc[:3, :3] = CLAW_IN_009_R; Tc[:3, 3] = CLAW_IN_009_T
    Tclaw = T009 @ Tc                                                # he kep -> the gioi
    m015 = load(stl_dir, "EBAmk2_015_claw_finger_dx")
    a_grip, pen_xy = grip_angle(m015)
    a_sx = -a_grip - MIRROR_DEG
    shaft = sg90_shaft()
    claw_local = {
        "EBAmk2_014_claw_base": np.eye(4),
        "EBAmk2_015_claw_finger_dx": Tz(a_grip, LP_DX, PIV_DX, Z_DX),
        "EBAmk2_017_claw_finger_sx": Tz(a_sx, LP_SX, PIV_SX, Z_SX),
        "EBAmk2_018_claw_gear_driven": Tz(a_sx + D018_DEG, LP_018, PIV_SX, Z_018),
        "EBAmk2_016_claw_gear_drive": Tz(D016_DEG, LP_016, shaft, Z_018 + 3.03 / 2 - 2.55),
    }
    for name, Tl in claw_local.items():
        mm_ = m015 if name == "EBAmk2_015_claw_finger_dx" else load(stl_dir, name)
        meshes[name] = (mm_, Tclaw @ Tl)

    return dict(meshes=meshes, static=static, W3=W3, Tclaw=Tclaw, a_grip=a_grip, pen_xy=pen_xy,
                shaft=shaft, gs_axis=gs_axis, gs_z0=gs_z0, gs_T=gs_T, base_bottom=base_bottom, gm=gm, m015=m015)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl-dir", required=True)
    args = ap.parse_args()
    os.makedirs(MESH_OUT, exist_ok=True)
    for f in os.listdir(MESH_OUT):
        if f.endswith(".stl"): os.remove(os.path.join(MESH_OUT, f))

    A_ = assemble(args.stl_dir, verbose=True)
    meshes, static, W3, Tclaw = A_["meshes"], A_["static"], A_["W3"], A_["Tclaw"]
    O, E, Wp, T, V, F, A, C, Q = (W3[k] for k in "O E W T V F A C Q".split())
    a_grip, pen_xy, shaft, gm = A_["a_grip"], A_["pen_xy"], A_["shaft"], A_["gm"]
    gs_axis, gs_z0, gs_T, base_bottom = A_["gs_axis"], A_["gs_z0"], A_["gs_T"], A_["base_bottom"]

    def cw(p):  # diem trong he kep -> the gioi (mm)
        return Tclaw[:3, :3] @ np.asarray(p, float) + Tclaw[:3, 3]
    zc = Tclaw[:3, 2]                                               # truc z kep (the gioi)
    # diem kep but: giua mat kep theo chieu cao (ma kep cao 9 mm tren ngon)
    grip_local = np.array([pen_xy[0], pen_xy[1], Z_DX + (0.11 + 9.11) / 2])
    grip_w = cw(grip_local)
    tip_w = grip_w - P.PEN_GRIP_TO_TIP * zc
    tilt = math.degrees(math.atan2(-zc[0], zc[2]))
    gW = grip_w - Wp
    print(f"  kep: goc ngon kep but D{P.PEN_DIAMETER:g} = {a_grip:.2f} do (mo {CLOSED_DEG - a_grip:.2f} do tu luc dong), "
          f"goc truc SG90 = {(CLOSED_DEG - a_grip) * P.GRIPPER_GEAR_RATIO:.2f} do tu luc dong")
    print(f"  diem kep - W = ({gW[0]:.2f}, {gW[1]:.3f}, {gW[2]:.2f}) mm, kep nghieng {tilt:.2f} do")
    if (np.max(np.abs(gW - np.array(P.GRIP_POINT_W))) > 0.02 or abs(tilt - P.CLAW_TILT_DEG) > 0.01):
        sys.exit(f"params.GRIP_POINT_W/CLAW_TILT_DEG khong khop hinh hoc STL. Sua params.py thanh:\n"
                 f"GRIP_POINT_W = ({gW[0]:.2f}, {gW[1]:.3f}, {gW[2]:.2f})\nCLAW_TILT_DEG = {tilt:.2f}")
    fk_tip = K.fk(0.0, math.radians(90.0), 0.0)
    print(f"  ngoi but o home: STL {np.round(tip_w, 2)}  FK {np.round(fk_tip, 2)}  lech {np.linalg.norm(tip_w - fk_tip):.3f} mm")

    # ---------------- body frames (goc tai khop, truc song song the gioi o home, mm)
    body_origin = {
        "turret": np.zeros(3), "mainarm": O, "horarm": E, "wrist": Wp,
        "varm": O, "link004": V, "link005": F, "trialink": A, "link008": C,
        "finger_dx": cw([*PIV_DX, 0]), "finger_sx": cw([*PIV_SX, 0]), "gear016": cw([*shaft, 0]),
        "gearservo": np.array([gs_axis[0], gs_axis[1], 0.0]),
    }
    body_parts = {
        "turret": ["EBAmk2_001_base", gm], "mainarm": ["EBAmk2_002_mainarm"],
        "horarm": ["EBAmk2_006_horarm__"], "wrist": ["EBAmk2_009_trialinkfront", "EBAmk2_014_claw_base"],
        "varm": ["EBAmk2_003_varm"], "link004": ["EBAmk2_004_link135"],
        "link005": ["EBAmk2_005_link135angled"], "trialink": ["EBAmk2_007_trialink"],
        "link008": ["EBAmk2_008_link147_new"],
        "finger_dx": ["EBAmk2_015_claw_finger_dx"],
        "finger_sx": ["EBAmk2_017_claw_finger_sx", "EBAmk2_018_claw_gear_driven"],
        "gear016": ["EBAmk2_016_claw_gear_drive"],
    }

    # ---------------- hop servo (khong co trong STL): kich thuoc + khoi luong datasheet
    L_, W_, H_ = P.MG996R_SIZE
    off = P.MG996R_SHAFT_OFFSET
    y_003 = ARM["EBAmk2_003_varm"][3] + 5.06            # mat ngoai vau O cua 003 (phia servo khuyu)
    y_002 = ARM["EBAmk2_002_mainarm"][3] - 28.17        # mat ngoai ma mong cua 002 (phia servo vai)
    horn = 1.5                                          # khe cho dia servo (gan dung)
    boxes = {   # ten: (body, tam the gioi mm, kich thuoc toan phan mm, khoi luong g)
        "mg996r_elbow": ("turret", np.array([0, y_003 + horn + H_ / 2, O[2] - off]), (W_, H_, L_), P.SERVO_MASS_G),
        "mg996r_shoulder": ("turret", np.array([0, y_002 - horn - H_ / 2, O[2] - off]), (W_, H_, L_), P.SERVO_MASS_G),
        "mg996r_base": (None, np.array([gs_axis[0] - off, 0, gs_z0 - H_ / 2]), (L_, W_, H_), P.SERVO_MASS_G),
    }
    sgL, sgW, sgH = P.SG90["size"]
    sg_bottom = Z_018 + 3.03 / 2 + 2.5                  # mat tren banh rang 016 (truc SG90 huong xuong)
    sg_c_local = np.array([(SG90_WIN[0] + SG90_WIN[1]) / 2, shaft[1], sg_bottom + sgH / 2])
    sg_box = ("wrist", cw(sg_c_local), (sgL, sgW, sgH), P.SG90["mass_g"])

    # ---------------- xuat luoi (dat vao he body, don vi m) + khoi luong
    report = ["chi tiet | khoi luong (g) | the tich (cm3) | ghi chu"]
    inertials = {}
    for body, parts in body_parts.items():
        items = []
        for name in parts:
            m, Tm = meshes[name]
            mb = m.copy(); mb.apply_transform(Tm)
            mb.apply_translation(-body_origin[body]); mb.apply_scale(MM)
            mb.export(os.path.join(MESH_OUT, name + ".stl"))
            mass, com, I, vol, note = mass_props(m, name)
            Rm = Tm[:3, :3]
            com_b = (Rm @ (com / MM) + Tm[:3, 3] - body_origin[body]) * MM
            items.append((mass, com_b, Rm @ I @ Rm.T))
            report.append(f"{name} | {mass*1000:.1f} | {vol/1000:.1f} | {note}")
        for bname, (b, c, size, mg) in list(boxes.items()) + [("sg90", sg_box)]:
            if b == body:
                ms = mg / 1000
                items.append((ms, (c - body_origin[body]) * MM, box_inertia(ms, *(np.array(size) * MM))))
                report.append(f"{bname} (hop datasheet) | {mg:.1f} | - | vi tri gan dung")
        if body == "wrist":
            mt = P.TOOL_MASS_G / 1000
            pc = grip_w + (P.PEN_LENGTH / 2 - P.PEN_GRIP_TO_TIP) * zc
            items.append((mt, (pc - body_origin[body]) * MM, box_inertia(mt, *(np.array([10, 10, P.PEN_LENGTH]) * MM))))
            report.append(f"but D{P.PEN_DIAMETER:g} x {P.PEN_LENGTH:g} mm | {P.TOOL_MASS_G:.1f} | - | [GIA-DINH]")
        inertials[body] = combine(items)
    for name, (m, Tm) in static.items():
        mb = m.copy(); mb.apply_transform(Tm); mb.apply_scale(MM)
        mb.export(os.path.join(MESH_OUT, name + ".stl"))
    m010 = load(args.stl_dir, "EBAmk2_010_gearservo")
    mb = m010.copy(); mb.apply_transform(gs_T); mb.apply_translation(-body_origin["gearservo"]); mb.apply_scale(MM)
    mb.export(os.path.join(MESH_OUT, "EBAmk2_010_gearservo.stl"))
    gs_mass, gs_com, gs_I, _, _ = mass_props(m010, "EBAmk2_010_gearservo")
    inertials["gearservo"] = (gs_mass, (gs_com / MM + gs_T[:3, 3] - body_origin["gearservo"]) * MM, gs_I)

    # ---------------- servo model
    sv = P.MG996R[P.SUPPLY]
    tau_s = sv["stall_kgfcm"] * 9.80665 / 100.0          # N m
    w0 = math.radians(60.0) / sv["s_per_60deg"]          # rad/s
    b_emf = tau_s / w0
    kp = tau_s / math.radians(P.SERVO_ERR_SAT_DEG)
    g = P.BASE_GEAR_RATIO
    bl = math.radians(P.BACKLASH_DEG)
    tau_g = P.SG90["stall_kgfcm"] * 9.80665 / 100.0
    w0_g = math.radians(60.0) / P.SG90["s_per_60deg"]
    kp_g = tau_g / math.radians(P.SERVO_ERR_SAT_DEG)
    gr = P.GRIPPER_GEAR_RATIO
    fo = math.radians(P.FINGER_OPEN_MAX_DEG)

    def inertial_xml(body):
        M, c, I = inertials[body]
        full = [I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]
        return f'<inertial pos="{fmt(c)}" mass="{M:.6g}" fullinertia="{fmt(full)}"/>'

    def joint(name, damping=0.0, arm=0.0, fl=0.0, rng=None, axis="0 -1 0", play=False):
        s = f'<joint name="{name}" type="hinge" axis="{axis}" damping="{damping:.6g}" armature="{arm:.6g}" frictionloss="{fl:.6g}"'
        if rng is not None: s += f' limited="true" range="{rng[0]:.6g} {rng[1]:.6g}"'
        s += "/>"
        if play and bl > 0:
            s += f'\n<joint name="{name}_play" type="hinge" axis="{axis}" limited="true" range="{-bl/2:.6g} {bl/2:.6g}" armature="1e-6" solreflimit="0.002 1" solimplimit="0.95 0.99 0.0005"/>'
        return s

    def mesh_geom(name, rgba):
        return f'<geom type="mesh" mesh="{name}" rgba="{rgba}" contype="0" conaffinity="0" group="1"/>'

    def box_geom(key, body, rgba="0.12 0.12 0.12 1"):
        b, c, size, _ = boxes[key] if key in boxes else sg_box
        org = body_origin[body] if body else np.zeros(3)
        return (f'<geom name="{key}" type="box" pos="{fmt((c - org) * MM)}" size="{fmt(np.array(size) * MM / 2)}" '
                f'rgba="{rgba}" contype="0" conaffinity="0" group="1"/>')

    def rel(a, b):
        return fmt((body_origin[b] - body_origin[a]) * MM)

    def lw(p, body):   # diem the gioi (mm) -> he body (m)
        return fmt((np.asarray(p) - body_origin[body]) * MM)

    arm_c = "0.95 0.45 0.1 1"; link_c = "0.2 0.4 0.85 1"; tri_c = "0.15 0.6 0.3 1"; claw_c = "0.85 0.85 0.9 1"
    arm_motor = max(P.SERVO_ARMATURE, 1e-5 if bl > 0 else 0.0)
    zax = fmt(zc)

    # but: ong than D10 phia tren, phan thon o dau, bi ngoi D1 (hinh hoc va cham voi giay)
    r_tip = P.INK_WIDTH / 2
    tip_c = tip_w + r_tip * zc
    pen_top = grip_w + (P.PEN_LENGTH - P.PEN_GRIP_TO_TIP) * zc
    pen_xml = "\n".join([
        f'<geom name="pen_body" type="cylinder" fromto="{lw(tip_w + 12 * zc, "wrist")} {lw(pen_top, "wrist")}" size="{P.PEN_DIAMETER/2*MM:.6g}" rgba="0.1 0.25 0.8 1" contype="0" conaffinity="0" group="1"/>',
        f'<geom name="pen_cone" type="cylinder" fromto="{lw(tip_w + 3 * zc, "wrist")} {lw(tip_w + 12 * zc, "wrist")}" size="{P.PEN_DIAMETER/4*MM:.6g}" rgba="0.75 0.75 0.75 1" contype="0" conaffinity="0" group="1"/>',
        f'<geom name="pen_neck" type="cylinder" fromto="{lw(tip_c, "wrist")} {lw(tip_w + 3 * zc, "wrist")}" size="{0.8*MM:.6g}" rgba="0.6 0.6 0.6 1" contype="0" conaffinity="0" group="1"/>',
        f'<geom name="pen_tip" type="sphere" pos="{lw(tip_c, "wrist")}" size="{r_tip*MM:.6g}" rgba="0.05 0.05 0.05 1" '
        f'contype="2" conaffinity="0" condim="3" friction="{P.PEN_TIP_FRICTION} 0.005 0.0001" solref="0.002 1" '
        f'margin="{P.INK_GAP*MM:.6g}" gap="{P.INK_GAP*MM:.6g}"/>',
        f'<site name="tool" pos="{lw(tip_w, "wrist")}" size="0.002" rgba="1 0 0 1"/>',
        f'<site name="grip" pos="{lw(grip_w, "wrist")}" size="0.002" rgba="0 1 0 1"/>',
    ])
    px, py = P.PAPER_CENTER; sx_, sy_ = P.PAPER_SIZE; th = P.PAPER_THICKNESS
    finger_rng_dx = (-fo, 0.0); finger_rng_sx = (0.0, fo)

    xml = f"""<mujoco model="eezybotarm_mk2">
  <!-- Sinh tu dong boi build_model.py — KHONG sua tay, sua params.py roi build lai -->
  <compiler angle="radian" inertiafromgeom="false" meshdir="meshes"/>
  <option timestep="{P.TIMESTEP}" integrator="implicitfast" gravity="0 0 -9.81"/>
  <visual><global offwidth="1280" offheight="960"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.9 0.9 0.9" rgb2="0.8 0.8 0.8" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="10 10"/>
{chr(10).join(f'    <mesh name="{n}" file="{n}.stl"/>' for n in [n for ps in body_parts.values() for n in ps] + list(static) + ["EBAmk2_010_gearservo"])}
  </asset>
  <worldbody>
    <light pos="0.3 -0.3 0.8" dir="-0.3 0.3 -0.8"/>
    <geom name="table" type="plane" size="0.6 0.6 0.01" material="grid" contype="0" conaffinity="0"/>
    <geom name="paper" type="box" pos="{px*MM:.6g} {py*MM:.6g} {th/2*MM:.6g}" size="{sx_/2*MM:.6g} {sy_/2*MM:.6g} {th/2*MM:.6g}" rgba="1 1 1 1"
          contype="0" conaffinity="2" friction="{P.PEN_TIP_FRICTION} 0.005 0.0001" solref="0.002 1"/>
    {mesh_geom("EBAmk2_012_mainbase", "0.6 0.6 0.6 1")}
    {mesh_geom("EBAmk2_013_lower_base", "0.5 0.5 0.5 1")}
    {mesh_geom("EBAmk2_019_drive_cover", "0.95 0.45 0.1 1")}
    {box_geom("mg996r_base", None)}
    <body name="gearservo" pos="{fmt(body_origin['gearservo'] * MM)}">
      {inertial_xml("gearservo")}
      <joint name="gearservo" type="hinge" axis="0 0 1" armature="{P.GRIPPER_ARMATURE:.6g}"/>
      {mesh_geom("EBAmk2_010_gearservo", "0.3 0.3 0.3 1")}
    </body>
    <body name="turret" pos="0 0 0">
      {inertial_xml("turret")}
      {joint("base_yaw", g*g*b_emf, g*g*arm_motor, g*P.SERVO_FRICTIONLOSS, axis="0 0 1", play=True)}
      {mesh_geom("EBAmk2_001_base", arm_c)}
      {mesh_geom(gm, "0.3 0.3 0.3 1")}
      {box_geom("mg996r_elbow", "turret")}
      {box_geom("mg996r_shoulder", "turret")}
      <body name="mainarm" pos="{rel('turret','mainarm')}">
        {inertial_xml("mainarm")}
        {joint("shoulder", b_emf, arm_motor, P.SERVO_FRICTIONLOSS, play=True)}
        {mesh_geom("EBAmk2_002_mainarm", arm_c)}
        <site name="E_main" pos="{fmt((E - O) * MM)}" size="0.003"/>
        <body name="horarm" pos="{rel('mainarm','horarm')}">
          {inertial_xml("horarm")}
          {joint("elbow_rel")}
          {mesh_geom("EBAmk2_006_horarm__", arm_c)}
          <site name="T_hor" pos="{fmt((T - E) * MM)}" size="0.003"/>
          <body name="wrist" pos="{rel('horarm','wrist')}">
            {inertial_xml("wrist")}
            {joint("wrist_rel")}
            {mesh_geom("EBAmk2_009_trialinkfront", tri_c)}
            {mesh_geom("EBAmk2_014_claw_base", claw_c)}
            <geom name="sg90" type="box" pos="{lw(sg_box[1], 'wrist')}" size="{fmt(np.array(sg_box[2]) * MM / 2)}" quat="{fmt(mat2quat(Tclaw[:3, :3]))}" rgba="0.1 0.3 0.8 1" contype="0" conaffinity="0" group="1"/>
            <site name="Q_wrist" pos="{fmt((Q - Wp) * MM)}" size="0.003"/>
            {pen_xml}
            <body name="finger_dx" pos="{rel('wrist','finger_dx')}">
              {inertial_xml("finger_dx")}
              <joint name="finger_dx" type="hinge" axis="{zax}" armature="{P.GRIPPER_ARMATURE:.6g}" limited="true" range="{finger_rng_dx[0]:.6g} {finger_rng_dx[1]:.6g}"/>
              {mesh_geom("EBAmk2_015_claw_finger_dx", claw_c)}
            </body>
            <body name="finger_sx" pos="{rel('wrist','finger_sx')}">
              {inertial_xml("finger_sx")}
              <joint name="finger_sx" type="hinge" axis="{zax}" armature="{P.GRIPPER_ARMATURE:.6g}" limited="true" range="{finger_rng_sx[0]:.6g} {finger_rng_sx[1]:.6g}"/>
              {mesh_geom("EBAmk2_017_claw_finger_sx", claw_c)}
              {mesh_geom("EBAmk2_018_claw_gear_driven", "0.3 0.3 0.3 1")}
            </body>
            <body name="gear016" pos="{rel('wrist','gear016')}">
              {inertial_xml("gear016")}
              <joint name="gripper" type="hinge" axis="{zax}" armature="{P.GRIPPER_ARMATURE:.6g}" damping="{tau_g / w0_g:.6g}" limited="true" range="{-gr*fo:.6g} 0"/>
              {mesh_geom("EBAmk2_016_claw_gear_drive", "0.3 0.3 0.3 1")}
            </body>
          </body>
        </body>
      </body>
      <body name="varm" pos="{rel('turret','varm')}">
        {inertial_xml("varm")}
        {joint("elbow_servo", b_emf, arm_motor, P.SERVO_FRICTIONLOSS, play=True)}
        {mesh_geom("EBAmk2_003_varm", link_c)}
        <body name="link004" pos="{rel('varm','link004')}">
          {inertial_xml("link004")}
          {joint("l004")}
          {mesh_geom("EBAmk2_004_link135", link_c)}
          <site name="T_l004" pos="{fmt((T - V) * MM)}" size="0.003"/>
        </body>
      </body>
      <body name="link005" pos="{rel('turret','link005')}">
        {inertial_xml("link005")}
        {joint("l005")}
        {mesh_geom("EBAmk2_005_link135angled", tri_c)}
        <body name="trialink" pos="{rel('link005','trialink')}">
          {inertial_xml("trialink")}
          {joint("tri")}
          {mesh_geom("EBAmk2_007_trialink", tri_c)}
          <site name="E_tri" pos="{fmt((E - A) * MM)}" size="0.003"/>
          <body name="link008" pos="{rel('trialink','link008')}">
            {inertial_xml("link008")}
            {joint("l008")}
            {mesh_geom("EBAmk2_008_link147_new", tri_c)}
            <site name="Q_l008" pos="{fmt((Q - C) * MM)}" size="0.003"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <equality>
    <!-- 3 vong hinh binh hanh -->
    <connect name="loop_elbow_drive" site1="T_hor" site2="T_l004" solref="0.002 1"/>
    <connect name="loop_triangle" site1="E_main" site2="E_tri" solref="0.002 1"/>
    <connect name="loop_wrist" site1="Q_wrist" site2="Q_l008" solref="0.002 1"/>
    <!-- banh rang: 2 ngon an khop 1:1; 016 (10 rang) - 018 (16 rang) tren ngon trai; 010 - 011 de -->
    <joint name="fingers_mesh" joint1="finger_sx" joint2="finger_dx" polycoef="0 -1 0 0 0" solref="0.002 1"/>
    <joint name="gripper_gear" joint1="gripper" joint2="finger_dx" polycoef="0 {gr:.6g} 0 0 0" solref="0.002 1"/>
    <joint name="base_gear" joint1="gearservo" joint2="base_yaw" polycoef="0 {-g:.6g} 0 0 0"/>
  </equality>
  <actuator>
    <!-- Servo MG996R ({P.SUPPLY}): tau_stall={tau_s:.3f} Nm, w0={w0:.2f} rad/s,
         kp=tau_stall/{P.SERVO_ERR_SAT_DEG} deg, back-EMF dat o damping cua khop -->
    <position name="servo_base" joint="base_yaw" gear="{g}" kp="{kp:.6g}" forcelimited="true" forcerange="{-tau_s:.6g} {tau_s:.6g}" ctrllimited="true" ctrlrange="{-math.pi/2:.6g} {math.pi/2:.6g}"/>
    <position name="servo_shoulder" joint="shoulder" kp="{kp:.6g}" forcelimited="true" forcerange="{-tau_s:.6g} {tau_s:.6g}" ctrllimited="true" ctrlrange="{-math.pi/2:.6g} {math.pi/2:.6g}"/>
    <position name="servo_elbow" joint="elbow_servo" kp="{kp:.6g}" forcelimited="true" forcerange="{-tau_s:.6g} {tau_s:.6g}" ctrllimited="true" ctrlrange="{-math.pi/2:.6g} {math.pi/2:.6g}"/>
    <!-- Servo kep SG90: 0 = kep but D{P.PEN_DIAMETER:g}, am = mo -->
    <position name="servo_gripper" joint="gripper" kp="{kp_g:.6g}" forcelimited="true" forcerange="{-tau_g:.6g} {tau_g:.6g}" ctrllimited="true" ctrlrange="{-gr*fo:.6g} 0"/>
  </actuator>
  <sensor>
    <framepos name="tool_pos" objtype="site" objname="tool"/>
    <actuatorfrc name="frc_base" actuator="servo_base"/>
    <actuatorfrc name="frc_shoulder" actuator="servo_shoulder"/>
    <actuatorfrc name="frc_elbow" actuator="servo_elbow"/>
  </sensor>
</mujoco>
"""
    with open(os.path.join(OUT, "mk2.xml"), "w") as f:
        f.write(xml)
    moving = [b for b in inertials if b != "gearservo"]
    tot = sum(inertials[b][0] for b in moving)
    report.append(f"TONG khoi luong chuyen dong: {tot*1000:.1f} g  (FILL_FACTOR={P.FILL_FACTOR})")
    with open(os.path.join(OUT, "mass_report.txt"), "w") as f:
        f.write("\n".join(report) + "\n")
    print("\n".join(report))
    print(f"\nServo: tau_stall={tau_s:.3f} N m, w0={w0:.2f} rad/s, kp={kp:.3f} N m/rad, b={b_emf:.4f} N m s/rad")
    print(f"Da ghi {os.path.join(OUT, 'mk2.xml')}")


def mat2quat(R):
    """Ma tran quay -> quaternion (w, x, y, z)."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([w, x, y, z])


# 019 (nap che phan sau cua 012): lat ngua (quay 180 do quanh truc y), dau hep trung dau sau
# cua 012, tam doi xung trung truc doi xung 012, mat duoi nap nam tren mep tuong 012 (z = 55),
# 4 moc cai vao trong tuong. Nguon: so sanh tiet dien 012 (z = 50..55) va 019 (tools/holes.py).
DRIVE_COVER_R = np.diag([-1.0, 1.0, -1.0])
# truc doi xung 019: y = (0.39 + 78.45)/2 = 39.42; cua 012: y = 45.21 -> dich 5.79
DRIVE_COVER_T = np.array([99.0 - 98.98, 5.79 - 45.21, 55.0 + 2.0])


if __name__ == "__main__":
    main()
