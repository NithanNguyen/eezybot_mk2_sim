"""
build_model.py — Dung mo hinh MuJoCo (MJCF) cua EEZYbotARM Mk2 tu cac file STL goc.

Cach dung:
    python build_model.py --stl-dir <thu muc chua EBAmk2_*.STL>

Ket qua:
    model/meshes/*.stl   (STL nhi phan, da dat vao he toa do cua tung body)
    model/mk2.xml        (MJCF: 9 khop, 3 rang buoc vong kin, 3 servo)
    model/mass_report.txt

Nguyen tac:
  * Vi tri tung chi tiet suy ra tu TAM LO TRUC do tren STL (tools/holes.py):
    2 tam lo cua chi tiet duoc dat trung 2 diem khop tinh tu params.py.
  * Vi tri theo chieu ngang (truc y, doc truc khop) chi anh huong hien thi va
    rat it den quan tinh -> dung bo tri gan dung (LATERAL_MM ben duoi).
  * 3 vong hinh binh hanh dong bang <equality><connect site1 site2>.
  * Khoi luong/quan tinh tinh tu the tich luoi x mat do PLA x FILL_FACTOR
    (hoac khoi luong can thuc te trong params.MEASURED_MASS_G).
"""
import argparse, math, os, sys
import numpy as np
import trimesh

import params as P
import mk2_kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "model")
MESH_OUT = os.path.join(OUT, "meshes")
MM = 1e-3

# --------------------------------------------------------------------------
# Tam lo do tren STL (mm, he toa do rieng cua tung file). Nguon: tools/holes.py
# axis: truc lo trong he rieng; holes: toa do 2 truc con lai (theo thu tu x,y,z bo axis)
# --------------------------------------------------------------------------
HOLES = {
    "EBAmk2_001_base":           dict(axis=0, h={"O": (46.43, 43.00), "F": (8.46, 63.19)}),
    "EBAmk2_002_mainarm":        dict(axis=2, h={"O": (16.16, 16.16), "E": (16.16, 151.16)}),
    "EBAmk2_003_varm":           dict(axis=2, h={"O": (15.00, 15.00), "V": (72.00, 15.00)}),
    "EBAmk2_004_link135":        dict(axis=2, h={"V": (6.56, 6.57), "T": (6.57, 141.57)}),
    "EBAmk2_005_link135angled":  dict(axis=2, h={"F": (20.20, 7.57), "A": (20.20, 142.58)}),
    "EBAmk2_006_horarm__":       dict(axis=0, h={"W": (6.51, 17.66), "E": (152.96, 30.47), "T": (209.74, 35.44)}),
    "EBAmk2_007_trialink":       dict(axis=2, h={"A": P.TRI_A, "E": P.TRI_B, "C": P.TRI_C}),
    "EBAmk2_008_link147_new":    dict(axis=2, h={"C": (6.56, 6.57), "Q": (6.57, 153.57)}),
    "EBAmk2_009_trialinkfront":  dict(axis=1, h={"Q": (6.57, 39.68), "W": (34.45, 6.95)}),
}

# Vi tri ngang (mm, theo truc y the gioi) cua mat giua tung chi tiet — GAN DUNG
LATERAL_MM = {
    "EBAmk2_002_mainarm": 0.0, "EBAmk2_006_horarm__": 0.0, "EBAmk2_009_trialinkfront": 0.0,
    "EBAmk2_003_varm": -26.0, "EBAmk2_004_link135": -26.0,
    "EBAmk2_005_link135angled": 26.0, "EBAmk2_007_trialink": 24.0, "EBAmk2_008_link147_new": 24.0,
}
# Lat mat chi tiet (doi chieu truc lo) — chi anh huong hien thi
# 007_trialink BAT BUOC lat (de goc A-B-C khong bi doi xung guong so voi params);
# cac chi tiet khac chi anh huong hinh dang hien thi.
FLIP = {"EBAmk2_007_trialink": True, "EBAmk2_005_link135angled": False, "EBAmk2_006_horarm__": False}


def load(stl_dir, name):
    p = os.path.join(stl_dir, name + ".STL")
    if not os.path.exists(p):
        sys.exit(f"Khong tim thay {p}. Kiem tra --stl-dir (thu muc 'files' trong zip part 1).")
    return trimesh.load(p, force="mesh")


def hole3(name, key, mesh):
    """Tam lo -> diem 3D (mm) trong he rieng; toa do doc truc = giua chi tiet."""
    d = HOLES[name]
    ax = d["axis"]
    others = [i for i in range(3) if i != ax]
    v = np.zeros(3)
    v[others[0]], v[others[1]] = d["h"][key]
    v[ax] = mesh.bounds[:, ax].mean()
    return v


def rigid_from_holes(name, mesh, k1, k2, P1, P2, lateral, flip=False):
    """Tim T (4x4, mm) dua 2 tam lo cua chi tiet ve 2 diem the gioi P1, P2
    (trong mat phang x-z, y = lateral). Truc lo -> truc y the gioi (+/-)."""
    ax = HOLES[name]["axis"]
    a = np.zeros(3); a[ax] = 1.0
    h1, h2 = hole3(name, k1, mesh), hole3(name, k2, mesh)
    vl = h2 - h1
    vw = np.array(P2, float) - np.array(P1, float)
    if abs(np.linalg.norm(vl) - np.linalg.norm(vw)) > 0.3:
        raise RuntimeError(f"{name}: khoang cach lo {np.linalg.norm(vl):.2f} != {np.linalg.norm(vw):.2f}")
    s = -1.0 if flip else 1.0
    e1l = vl / np.linalg.norm(vl); e2l = s * a; e3l = np.cross(e1l, e2l)
    e1w = vw / np.linalg.norm(vw); e2w = np.array([0, 1.0, 0]); e3w = np.cross(e1w, e2w)
    R = np.c_[e1w, e2w, e3w] @ np.c_[e1l, e2l, e3l].T
    t = np.array(P1, float) - R @ h1
    t[1] = lateral - (R @ h1)[1]         # mat giua chi tiet tai y = lateral
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
        vox = m.voxelized(pitch=0.6).fill()
        pts = vox.points
        vol = len(pts) * 0.6 ** 3
        com = pts.mean(0)
        d = pts - com
        Iunit = np.eye(3) * (d ** 2).sum(1).sum() - d.T @ d   # per-unit-mass * N
        Iunit /= len(pts)
        note = "voxel 0.6mm"
    else:
        vol = abs(m.volume); com = m.center_mass
        Iunit = m.moment_inertia / m.mass if m.mass != 0 else np.zeros((3, 3))
        if m.volume < 0: Iunit = -Iunit
    rho = P.PLA_DENSITY * P.FILL_FACTOR
    mass = vol * (MM ** 3) * rho
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
    return " ".join(f"{x:.6g}" for x in v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl-dir", required=True)
    args = ap.parse_args()
    os.makedirs(MESH_OUT, exist_ok=True)

    # ---------------- diem khop o tu the home (th2 = 90 do, phi = 0), mat phang x-z, mm
    th2, phi = math.radians(90.0), 0.0
    pts = K.linkage_points(th2, phi)
    W3 = {k: np.array([v[0], 0.0, v[1]]) for k, v in pts.items()}   # (x, 0, z)
    O, E, Wp, T, V, F, A, C, Q = (W3[k] for k in "O E W T V F A C Q".split())

    # ---------------- body frames (goc tai khop, truc song song the gioi, mm)
    body_origin = {
        "turret": np.zeros(3), "mainarm": O, "horarm": E, "wrist": Wp,
        "varm": O, "link004": V, "link005": F, "trialink": A, "link008": C,
    }

    # ---------------- dat tung chi tiet (T: he rieng -> the gioi, mm)
    meshes = {}
    def place(name, k1, k2, P1, P2):
        m = load(args.stl_dir, name)
        Tm = rigid_from_holes(name, m, k1, k2, P1, P2, LATERAL_MM.get(name, 0.0), FLIP.get(name, False))
        meshes[name] = (m, Tm)

    place("EBAmk2_002_mainarm", "O", "E", O, E)
    place("EBAmk2_003_varm", "O", "V", O, V)
    place("EBAmk2_004_link135", "V", "T", V, T)
    place("EBAmk2_005_link135angled", "F", "A", F, A)
    place("EBAmk2_006_horarm__", "E", "W", E, Wp)
    place("EBAmk2_007_trialink", "E", "A", E, A)
    place("EBAmk2_008_link147_new", "C", "Q", C, Q)
    place("EBAmk2_009_trialinkfront", "W", "Q", Wp, Q)

    # kiem tra diem thu 3 (lo C cua tam giac, lo T cua tay ngang)
    for name, key, target in [("EBAmk2_007_trialink", "C", C), ("EBAmk2_006_horarm__", "T", T)]:
        m, Tm = meshes[name]
        p = Tm[:3, :3] @ hole3(name, key, m) + Tm[:3, 3]
        err = np.linalg.norm((p - target)[[0, 2]])
        print(f"  kiem tra lo {key} cua {name}: lech {err:.2f} mm")
        if err > 1.0:
            raise RuntimeError("Hinh hoc khong khop — kiem tra lai HOLES/params")

    # 001_base (de quay): lo truc dung o (x=40.40, y=46.43) he rieng; truc lo vai = x rieng
    m = load(args.stl_dir, "EBAmk2_001_base")
    R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])          # rieng y->x, x->-y, z->z
    base_bottom = P.H_SHOULDER - 43.00
    Tm = np.eye(4); Tm[:3, :3] = R; Tm[:3, 3] = np.array([0, 0, base_bottom]) - R @ np.array([40.40, 46.43, 0])
    meshes["EBAmk2_001_base"] = (m, Tm)
    # kiem tra O, F tren 001_base
    for key, target in [("O", O), ("F", F)]:
        p = Tm[:3, :3] @ hole3("EBAmk2_001_base", key, m) + Tm[:3, 3]
        print(f"  kiem tra lo {key} cua 001_base: lech {np.linalg.norm((p - target)[[0, 2]]):.2f} mm")

    # 011_gearmast (quay theo de, in up nguoc): lat quanh x, tam (39.34, 40.40)
    gm = "EBAmk2_011_gearmast"
    m = load(args.stl_dir, gm)
    R = np.diag([1.0, -1.0, -1.0])
    Tm = np.eye(4); Tm[:3, :3] = R; Tm[:3, 3] = np.array([0, 0, base_bottom]) - R @ np.array([39.34, 40.40, 0])
    meshes[gm] = (m, Tm)

    # Chi tiet co dinh (khong dong luc hoc): 012_mainbase, 013_lower_base
    static = {}
    m = load(args.stl_dir, "EBAmk2_012_mainbase")
    Tm = np.eye(4); Tm[:3, 3] = -np.array([98.98, 45.21, 0])        # truc dung tai (98.98, 45.21)
    static["EBAmk2_012_mainbase"] = (m, Tm)
    m = load(args.stl_dir, "EBAmk2_013_lower_base")
    Tm = np.eye(4); Tm[:3, 3] = np.array([0, 0, 33.5]) - np.array([45.45, 41.21, 0])
    static["EBAmk2_013_lower_base"] = (m, Tm)

    body_parts = {
        "turret": ["EBAmk2_001_base", gm], "mainarm": ["EBAmk2_002_mainarm"],
        "horarm": ["EBAmk2_006_horarm__"], "wrist": ["EBAmk2_009_trialinkfront"],
        "varm": ["EBAmk2_003_varm"], "link004": ["EBAmk2_004_link135"],
        "link005": ["EBAmk2_005_link135angled"], "trialink": ["EBAmk2_007_trialink"],
        "link008": ["EBAmk2_008_link147_new"],
    }

    # ---------------- xuat luoi (da dat vao he body, don vi m) + khoi luong
    report = ["chi tiet | khoi luong (g) | the tich (cm3) | ghi chu"]
    inertials = {}
    for body, parts in body_parts.items():
        items = []
        for name in parts:
            m, Tm = meshes[name]
            mb = m.copy(); mb.apply_transform(Tm)
            mb.apply_translation(-body_origin[body]); mb.apply_scale(MM)
            mb.export(os.path.join(MESH_OUT, name + ".stl"))           # nhi phan
            mass, com, I, vol, note = mass_props(m, name)
            Rm = Tm[:3, :3]
            com_b = (Rm @ (com / MM) + Tm[:3, 3] - body_origin[body]) * MM
            items.append((mass, com_b, Rm @ I @ Rm.T))
            report.append(f"{name} | {mass*1000:.1f} | {vol/1000:.1f} | {note}")
        if body == "turret":   # 2 servo vai gan tren de quay, hai ben truc vai
            ms = P.SERVO_MASS_G / 1000
            for sgn in (+1, -1):
                items.append((ms, np.array([0, sgn * 0.035, P.H_SHOULDER * MM]), box_inertia(ms, 0.04, 0.02, 0.043)))
            report.append(f"2 x MG996R tren de quay | {2*P.SERVO_MASS_G:.1f} | - | datasheet, vi tri gan dung")
        if body == "wrist":
            mt = P.TOOL_MASS_G / 1000
            items.append((mt, np.array([P.TOOL_OFFSET[0], 0, P.TOOL_OFFSET[1]]) * MM, np.eye(3) * mt * 1e-4))
            report.append(f"dau cong tac (but) | {P.TOOL_MASS_G:.1f} | - | [GIA-DINH]")
        inertials[body] = combine(items)
    for name, (m, Tm) in static.items():
        mb = m.copy(); mb.apply_transform(Tm); mb.apply_scale(MM)
        mb.export(os.path.join(MESH_OUT, name + ".stl"))

    # ---------------- servo model
    sv = P.MG996R[P.SUPPLY]
    tau_s = sv["stall_kgfcm"] * 9.80665 / 100.0          # N m
    w0 = math.radians(60.0) / sv["s_per_60deg"]          # rad/s
    b_emf = tau_s / w0                                   # N m s/rad (doc duong cong mo-men/toc do)
    kp = tau_s / math.radians(P.SERVO_ERR_SAT_DEG)
    g = P.BASE_GEAR_RATIO
    bl = math.radians(P.BACKLASH_DEG)

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

    def rel(a, b):   # vi tri body b trong he body cha a (m)
        return fmt((body_origin[b] - body_origin[a]) * MM)

    arm_c = "0.95 0.45 0.1 1"; link_c = "0.2 0.4 0.85 1"; tri_c = "0.15 0.6 0.3 1"
    arm_motor = max(P.SERVO_ARMATURE, 1e-5 if bl > 0 else 0.0)

    xml = f"""<mujoco model="eezybotarm_mk2">
  <!-- Sinh tu dong boi build_model.py — KHONG sua tay, sua params.py roi build lai -->
  <compiler angle="radian" inertiafromgeom="false" meshdir="meshes"/>
  <option timestep="{P.TIMESTEP}" integrator="implicitfast" gravity="0 0 -9.81"/>
  <visual><global offwidth="1280" offheight="960"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.9 0.9 0.9" rgb2="0.8 0.8 0.8" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="10 10"/>
{chr(10).join(f'    <mesh name="{n}" file="{n}.stl"/>' for n in list(body_parts_flat(body_parts)) + list(static))}
  </asset>
  <worldbody>
    <light pos="0.3 -0.3 0.8" dir="-0.3 0.3 -0.8"/>
    <geom name="table" type="plane" size="0.6 0.6 0.01" material="grid" contype="0" conaffinity="0"/>
    {mesh_geom("EBAmk2_012_mainbase", "0.6 0.6 0.6 1")}
    {mesh_geom("EBAmk2_013_lower_base", "0.5 0.5 0.5 1")}
    <body name="turret" pos="0 0 0">
      {inertial_xml("turret")}
      {joint("base_yaw", g*g*b_emf, g*g*arm_motor, g*P.SERVO_FRICTIONLOSS, axis="0 0 1", play=True)}
      {mesh_geom("EBAmk2_001_base", arm_c)}
      {mesh_geom(gm, "0.3 0.3 0.3 1")}
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
            <site name="Q_wrist" pos="{fmt((Q - Wp) * MM)}" size="0.003"/>
            <site name="tool" pos="{P.TOOL_OFFSET[0]*MM:.6g} 0 {P.TOOL_OFFSET[1]*MM:.6g}" size="0.004" rgba="1 0 0 1"/>
            <geom type="capsule" fromto="0 0 0 {P.TOOL_OFFSET[0]*MM:.6g} 0 {P.TOOL_OFFSET[1]*MM:.6g}" size="0.003" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0"/>
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
  </equality>
  <actuator>
    <!-- Servo MG996R ({P.SUPPLY}): tau_stall={tau_s:.3f} Nm, w0={w0:.2f} rad/s,
         kp=tau_stall/{P.SERVO_ERR_SAT_DEG} deg, back-EMF dat o damping cua khop -->
    <position name="servo_base" joint="base_yaw" gear="{g}" kp="{kp:.6g}" forcelimited="true" forcerange="{-tau_s:.6g} {tau_s:.6g}" ctrllimited="true" ctrlrange="{-math.pi/2:.6g} {math.pi/2:.6g}"/>
    <position name="servo_shoulder" joint="shoulder" kp="{kp:.6g}" forcelimited="true" forcerange="{-tau_s:.6g} {tau_s:.6g}" ctrllimited="true" ctrlrange="{-math.pi/2:.6g} {math.pi/2:.6g}"/>
    <position name="servo_elbow" joint="elbow_servo" kp="{kp:.6g}" forcelimited="true" forcerange="{-tau_s:.6g} {tau_s:.6g}" ctrllimited="true" ctrlrange="{-math.pi/2:.6g} {math.pi/2:.6g}"/>
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
    tot = sum(v[0] for v in inertials.values())
    report.append(f"TONG khoi luong chuyen dong: {tot*1000:.1f} g  (FILL_FACTOR={P.FILL_FACTOR})")
    with open(os.path.join(OUT, "mass_report.txt"), "w") as f:
        f.write("\n".join(report) + "\n")
    print("\n".join(report))
    print(f"\nServo: tau_stall={tau_s:.3f} N m, w0={w0:.2f} rad/s, kp={kp:.3f} N m/rad, b={b_emf:.4f} N m s/rad")
    print(f"Da ghi {os.path.join(OUT, 'mk2.xml')}")


def body_parts_flat(bp):
    for parts in bp.values():
        for n in parts:
            yield n


if __name__ == "__main__":
    main()
