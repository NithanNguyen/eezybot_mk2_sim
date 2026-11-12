"""tools/export_arm_config.py — Sinh eezybot_mk2_robot/include/arm_config.h tu params.py.

params.py la NGUON DUY NHAT cua hinh hoc, gioi han khop, anh xa servo, gioi han quy dao va cac diem cua chu
trinh phan loai. Firmware khong sua tay arm_config.h; sua params.py roi chay lai script nay.
    python tools/export_arm_config.py                         # ghi ../eezybot_mk2_robot/include/arm_config.h
    python tools/export_arm_config.py --out duong/dan/arm_config.h
    python tools/export_arm_config.py --check                 # chi so sanh, ma thoat 1 neu file da cu
Header ghi SHA-256 cua params.py va sort_scene.py: --check bao "cu" khi mot trong hai file doi.
Cac diem cua chu trinh lay tu sort_scene.task_points() (cung ham run_sort.py dung).
"""
import argparse, hashlib, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
import params as P
import sort_scene as SS

DEFAULT_OUT = os.path.normpath(os.path.join(ROOT, "..", "eezybot_mk2_robot", "include", "arm_config.h"))


def ff(v):
    """float -> literal C++ float (luon co dau cham thap phan va hau to f)."""
    s = repr(round(float(v), 6))
    if "e" in s or "E" in s:
        s = f"{float(v):.6e}"
    elif "." not in s:
        s += ".0"
    return s + "f"


def source_hash():
    h = hashlib.sha256()
    for name in ("params.py", "sort_scene.py"):
        with open(os.path.join(ROOT, name), "rb") as f:
            h.update(f.read())
    return h.hexdigest()[:16]


def render():
    tp = SS.task_points()
    L = []
    a = L.append

    def c(name, v, comment=""):
        a(f"constexpr float {name} = {ff(v)};" + (f"  // {comment}" if comment else ""))

    def pt(name, p, comment=""):
        a(f"constexpr Point3 {name} = {{{ff(p[0])}, {ff(p[1])}, {ff(p[2])}}};" + (f"  // {comment}" if comment else ""))

    a("// arm_config.h - SINH TU DONG boi eezybot_mk2_sim/tools/export_arm_config.py tu params.py. KHONG SUA TAY:")
    a("// sua eezybot_mk2_sim/params.py roi chay lai `python tools/export_arm_config.py`.")
    a(f"// Nguon: params.py + sort_scene.py, SHA-256 {source_hash()}")
    a("// Don vi: mm, do, giay. Goc khop: theta1 = de, theta2 = canh tay chinh so voi phuong ngang,")
    a("// phi = tay ngang so voi phuong ngang (servo khuyu dat phi tuyet doi), phi - theta2 = goc khuyu tuong doi.")
    a("#pragma once")
    a("")
    a("namespace armcfg {")
    a("")
    a("struct Point3 { float x, y, z; };")
    a("")
    a("// ---- Hinh hoc co cau [DO-STL] (params.py muc 1)")
    c("L_MAIN_ARM", P.L_UPPER, "L2: vai O -> khuyu E")
    c("L_HORIZ_ARM", P.L_FORE, "L3: khuyu E -> co tay W")
    c("L_HORIZ_TAIL", P.L_TAIL)
    c("L_CRANK", P.L_CRANK)
    c("L_LINK_A", P.L_LINK135)
    c("L_LINK_B", P.L_LINK147)
    c("L_ELBOW_TRI", P.TRI_ARM)
    c("SHOULDER_HEIGHT", P.H_SHOULDER, "H: truc vai so voi mat ban (suy tu STL, nen do lai)")
    c("SHOULDER_OFFSET_X", P.SHOULDER_OFFSET_X, "truc vai so voi truc de theo phuong ngang")
    c("BASE_GEAR_RATIO", P.BASE_GEAR_RATIO, "goc servo de = ti so * theta1")
    a("")
    a("// ---- Dau cong tac so voi tam co tay W: (DX, DZ) trong mat phang canh tay, DY lech ngang (params.py muc 2)")
    c("TOOL_GRIPPER_DX", P.TOOL_OFFSETS["gripper"][0], "TCP gap vat")
    c("TOOL_GRIPPER_DZ", P.TOOL_OFFSETS["gripper"][1])
    c("TOOL_CAP_DX", P.TOOL_OFFSETS["cap"][0], "diem 'cap' = tam nap khi kep (bai phan loai)")
    c("TOOL_CAP_DZ", P.TOOL_OFFSETS["cap"][1])
    c("TOOL_DY", P.TOOL_OFFSET_Y)
    a("")
    a("// ---- Gioi han khop (do), tu kiem tra va cham STL (params.py muc 3). IK phai kiem TAT CA cac dieu kien")
    a("// duoi day giong mk2_kinematics.within_limits().")
    c("THETA1_MIN", P.THETA1_LIM[0]); c("THETA1_MAX", P.THETA1_LIM[1])
    c("THETA2_MIN", P.THETA2_LIM[0]); c("THETA2_MAX", P.THETA2_LIM[1])
    c("PHI_MIN", P.PHI_LIM[0]); c("PHI_MAX", P.PHI_LIM[1])
    c("ELBOW_MIN", P.REL_LIM[0], "phi - theta2"); c("ELBOW_MAX", P.REL_LIM[1])
    a("// Tam lenh actuator (do): de = BASE_GEAR_RATIO * theta1, vai = theta2 - 90, khuyu = phi")
    c("CTRL_BASE_MIN", P.CTRL_BASE_DEG[0]); c("CTRL_BASE_MAX", P.CTRL_BASE_DEG[1])
    c("CTRL_SHOULDER_MIN", P.CTRL_SHOULDER_DEG[0]); c("CTRL_SHOULDER_MAX", P.CTRL_SHOULDER_DEG[1])
    c("CTRL_ELBOW_MIN", P.CTRL_ELBOW_DEG[0]); c("CTRL_ELBOW_MAX", P.CTRL_ELBOW_DEG[1])
    a("")
    a("// ---- Anh xa goc khop -> goc servo (do) [GIA-DINH, phai hieu chuan] (params.py muc 4):")
    a("//   servo[0] = OFFSET[0] + SIGN[0] * BASE_GEAR_RATIO * theta1")
    a("//   servo[1] = OFFSET[1] + SIGN[1] * (theta2 - 90)")
    a("//   servo[2] = OFFSET[2] + SIGN[2] * phi")
    a("constexpr float SERVO_OFFSET_DEG[3] = {" + ", ".join(ff(v) for v in P.SERVO_OFFSET) + "};")
    a("constexpr float SERVO_SIGN[3] = {" + ", ".join(ff(v) for v in P.SERVO_SIGN) + "};")
    c("SERVO_MIN_DEG", P.SERVO_RANGE[0]); c("SERVO_MAX_DEG", P.SERVO_RANGE[1])
    a("// Home: theta1 = 0, theta2 = 90 (tay chinh dung), phi = 0 (tay ngang nam ngang)")
    c("HOME_THETA1", 0.0); c("HOME_THETA2", 90.0); c("HOME_PHI", 0.0)
    a("")
    a("// ---- Quy dao double-S (params.py muc 8). Gioi han cua DAU CONG TAC, dung cho ca duong thang (ha/nhac)")
    a("// va chuyen dong khop: q(t) = q0 + sigma(t) (q1 - q0) — gian thoi gian de dau cong tac khong vuot.")
    c("V_MAX", P.TRAJ_V, "mm/s")
    c("A_MAX", P.TRAJ_A, "mm/s^2")
    c("J_MAX", P.TRAJ_J, "mm/s^3")
    c("TRAJ_R_REF", P.TRAJ_R_REF, "mm, ban kinh quy doi sang gioi han khop [CHON]")
    a("constexpr float JOINT_V_MAX = V_MAX / TRAJ_R_REF;  // rad/s")
    a("constexpr float JOINT_A_MAX = A_MAX / TRAJ_R_REF;  // rad/s^2")
    a("constexpr float JOINT_J_MAX = J_MAX / TRAJ_R_REF;  // rad/s^3")
    c("CONTROL_PERIOD_S", P.CMD_PERIOD, "= chu ky xung servo 20 ms")
    a("")
    a("// ---- Kep SG90 (params.py muc 5 va 8). Goc kep tinh tu luc dong (0 = dong).")
    a(f"constexpr unsigned SG90_MIN_US = {int(P.SG90_PULSE_US[0])}, SG90_MAX_US = {int(P.SG90_PULSE_US[1])};")
    c("GRIPPER_GEAR_RATIO", P.GRIPPER_GEAR_RATIO)
    c("GRIP_OPEN_DEG", P.GRIP_OPEN_DEG, "mo khi ha xuong")
    c("GRIP_HOLD_DEG", P.GRIP_HOLD_DEG, "lenh kep nap")
    c("GRIP_MOVE_S", P.GRIP_MOVE_S, "cho sau moi lenh dong/mo")
    c("DROP_WAIT_S", P.DROP_WAIT_S, "cho nap roi sau khi mo kep")
    a("")
    a("// ---- Chu trinh phan loai: diem 'cap' (tam nap khi kep, mm, he the gioi; dung TOOL_CAP_*) — sort_scene.task_points()")
    c("DROP_HEIGHT_ABOVE_BIN", P.DROP_ABOVE, "day nap cach mat tren hop / mat ban khi tha")
    c("PRE_GRASP_LIFT", P.PRE_GRASP_LIFT)
    pt("P_GRASP", tp["grasp"], "diem kep tai tram")
    pt("P_PRE_GRASP", tp["pre"], "phia tren diem kep")
    for col in ("red", "green", "white"):
        pt(f"P_DROP_{col.upper()}", tp["drop"][col], f"ngan {P.BIN_NUMBER[col]}")
    pt("P_REJECT_PRE", tp["reject_pre"], "phia tren vung vut bo, roi ha thang")
    pt("P_DROP_REJECT", tp["drop"]["reject"])
    a("")
    a("}  // namespace armcfg")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=DEFAULT_OUT, help=f"duong dan arm_config.h (mac dinh {DEFAULT_OUT})")
    ap.add_argument("--check", action="store_true", help="chi so sanh; ma thoat 1 neu file khac noi dung sinh ra")
    args = ap.parse_args()
    text = render()
    if args.check:
        try:
            with open(args.out) as f:
                same = f.read() == text
        except FileNotFoundError:
            same = False
        print(f"{args.out}: {'KHOP params.py' if same else 'CU — chay lai tools/export_arm_config.py'}")
        sys.exit(0 if same else 1)
    if not os.path.isdir(os.path.dirname(os.path.abspath(args.out))):
        sys.exit(f"Khong thay thu muc {os.path.dirname(args.out)}. Clone eezybot_mk2_robot canh eezybot_mk2_sim "
                 f"hoac dung --out.")
    with open(args.out, "w") as f:
        f.write(text)
    print(f"Da ghi {args.out}")


if __name__ == "__main__":
    main()
