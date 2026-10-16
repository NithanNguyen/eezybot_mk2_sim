"""snapshot.py — Chup anh tay may o mot tu the (dung cho bao cao).
Mac dinh: mo hinh gripper KHONG but, --xyz la vi tri TCP gap vat (giua 2 dem mui ngon).
Co --pen: gan but vao gripper, --xyz la vi tri ngoi but.
Vi du:  python snapshot.py --servo 90 90 90 --out results/home.png
        python snapshot.py --xyz 260 0 120 --out results/pose_gripper.png      (dung IK, TCP gripper)
        python snapshot.py --pen --xyz 290 0 20 --out results/pose.png         (dung IK, ngoi but)
        python snapshot.py --grip 90 --servo 90 90 90 --out results/open.png   (mo gripper 90 do)
"""
import argparse, math, os
import numpy as np
import mujoco
from PIL import Image
import params as P
import mk2_kinematics as K
import sim_common as S

ap = argparse.ArgumentParser()
g = ap.add_mutually_exclusive_group()
g.add_argument("--servo", type=float, nargs=3, help="goc lenh 3 servo (do)")
g.add_argument("--xyz", type=float, nargs=3, help="vi tri dau cong tac (mm) -> IK")
ap.add_argument("--pen", action="store_true", help="gan but vao gripper; dau cong tac = ngoi but")
ap.add_argument("--grip", type=float, default=None,
                help="goc servo kep (do, 0 = dong, 90 = mo het); mac dinh: dong, hoac goc kep but neu --pen")
ap.add_argument("--out", default=os.path.join(S.HERE, "results", "snapshot.png"))
ap.add_argument("--azimuth", type=float, default=-125)
args = ap.parse_args()
K.set_tool("pen" if args.pen else "gripper")
tool = "ngoi but" if args.pen else "TCP gripper"

if args.xyz:
    q = K.ik(np.array(args.xyz)); sv = K.to_servo(q)
else:
    sv = np.array(args.servo if args.servo else [90, 90, 90]); q = K.from_servo(sv)
print(f"servo (deg): {np.round(sv, 2)} | FK {tool} (mm): {np.round(K.fk(*q), 2)}")

m, d = S.load_model(pen=args.pen)
if args.grip is not None:
    d.ctrl[m.actuator("servo_gripper").id] = P.GRIPPER_GEAR_RATIO * math.radians(args.grip)
S.settle(m, d, S.q_to_ctrl(q), seconds=2.0)
print(f"MuJoCo {tool} (mm): {np.round(d.site('tool').xpos * 1000, 2)}")
r = mujoco.Renderer(m, 720, 960)
cam = mujoco.MjvCamera(); cam.lookat[:] = [0.08, 0, 0.13]; cam.distance = 0.7
cam.azimuth = args.azimuth; cam.elevation = -18
r.update_scene(d, cam)
os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
Image.fromarray(r.render()).save(args.out)
print("Da luu", args.out)
