"""snapshot.py — Chup anh tay may o mot tu the (dung cho bao cao).
Vi du:  python snapshot.py --servo 90 90 90 --out results/home.png
        python snapshot.py --xyz 180 0 60 --out results/pose.png   (dung IK)
"""
import argparse, os
import numpy as np
import mujoco
from PIL import Image
import mk2_kinematics as K
import sim_common as S

ap = argparse.ArgumentParser()
g = ap.add_mutually_exclusive_group()
g.add_argument("--servo", type=float, nargs=3, help="goc lenh 3 servo (do)")
g.add_argument("--xyz", type=float, nargs=3, help="vi tri dau but (mm) -> IK")
ap.add_argument("--out", default=os.path.join(S.HERE, "results", "snapshot.png"))
ap.add_argument("--azimuth", type=float, default=-125)
args = ap.parse_args()

if args.xyz:
    q = K.ik(np.array(args.xyz)); sv = K.to_servo(q)
else:
    sv = np.array(args.servo if args.servo else [90, 90, 90]); q = K.from_servo(sv)
print("servo (deg):", np.round(sv, 2), "| FK dau but (mm):", np.round(K.fk(*q), 2))

m, d = S.load_model()
S.settle(m, d, S.q_to_ctrl(q), seconds=2.0)
print("MuJoCo dau but (mm):", np.round(d.site("tool").xpos * 1000, 2))
r = mujoco.Renderer(m, 720, 960)
cam = mujoco.MjvCamera(); cam.lookat[:] = [0.08, 0, 0.13]; cam.distance = 0.7
cam.azimuth = args.azimuth; cam.elevation = -18
r.update_scene(d, cam)
os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
Image.fromarray(r.render()).save(args.out)
print("Da luu", args.out)
