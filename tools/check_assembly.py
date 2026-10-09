"""tools/check_assembly.py — Kiem chung lap rap STL cua build_model.py.

  [A] Tai tu the home: khe ho nho nhat giua moi cap chi tiet thuoc 2 vat khac nhau
      (am = lan vao nhau). Cac cap "cham nhau" (khe < 0.5 mm) cho thay chi tiet xep sat nhau tren truc.
  [B] Quet luoi tu the trong gioi han khop (params.THETA2_LIM, PHI_LIM, REL_LIM, THETA1_LIM):
      khong cap nao duoc lan vao nhau sau hon TOL.
  [C] (tuy chon --limits) Quet rong ngoai gioi han, in bang va cham de suy ra gioi han khop.

Phuong phap: moi chi tiet -> truong khoang cach co dau (SDF) tren luoi voxel (lat cat z, quy tac
chan-le, nen dung ca voi luoi STL con ho); lay mau 5000 diem tren be mat chi tiet kia va tra SDF.
Do phan giai luoi (mac dinh 0.3 mm) => mat tiep xuc that hien ra khe ~ -0.2 mm, nen TOL = 0.35 mm.

Chay:  python tools/check_assembly.py --stl-dir "$STL_DIR"           (~4 phut)
       python tools/check_assembly.py --stl-dir "$STL_DIR" --limits  (them ~10 phut)
"""
import argparse, logging, math, os, sys
import numpy as np
import trimesh
logging.getLogger("trimesh").setLevel(logging.ERROR)
from matplotlib.path import Path
from scipy.ndimage import distance_transform_edt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import params as P
import build_model as B

TOL = 0.35
BODY = {  # chi tiet -> vat ran (khong kiem tra 2 chi tiet cung vat)
    "EBAmk2_001_base": "turret", "EBAmk2_011_gearmast": "turret", "EBAmk2_002_mainarm": "mainarm",
    "EBAmk2_006_horarm__": "horarm", "EBAmk2_009_trialinkfront": "wrist", "EBAmk2_014_claw_base": "wrist",
    "EBAmk2_003_varm": "varm", "EBAmk2_004_link135": "link004", "EBAmk2_005_link135angled": "link005",
    "EBAmk2_007_trialink": "trialink", "EBAmk2_008_link147_new": "link008",
    "EBAmk2_015_claw_finger_dx": "finger_dx", "EBAmk2_017_claw_finger_sx": "finger_sx",
    "EBAmk2_018_claw_gear_driven": "finger_sx", "EBAmk2_016_claw_gear_drive": "gear016",
    "EBAmk2_012_mainbase": "ground", "EBAmk2_013_lower_base": "ground", "EBAmk2_019_drive_cover": "ground",
    "pen": "wrist",
}


def sdf_of(mesh, pitch):
    lo, hi = mesh.bounds
    pad = 6
    xs = np.arange(lo[0] + pitch / 2, hi[0], pitch); ys = np.arange(lo[1] + pitch / 2, hi[1], pitch)
    zs = np.arange(lo[2] + pitch / 2, hi[2], pitch)
    X, Y = np.meshgrid(xs, ys, indexing="ij"); XY = np.c_[X.ravel(), Y.ravel()]
    occ = np.zeros((len(xs), len(ys), len(zs)), bool)
    for k, z in enumerate(zs):
        s = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if s is None: continue
        c = np.zeros(len(XY), int)
        for d in s.discrete:
            if len(d) > 3: c += Path(d[:, :2]).contains_points(XY)
        occ[:, :, k] = (c % 2 == 1).reshape(X.shape)
    occ = np.pad(occ, pad)
    origin = np.array([xs[0], ys[0], zs[0]]) - pad * pitch
    sdf = ((distance_transform_edt(~occ) - distance_transform_edt(occ)) * pitch).astype(np.float32)
    return dict(sdf=sdf, origin=origin, pitch=pitch)


def d_at(S, Pts):
    idx = (Pts - S["origin"]) / S["pitch"]
    i0 = np.floor(idx).astype(int); fr = idx - i0
    sh = np.array(S["sdf"].shape)
    out = np.full(len(Pts), 99.0, np.float32)
    ok = np.all((i0 >= 0) & (i0 < sh - 1), axis=1)
    i0, fr, s = i0[ok], fr[ok], S["sdf"]
    acc = 0
    for dx in (0, 1):
        for dy in (0, 1):
            for dz in (0, 1):
                w = (fr[:, 0] if dx else 1 - fr[:, 0]) * (fr[:, 1] if dy else 1 - fr[:, 1]) * (fr[:, 2] if dz else 1 - fr[:, 2])
                acc = acc + w * s[i0[:, 0] + dx, i0[:, 1] + dy, i0[:, 2] + dz]
    out[ok] = acc
    return out


def pen_samples(A):
    """Diem tren mat tru but (he the gioi o tu the A) — but gan cung voi co tay."""
    Tc = A["Tclaw"]; zc = Tc[:3, 2]
    grip = Tc[:3, :3] @ np.array([A["pen_xy"][0], A["pen_xy"][1], B.Z_DX + (0.11 + 9.11) / 2]) + Tc[:3, 3]
    ex = np.cross(zc, [0, 1, 0]); ex /= np.linalg.norm(ex); ey = np.cross(zc, ex)
    rng = np.random.default_rng(1)
    s = rng.uniform(-P.PEN_GRIP_TO_TIP + 12, P.PEN_LENGTH - P.PEN_GRIP_TO_TIP, 3000)
    a = rng.uniform(0, 2 * np.pi, 3000); r = P.PEN_DIAMETER / 2
    return grip + np.outer(s, zc) + r * (np.outer(np.cos(a), ex) + np.outer(np.sin(a), ey))


def rotz(t_deg):
    t = math.radians(t_deg); T = np.eye(4)
    T[:2, :2] = [[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]]
    return T


class Checker:
    def __init__(self, stl_dir, pitch):
        A = B.assemble(stl_dir)
        names = list(A["meshes"]) + list(A["static"])
        print(f"Dung SDF cho {len(names)} chi tiet (luoi {pitch} mm)...", flush=True)
        self.S, self.pts = {}, {}
        for n in names:
            m = (A["meshes"].get(n) or A["static"].get(n))[0]
            self.S[n] = sdf_of(m, pitch)
            self.pts[n] = np.asarray(trimesh.sample.sample_surface_even(m, 5000, radius=0.6, seed=0)[0])
        self.stl_dir = stl_dir

    def poses(self, th2, phi, th1=0.0):
        A = B.assemble(self.stl_dir, th2, phi)
        Rz = rotz(th1)
        T = {n: Rz @ Tm for n, (_, Tm) in A["meshes"].items()}
        T.update({n: Tm for n, (_, Tm) in A["static"].items()})
        pen = (Rz[:3, :3] @ pen_samples(A).T).T
        return T, pen

    def worst(self, T, pen, only_pairs=None):
        names = list(T); res = {}
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                if BODY[a] == BODY[b] or (BODY[a] == "ground" and BODY[b] == "ground"): continue
                if only_pairs and (a, b) not in only_pairs and (b, a) not in only_pairs: continue
                Ta, Tb = T[a], T[b]
                pa = self.pts[a] @ Ta[:3, :3].T + Ta[:3, 3]
                d1 = d_at(self.S[b], (pa - Tb[:3, 3]) @ Tb[:3, :3]).min()
                pb = self.pts[b] @ Tb[:3, :3].T + Tb[:3, 3]
                d2 = d_at(self.S[a], (pb - Ta[:3, 3]) @ Ta[:3, :3]).min()
                res[(a, b)] = float(min(d1, d2))
        for b in names:
            if BODY[b] == "wrist": continue
            Tb = T[b]
            res[("pen", b)] = float(d_at(self.S[b], (pen - Tb[:3, 3]) @ Tb[:3, :3]).min())
        return res


def short(n):
    return n.replace("EBAmk2_", "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl-dir", required=True)
    ap.add_argument("--pitch", type=float, default=0.3)
    ap.add_argument("--limits", action="store_true", help="quet ca ngoai gioi han khop (cham)")
    args = ap.parse_args()
    C = Checker(args.stl_dir, args.pitch)

    # [A] home
    T, pen = C.poses(90.0, 0.0)
    w = C.worst(T, pen)
    touch = sorted((v, short(a), short(b)) for (a, b), v in w.items() if v < 0.5)
    print("[A] Tu the home — cac cap cham nhau (khe < 0.5 mm; am nho la sai so luoi):")
    for v, a, b in touch:
        print(f"    {a:28s} - {b:28s} khe {v:+.2f} mm")
    bad_home = [x for x in touch if x[0] < -TOL]

    # [B] luoi tu the trong gioi han
    t2s = np.linspace(P.THETA2_LIM[0], P.THETA2_LIM[1], 7)
    phs = np.linspace(P.PHI_LIM[0], P.PHI_LIM[1], 7)
    poses = [(t2, ph) for t2 in t2s for ph in phs if P.REL_LIM[0] <= ph - t2 <= P.REL_LIM[1]]
    worst_all = {}
    for t2, ph in poses:
        for th1 in (P.THETA1_LIM[0], 0.0, P.THETA1_LIM[1]) if (t2, ph) == poses[0] else (0.0,):
            T, pen = C.poses(t2, ph, th1)
            for k, v in C.worst(T, pen).items():
                if v < worst_all.get(k, (99,))[0]: worst_all[k] = (v, (round(t2, 1), round(ph, 1), th1))
    bad = sorted(((v, k, p) for k, (v, p) in worst_all.items() if v < -TOL), key=lambda x: x[0])
    print(f"[B] {len(poses)} tu the trong gioi han (theta2 {P.THETA2_LIM}, phi {P.PHI_LIM}, rel {P.REL_LIM}):")
    vmin, kmin = min((v, k) for k, (v, p) in worst_all.items())
    print(f"    khe nho nhat = {vmin:+.2f} mm ({short(kmin[0])} - {short(kmin[1])} tai {worst_all[kmin][1]})")
    for v, k, p in bad:
        print(f"    LAN VAO NHAU {v:+.2f} mm: {short(k[0])} - {short(k[1])} tai (theta2, phi, theta1) = {p}")
    ok = not bad and not bad_home
    print("PASS: khong chi tiet nao lan vao nhau trong gioi han khop" if ok else "FAIL: xem cac dong LAN VAO NHAU o tren")

    if args.limits:
        print("[C] Bang va cham theo (theta2 hang, phi cot); '.' = khong va cham, chu = cap va cham dau tien")
        phs = np.arange(-90, 41, 5.0)
        print("phi:  " + "".join(f"{int(p):>5d}" for p in phs))
        for t2 in np.arange(30, 136, 5.0):
            row = ""
            for ph in phs:
                T, pen = C.poses(t2, ph)
                w = C.worst(T, pen); k = min(w, key=w.get)
                row += f"{'.' if w[k] >= -TOL else short(k[0])[:2] + short(k[1])[:2]:>5s}"
            print(f"{t2:5.0f} {row}", flush=True)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
