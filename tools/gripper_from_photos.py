"""tools/gripper_from_photos.py — Trich bien dang gripper THAT tu anh chup, sinh gripper_shape.py.

Gripper tren robot that la thiet ke rieng (khong phai kep goc 014-018) va khong co file CAD.
Kich thuoc co cau (khoang cach truc, chieu dai khau...) do bang thuoc kep va nam trong params.py.
Script nay chi lay tu anh nhung thu thuoc kep khong do duoc: BIEN DANG ngon, vien de, khoi giu chot.

Can 3 anh (nam trong file du an, KHONG co trong repo):
  --closed  "Gripper đóng - chụp từ trên xuống.png"
  --open    "Gripper mở - chụp từ trên xuống.png"
  --bottom  "gripper mở - chụp từ dưới lên.png"

Phuong phap
  1. Nen anh da duoc xoa thanh den tuyet doi (0, 0, 0) -> mat na = pixel khac 0; bo vach do/nhan.
  2. Bien dang ngon trai: quet tung hang anh "dong" (mep ngoai = pixel trai nhat, mat kep = mep trong),
     noi suy qua cac hang bi vach do/mui ten che.
  3. Dat vao he ngon (goc = oc duoi Lw, oc tren U = F) bang phep dong dang xac dinh boi CHINH 2 lo oc
     cua ngon (cach nhau GRIP_FINGER_HOLES = 20 mm, do bang thuoc kep) -> khong dung nhan ve tay.
  4. Lam tron 0.5 mm, don gian hoa 0.2 mm; lop long ngon = bien dang thut vao 1.5 mm.
  5. Kiem chung: khop dong dang (ti le + xoay + tinh tien) giua vi tri cac oc trong mo hinh
     (params.py) va tam oc tren 3 anh -> in sai so RMS; vien de lay tu anh duoi, khoi chot tu anh mo.

Toa do pixel cac tam oc duoi day doc tay tren anh goc, roi tinh chinh tu dong theo cum diem
sang mau bac (refine_bright). Oc den o giua (anh tren) khong sang nen giu nguyen.

Chay:  python tools/gripper_from_photos.py --closed A.png --open B.png --bottom C.png
"""
import argparse, math, os, sys
import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.spatial import ConvexHull
import contourpy
from matplotlib.path import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import params as P

# ------------------------------------------------------------------ du lieu doc tren anh (pixel anh goc)
PTS = {
    "closed": dict(Lw_L=(1042, 2580), U_L=(1312, 2008), U_R=(1695, 2017), Lw_R=(1970, 2580),
                   C_L=(1350, 3083), C_R=(1670, 3075)),
    "open": dict(Lw_L=(812, 2233), U_L=(1067, 1758), U_R=(3050, 1775), Lw_R=(3283, 2242),
                 C_L=(1925, 1708), C_R=(2180, 1708)),
    # anh tu duoi len la anh guong: ngon phia tren anh <-> ngon trai (fit cho phep phan xa)
    "bottom": dict(Lw_L=(3300, 778), U_L=(3750, 925), U_R=(3822, 2889), Lw_R=(3389, 3078)),
}
BOTTOM_BASE_PX = [(2939, 1514), (2744, 1722), (2744, 2111), (2953, 2347), (3967, 2417),
                  (4036, 2222), (4036, 1806), (3967, 1458)]          # vien de (mat duoi)
OPEN_BLOCK_PX = [(1770, 1666), (1848, 1605), (2273, 1605), (2360, 1683)]   # mep truoc khoi chot
OPEN_SERVO_PX = [(1405, 1978), (2117, 1978), (2117, 2325), (1405, 2325)]   # than SG90 (anh mo)
CLOSED_MID_X = 1506                     # duong tam anh "dong" (trung binh 3 cap oc)
# cac dai hang bi chu thich che tren anh "dong": vach "2.5 cm", mui ten "Oc vit", vach "4 cm"
COVERED_ROWS = ((1150, 1330), (1900, 2090), (2230, 2410))

# ------------------------------------------------------------------ hinh hoc co cau (params.py)
G = P.GRIP_GEAR_DIST / 2
CX = P.GRIP_PIVOT_DIST / 2
F = np.array([G - CX, math.sqrt(P.GRIP_FINGER_HOLES ** 2 - (G - CX) ** 2)])
L = P.GRIP_LINK


def pose(phi_deg, s):
    ph = math.radians(phi_deg)
    g = np.array([s * G, 0.0]); c = np.array([s * CX, F[1]])
    d = np.array([s * math.sin(ph), math.cos(ph)]) * L
    return dict(g=g, c=c, lw=g + d, up=c + d)


def load(path):
    return np.asarray(Image.open(path).convert("RGB")).astype(int)


def refine_bright(img, p, r=70):
    """Tam cum diem sang mau bac (dau oc vit) quanh p; giu nguyen p neu khong co."""
    x0, y0 = int(p[0]), int(p[1])
    win = img[y0 - r:y0 + r, x0 - r:x0 + r]
    mx, mn = win.max(2), win.min(2)
    m = (mn > 120) & (mx - mn < 70)
    if m.sum() < 40:
        return np.array(p, float)
    ys, xs = np.nonzero(m)
    return np.array([x0 - r + xs.mean(), y0 - r + ys.mean()])


def px2m(p):            # pixel -> toa do toan hoc (truc y huong len)
    return np.array([p[0], -p[1]], float)


def umeyama(src, dst, allow_reflection=False):
    """dst ~ s R src + t (Umeyama 1991). Tra ve s, R, t, rms (don vi dst)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    A, B = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(B.T @ A / len(src))
    D = np.eye(2)
    if np.linalg.det(U @ Vt) < 0 and not allow_reflection:
        D[1, 1] = -1
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / (A ** 2).sum(1).mean()
    t = mu_d - s * R @ mu_s
    res = dst - (s * (R @ src.T).T + t)
    return s, R, t, math.sqrt((res ** 2).sum(1).mean())


def douglas_peucker(pts, tol):
    """Don gian hoa duong cong kin (mang N x 2, diem dau != diem cuoi)."""
    def dp(a):
        if len(a) < 3:
            return a
        p0, p1 = a[0], a[-1]
        d = p1 - p0; n = np.hypot(*d)
        dist = (np.abs(np.cross(d, a - p0)) / n) if n > 1e-12 else np.hypot(*(a - p0).T)
        i = int(np.argmax(dist))
        if dist[i] <= tol:
            return np.array([p0, p1])
        return np.vstack([dp(a[:i + 1])[:-1], dp(a[i:])])
    i0 = int(np.argmax(np.hypot(*(pts - pts[0]).T)))       # chia vong kin thanh 2 nua
    return np.vstack([dp(pts[:i0 + 1])[:-1], dp(np.vstack([pts[i0:], pts[:1]]))[:-1]])


def raster_ops(poly, res=0.05, close_r=0.5, inset=0.0):
    """Lam tron (dong hinh thai ban kinh close_r) va thut vao 'inset' mm bang bien doi khoang cach
    tren luoi res mm; tra ve duong vien ngoai (N x 2)."""
    lo, hi = poly.min(0) - 3, poly.max(0) + 3
    xs = np.arange(lo[0], hi[0], res); ys = np.arange(lo[1], hi[1], res)
    X, Y = np.meshgrid(xs, ys)
    inside = Path(poly).contains_points(np.c_[X.ravel(), Y.ravel()]).reshape(X.shape)
    if close_r > 0:
        dil = ndimage.distance_transform_edt(~inside) * res <= close_r
        inside = ndimage.distance_transform_edt(dil) * res > close_r
    if inset > 0:
        inside = ndimage.distance_transform_edt(inside) * res > inset
    gen = contourpy.contour_generator(xs, ys, inside.astype(float))
    lines = gen.lines(0.5)
    c = max(lines, key=len)
    if np.allclose(c[0], c[-1]):
        c = c[:-1]
    area = 0.5 * np.sum(c[:, 0] * np.roll(c[:, 1], -1) - np.roll(c[:, 0], -1) * c[:, 1])
    return c if area > 0 else c[::-1]          # nguoc chieu kim dong ho


def finger_outline(img_closed):
    """Bien dang ngon trai (pixel anh 'dong')."""
    img = img_closed
    m = img.sum(2) > 0
    r, g, b = img[..., 0], img[..., 1], img[..., 2]
    m[(r > 180) & (g < 110) & (b < 110)] = False              # vach do
    m[:340] = False; m[:, :800] = False                      # tieu de + chu thich ben trai
    m = ndimage.binary_closing(m, structure=np.ones((15, 15), bool))
    MID = CLOSED_MID_X
    ys = np.arange(700, 2560, 4)
    outer = np.full(len(ys), np.nan); inner = np.full(len(ys), np.nan)
    for i, y in enumerate(ys):
        row = np.nonzero(m[y, 800:MID + 1])[0]
        if len(row) == 0:
            continue
        outer[i] = 800 + row[0]
        if y < 2150:
            if m[y, MID]:
                inner[i] = MID                                  # 2 ngon cham nhau tai duong tam
            else:
                k = MID
                while k > 800 and not m[y, k]:
                    k -= 1
                inner[i] = k
    covered = np.zeros(len(ys), bool)
    for a, b_ in COVERED_ROWS:
        covered |= (ys > a) & (ys < b_)
    outer[covered] = np.nan
    inner[covered & (ys < 2150)] = np.nan
    for arr, rng in ((outer, np.ones(len(ys), bool)), (inner, ys < 2150)):
        ok = ~np.isnan(arr) & rng; fill = np.isnan(arr) & rng
        arr[fill] = np.interp(ys[fill], ys[ok], arr[ok])
    sel = ys < 2150
    pts = [(x, y) for x, y in zip(outer, ys)]
    # day ngon: vong quanh oc duoi; mat trong phan goc ngon (khe giua ngon va khau tren, anh dong)
    pts += [(outer[-1], 2620), (960, 2700), (1042, 2730), (1130, 2700), (1215, 2640)]
    pts += [(1215, 2330), (1440, 2150)]
    pts += [(x, y) for x, y in zip(inner[sel][::-1], ys[sel][::-1])]
    return np.array(pts, float)


def model_points(phi):
    pl, pr = pose(phi, -1), pose(phi, 1)
    return dict(Lw_L=pl["lw"], U_L=pl["up"], C_L=pl["c"], Lw_R=pr["lw"], U_R=pr["up"], C_R=pr["c"])


def fmt_poly(name, a, note):
    rows = ",\n".join("    (%.2f, %.2f)" % (x, y) for x, y in a)
    return f"# {note}\n{name} = [\n{rows},\n]\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--closed", required=True); ap.add_argument("--open", required=True)
    ap.add_argument("--bottom", required=True)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(HERE), "gripper_shape.py"))
    args = ap.parse_args()
    imgs = {k: load(getattr(args, k)) for k in ("closed", "open", "bottom")}

    # --- bien dang ngon, he ngon (goc = Lw, U = F)
    lw = refine_bright(imgs["closed"], PTS["closed"]["Lw_L"])
    up = refine_bright(imgs["closed"], PTS["closed"]["U_L"])
    z1, z2 = complex(*px2m(lw)), complex(*px2m(up))
    a = complex(*F) / (z2 - z1)
    zz = np.array([complex(*px2m(p)) for p in finger_outline(imgs["closed"])])
    w = a * (zz - z1)
    raw = np.c_[w.real, w.imag]
    fin = douglas_peucker(raster_ops(raw, close_r=0.5), 0.2)
    fin_in = douglas_peucker(raster_ops(raw, close_r=0.5, inset=1.5), 0.2)
    scale = 1 / abs(a)
    print(f"Ngon: ti le anh 'dong' {scale:.2f} px/mm (theo 2 lo oc cach {P.GRIP_FINGER_HOLES:g} mm), "
          f"xoay {math.degrees(np.angle(a)):.2f} do; {len(fin)} dinh, long ngon {len(fin_in)} dinh")
    xmax, ymax = fin[:, 0].max(), fin[:, 1].max()
    band = fin[fin[:, 1] > ymax - 0.5]
    print(f"  mat kep (x max) = {xmax:.2f} mm, mut ngon (y max) = {ymax:.2f} mm, "
          f"be rong mut (0.5 mm tren cung) ~ {band[:, 0].max() - band[:, 0].min():.2f} mm (thuoc kep: 4.0)")
    print(f"  goc khau de 2 mut cham nhau = {math.degrees(math.asin((xmax - G) / L)):.2f} do "
          f"(nguoi dung: dong = khau thang 0 do)")

    # --- khop dong dang voi 3 anh
    fits = {}
    for st, phi in (("closed", 0.0), ("open", P.GRIP_TRAVEL_DEG), ("bottom", P.GRIP_TRAVEL_DEG)):
        mp = model_points(phi)
        keys = list(PTS[st])
        meas = {k: (refine_bright(imgs[st], v) if k[0] in "LU" else np.array(v, float)) for k, v in PTS[st].items()}
        src = np.array([mp[k] for k in keys]); dst = np.array([px2m(meas[k]) for k in keys])
        s, R, t, rms = umeyama(src, dst, allow_reflection=(st == "bottom"))
        fits[st] = (s, R, t)
        print(f"Khop anh '{st}': {s:.2f} px/mm, sai so RMS vi tri oc = {rms / s:.2f} mm ({len(keys)} oc)")
    inv = lambda st, p: np.linalg.solve(fits[st][0] * fits[st][1], px2m(p) - fits[st][2])
    base = np.array([inv("bottom", p) for p in BOTTOM_BASE_PX])
    base = base[ConvexHull(base).vertices]
    blk = [inv("open", p) for p in OPEN_BLOCK_PX]
    block = np.array([*blk, (blk[-1][0], 8.0), (blk[0][0], 8.0)])
    servo = np.array([inv("open", p) for p in OPEN_SERVO_PX])
    servo_gx = float(servo[:, 0].mean())
    print(f"De: gx {base[:, 0].min():.1f}..{base[:, 0].max():.1f}, gy {base[:, 1].min():.1f}..{base[:, 1].max():.1f} mm; "
          f"khoi chot gy {block[:, 1].min():.1f}..{block[:, 1].max():.1f}; tam ngang SG90 gx = {servo_gx:.1f} mm")

    txt = ['"""gripper_shape.py — SINH TU DONG boi tools/gripper_from_photos.py, KHONG sua tay.',
           "",
           "Bien dang lay tu anh chup gripper that (he G va he ngon dinh nghia trong params.py, muc 2):",
           "  FINGER: ngon TRAI, he ngon (goc = tam oc duoi Lw, x = gx, y = gy); mat kep huong +x.",
           "          Dat theo 2 lo oc cua ngon (cach 20 mm, thuoc kep). Ngon phai = guong qua truc y.",
           "  FINGER_INNER: lop long ngon, thut 1.5 mm so voi FINGER.",
           "  BASE: vien de (mat duoi), he G, tu anh 'mo - chup tu duoi len'.",
           "  BLOCK: khoi giu 2 chot khau tren, he G, tu anh 'mo - chup tu tren xuong'.",
           "  SERVO_GX: tam SG90 theo gx (he G), tu anh 'mo - chup tu tren xuong'.",
           "Don vi mm. [ANH]",
           '"""', ""]
    txt.append(fmt_poly("FINGER", fin, "ngon trai, he ngon"))
    txt.append(fmt_poly("FINGER_INNER", fin_in, "long ngon (thut 1.5 mm), he ngon"))
    txt.append(fmt_poly("BASE", base, "vien de, he G"))
    txt.append(fmt_poly("BLOCK", block, "khoi giu chot khau tren, he G"))
    txt.append(f"SERVO_GX = {servo_gx:.2f}   # tam than SG90 theo gx, he G\n")
    with open(args.out, "w") as f:
        f.write("\n".join(txt))
    print("Da ghi", args.out)


if __name__ == "__main__":
    main()
