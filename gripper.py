"""gripper.py — Gripper THAT tren robot: hinh hoc, dong hoc, luoi va khoi luong.

Co cau (params.py, muc 2): moi ben 1 chi tiet "banh rang + khau duoi" quay quanh truc banh rang G,
1 "khau tren" quay quanh chot C tren than; ngon noi dau 2 khau -> 2 hinh binh hanh G-Lw-U-C,
ngon TINH TIEN (khong xoay). 2 banh rang bang nhau an khop 1:1; truc SG90 = truc banh rang trai.
Goc servo phi: 0 = dong (khau duoi thang, huong mui ngon), 90 do = mo (khau vuong goc canh tay).

He toa do
  He G (mm): goc = trung diem 2 truc banh rang; gx = ngang (phai khi nhin tu tren, mui ngon huong len
             tren anh), gy = huong mui ngon; z = len, z = 0 o mat day gripper.
  He co tay (mm): goc = tam lo W (dau tay ngang 006), truc song song the gioi o tu the home
             (x truoc, y trai, z len) — trung he body "wrist" cua build_model.py.
             x = GRIP_W_TO_GEAR + gy,  y = GRIP_CENTER_Y - gx,  z = z_G - GRIP_W_HEIGHT.
  He body MuJoCo cua tung chi tiet: goc tai truc khop, truc song song he co tay khi phi = 0.

Lop theo chieu day (z_G, mm; nguoi dung xac nhan 2026-10-16):
  de 0-3.5 | banh rang + khau duoi + khau tren CUNG 1 lop 3.5-10.5 (mat giua 7.0 = do cao tam W)
  khoi giu chot 10.5-15 | ngon: tam duoi 0-3.3, long 3.3-10.7 (thut 1.5 mm, chi o phan mut ngon,
  phan goc ngon de trong cho 2 khau lot vao), tam tren 10.7-15 | SG90 dat tren mat lop banh rang.
"""
import math
import numpy as np
import trimesh
from scipy.optimize import brentq
from scipy.spatial import ConvexHull
from matplotlib.path import Path
import params as P
import mk2_kinematics as K
import gripper_shape as GS

# ------------------------------------------------------------------ kich thuoc (mm)
G = P.GRIP_GEAR_DIST / 2                       # |gx| truc banh rang
CX = P.GRIP_PIVOT_DIST / 2                     # |gx| chot khau tren
F = np.array([G - CX, math.sqrt(P.GRIP_FINGER_HOLES ** 2 - (G - CX) ** 2)])   # Lw -> U (= G -> C)
CY = F[1]                                      # gy chot khau tren
L = P.GRIP_LINK
R_GEAR = G                                     # 2 banh rang bang nhau, an khop tai gx = 0
W_LINK = P.GRIP_LINK_WIDTH
T = P.GRIP_THICKNESS
D_WG, Z_W, Y_C = P.GRIP_W_TO_GEAR, P.GRIP_W_HEIGHT, P.GRIP_CENTER_Y
Z_BASE, Z_MID, Z_TOP = (0.0, 3.5), (3.5, 10.5), (10.5, 15.0)
Z_FIN = ((0.0, 3.3), (3.3, 10.7), (10.7, 15.0))
SG_L, SG_W, SG_H = P.SG90["size"]              # 22.2 (dai, theo gx) x 11.8 (rong, theo gy) x 31 (cao)
# Vau ga cua 009 goc (do tren STL da lap, build_model.assemble): vau W rong 12 mm lot khe 006,
# vau Q rong 22 mm tua vao 008. Thap ga cua gripper thay 009 nen giu dung be rong/vi tri nay.
Y_LUG_W = (-2.75, 9.25)
Y_LUG_Q = (-12.75, 9.25)
R_LUG_W = 6.5                  # ban kinh vau W (009 goc: 6.55)
OFF_TOP = 4.0                  # canh tren nhanh W->Q cach duong tam W-Q (009 goc: 4.0); 006 quay quanh W khong cham

FINGER = np.array(GS.FINGER, float)            # ngon trai, he ngon (goc = Lw)
FINGER_INNER = np.array(GS.FINGER_INNER, float)
FIN_XMAX, FIN_YMAX = FINGER[:, 0].max(), FINGER[:, 1].max()


def _wq():
    """Q - W (doc tay, dung) — hang so cua hinh binh hanh 006/008 (khong phu thuoc tu the)."""
    p = K.linkage_points(math.radians(90.0), 0.0)
    return p["Q"] - p["W"]


WQ = _wq()                                       # (32.60, 28.04)


# ------------------------------------------------------------------ dong hoc
def pose(phi_deg, s):
    """Diem khop (he G) cua ben s (-1 trai, +1 phai) o goc servo phi (do)."""
    ph = math.radians(phi_deg)
    g = np.array([s * G, 0.0]); c = np.array([s * CX, CY])
    d = np.array([s * math.sin(ph), math.cos(ph)]) * L
    return dict(g=g, c=c, lw=g + d, up=c + d)


def g2w(gx, gy, zg):
    """He G -> he co tay (mm)."""
    return np.array([D_WG + gy, Y_C - gx, zg - Z_W])


def finger_G(phi_deg, s):
    """Bien dang ngon ben s trong he G."""
    loc = FINGER if s < 0 else FINGER * [-1, 1]
    return loc + pose(phi_deg, s)["lw"]


def gap_at(phi_deg):
    """Khe ho nho nhat giua 2 ngon (mm) — bang 2 lan khoang cach mat kep toi truc gx = 0."""
    return 2 * (-finger_G(phi_deg, -1)[:, 0].max())


def _dist_point_poly(p, poly):
    a = poly; b = np.roll(poly, -1, axis=0); ab = b - a
    t = np.clip(((p - a) * ab).sum(1) / (ab * ab).sum(1), 0, 1)
    return np.hypot(*(a + ab * t[:, None] - p).T).min()


def pen_center_G(phi_deg):
    return np.array([0.0, L * math.cos(math.radians(phi_deg)) + FIN_YMAX - P.PEN_FROM_TIP])


def pen_grip_deg():
    """Goc servo (do) de but D = PEN_DIAMETER (tam o truc gripper, lui PEN_FROM_TIP so voi mut ngon)
    vua cham ca 2 mat kep."""
    def f(ph):
        c = pen_center_G(ph); poly = finger_G(ph, -1)
        inside = Path(poly).contains_point(c)
        d = _dist_point_poly(c, poly)
        return (-d if inside else d) - P.PEN_DIAMETER / 2
    return brentq(f, 0.0, 45.0, xtol=1e-7)


def tcp_w():
    """TCP gap vat (he co tay): giua 2 dem mui ngon luc dong, lui GRIP_TCP_BACK, giua chieu day."""
    return g2w(0.0, L + FIN_YMAX - P.GRIP_TCP_BACK, T / 2)


def pen_grip_w(phi_deg=None):
    ph = pen_grip_deg() if phi_deg is None else phi_deg
    return g2w(*pen_center_G(ph), T / 2)


# ------------------------------------------------------------------ hinh 2D
def circle(c, r, n=48):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a)]


def stadium(p, q, w, n=24):
    """Hinh chu nhat bo tron 2 dau tu p toi q, be rong w (nguoc chieu kim dong ho)."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = q - p; th = math.atan2(d[1], d[0]); r = w / 2
    a1 = np.linspace(th - math.pi / 2, th + math.pi / 2, n)
    a2 = np.linspace(th + math.pi / 2, th + 3 * math.pi / 2, n)
    return np.vstack([q + r * np.c_[np.cos(a1), np.sin(a1)], p + r * np.c_[np.cos(a2), np.sin(a2)]])


def gear_link_outline(n=96):
    """Banh rang (vong chia R_GEAR) + khau duoi la MOT chi tiet; he body: truc tai goc, khau theo +x."""
    r = W_LINK / 2
    a0 = math.asin(r / R_GEAR)
    arc = np.linspace(a0, 2 * math.pi - a0, n)
    gear = np.c_[R_GEAR * np.cos(arc), R_GEAR * np.sin(arc)]           # (x0, r) -> (x0, -r)
    end = np.linspace(-math.pi / 2, math.pi / 2, 24)
    tip = np.c_[L + r * np.cos(end), r * np.sin(end)]                   # (L, -r) -> (L, r)
    return np.vstack([gear, tip])


def hull(points):
    pts = np.vstack(points)
    return pts[ConvexHull(pts).vertices]


def clip_halfplane(poly, n, c):
    """Sutherland-Hodgman: giu phan n . p >= c."""
    out = []
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        da, db = n @ a - c, n @ b - c
        if da >= 0: out.append(a)
        if (da >= 0) != (db >= 0):
            out.append(a + (b - a) * (da / (da - db)))
    return np.array(out)


def _signed_area(r):
    return 0.5 * np.sum(r[:, 0] * np.roll(r[:, 1], -1) - np.roll(r[:, 0], -1) * r[:, 1])


def _clean_ring(r, ccw=True, eps=1e-9):
    """Bo diem trung va diem thang hang; dat chieu (ccw = nguoc chieu kim dong ho)."""
    r = np.asarray(r, float)
    keep = np.hypot(*(r - np.roll(r, -1, axis=0)).T) > 1e-7
    r = r[keep]
    changed = True
    while changed and len(r) > 3:
        a, b, c = np.roll(r, 1, axis=0), r, np.roll(r, -1, axis=0)
        cr = np.cross(b - a, c - b)
        col = np.abs(cr) <= eps * np.maximum(1.0, np.hypot(*(c - a).T))
        changed = bool(col.any())
        if changed:
            r = np.delete(r, int(np.argmax(col)), axis=0)
    if (_signed_area(r) > 0) != ccw:
        r = r[::-1]
    return r


def _seg_cross(p1, p2, q1, q2):
    """2 doan thang cat nhau tai diem trong (khong tinh cham dau mut)."""
    d = lambda a, b, c: np.cross(b - a, c - a)
    d1, d2, d3, d4 = d(q1, q2, p1), d(q1, q2, p2), d(p1, p2, q1), d(p1, p2, q2)
    return (d1 * d2 < -1e-12) and (d3 * d4 < -1e-12)


def _bridge(outer, hole):
    """Noi lo vao vien ngoai bang 1 cau (2 canh trung nhau) -> 1 vong don."""
    edges = [(outer[i], outer[(i + 1) % len(outer)]) for i in range(len(outer))] + \
            [(hole[i], hole[(i + 1) % len(hole)]) for i in range(len(hole))]
    best = None
    for j, h in enumerate(hole):
        for i, o in enumerate(outer):
            dd = np.hypot(*(o - h))
            if best is not None and dd >= best[0]:
                continue
            if any(_seg_cross(h, o, a, b) for a, b in edges):
                continue
            best = (dd, i, j)
    _, i, j = best
    hj = np.roll(hole, -j, axis=0)
    return np.vstack([outer[:i + 1], hj, hj[:1], outer[i:]])


def _earclip(V):
    """Ear clipping cho da giac don (ccw, co the co dinh trung do cau noi lo). Tra ve tam giac (idx)."""
    idx = list(range(len(V))); tris = []

    def inside(p, a, b, c):
        if (np.allclose(p, a) or np.allclose(p, b) or np.allclose(p, c)):
            return False
        d1, d2, d3 = np.cross(b - a, p - a), np.cross(c - b, p - b), np.cross(a - c, p - c)
        return d1 >= -1e-12 and d2 >= -1e-12 and d3 >= -1e-12
    guard = 0
    while len(idx) > 3:
        n = len(idx); done = False
        for k in range(n):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % n]
            a, b, c = V[i0], V[i1], V[i2]
            if np.cross(b - a, c - b) <= 1e-12:
                continue
            if any(inside(V[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2)); idx.pop(k); done = True
            break
        guard += 1
        if not done or guard > 100000:
            raise RuntimeError("ear clipping that bai (da giac tu cat?)")
    tris.append(tuple(idx))
    return np.array(tris)


def triangulate(poly, holes=()):
    """Tam giac hoa da giac don (co the lom, co lo). Kiem tra tong dien tich bang cong thuc Gauss."""
    outer = _clean_ring(poly, ccw=True)
    ring = outer
    a_ref = _signed_area(outer)
    for h in holes:
        hc = _clean_ring(h, ccw=False)
        a_ref += _signed_area(hc)
        ring = _bridge(ring, hc)
    tri = _earclip(ring)
    e = ring[tri]
    a_tri = 0.5 * np.cross(e[:, 1] - e[:, 0], e[:, 2] - e[:, 0]).sum()
    if abs(a_tri - a_ref) > 1e-6 * max(1.0, a_ref):
        raise RuntimeError(f"tam giac hoa sai: {a_tri:.4f} != {a_ref:.4f} mm2")
    # bo dinh trung (cau noi lo) de luoi kin
    V, inv = np.unique(np.round(ring, 9), axis=0, return_inverse=True)
    return V, inv.reshape(-1)[tri]


def extrude(poly, z0, z1, holes=(), plane="xy"):
    """Lang tru (trimesh, mm). plane 'xy': poly theo (x, y), cao theo z.
    plane 'xz': poly theo (x, z), day theo y tu z0 toi z1."""
    V, tri = triangulate(poly, holes)
    m = trimesh.creation.extrude_triangulation(V, tri, z1 - z0)
    m.apply_translation([0, 0, z0])
    if plane == "xz":                                     # (x, y, z) -> (x, z, y)
        M = np.eye(4); M[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, 1, 0]]
        m.apply_transform(M)                               # trimesh tu lat thu tu dinh khi det < 0
    if not m.is_watertight or m.volume <= 0:
        raise RuntimeError("luoi lang tru khong kin hoac sai chieu phap tuyen")
    return m


# ------------------------------------------------------------------ chi tiet (he body, mm)
def servo_box_w():
    """(tam, kich thuoc) hop SG90 trong he co tay: truc ra tren truc banh rang trai, than nam ngang,
    dai theo gx (tam theo anh), rong theo gy (tam o gy = 0), dat tren mat lop banh rang."""
    c = g2w(GS.SERVO_GX, 0.0, Z_MID[1] + SG_H / 2)
    return c, np.array([SG_W, SG_L, SG_H])


def gear_clear_x(y_lo, y_hi, clearance=0.5):
    """x (he co tay) lon nhat ma chi tiet trong khoang y [y_lo, y_hi] o lop banh rang con cach
    2 banh rang (vong chia R_GEAR) it nhat 'clearance' mm."""
    ys = np.linspace(y_lo, y_hi, 200)
    xs = []
    for s in (-1, 1):
        c = g2w(*pose(0.0, s)["g"], 0.0)
        dy = np.abs(ys - c[1])
        ok = dy < R_GEAR
        xs.append((c[0] - np.sqrt(R_GEAR ** 2 - dy[ok] ** 2)).min() if ok.any() else np.inf)
    return min(xs) - clearance


def _raster_poly(inside_fn, lo, hi, res=0.05, tol=0.05):
    """Duong vien (ccw) cua mien {inside_fn(x, y)} trong hop [lo, hi], luoi res mm."""
    import contourpy
    xs = np.arange(lo[0], hi[0], res); ys = np.arange(lo[1], hi[1], res)
    X, Y = np.meshgrid(xs, ys)
    Z = inside_fn(X, Y).astype(float)
    c = max(contourpy.contour_generator(xs, ys, Z).lines(0.5), key=len)
    if np.allclose(c[0], c[-1]): c = c[:-1]

    def dp(a):                                   # Douglas-Peucker, sai lech <= tol
        if len(a) < 3:
            return a
        d = a[-1] - a[0]; n = np.hypot(*d)
        dist = np.abs(np.cross(d, a - a[0])) / n if n > 1e-12 else np.hypot(*(a - a[0]).T)
        i = int(np.argmax(dist))
        if dist[i] <= tol:
            return np.array([a[0], a[-1]])
        return np.vstack([dp(a[:i + 1])[:-1], dp(a[i:])])
    i0 = int(np.argmax(np.hypot(*(c - c[0]).T)))
    c = np.vstack([dp(c[:i0 + 1])[:-1], dp(np.vstack([c[i0:], c[:1]]))[:-1]])
    return c if _signed_area(c) > 0 else c[::-1]


def tower_xz():
    """Thap ga (mat phang x-z he co tay), be day theo y = vau W cua 009.
    Phan tren lop banh rang: mat truoc = mat sau SG90 (nguoi dung: servo nam ngoai, sat thap).
    Phan trong lop banh rang + de: lui ve sau de khong cham 2 banh rang (khuyet goc duoi-truoc)."""
    c_sv, s_sv = servo_box_w()
    x_front = c_sv[0] - s_sv[0] / 2
    rq = x_front - WQ[0]                              # vau Q vua cham mat sau servo
    z_bt, z_tp = Z_BASE[1] - Z_W, Z_TOP[1] - Z_W       # tu mat tren de toi mat tren gripper
    z_gear_top = Z_MID[1] - Z_W
    x_notch = gear_clear_x(*Y_LUG_W)
    # Bien dang theo 009 goc (thap ga thay 009, cung 2 lo W, Q): phia sau W chi co vau tron; canh tren
    # nhanh W->Q song song duong W-Q, cach OFF_TOP; chan thap bat dau tu x = X_FOOT.
    X_FOOT = 9.0
    hull_pts = hull([circle((0, 0), R_LUG_W, 96), circle(WQ, rq, 96),
                     np.array([[X_FOOT, z_bt], [x_front, z_bt], [x_front, z_tp], [X_FOOT, z_tp]])])
    hp = Path(hull_pts)
    u = WQ / np.hypot(*WQ); nrm = np.array([-u[1], u[0]])

    def inside(X, Y):
        pts = np.c_[X.ravel(), Y.ravel()]
        a = hp.contains_points(pts).reshape(X.shape) & ((X * nrm[0] + Y * nrm[1]) <= OFF_TOP)
        a |= (X ** 2 + Y ** 2 <= R_LUG_W ** 2) | ((X - WQ[0]) ** 2 + (Y - WQ[1]) ** 2 <= rq ** 2)
        return a & ~((X > x_notch) & (Y < z_gear_top)) & (X <= x_front)
    lo = hull_pts.min(0) - 1; hi = hull_pts.max(0) + 1
    outer = _raster_poly(inside, lo, hi)
    hole = np.array([[12.0, z_tp + 2], [WQ[0] - 4.6, z_tp + 2], [WQ[0] - 4.6, z_tp + 12]])   # cua so (anh chup)
    return outer, hole, rq, x_front


def parts():
    """Danh sach thanh phan lang tru cua tung body (he body, mm):
    {body: [(ten, poly, z0, z1, plane, holes), ...]}. Body 'wrist' chi gom phan cua gripper."""
    b2w = lambda poly: np.c_[D_WG + poly[:, 1], Y_C - poly[:, 0]]     # he G (gx, gy) -> co tay (x, y)
    base = b2w(np.array(GS.BASE, float))
    block = b2w(np.array(GS.BLOCK, float))
    tw, hole, rq, _ = tower_xz()
    out = {"wrist": [("de", base, Z_BASE[0] - Z_W, Z_BASE[1] - Z_W, "xy", ()),
                     ("khoi chot", block, Z_TOP[0] - Z_W, Z_TOP[1] - Z_W, "xy", ()),
                     ("thap ga", tw, Y_LUG_W[0], Y_LUG_W[1], "xz", (hole,)),
                     ("vau Q", circle(WQ, rq, 64), Y_LUG_Q[0], Y_LUG_Q[1], "xz", ())]}
    gl = gear_link_outline()
    ul = stadium((0, 0), (L, 0), W_LINK)
    tip_cut = F[1] + W_LINK / 2 + 0.5                 # long ngon chi co o phan mut (de khau lot vao)
    inner = clip_halfplane(FINGER_INNER, np.array([0.0, 1.0]), tip_cut)
    for s, name in ((-1, "l"), (1, "r")):
        out[f"gear_{name}"] = [("banh rang + khau duoi", gl, *Z_MID, "xy", ())]
        out[f"uplink_{name}"] = [("khau tren", ul, *Z_MID, "xy", ())]
        # he ngon (fx, fy) -> body (x = fy, y = -gx); ngon phai = guong cua ngon trai (gx = -fx)
        f2b = (lambda q: np.c_[q[:, 1], -q[:, 0]]) if s < 0 else (lambda q: np.c_[q[:, 1], q[:, 0]])
        fo, fi = f2b(FINGER), f2b(inner)
        out[f"finger_{name}"] = [("ngon tam duoi", fo, *Z_FIN[0], "xy", ()),
                                 ("ngon long", fi, *Z_FIN[1], "xy", ()),
                                 ("ngon tam tren", fo, *Z_FIN[2], "xy", ())]
    return out


def body_origins_w():
    """Goc body (he co tay, mm) o phi = 0, va truc khop (+z hoac -z: chieu duong = mo)."""
    o = {}
    for s, name in ((-1, "l"), (1, "r")):
        p = pose(0.0, s)
        o[f"gear_{name}"] = g2w(*p["g"], 0.0)
        o[f"uplink_{name}"] = g2w(*p["c"], 0.0)
        o[f"finger_{name}"] = g2w(*p["lw"], 0.0)
    axis = {k: (1.0 if k.endswith("_l") else -1.0) for k in o}
    return o, axis


def meshes():
    """{body: trimesh (mm, he body)} — moi body gop cac thanh phan."""
    out = {}
    for body, comps in parts().items():
        ms = [extrude(poly, z0, z1, holes, plane) for _, poly, z0, z1, plane, holes in comps]
        out[body] = trimesh.util.concatenate(ms)
    return out


def mass_props_union(comps, density_kg_m3, pitch=0.25):
    """Khoi luong (kg), tam (mm), ten-xo quan tinh quanh tam (kg mm^2) cua HOP cac lang tru
    (dem diem luoi pitch mm; phan chong lan chi tinh 1 lan)."""
    ms = [extrude(poly, z0, z1, holes, plane) for _, poly, z0, z1, plane, holes in comps]
    lo = np.min([m.bounds[0] for m in ms], 0); hi = np.max([m.bounds[1] for m in ms], 0)
    ax = [np.arange(lo[i] + pitch / 2, hi[i], pitch) for i in range(3)]
    X, Y = np.meshgrid(ax[0], ax[1], indexing="ij")
    occ = np.zeros((len(ax[0]), len(ax[1]), len(ax[2])), bool)
    for _, poly, z0, z1, plane, holes in comps:
        if plane == "xy":
            inside = Path(poly).contains_points(np.c_[X.ravel(), Y.ravel()]).reshape(X.shape)
            for h in holes:
                inside &= ~Path(h).contains_points(np.c_[X.ravel(), Y.ravel()]).reshape(X.shape)
            kz = (ax[2] >= z0) & (ax[2] <= z1)
            occ |= inside[:, :, None] & kz[None, None, :]
        else:
            XZ, ZZ = np.meshgrid(ax[0], ax[2], indexing="ij")
            inside = Path(poly).contains_points(np.c_[XZ.ravel(), ZZ.ravel()]).reshape(XZ.shape)
            for h in holes:
                inside &= ~Path(h).contains_points(np.c_[XZ.ravel(), ZZ.ravel()]).reshape(XZ.shape)
            ky = (ax[1] >= z0) & (ax[1] <= z1)
            occ |= inside[:, None, :] & ky[None, :, None]
    idx = np.argwhere(occ)
    pts = np.c_[ax[0][idx[:, 0]], ax[1][idx[:, 1]], ax[2][idx[:, 2]]]
    vol = len(pts) * pitch ** 3                              # mm^3
    mass = vol * 1e-9 * density_kg_m3
    com = pts.mean(0); d = pts - com
    I = mass * (np.eye(3) * (d ** 2).sum(1).mean() - d.T @ d / len(pts))     # kg mm^2
    return mass, com, I, vol
