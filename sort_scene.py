"""sort_scene.py — Cac doi tuong cua bai phan loai mau nap chai (model/mk2_sort.xml).

Sinh cac doan MJCF chen vao mo hinh mac dinh boi build_model.py:
  * ke tram cam bien: mat mica 60 x 60 x 3 co lo D23 (ghep tu 16 manh loi), 4 chan luc giac cao 40 mm;
  * module GY-33 25 x 25 x 12 sat mat duoi mica, tam ngay duoi lo (khong mo hinh gia do);
  * hop dich 1 khoi 3 ngan (thanh/day 3 mm), truc dai huong xuyen tam; vung vut bo ve tren ban;
  * nap chai rong dat up (mat tren D30 x 1 mm + thanh 16 tam phang), khoi luong 2.5 g, khop tu do;
  * tam kep cua 2 ngon (hop mong doc theo bien dang mat trong ngon) de ngon kep nap bang tiep xuc that.

Nhom va cham (contype / conaffinity):
  NAP  4 / 4|8|16 : nap cham nap, moi truong va ngon
  ENV  8 / 0      : ban, ke, cam bien, hop
  NGON 16 / 8     : tam kep cua ngon cham nap va moi truong (de lo ra neu ngon cham ke)
Cac chi tiet khac cua tay may/gripper KHONG co va cham (giong mo hinh mac dinh); khe giua nap va chung
duoc kiem tra hinh hoc boi tools/check_sort_layout.py.
Don vi trong file nay: mm (doi sang m khi ghi XML).
"""
import math
import numpy as np
import params as P
import gripper as GR

MM = 1e-3
CAP_T, CAP_C = 4, 4 | 8 | 16
ENV_T, ENV_C = 8, 0
FIN_T, FIN_C = 16, 8
FOREIGN_CAPS = 2                     # vat la (xanh duong) de thu nhanh "vut bo" (run_sort.py --foreign)
CAP_COL_N, CAP_COL_R = 12, 2.0       # va cham nap: 12 vien nang R2 mm quanh chu vi (khe giua 2 vien 2.8 mm)
CAP_RGBA = {"green": "0.10 0.62 0.25 1", "red": "0.85 0.10 0.10 1", "white": "0.96 0.96 0.96 1",
            "other": "0.15 0.30 0.85 1"}


def f(v):
    return " ".join(f"{x:.6g}" for x in np.ravel(v))


def contact_attr(contype, conaffinity, condim=3):
    s = P.CONTACT_SOLREF; i = P.CONTACT_SOLIMP
    return (f'contype="{contype}" conaffinity="{conaffinity}" condim="{condim}" '
            f'friction="{P.CONTACT_FRICTION} 0.005 0.0001" solref="{s[0]} {s[1]}" solimp="{i[0]} {i[1]} {i[2]}"')


def zrot_quat(a):
    return f"{math.cos(a / 2):.9g} 0 0 {math.sin(a / 2):.9g}"


# ------------------------------------------------------------------ vi tri trong the gioi (mm)
def polar(r, deg):
    a = math.radians(deg)
    return np.array([r * math.cos(a), r * math.sin(a)])


def bin_frame():
    """(tam hop xy, goc rad): truc u cua hop huong xuyen tam ra xa robot."""
    return polar(P.BIN_CENTER_R, P.BIN_ANGLE_DEG), math.radians(P.BIN_ANGLE_DEG)


def bin_pitch():
    return P.BIN_INNER[0] + P.BIN_WALL


def bin_targets():
    """{mau: (x, y)} tam long tung ngan."""
    c, a = bin_frame(); u = np.array([math.cos(a), math.sin(a)])
    return {col: c + (i - 1) * bin_pitch() * u for i, col in enumerate(P.BIN_ORDER)}


def bin_outer():
    return 3 * P.BIN_INNER[0] + 4 * P.BIN_WALL, P.BIN_INNER[1] + 2 * P.BIN_WALL, P.BIN_HEIGHT


def reject_target():
    return polar(P.REJECT_R, P.REJECT_ANGLE_DEG)


def station():
    return np.array([P.SHELF_CENTER[0], P.SHELF_CENTER[1], P.SHELF_HEIGHT])


def queue_positions(n):
    """Cho nap cho: 2 hang ngoai tam voi (x = CAP_QUEUE_X), tam nap (mm)."""
    out = []
    for k in range(n):
        row, col = divmod(k, 6)
        out.append(np.array([P.CAP_QUEUE_X + 45.0 * row, -112.5 + 45.0 * col, P.CAP_HEIGHT / 2]))
    return out


# ------------------------------------------------------------------ hinh hoc
def shelf_pieces(n=16):
    """Mat mica = n manh loi: hinh nem (tam = tam lo) giao hinh vuong, bo phan trong day cung cua lo.
    Lo la da giac deu ngoai tiep duong tron D23 (moi diem cua lo cach tam >= 11.5 mm). He ke, mm."""
    hs = P.SHELF_SIZE / 2
    rh = P.SHELF_HOLE_D / 2 / math.cos(math.pi / n)
    corners = [np.array(c) for c in ((hs, hs), (-hs, hs), (-hs, -hs), (hs, -hs))]

    def ray_square(a):
        d = np.array([math.cos(a), math.sin(a)])
        t = min(hs / abs(d[0]) if abs(d[0]) > 1e-12 else 1e9, hs / abs(d[1]) if abs(d[1]) > 1e-12 else 1e9)
        return t * d

    pieces = []
    for i in range(n):
        a0 = 2 * math.pi * i / n + math.pi / n; a1 = a0 + 2 * math.pi / n
        h0 = rh * np.array([math.cos(a0), math.sin(a0)]); h1 = rh * np.array([math.cos(a1), math.sin(a1)])
        poly = [h0, ray_square(a0)]
        for c in corners:
            ac = math.atan2(c[1], c[0]) % (2 * math.pi)
            lo, hi = a0 % (2 * math.pi), a1 % (2 * math.pi)
            inside = (lo < ac < hi) if lo < hi else (ac > lo or ac < hi)
            if inside:
                poly.append(c)
        poly += [ray_square(a1), h1]
        pieces.append(np.array(poly))
    return pieces


def prism_vertices(poly, z0, z1):
    return np.vstack([np.c_[poly, np.full(len(poly), z0)], np.c_[poly, np.full(len(poly), z1)]])


def hexagon(r):
    return np.array([[r * math.cos(k * math.pi / 3), r * math.sin(k * math.pi / 3)] for k in range(6)])


def cap_inertial():
    """Khoi luong, tam, quan tinh (he nap: goc tai tam hinh hoc, z len, mat kin o tren) — nap = dia D x t
    + vanh tru (r_ngoai, r_ngoai - t). Khoi luong chia theo the tich, tong = CAP_MASS_G."""
    R = P.CAP_DIAMETER / 2; t = P.CAP_WALL; H = P.CAP_HEIGHT; ri = R - t; hw = H - t
    v_top = math.pi * R * R * t; v_wall = math.pi * (R * R - ri * ri) * hw
    m = P.CAP_MASS_G / 1000.0; m_top = m * v_top / (v_top + v_wall); m_wall = m - m_top
    z_top = H / 2 - t / 2; z_wall = -H / 2 + hw / 2
    zc = (m_top * z_top + m_wall * z_wall) / m
    Izz = m_top * R * R / 2 + m_wall * (R * R + ri * ri) / 2
    Ixx = (m_top * (3 * R * R + t * t) / 12 + m_top * (z_top - zc) ** 2 +
           m_wall * (3 * (R * R + ri * ri) + hw * hw) / 12 + m_wall * (z_wall - zc) ** 2)
    return m, np.array([0, 0, zc]) * MM, np.array([Ixx, Ixx, Izz]) * MM * MM


def finger_pads(side):
    """Hop mong (day 1 mm, lun vao trong vat lieu) doc moi canh mat kep cua ngon, theo 3 lop:
    tam duoi va tam tren theo FINGER, lop long theo FINGER_INNER (chi phan mut, fy >= tip_cut).
    Tra ve list (tam he body mm, goc quanh z rad, nua kich thuoc mm)."""
    tip_cut = GR.F[1] + GR.W_LINK / 2 + 0.5
    inner = GR.clip_halfplane(GR.FINGER_INNER, np.array([0.0, 1.0]), tip_cut)
    layers = [(GR.FINGER, GR.Z_FIN[0]), (inner, GR.Z_FIN[1]), (GR.FINGER, GR.Z_FIN[2])]
    t = 1.0
    out = []
    for poly, (z0, z1) in layers:
        poly = np.asarray(poly, float)
        if GR._signed_area(poly) < 0: poly = poly[::-1]
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            e = b - a; L = np.hypot(*e)
            if L < 0.2: continue
            n = np.array([e[1], -e[0]]) / L                  # phap tuyen ngoai (da giac nguoc chieu kim dong ho)
            mid = (a + b) / 2
            if n[0] < 0.3 or mid[1] < 10.0:                   # chi mat kep (huong +fx), bo vung goc ngon
                continue
            c = mid - n * t / 2
            # he ngon (fx, fy) -> body: trai (fy, -fx), phai (fy, +fx)
            sgn = -1.0 if side == "l" else 1.0
            cb = np.array([c[1], sgn * c[0], (z0 + z1) / 2])
            eb = np.array([e[1], sgn * e[0]])
            out.append((cb, math.atan2(eb[1], eb[0]), np.array([L / 2 + 0.1, t / 2, (z1 - z0) / 2])))
    return out


# ------------------------------------------------------------------ MJCF
def assets_xml():
    lines = []
    for i, poly in enumerate(shelf_pieces()):
        v = prism_vertices(poly, -P.SHELF_THICKNESS / 2, P.SHELF_THICKNESS / 2) * MM
        lines.append(f'    <mesh name="shelf_p{i}" vertex="{f(v)}"/>')
    v = prism_vertices(hexagon(P.SHELF_LEG_ACROSS / 2), -P.SHELF_LEG_HEIGHT / 2, P.SHELF_LEG_HEIGHT / 2) * MM
    lines.append(f'    <mesh name="shelf_leg" vertex="{f(v)}"/>')
    return "\n".join(lines)


def table_xml():
    return (f'<geom name="table" type="plane" size="0.6 0.6 0.01" material="grid" {contact_attr(ENV_T, ENV_C)}/>')


def world_xml():
    out = []
    env = contact_attr(ENV_T, ENV_C)
    vis = 'contype="0" conaffinity="0" group="1"'
    # ---- ke + cam bien
    sx, sy = P.SHELF_CENTER
    hs = P.SHELF_SIZE / 2; leg_r = P.SHELF_LEG_ACROSS / 2; leg_ry = leg_r * math.sin(math.pi / 3)
    out.append(f'<body name="shelf" pos="{f(np.array([sx, sy, 0]) * MM)}">')
    zt = P.SHELF_LEG_HEIGHT + P.SHELF_THICKNESS / 2
    for i in range(len(shelf_pieces())):
        out.append(f'  <geom name="shelf_p{i}" type="mesh" mesh="shelf_p{i}" pos="0 0 {zt*MM:.6g}" '
                   f'rgba="0.75 0.88 0.95 0.35" group="1" {env}/>')
    for k, (ux, uy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        p = np.array([ux * (hs - leg_r), uy * (hs - leg_ry), P.SHELF_LEG_HEIGHT / 2])
        out.append(f'  <geom name="shelf_leg{k}" type="mesh" mesh="shelf_leg" pos="{f(p * MM)}" rgba="0.8 0.8 0.82 1" group="1" {env}/>')
    s = np.array(P.SENSOR_SIZE)
    zs = P.SHELF_LEG_HEIGHT - s[2] / 2                        # mat tren module = mat duoi mica
    out.append(f'  <geom name="gy33" type="box" pos="0 0 {zs*MM:.6g}" size="{f(s / 2 * MM)}" rgba="0.12 0.2 0.5 1" group="1" {env}/>')
    out.append(f'  <site name="station" pos="0 0 {P.SHELF_HEIGHT*MM:.6g}" size="0.002" rgba="1 1 0 1"/>')
    out.append(f'  <site name="sensor" pos="0 0 {P.SHELF_LEG_HEIGHT*MM:.6g}" size="0.002" rgba="1 1 1 1"/>')
    out.append('</body>')
    # ---- hop dich 3 ngan (he hop: u xuyen tam, v tiep tuyen)
    c, a = bin_frame()
    Lu, Lv, H = bin_outer(); w = P.BIN_WALL; hu, hv = Lu / 2, Lv / 2; hz = (H - w) / 2; zc = w + hz
    out.append(f'<body name="bin" pos="{f(np.array([c[0], c[1], 0]) * MM)}" quat="{zrot_quat(a)}">')
    bc = "0.98 0.74 0.35 1"
    parts = [("bin_floor", (0, 0, w / 2), (hu, hv, w / 2)),
             ("bin_side_l", (0, hv - w / 2, zc), (hu, w / 2, hz)), ("bin_side_r", (0, -hv + w / 2, zc), (hu, w / 2, hz)),
             ("bin_end_near", (-hu + w / 2, 0, zc), (w / 2, hv - w, hz)), ("bin_end_far", (hu - w / 2, 0, zc), (w / 2, hv - w, hz))]
    for k in (-1, 1):
        parts.append((f"bin_div{'n' if k < 0 else 'f'}", (k * (P.BIN_INNER[0] / 2 + w / 2), 0, zc), (w / 2, hv - w, hz)))
    for nm, p, hsz in parts:
        out.append(f'  <geom name="{nm}" type="box" pos="{f(np.array(p) * MM)}" size="{f(np.array(hsz) * MM)}" rgba="{bc}" group="1" {env}/>')
    for i, col in enumerate(P.BIN_ORDER):
        u = (i - 1) * bin_pitch()
        rgba = CAP_RGBA[col].rsplit(" ", 1)[0] + " 0.45"
        out.append(f'  <geom name="bin_mark_{col}" type="box" pos="{f(np.array([u, 0, w + 0.1]) * MM)}" '
                   f'size="{f(np.array([P.BIN_INNER[0] / 2 - 4, P.BIN_INNER[1] / 2 - 4, 0.1]) * MM)}" rgba="{rgba}" {vis}/>')
        out.append(f'  <site name="bin_{col}" pos="{f(np.array([u, 0, H]) * MM)}" size="0.002" rgba="0 0 0 1"/>')
    out.append('</body>')
    # ---- vung vut bo (chi ve, nam tren mat ban)
    rc = reject_target(); ar = math.radians(P.REJECT_ANGLE_DEG); h = P.REJECT_SIZE / 2
    out.append(f'<body name="reject" pos="{f(np.array([rc[0], rc[1], 0]) * MM)}" quat="{zrot_quat(ar)}">')
    out.append(f'  <geom name="reject_fill" type="box" pos="0 0 {0.1*MM:.6g}" size="{f(np.array([h, h, 0.1]) * MM)}" rgba="1 0.3 0.3 0.25" {vis}/>')
    for k, (p, hsz) in enumerate((((0, h - 1, 0.25), (h, 1, 0.25)), ((0, -h + 1, 0.25), (h, 1, 0.25)),
                                   ((h - 1, 0, 0.25), (1, h, 0.25)), ((-h + 1, 0, 0.25), (1, h, 0.25)))):
        out.append(f'  <geom name="reject_edge{k}" type="box" pos="{f(np.array(p) * MM)}" size="{f(np.array(hsz) * MM)}" rgba="0.9 0.1 0.1 1" {vis}/>')
    out.append('  <site name="reject" pos="0 0 0" size="0.002" rgba="0 0 0 1"/>')
    out.append('</body>')
    # ---- nap chai
    m, com, I = cap_inertial()
    R = P.CAP_DIAMETER / 2; t = P.CAP_WALL; Hc = P.CAP_HEIGHT; nseg = P.CAP_WALL_SEGMENTS
    a_o = R * math.cos(math.pi / nseg)                        # da giac noi tiep D: mat ngoai thanh
    half_len = R * math.sin(math.pi / nseg)
    caps = [(f"cap_{col}_{k}", col) for col in P.CAP_COLORS for k in range(P.CAP_PER_COLOR)]
    caps += [(f"cap_other_{k}", "other") for k in range(FOREIGN_CAPS)]
    # Va cham (an, group 3): mat ngoai nap = vong CAP_COL_N vien nang dung (ban kinh CAP_COL_R, mat ngoai cham
    # duong tron D, dau tron cham mat tren/duoi nap) + 1 tam hop mong o mat tren. Chi dung cap va cham giai tich
    # cua MuJoCo (vien nang-vien nang/hop/mat phang, hop-hop) — tru-tru di qua bo va cham loi tong quat va lam nap
    # nay manh khi roi chong len nhau (da thu, xem README). Hinh ve: mat tren + thanh 16 tam (rong), khong va cham.
    # Quan tinh: nap rong that (cap_inertial).
    capc = contact_attr(CAP_T, CAP_C, condim=4)
    rc = CAP_COL_R; rr = R - rc; n_c = CAP_COL_N
    s_top = rr * math.cos(math.pi / 4)                        # hop vuong noi tiep vong tam vien nang
    for (name, col), q in zip(caps, queue_positions(len(caps))):
        out.append(f'<body name="{name}" pos="{f(q * MM)}">')
        out.append('  <freejoint/>')
        out.append(f'  <inertial pos="{f(com)}" mass="{m:.6g}" diaginertia="{f(I)}"/>')
        crgba = CAP_RGBA[col].rsplit(" ", 1)[0] + " 0.3"
        for k in range(n_c):
            ang = 2 * math.pi * k / n_c
            x, y = rr * math.cos(ang), rr * math.sin(ang)
            out.append(f'  <geom name="{name}_c{k}" type="capsule" fromto="{f(np.array([x, y, -Hc / 2 + rc, x, y, Hc / 2 - rc]) * MM)}" '
                       f'size="{rc*MM:.6g}" group="3" rgba="{crgba}" {capc}/>')
        out.append(f'  <geom name="{name}_lid" type="box" pos="0 0 {(Hc / 2 - t / 2)*MM:.6g}" '
                   f'size="{f(np.array([s_top, s_top, t / 2]) * MM)}" group="3" rgba="{crgba}" {capc}/>')
        out.append(f'  <geom name="{name}_top" type="cylinder" pos="0 0 {(Hc / 2 - t / 2)*MM:.6g}" '
                   f'size="{R*MM:.6g} {t/2*MM:.6g}" rgba="{CAP_RGBA[col]}" {vis}/>')
        hz = (Hc - t) / 2; zw = -Hc / 2 + hz
        for k in range(nseg):
            ang = 2 * math.pi * k / nseg
            p = np.array([(a_o - t / 2) * math.cos(ang), (a_o - t / 2) * math.sin(ang), zw])
            out.append(f'  <geom name="{name}_w{k}" type="box" pos="{f(p * MM)}" quat="{zrot_quat(ang)}" '
                       f'size="{f(np.array([t / 2, half_len, hz]) * MM)}" rgba="{CAP_RGBA[col]}" {vis}/>')
        out.append('</body>')
    return "\n    ".join(out)


def pads_xml(side):
    fc = contact_attr(FIN_T, FIN_C)
    return "\n".join(f'                <geom name="pad_{side}{k}" type="box" pos="{f(c * MM)}" quat="{zrot_quat(a)}" '
                     f'size="{f(h * MM)}" rgba="0.9 0.5 0.1 0.6" group="3" {fc}/>'
                     for k, (c, a, h) in enumerate(finger_pads(side)))


def cap_names():
    return ([f"cap_{col}_{k}" for col in P.CAP_COLORS for k in range(P.CAP_PER_COLOR)] +
            [f"cap_other_{k}" for k in range(FOREIGN_CAPS)])
