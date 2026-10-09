"""trajectories.py — Hinh thu nghiem (ngoi sao 5 canh + duong tron) va hai
phuong phap sinh quy dao theo de cuong:
  M0: noi suy tuyen tinh, van toc tiep tuyen khong doi v = L/T
  M2: double-S gioi han v, a, j theo do dai cung, dung tai moi dinh
      (dung Ruckig lam chuan tham chieu toi uu thoi gian — de cuong [2])
Ca hai lay mau moi CMD_PERIOD (20 ms) va goi IK tai moi mau.
"""
import math
import numpy as np
import params as P
import mk2_kinematics as K


def star_circle_path(cx, cy, R, z):
    """Danh sach doan: ('line', p0, p1) x10 roi ('circle', center, R, goc_bat_dau).
    Mui sao tren cung huong +x (ra xa robot)."""
    r_in = R * math.sin(math.radians(18)) / math.sin(math.radians(126))   # = 0.381966 R
    verts = []
    for k in range(5):
        a_tip = math.radians(0 + 72 * k)
        a_in = a_tip + math.radians(36)
        verts.append((cx + R * math.cos(a_tip), cy + R * math.sin(a_tip)))
        verts.append((cx + r_in * math.cos(a_in), cy + r_in * math.sin(a_in)))
    segs = []
    for i in range(10):
        p0 = np.array([*verts[i], z]); p1 = np.array([*verts[(i + 1) % 10], z])
        segs.append(("line", p0, p1))
    segs.append(("circle", np.array([cx, cy, z]), R, 0.0))
    return segs, verts


def seg_length(s):
    return np.linalg.norm(s[2] - s[1]) if s[0] == "line" else 2 * math.pi * s[2]


def seg_point(s, u):
    """Diem tai do dai cung u (mm) tren doan s."""
    if s[0] == "line":
        p0, p1 = s[1], s[2]
        return p0 + (p1 - p0) * (u / np.linalg.norm(p1 - p0))
    c, R, a0 = s[1], s[2], s[3]
    a = a0 + u / R
    return c + np.array([R * math.cos(a), R * math.sin(a), 0.0])


def ruckig_1d(length, vmax, amax, jmax, dt):
    """Bien dang double-S nghi->nghi (Ruckig), lay mau moi dt. Tra ve s[k]."""
    from ruckig import InputParameter, OutputParameter, Result, Ruckig
    otg = Ruckig(1, dt)
    inp, out = InputParameter(1), OutputParameter(1)
    inp.current_position = [0.0]; inp.current_velocity = [0.0]; inp.current_acceleration = [0.0]
    inp.target_position = [length]; inp.target_velocity = [0.0]; inp.target_acceleration = [0.0]
    inp.max_velocity = [vmax]; inp.max_acceleration = [amax]; inp.max_jerk = [jmax]
    s = []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)
        s.append(out.new_position[0])
        out.pass_to_input(inp)
    if res != Result.Finished:
        raise RuntimeError(f"Ruckig loi: {res}")
    return np.array(s)


def m2_path(segs, vmax, amax, jmax, dt=P.CMD_PERIOD):
    """M2: moi doan double-S rieng, dung tai dinh. Tren cung tron: v^2/R <= amax."""
    pts, seg_id = [], []
    for i, s in enumerate(segs):
        v = vmax if s[0] == "line" else min(vmax, math.sqrt(amax * s[2]))
        for u in ruckig_1d(seg_length(s), v, amax, jmax, dt):
            pts.append(seg_point(s, u)); seg_id.append(i)
    return np.array(pts), np.array(seg_id)


def m0_path(segs, T, dt=P.CMD_PERIOD):
    """M0: van toc khong doi v = L/T tren toan bo duong, doi huong tuc thoi."""
    L = [seg_length(s) for s in segs]
    cum = np.r_[0, np.cumsum(L)]
    n = int(round(T / dt))
    pts, seg_id = [], []
    for k in range(1, n + 1):
        u = cum[-1] * k / n
        i = min(np.searchsorted(cum, u, side="right") - 1, len(segs) - 1)
        pts.append(seg_point(segs[i], u - cum[i])); seg_id.append(i)
    return np.array(pts), np.array(seg_id)


def to_joint(pts):
    """IK cho tung mau. Bao loi neu co diem ngoai gioi han."""
    return np.array([K.ik(p) for p in pts])


def derivs(x, dt=P.CMD_PERIOD):
    """Sai phan lui cua tin hieu lenh (giong cach log tu ESP32)."""
    v = np.diff(x, axis=0, prepend=x[:1]) / dt
    a = np.diff(v, axis=0, prepend=v[:1]) / dt
    j = np.diff(a, axis=0, prepend=a[:1]) / dt
    return v, a, j


def line_path(p0, p1, T, dt=P.CMD_PERIOD):
    """Doan thang p0 -> p1 trong T giay, bien dang van toc hinh sin (bat dau/ket thuc v = 0).
    Dung cho ha but xuong giay va nhac but len (khong tinh vao chi so M0/M2)."""
    n = max(1, int(round(T / dt)))
    s = (1 - np.cos(np.pi * np.arange(1, n + 1) / n)) / 2
    return np.asarray(p0, float) + np.outer(s, np.asarray(p1, float) - np.asarray(p0, float))
