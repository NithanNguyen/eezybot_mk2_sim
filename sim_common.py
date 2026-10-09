"""sim_common.py — Cac ham dung chung: nap mo hinh, doi lenh servo -> ctrl,
doc goc thuc te (th1, th2, phi) tu trang thai MuJoCo, chay mo phong voi lenh
giu 20 ms (zero-order hold) giong servo that, va ghi vet muc khi ngoi but cham giay."""
import math, os
import numpy as np
import mujoco
import params as P
import mk2_kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "model", "mk2.xml")
INK_RGBA = {"M0": (0.85, 0.1, 0.1, 1.0), "M2": (0.05, 0.2, 0.85, 1.0)}


def load_model(gravity=True, contact=True):
    if not os.path.exists(MODEL):
        raise SystemExit("Chua co model/mk2.xml — chay: python build_model.py --stl-dir <thu muc STL>")
    m = mujoco.MjModel.from_xml_path(MODEL)
    if not gravity:
        m.opt.gravity[:] = 0
    if not contact:
        m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    return m, mujoco.MjData(m)


def q_to_ctrl(q):
    """(th1, th2, phi) rad -> ctrl cua 3 servo tay (do dai actuator = gear * khop)."""
    th1, th2, phi = q
    return np.array([P.BASE_GEAR_RATIO * th1, th2 - math.pi / 2, phi])


def servo_to_ctrl(sv_deg):
    return q_to_ctrl(K.from_servo(sv_deg))


def _jq(m, d, name):
    j = m.joint(name)
    v = d.qpos[j.qposadr[0]]
    try:
        v += d.qpos[m.joint(name + "_play").qposadr[0]]
    except KeyError:
        pass
    return float(v)


def actual_q(m, d):
    """Goc thuc (th1, th2, phi) tu hinh hoc mo phong (da gom do ro neu co)."""
    th1 = _jq(m, d, "base_yaw")
    sh = _jq(m, d, "shoulder")
    th2 = math.pi / 2 + sh
    phi = sh + float(d.qpos[m.joint("elbow_rel").qposadr[0]])
    return np.array([th1, th2, phi])


def loop_violation_mm(d):
    pairs = [("E_main", "E_tri"), ("T_hor", "T_l004"), ("Q_wrist", "Q_l008")]
    return 1000 * max(np.linalg.norm(d.site(a).xpos - d.site(b).xpos) for a, b in pairs)


def settle(m, d, ctrl, seconds=2.0):
    d.ctrl[:3] = ctrl
    for _ in range(int(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)


# ----------------------------------------------------------------------------- vet muc
class Ink:
    """Theo doi tiep xuc ngoi but - giay. Chi ghi muc khi MuJoCo bao co tiep xuc giua
    geom 'pen_tip' va geom 'paper' (pen_tip co margin = gap = INK_GAP: tiep xuc duoc bao khi
    khe <= INK_GAP, luc chi xuat hien khi lun vao giay). Vet muc = chuoi cac net (x, y, k)."""

    def __init__(self, m, spacing=0.2):
        self.m = m
        self.g_tip = m.geom("pen_tip").id
        self.g_paper = m.geom("paper").id
        self.r_tip = float(m.geom_size[self.g_tip][0])
        self.spacing = spacing
        self.strokes = []          # list of list[(x, y, k)]: mm, chi so lenh k
        self.down = False
        self._f6 = np.zeros(6)

    def contact(self, d):
        """(co tiep xuc?, luc phap tuyen N)."""
        for i in range(d.ncon):
            c = d.contact[i]
            if {c.geom1, c.geom2} == {self.g_tip, self.g_paper}:
                mujoco.mj_contactForce(self.m, d, i, self._f6)
                return True, float(self._f6[0])
        return False, 0.0

    def update(self, d, k=0):
        touch, fn = self.contact(d)
        if touch:
            p = d.geom_xpos[self.g_tip][:2] * 1000
            if not self.down:
                self.strokes.append([(p[0], p[1], k)]); self.down = True
            elif np.hypot(p[0] - self.strokes[-1][-1][0], p[1] - self.strokes[-1][-1][1]) >= self.spacing:
                self.strokes[-1].append((p[0], p[1], k))
        else:
            self.down = False
        return touch, fn

    def clear(self):
        self.strokes = []; self.down = False


class InkDrawer:
    """Ve vet muc (con nhong rong INK_WIDTH) vao mot MjvScene: viewer.user_scn hoac renderer.scene."""

    def __init__(self, rgba):
        self.rgba = np.array(rgba, np.float32)
        self.done = {}             # stroke index -> so doan da ve

    def draw(self, scn, ink, append=True):
        """Them cac doan muc moi vao scn (append=True: chi doan chua ve; False: ve lai tat ca)."""
        z = (P.PAPER_THICKNESS + 0.05) / 1000
        w = P.INK_WIDTH / 2 / 1000
        if not append: self.done = {}
        for si, st in enumerate(ink.strokes):
            for k in range(self.done.get(si, 0), len(st) - 1):
                if scn.ngeom >= scn.maxgeom: return
                g = scn.geoms[scn.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).ravel(), self.rgba)
                a = np.array([st[k][0] / 1000, st[k][1] / 1000, z])
                b = np.array([st[k + 1][0] / 1000, st[k + 1][1] / 1000, z])
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, w, a, b)
                scn.ngeom += 1
                self.done[si] = k + 1


def run_commands(m, d, ctrl_seq, dt_cmd=P.CMD_PERIOD, tail=0.5, viewer=None, realtime=False, ink=None, drawer=None):
    """Chay chuoi lenh, moi lenh giu dt_cmd giay. Tra ve dict log (lay mau cuoi moi chu ky lenh).
    Neu co ink: kiem tra tiep xuc ngoi-giay o moi buoc tich phan va ghi vet muc."""
    import time
    n_sub = int(round(dt_cmd / m.opt.timestep))
    seq = list(ctrl_seq) + [ctrl_seq[-1]] * int(tail / dt_cmd)
    log = {k: [] for k in ("t", "tool", "q", "frc", "viol", "k", "ink", "fn")}
    t0 = time.time()
    for k, c in enumerate(seq):
        d.ctrl[:3] = c
        touched = False; fmax = 0.0
        for _ in range(n_sub):
            mujoco.mj_step(m, d)
            if ink is not None:
                t_, f_ = ink.update(d, min(k, len(ctrl_seq) - 1))
                touched |= t_; fmax = max(fmax, f_)
        log["t"].append(d.time); log["tool"].append(d.site("tool").xpos * 1000)
        log["q"].append(actual_q(m, d)); log["frc"].append(d.actuator_force[:3].copy())
        log["viol"].append(loop_violation_mm(d)); log["k"].append(min(k, len(ctrl_seq) - 1))
        log["ink"].append(touched); log["fn"].append(fmax)
        if viewer is not None:
            if ink is not None and drawer is not None:
                with viewer.lock():
                    drawer.draw(viewer.user_scn, ink)
            viewer.sync()
            if realtime:
                lag = (k + 1) * dt_cmd - (time.time() - t0)
                if lag > 0: time.sleep(lag)
            if not viewer.is_running(): break
    return {k: np.array(v) for k, v in log.items()}
