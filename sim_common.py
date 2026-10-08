"""sim_common.py — Cac ham dung chung: nap mo hinh, doi lenh servo -> ctrl,
doc goc thuc te (th1, th2, phi) tu trang thai MuJoCo, chay mo phong voi lenh
giu 20 ms (zero-order hold) giong servo that."""
import math, os
import numpy as np
import mujoco
import params as P
import mk2_kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "model", "mk2.xml")


def load_model(gravity=True):
    if not os.path.exists(MODEL):
        raise SystemExit("Chua co model/mk2.xml — chay: python build_model.py --stl-dir <thu muc STL>")
    m = mujoco.MjModel.from_xml_path(MODEL)
    if not gravity:
        m.opt.gravity[:] = 0
    return m, mujoco.MjData(m)


def q_to_ctrl(q):
    """(th1, th2, phi) rad -> ctrl cua 3 actuator (do dai actuator = gear * khop)."""
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
    d.ctrl[:] = ctrl
    for _ in range(int(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)


def run_commands(m, d, ctrl_seq, dt_cmd=P.CMD_PERIOD, tail=0.5, viewer=None, realtime=False):
    """Chay chuoi lenh, moi lenh giu dt_cmd giay. Tra ve dict log (lay mau moi buoc)."""
    import time
    n_sub = int(round(dt_cmd / m.opt.timestep))
    seq = list(ctrl_seq) + [ctrl_seq[-1]] * int(tail / dt_cmd)
    log = {k: [] for k in ("t", "tool", "q", "frc", "viol", "k")}
    t0 = time.time()
    for k, c in enumerate(seq):
        d.ctrl[:] = c
        for _ in range(n_sub):
            mujoco.mj_step(m, d)
        log["t"].append(d.time); log["tool"].append(d.site("tool").xpos * 1000)
        log["q"].append(actual_q(m, d)); log["frc"].append(d.actuator_force.copy())
        log["viol"].append(loop_violation_mm(d)); log["k"].append(min(k, len(ctrl_seq) - 1))
        if viewer is not None:
            viewer.sync()
            if realtime:
                lag = (k + 1) * dt_cmd - (time.time() - t0)
                if lag > 0: time.sleep(lag)
            if not viewer.is_running(): break
    return {k: np.array(v) for k, v in log.items()}
