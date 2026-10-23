# Installation and model build

Step-by-step setup on Ubuntu 20.04 (Python 3.8), from a clean machine to a rebuilt MuJoCo model.
Every step ends with a **Check** block: run it and move on only when it passes. If a step fails, see the
"If it fails" notes or [troubleshooting.md](troubleshooting.md).

Conventions:
- `~` is your home directory.
- From step 3.3 on, every `python` command must run **inside the virtual environment** (prompt shows `(mk2)`).

> The repository already ships the built models (`model/*.xml`, `model/meshes/`). Steps 4–5 are only
> needed if you want to rebuild them, e.g. after editing `params.py`, `gripper.py` or `gripper_shape.py`.

## Contents

- [Verification status](#verification-status)
- [0. Tool stack and version pins](#0-tool-stack-and-version-pins)
- [1. Check the machine](#1-check-the-machine)
- [2. Install](#2-install)
- [3. Get the original STL files](#3-get-the-original-stl-files)
- [4. Build the model from STL](#4-build-the-model-from-stl)
- [5. Assembly check (recommended, run once)](#5-assembly-check-recommended-run-once)

---

## Verification status

**2026-10-16 (real gripper + `--pen` flag).** The whole workflow (model build → snapshots) was re-run from scratch in a
**Python 3.8.20** venv with exactly the versions pinned in `requirements.txt` and the `EBAmk2_*.STL` set (step 3).
All "Expected output" blocks in these docs are real output of that run.
- `cmd_derivatives.png` and `pen_trace.png` in `assets/simulation_results/` are **byte-identical** (md5) to freshly generated images.
- `run_sim.py --pen` run twice in a row gives the same md5 and the same `log_M2.csv` (deterministic).

That run was **not** performed on a real Ubuntu 20.04 machine but on another Linux x86_64 host. Ubuntu 20.04 compatibility is
inferred from the wheel tags (every package is `cp38 manylinux`, needing glibc ≤ 2.28; Ubuntu 20.04 ships glibc 2.31).
Step 1 lets you confirm this on your own machine. OpenGL images were rendered on a virtual display (Xvfb); the viewer
window has not been tested with a real NVIDIA driver.

**2026-10-23 addendum (sorting task; `CTRL_ELBOW_DEG` changed to −62.5°).** The model build, kinematics check, M0/M2
experiment, the first two snapshot commands and the sorting task were re-run in the same environment. Expected outputs were updated.
- `mk2.xml` and `mk2_pen.xml` differ from the previous version only in the `ctrlrange` of `servo_elbow`.
- `run_sim.py --pen` still produces two plots byte-identical to the reference images.
- Only the R_max table of `check_kinematics.py --pen` changed.
- The `--view` viewer of `run_sort.py` has only been tested on a virtual display (Xvfb).

---

## 0. Tool stack and version pins

| Layer | Tool | Used for |
|---|---|---|
| Kinematics + trajectories | Python 3.8, NumPy, Ruckig 0.12.2 | FK/IK, workspace, command q̇/q̈/jerk, trajectory duration, drawing-size selection |
| Dynamics | MuJoCo 3.2.3 | Closed parallelogram loops of the arm (`equality/connect`), gripper gears and parallelograms (`equality/joint`), inertia from STL, torque-saturated servos, backlash, 20 ms command hold, pen-tip/paper contact |

Why exactly these versions:
- Ubuntu 20.04 uses Python 3.8.
- According to PyPI, **mujoco 3.2.3** is the last release with a `cp38` wheel (3.2.4 requires Python ≥ 3.9).
- **ruckig 0.12.2** is the last release with a `cp38` wheel. For Python 3.8, 0.14.0 and 0.15.3 are source-only and pip would have to compile them.
- NumPy 1.24.4, SciPy 1.10.1, Matplotlib 3.7.5 and NetworkX 3.1 are the last releases supporting Python 3.8 (the next release of each requires Python ≥ 3.9).

**Do not upgrade packages individually.**

The gripper model needs no extra library. It only uses the packages above plus `contourpy`, which Matplotlib 3.7.5 installs automatically.

## 1. Check the machine

### 1.1 OS, glibc, Python

```bash
lsb_release -ds            # operating system
uname -m                   # CPU architecture
ldd --version | head -1    # glibc version
python3 --version          # system Python
```

**Check — passes when:**
- Line 1 is `Ubuntu 20.04.x LTS`.
- Line 2 is `x86_64`.
- glibc is **≥ 2.28** (Ubuntu 20.04 has `2.31`).
- Python is `Python 3.8.x` (any x).

If it fails:
- Wrong Python (e.g. 3.10, 3.12): the versions pinned in `requirements.txt` **will not install**. Stop; do not edit `requirements.txt` by guesswork.
- Why glibc is checked: the `glfw` and `pillow` wheels are built to the `manylinux_2_28` standard, i.e. they need glibc ≥ 2.28.

### 1.2 GPU and OpenGL (only needed for the viewer and rendered images)

```bash
sudo apt update
sudo apt install -y mesa-utils           # provides glxinfo
nvidia-smi                               # is the NVIDIA driver working?
glxinfo -B | grep -E "direct rendering|OpenGL renderer"
```

**Check — passes when:**
- `nvidia-smi` prints a table with the GPU name (RTX 3050) and driver version.
- `direct rendering: Yes`.
- `OpenGL renderer` is a real GPU (NVIDIA or Intel), **not `llvmpipe`**.

If it fails:
- `llvmpipe` means rendering on the CPU (software).
- Model build, kinematics check and the M0/M2 experiment (without `--view`) **still run**, since they need no OpenGL; only the `ink_*.png` images will be missing.
- The viewer will be very slow or will not open. Fix the driver before using the viewer.

*Optional (dual-GPU laptop):* if `glxinfo` reports the Intel GPU and you want the viewer on NVIDIA, prefix the command with the two
NVIDIA PRIME Render Offload variables, for example:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo -B | grep "OpenGL renderer"   # -> NVIDIA ...
```

If this prints NVIDIA, you can put the same prefix before the viewer `python ...` commands. The Intel GPU is also sufficient for this model.

## 2. Install

### 2.1 System packages

```bash
sudo apt install -y python3-venv python3-pip unzip git
```

**Check:**

```bash
python3 -m venv --help > /dev/null && echo "venv OK"
git --version && unzip -v | head -1
```

Passes when it prints `venv OK`, the git version and the unzip version.

### 2.2 Get the source

```bash
git clone https://github.com/NithanNguyen/eezybot_mk2_sim.git ~/eezybot_mk2_sim
```

If already cloned, update with `cd ~/eezybot_mk2_sim && git pull`.

**Check:**

```bash
ls ~/eezybot_mk2_sim/requirements.txt ~/eezybot_mk2_sim/build_model.py ~/eezybot_mk2_sim/gripper.py
```

Passes when all three paths are printed without `No such file or directory`.

### 2.3 Dedicated virtual environment

```bash
python3 -m venv ~/venvs/mk2
source ~/venvs/mk2/bin/activate
pip install --upgrade pip
```

**Check:**

```bash
which python      # -> /home/<user>/venvs/mk2/bin/python
python --version  # -> Python 3.8.x
pip --version     # -> pip 25.0.1 ... (python 3.8)
```

Passes when `which python` points into `venvs/mk2`. If it points to `/usr/bin/python3`, the venv is **not activated**: re-run `source`.
(pip 25.0.1 is the last pip supporting Python 3.8; pip selects it automatically.)

**In every new terminal**, run `source ~/venvs/mk2/bin/activate` again and re-check with `which python`.

### 2.4 Install the libraries

```bash
cd ~/eezybot_mk2_sim
pip install -r requirements.txt
```

**Check 1 — consistent dependencies:**

```bash
pip check     # -> No broken requirements found.
```

**Check 2 — correct versions and importable** (paste the whole block):

```bash
python - <<'EOF'
from importlib.metadata import version
want = {"mujoco": "3.2.3", "numpy": "1.24.4", "matplotlib": "3.7.5", "ruckig": "0.12.2",
        "trimesh": "4.4.9", "networkx": "3.1", "scipy": "1.10.1"}
bad = [k for k, v in want.items() if version(k) != v]
for k, v in want.items():
    print(f"{'OK ' if version(k) == v else 'BAD'} {k:10s} {version(k):8s} (need {v})")
import mujoco, ruckig, trimesh, scipy, matplotlib, PIL, glfw, contourpy
print("PASS" if not bad else f"FAIL: wrong versions {bad}")
EOF
```

Expected: 7 lines starting with `OK`, last line `PASS`.

If it fails with `No matching distribution found for mujoco==3.2.3`: Python is not 3.8 or the machine is not x86_64 → back to step 1.1.

## 3. Get the original STL files

`build_model.py` needs the following **13 files**, with names in **exact case** (Linux distinguishes `.STL` from `.stl`):

```
EBAmk2_001_base  002_mainarm  003_varm  004_link135  005_link135angled  006_horarm__
EBAmk2_007_trialink  008_link147_new  010_gearservo  011_gearmast  012_mainbase  013_lower_base  019_drive_cover
```

Files **no longer used**:
- `009_trialinkfront` and the original gripper `014`–`018`: the real gripper replaces this whole block. It is built by `gripper.py` and needs no STL.
- `006_horarm_plate`, `010_gearservo_22DENTI`, `011_gearmast_full`: alternative variants.

**Option A — original source (recommended):** download all files of *EEZYbotARM Mk2* from Thingiverse thing:1454048
(https://www.thingiverse.com/thing:1454048), then unzip:

```bash
mkdir -p ~/mk2_stl
unzip ~/Downloads/<downloaded-zip-file>.zip -d ~/mk2_stl
# If Thingiverse splits the download into several zips, unzip the one containing files/ with the EBAmk2_*.STL files
```

**Option B — GitHub mirror** (when Thingiverse is unavailable). This is a third-party re-upload, not the original source;
this file set produced **exactly** the expected numbers below:

```bash
git clone --depth 1 https://github.com/mchora2/EEZYbotARMMK2 ~/mk2_stl/EEZYbotARMMK2
```

Then **locate the STL directory** (do not guess the path) and store it in `STL_DIR`:

```bash
find ~/mk2_stl -iname "EBAmk2_001_base.stl"
# e.g. prints /home/<user>/mk2_stl/files/EBAmk2_001_base.STL  -> the directory is ~/mk2_stl/files
STL_DIR=<directory printed by find>
echo "$STL_DIR"
```

**Check — all 13 files present:**

```bash
miss=0; for f in 001_base 002_mainarm 003_varm 004_link135 005_link135angled 006_horarm__ \
  007_trialink 008_link147_new 010_gearservo 011_gearmast 012_mainbase 013_lower_base 019_drive_cover; do
  if [ -f "$STL_DIR/EBAmk2_$f.STL" ]; then echo "OK      EBAmk2_$f.STL"; else echo "MISSING EBAmk2_$f.STL"; miss=1; fi
done; [ $miss -eq 0 ] && echo PASS || echo FAIL
```

Passes with 13 `OK` lines and a final `PASS`.

Notes:
- If `find` only finds lowercase `.stl` names, the check reports `MISSING`; rename the files to `.STL` or use option B.
- `STL_DIR` only exists in the current terminal.

## 4. Build the model from STL

```bash
cd ~/eezybot_mk2_sim
python build_model.py --stl-dir "$STL_DIR"
```

Expected output (default parameters in `params.py`):

```
  kiem tra lo C cua EBAmk2_007_trialink: lech 0.01 mm
  kiem tra lo T cua EBAmk2_006_horarm__: lech 0.00 mm
  kiem tra lo O cua 001_base: lech 0.00 mm
  kiem tra lo F cua 001_base: lech 0.00 mm
  gripper: Q - W = (32.60, 28.04) mm (|Q-W| = 43.00); khe 2 mui ngon: dong 0.78 mm, mo het 70.8 mm
  but D10 lui 27 mm tu mut ngon: goc servo kep = 4.29 do; khe 2 mui ngon khi kep but = 6.01 mm
  TCP - W = (132.69, 0.480, 0.50) mm; diem kep but - W = (110.59, 0.480, 0.50) mm
  nap D30 (sort): tam nap - W = (110.00, 0.480, 0.50) mm; goc servo cham nap = 21.27 do; khe nho nhat toi chi tiet khong va cham = 4.99 mm (khau tren trai); mo 40 do: ngon cach nap 8.90 mm
  TCP o home: hinh hoc [279.69   0.48 229.75]  FK [279.69   0.48 229.75]  lech 0.000 mm
  ngoi but o home (--pen): hinh hoc [257.59   0.48 139.75]  FK [257.59   0.48 139.75]  lech 0.002 mm
chi tiet | khoi luong (g) | the tich (cm3) | ghi chu
EBAmk2_001_base | 178.2 | 143.7 | luoi da va lo
...
GR_body | 26.3 | 21.2 | gripper that, hop lang tru (luoi diem 0.25 mm)
sg90 (hop datasheet) | 9.0 | - | vi tri gan dung
...
GR_gear_l | 6.2 | 5.0 | gripper that, hop lang tru (luoi diem 0.25 mm)
GR_finger_l | 8.6 | 6.9 | gripper that, hop lang tru (luoi diem 0.25 mm)
GR_uplink_l | 2.1 | 1.7 | gripper that, hop lang tru (luoi diem 0.25 mm)
...
TONG khoi luong chuyen dong: 588.7 g  (FILL_FACTOR=1.0)
  + but (chi --pen): 15.0 g [GIA-DINH]

Servo: tau_stall=0.922 N m, w0=5.51 rad/s, kp=10.563 N m/rad, b=0.1673 N m s/rad
Da ghi .../eezybot_mk2_sim/model/mk2.xml (mac dinh, khong but), .../eezybot_mk2_sim/model/mk2_pen.xml (--pen) va .../eezybot_mk2_sim/model/mk2_sort.xml (bai phan loai)
```

Meaning of the check lines:
- **`lech` (deviation) lines** are geometric checks: the third hole of each part must land exactly on the joint computed from `params.py`. If the deviation exceeds 1 mm, the script stops.
- **`gripper` line:**
  - `Q - W` is the distance between the two gripper mounting holes (W: end of horizontal arm 006, Q: end of link 008), computed from the arm geometry in the STL.
  - It matches the caliper measurement (horizontal 32.6 mm, vertical 28 mm, straight 43 mm), so the gripper is level, not tilted.
  - Fingertip gap: 0.78 mm closed and 70.8 mm fully open (= 0.78 + 2 × 35 mm link length).
- **`but D10` line:** gripper servo angle at which the inner faces of both fingers just touch the Ø10 pen. Computed from the finger profile and pen position (27 mm back from the fingertip).
- **`TCP - W` / `diem kep but - W` lines** are compared with `params.TCP_W`, `PEN_GRIP_W`, `PEN_GRIP_DEG`. If they differ by more than 0.02 mm or 0.01°, the script stops and prints the correct values to copy into `params.py`.
- **`nap D30 (sort)` line** (sorting task, see [sorting-task.md](sorting-task.md)) is compared with `params.CAP_GRIP_W` and `CAP_GRIP_DEG` the same way:
  - The centre of the Ø30 cap when gripped lies `GRIP_CAP_GY` = 65 mm from the gear-axis midpoint, i.e. 110 mm in front of W.
  - The servo angle at which both fingers just touch the cap is 21.27°.
  - The clearance to parts that have **no** collision geometry in the simulation (upper link, lower link, gears, base) must be > 0, otherwise the script stops.
- **`TCP o home` and `ngoi but o home` lines:** the position computed from geometry must match the analytic FK of `mk2_kinematics.py` within 0.05 mm.
- **`GR_*` masses** are computed on the extruded gripper prisms (0.25 mm point grid, PLA 1.24 g/cm³, 100 % infill). The pen mass is added only to the `--pen` model.

**Check — effective servo command ranges:**

```bash
python -c "import mujoco,numpy as np; m=mujoco.MjModel.from_xml_path('model/mk2.xml'); [print(f'{m.actuator(i).name:15s}', np.round(m.actuator_ctrlrange[i],4), 'rad =', np.round(np.degrees(m.actuator_ctrlrange[i]),1), 'deg') for i in range(m.nu)]"
```

Expected:

```
servo_base      [-1.5708  1.5708] rad = [-90.  90.] deg
servo_shoulder  [-0.9948  0.6981] rad = [-57.  40.] deg
servo_elbow     [-1.0908  0.3054] rad = [-62.5  17.5] deg
servo_gripper   [0.     1.5708] rad = [ 0. 90.] deg
```

The first three lines come from `CTRL_BASE_DEG`, `CTRL_SHOULDER_DEG`, `CTRL_ELBOW_DEG` in `params.py`. The last comes from `GRIP_TRAVEL_DEG`: 0 is closed, 90° fully open.
The elbow lower bound is −62.5°, replacing −57.3° before 2026-10-23. Reason and what to verify on the real arm: [sorting-task.md](sorting-task.md#2-layout).

**Check — MuJoCo loads all three models:**

```bash
for f in model/mk2.xml model/mk2_pen.xml model/mk2_sort.xml; do python -c "import mujoco,sys; m=mujoco.MjModel.from_xml_path(sys.argv[1]); print(sys.argv[1], 'nbody',m.nbody,'njnt',m.njnt,'neq',m.neq,'nu',m.nu,'nmesh',m.nmesh,'nkey',m.nkey)" $f; done
ls model/meshes | wc -l
```

Expected:

```
model/mk2.xml nbody 17 njnt 16 neq 9 nu 4 nmesh 20 nkey 0
model/mk2_pen.xml nbody 17 njnt 16 neq 9 nu 4 nmesh 20 nkey 1
model/mk2_sort.xml nbody 31 njnt 27 neq 9 nu 4 nmesh 37 nkey 0
20
```

Passes when all of the following hold:
- Every `lech` line ≤ 0.05 mm.
- Both `o home` lines deviate < 0.05 mm.
- Total mass is `588.7 g` (if `params.py` is unchanged).
- The three model-load lines print exactly as above.

Meaning of the numbers:
- **17 bodies:** world, 9 arm links (the `wrist` link carries the gripper body), 6 gripper parts, gearservo 010.
  - The 6 gripper parts are: 2 "gear + lower link", 2 upper links, 2 fingers.
- **16 joints:** 9 arm joints, 6 gripper joints, the gearservo joint.
  - The `gripper` joint is the SG90 shaft; the other gripper joints are constrained to it.
- **9 equality constraints:** 3 arm parallelogram loops, the gripper gear pair, 4 gripper parallelogram constraints (2 fingers, 2 upper links), the 010–011 base gear pair.
- **4 servos:** base, shoulder, elbow, gripper.
- `mk2_sort.xml` adds:
  - 14 bodies: rack, bin, reject area, 9 caps and 2 foreign objects.
  - 11 free joints (`freejoint`), one per cap/object.
  - 17 convex meshes declared inline in the XML (16 acrylic pieces, 1 hex standoff), not written to `meshes/`.
- **20 meshes:** 13 original STL and 7 gripper meshes `GR_*`.
- **`nkey 1`:** `mk2_pen.xml` has a keyframe `pen` in which the gripper is opened 4.29° to hold the pen. The scripts load this keyframe automatically.

If `BACKLASH_DEG > 0`, each driven arm joint (base, shoulder, elbow) gets an extra "play" joint, so `njnt` becomes 19 (tested with 0.5°).

**Important:** after every edit to `params.py`, `gripper.py` or `gripper_shape.py`, re-run `build_model.py`.

## 5. Assembly check (recommended, run once)

```bash
python tools/check_assembly.py --stl-dir "$STL_DIR"
```

The script builds signed distance fields (SDF) for 19 parts (12 arm/base STL, 7 gripper parts); the pen body is checked via sample points on its surface. It then checks:
- **[A]** At the home pose, list the part pairs that are close to each other.
- **[B]** At 33 poses covering the joint limits, check that no pair interpenetrates. The first pose is also tested with the base joint at ±45°.
- **[D]** Gripper at the home pose, servo angle 0 → 90° (10° steps): gripper parts do not interpenetrate.

The script runs for a few minutes. Expected output (abridged):

```
[A] Tu the home — cac cap cham nhau (khe < 0.5 mm; am nho la sai so luoi):
    004_link135                  - 006_horarm__                 khe -0.20 mm
    006_horarm__                 - GR_body                      khe -0.20 mm
    ...
    pen                          - GR_finger_l                  khe -0.08 mm
    pen                          - GR_finger_r                  khe -0.07 mm
    ...
    005_link135angled            - 001_base                     khe +0.50 mm
[B] 33 tu the trong gioi han (theta2 (42.5, 125.0), phi (-65.0, 20.0), rel (-145.0, -60.0)):
    khe nho nhat = -0.20 mm (004_link135 - 006_horarm__ tai (42.5, -65.0, -45.0))
PASS: khong chi tiet nao lan vao nhau trong gioi han khop
[D] Gripper, goc servo kep 0..90 do (buoc 10): khe nho nhat = -0.24 mm (GR_gear_l - GR_gear_r tai 10 do)
PASS: gripper khong tu va cham
```

**Passes when** both `PASS` lines appear.
- Negative gaps of about −0.2 mm are mesh error (0.3 mm grid) on faces that rest against each other, not real collisions. Examples: two meshing gears, a finger resting on the pen body.
- The failure threshold is −0.35 mm.

Add `--limits` to print the collision table **[C]** over (θ2, φ) beyond the joint limits. This adds about 10 minutes; it is how
`THETA2_LIM`, `PHI_LIM`, `REL_LIM` were set (see [calibration-and-limitations.md](calibration-and-limitations.md)).
With the real gripper, horizontal arm `006` hits gripper body `GR_body` at φ ≤ −70°, so `PHI_LIM = (-65, 20)` is kept.
