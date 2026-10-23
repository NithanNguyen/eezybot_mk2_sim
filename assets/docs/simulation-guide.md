# Simulation guide

How to view the models, verify the kinematics, run the M0/M2 drawing experiment and take pose snapshots.
Every step lists the command, the expected output and a pass criterion. Run all commands from the repository root,
inside the virtual environment (see [installation.md](installation.md)). The colour-sorting task has its own page:
[sorting-task.md](sorting-task.md).

## Contents

- [1. View the model and drive it by hand](#1-view-the-model-and-drive-it-by-hand)
- [2. Kinematics verification](#2-kinematics-verification)
- [3. M0 vs M2 drawing experiment](#3-m0-vs-m2-drawing-experiment)
- [4. Pose snapshots for the report](#4-pose-snapshots-for-the-report)

---

## 1. View the model and drive it by hand

Requires a working OpenGL renderer (not `llvmpipe`, see [installation.md §1.2](installation.md#12-gpu-and-opengl-only-needed-for-the-viewer-and-rendered-images)).

### 1.1 Default model (pick-and-place, no pen)

```bash
python -m mujoco.viewer --mjcf=model/mk2.xml
```

**Visual check — passes when:**
1. The window opens and shows the arm with the gripper at its end. No pen, no paper. No errors in the terminal.
   - Gripper colours: body dark grey, "gear + lower link" teal, upper links blue, fingers light grey, SG90 a blue box.
2. At the initial pose (all sliders = 0): main arm vertical, horizontal arm level, gripper closed and pointing forward.
3. Open the right panel → **Control** section (shortcut `Alt`+`C`) and drag `servo_base`, `servo_shoulder`, `servo_elbow`. The arm follows and **the gripper stays level** thanks to the parallelograms.
4. Drag `servo_gripper` from 0 to 1.571:
   - The two gears rotate in opposite directions.
   - The two fingers separate symmetrically and **translate**: they stay parallel and do not rotate.
   - When fully open, the lower links are perpendicular to the arm.

Slider units are radians.
- The three arm servos are limited by `CTRL_*_DEG` (see [installation.md §4](installation.md#4-build-the-model-from-stl)). `servo_base` is the **servo shaft** angle; through the 2:1 gearbox the base joint turns half as much.
- `servo_gripper`: 0 is closed (lower links straight), 1.571 rad (90°) fully open.

### 1.2 Pen model (drawing)

```bash
python -m mujoco.viewer --mjcf=model/mk2_pen.xml
```

**Passes when:** a white A4 sheet appears in front of the base and a blue pen stands vertically between the fingers.

Initial pose:
- The viewer starts with every slider at 0, so the fingers may close through the pen body.
- To set the correct pen-holding pose: left panel → **Simulation** → **Load key** (key 0 = `pen`). `servo_gripper` is then 0.0748 rad (4.29°).
- The pen is rigidly attached to the wrist, so it does not fall when the gripper opens. Grip force is not simulated (see [calibration-and-limitations.md](calibration-and-limitations.md#model-limitations)).

### 1.3 Mouse and keyboard

From the MuJoCo "simulate" documentation:
- `Space`: run/pause.
- Double-click a part: select it.
- `Ctrl` + drag **left mouse**: rotate the selected part (applies torque).
- `Ctrl` + drag **right mouse**: push the part in the vertical plane (applies force); add `Shift` for the horizontal plane.

Close the window to exit. This viewer shows **no ink trace**; the ink is drawn by `run_sim.py --pen` (section 3).

## 2. Kinematics verification

Run twice: for the gripper TCP (default) and for the pen tip.

```bash
python check_kinematics.py          # ~6 s on the test machine
python check_kinematics.py --pen    # ~21 s (part [4] sweeps the workspace)
```

The gripper TCP is the midpoint of the two fingertip pads when closed, 5 mm back from the fingertip (`GRIP_TCP_BACK`).

Expected output of the first command:

```
Dau cong tac: gripper (132.69, 0.48, 0.5) mm so voi W. CANH BAO: PEN_LENGTH, TOOL_MASS_G, GRIP_CENTER_Y va gioi han khop trong params.py la gia tri tam [GIA-DINH]. Hay do tren tay may that roi sua lai.
[1] FK(IK(p)) tren 5000 diem: max |FK(IK(p))-p| = 1.30e-13 mm, max |IK(FK(q))-q| = 1.14e-12 mrad
[2] MuJoCo (vong kin, g=0) vs FK giai tich (TCP gripper), 40 tu the: max sai lech = 0.011 mm, max vi pham rang buoc vong = 0.0000 mm
[3] Do vong do trong luc (servo P huu han, params.SERVO_ERR_SAT_DEG):
    q=(0, 90, 0): lech goc (deg) = [ 0.   -0.02 -0.63], lech TCP gripper = 1.72 mm, ...
    q=(0, 60, -30): lech goc (deg) = [ 0.   -0.72 -0.56], lech TCP gripper = 2.28 mm, ...
    q=(0, 45, -55): lech goc (deg) = [ 0.   -1.01 -0.39], lech TCP gripper = 2.46 mm, ...
    q=(0, 110, 0): lech goc (deg) = [ 0.    0.47 -0.63], lech TCP gripper = 2.39 mm, ...
[4] Bang hinh ve kha thi chi danh cho but: chay  python check_kinematics.py --pen
Da luu .../results/workspace_rz_gripper.png
```

Expected output of the `--pen` command:

```
Dau cong tac: pen (110.59, 0.48, -89.5) mm so voi W. CANH BAO: ...
[1] FK(IK(p)) tren 5000 diem: max |FK(IK(p))-p| = 1.39e-13 mm, max |IK(FK(q))-q| = 1.20e-12 mrad
[2] MuJoCo (vong kin, g=0) vs FK giai tich (ngoi but), 40 tu the: max sai lech = 0.013 mm, max vi pham rang buoc vong = 0.0000 mm
[3] Do vong do trong luc (servo P huu han, params.SERVO_ERR_SAT_DEG):
    q=(0, 90, 0): lech goc (deg) = [ 0.   -0.02 -0.76], lech ngoi but = 2.17 mm, ...
    q=(0, 60, -30): lech goc (deg) = [ 0.   -0.78 -0.67], lech ngoi but = 2.59 mm, ...
    q=(0, 45, -55): lech goc (deg) = [ 0.   -1.09 -0.46], lech ngoi but = 2.71 mm, ...
    q=(0, 110, 0): lech goc (deg) = [ 0.    0.51 -0.76], lech ngoi but = 2.95 mm, ...
[4] Hinh ve lon nhat (sao + tron ban kinh R) theo do cao mat giay z (mm), cach gioi han >= 3 deg:
    z_giay =   1.0 mm: tam x = 275 mm, R_max =  34.1 mm   <- mat giay (params.PAPER_THICKNESS)
    z_giay =  10.0 mm: tam x = 265 mm, R_max =  51.3 mm
    z_giay =  20.0 mm: tam x = 265 mm, R_max =  62.9 mm
    ...
    z_giay =  70.0 mm: tam x = 260 mm, R_max =  82.1 mm
    ...
    z_giay = 100.0 mm: tam x = 260 mm, R_max =  71.4 mm
Da luu .../results/workspace_rz_pen.png
```

The `CANH BAO` (warning) line is a normal reminder, not an error. Parts [2] and [3] disable pen–paper contact to compare pure kinematics.

**Check — passes when:**

| Line | Criterion | Meaning if it fails |
|---|---|---|
| [1] | both errors < 1e-9 | IK is no longer the inverse of FK (`mk2_kinematics.py` edited by mistake?) |
| [2] | deviation < 0.05 mm **and** loop violation ≈ 0 | The parallelogram mapping layer or `model/*.xml` does not match `params.py` → rebuild the model |
| [3] | sag about 1.5–2.5 mm (TCP) and 2.0–3.0 mm (pen tip) | Much larger: `SERVO_ERR_SAT_DEG` or masses were changed |
| [4] (`--pen`) | the `<- mat giay` line has `R_max` > 20 mm, and `results/workspace_rz_pen.png` exists | The default `R = 20` of section 3 would leave the workspace |

What each line shows:
1. IK is an exact inverse of FK. FK/IK include the 0.48 mm lateral offset of the tool from the arm plane (`GRIP_CENTER_Y`).
2. The closed-loop physical model gives the same tool position as the analytic formula, which validates the **parallelogram mapping layer**.
3. The servos are finite-gain P controllers, so the tool sags a few mm under gravity. This is a real effect that a pure kinematic model cannot see.
   - The third test pose is (0, 45, −55), not −60: when this step was written, φ = −60° was outside the old `CTRL_ELBOW_DEG` range (−57.3°). The pose was kept so results stay comparable.
4. A table for **choosing R and the drawing centre**. The marked line is the real paper surface (1 mm thick, on the table). Choose R well below R_max to keep a margin.

**Why R_max on the paper surface is 34.1 mm (old clamp: 56.4 mm).** Two causes. Table [4] was re-run while changing one factor at a time:

| Case | R_max at z = 1 mm |
|---|---|
| Old clamp, no servo command range (old README) | 56.4 mm |
| Old clamp + `CTRL_ELBOW_DEG` | 34.9 mm |
| Real gripper, `PHI_LIM` only | 42.2 mm |
| Real gripper + `CTRL_ELBOW_DEG` lower bound −57.3° (2026-10-16) | 22.9 mm |
| Real gripper + `CTRL_ELBOW_DEG` lower bound −62.5° (current, since 2026-10-23) | **34.1 mm** (centre x = 275 mm) |

- **The real gripper holds the pen higher.** The pen grip point is 0.5 mm above W; the old clamp was 8 mm below W. The tip is therefore 8.5 mm higher relative to the wrist, so the horizontal arm must tilt down further to reach the paper.
- **The elbow servo command range stops at φ ≥ −62.5°** (`CTRL_ELBOW_DEG`; −57.3° before 2026-10-23, commit "update - Operating Angle"). This is still tighter than the collision limit `PHI_LIM` = −65°. IK checks this range too, so out-of-range points raise `IKError` instead of being silently clipped.
- For a larger drawing:
  - Raise the paper: z = 10 mm gives R_max 51.3 mm; z = 20 mm gives 62.9 mm.
  - Or let the pen protrude further: k mm more is equivalent to raising the paper by k mm.
  - Or widen `CTRL_ELBOW_DEG` if the real servo can reach it (never beyond `PHI_LIM` = −65°).

## 3. M0 vs M2 drawing experiment

Drawing **needs the pen**, so `--pen` is required:

```bash
python run_sim.py --pen     # default: R = 20 mm, centre (290, 0) = centre of the A4 sheet, tip at z = 1 mm (paper surface)
```

Without `--pen` the script stops immediately with `run_sim.py la bai VE hinh, can but. ... python run_sim.py --pen`.

Sequence of each drawing (each method draws on a fresh sheet):
1. Move the pen to 15 mm vertically above the start point (`PEN_LIFT`).
2. Lower the pen straight down in 1 s, dwell 0.3 s.
3. Draw one continuous stroke (M0 or M2).
4. Dwell 0.3 s, lift the pen in 0.5 s.

**Ink is recorded only when MuJoCo reports pen–paper contact**, i.e. tip–paper gap ≤ `INK_GAP` = 0.05 mm. The stroke is 1 mm wide.
Metrics are measured on the **ink trace**, as on real paper (definitions: [methods.md](methods.md#4-drawing-metrics)).

Expected output with default parameters (~7 s on the test machine):

```
Dau cong tac: pen (110.59, 0.48, -89.5) mm so voi W. CANH BAO: ...
Duong di L = 271.0 mm (ly thuyet 13.55R = 271.0); T(M2) = 3.60 s; v(M0) = L/T = 75.3 mm/s
[M0] toc do servo lon nhat = [0.56 0.38 0.33] rad/s (MG996R khong tai 5.51 rad/s @4.8V) -> OK
[M2] toc do servo lon nhat = [1.11 0.74 0.64] rad/s (MG996R khong tai 5.51 rad/s @4.8V) -> OK
[M0] max vi pham rang buoc vong = 0.055 mm, max |mo-men servo| = [0.083 0.219 0.125] Nm
[M0] ngoi cham giay 100.0% thoi gian ve; so net muc = 1; luc ep ngoi TB/max = 3.09/4.05 N
[M2] max vi pham rang buoc vong = 0.054 mm, max |mo-men servo| = [0.107 0.219 0.125] Nm
[M2] ngoi cham giay 100.0% thoi gian ve; so net muc = 1; luc ep ngoi TB/max = 3.07/5.86 N

Chi so du doan tu mo phong (cung T), do tren VET MUC: 5 mui sao / duong tron / 10 canh:
PP    vot lo TB  vot lo max  sai lech dinh TB  do tron  lech thang max  co muc %   (mm)
M0        -0.81       -0.10              0.85     1.02            1.19     100.0
M2        -0.44        0.13              0.50     0.55            1.17     100.0
  vot lo < 0: net ve cat goc, khong cham giao diem 2 canh keo dai
```

**Check:**

```bash
ls results/cmd_derivatives.png results/pen_trace.png results/log_M0.csv results/log_M2.csv results/ink_M0.png results/ink_M2.png
wc -l results/log_M0.csv results/log_M2.csv          # -> 311 lines each (1 header + 310 samples)
md5sum results/cmd_derivatives.png assets/simulation_results/cmd_derivatives.png
md5sum results/pen_trace.png       assets/simulation_results/pen_trace.png
```

Passes when:
- Both servo-speed lines end with `-> OK`.
- Loop-constraint violation < 0.1 mm.
- **The tip touches the paper 100 % of the drawing time, with exactly 1 ink stroke** (continuous, unbroken).
- All 6 files exist, each CSV has 311 lines.
- The metrics table matches the one above (default parameters).

The two `md5sum` commands are the strictest comparison: if each pair gives the same hash, your image is byte-identical to the reference.
- If the hashes differ but the metrics table matches, compare the two images visually. Small pixel differences can come from a different CPU or fonts and are not an error.
- `ink_*.png` is rendered with OpenGL, so pixels depend on the GPU; compare only visually with `assets/simulation_results/ink_M2.png`.

Other options:

```bash
python run_sim.py --pen --view       # watch the robot draw, ink appears progressively (needs a working viewer)
python run_sim.py --pen --R 15 --vmax 150 --amax 1500 --jmax 30000
```

With `--view`:
- The camera points at the sheet. Each method starts on a blank sheet: M0 draws in red, M2 in blue.
- The viewer runs in "passive" mode, so `Space` (pause) has no effect (per the MuJoCo documentation).
- The script closes the window when done.

With `--R 15`, the expected metrics are `M0 -0.77 -0.38 0.80 0.84 1.18 100.0` and `M2 -0.39 0.03 0.42 0.56 1.16 100.0`.

Other R values:
- `--R 30` still runs, but the drawing enters the region less than 3° from a joint limit (table [4], section 2).
- `--R 35` raises `IKError: th2=42.5 ngoai (42.5, 125.0)`.

Files in `results/`:

| File | Content |
|---|---|
| `cmd_derivatives.png` | **Evidence 1**: q̇, q̈, jerk of the M0 and M2 commands (drawing phase) overlaid |
| `pen_trace.png` | **Ink trace** of M0 and M2 (only where the tip touched the paper) vs the ideal shape, with a zoom on a star tip |
| `ink_M0.png`, `ink_M2.png` | 3D render of the arm and the inked sheet after drawing |
| `log_M0.csv`, `log_M2.csv` | 20 ms commands, phase (0 lower, 1 draw, 2 lift), servo angles, tip position, actual angles, servo torques, ink (0/1), tip normal force (N) |

## 4. Pose snapshots for the report

Requires a working OpenGL renderer.

```bash
python snapshot.py --pen --xyz 290 0 20 --out results/pose.png    # put the PEN TIP at (290, 0, 20) mm via IK
python snapshot.py --xyz 260 0 100 --out results/pose_gripper.png # put the gripper TCP at (260, 0, 100) mm via IK
python snapshot.py --servo 90 90 90 --out results/home.png        # set the three servo angles directly (degrees)
python snapshot.py --grip 90 --servo 90 90 90 --out results/open.png   # gripper opened 90°
```

Expected output of the first three commands:

```
servo (deg): [89.81 58.18 47.42] | FK ngoi but (mm): [290.   0.  20.]
MuJoCo ngoi but (mm): [290.55  -0.    17.78]
Da luu results/pose.png
servo (deg): [89.79 70.43 33.94] | FK TCP gripper (mm): [260.   0. 100.]
MuJoCo TCP gripper (mm): [260.28  -0.    99.  ]
Da luu results/pose_gripper.png
servo (deg): [90. 90. 90.] | FK TCP gripper (mm): [279.69   0.48 229.75]
MuJoCo TCP gripper (mm): [279.72   0.48 228.03]
Da luu results/home.png
```

**Check — passes when** `results/pose.png` exists and resembles `assets/simulation_results/pose_xyz_290_0_20.png`.
- The 1–2.3 mm difference between FK and MuJoCo is gravity sag, the same effect as line [3] in section 2, not an error.
- The base servo angle is 89.8°, not 90°, because IK compensates the 0.48 mm lateral tool offset.
- `--grip` is the gripper servo angle (degrees). Without `--grip` the gripper is closed, or holds the pen at 4.29° with `--pen`.
