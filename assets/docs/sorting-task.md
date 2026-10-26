# Bottle-cap colour-sorting task (`run_sort.py`)

The pick-and-place task of the project runs on `model/mk2_sort.xml`. This model is built together with the other two
by `build_model.py` + `sort_scene.py` ([installation.md §4](installation.md#4-build-the-model-from-stl)).

## Contents

- [1. Objects](#1-objects)
- [2. Layout](#2-layout)
- [3. Cycle (per cap)](#3-cycle-per-cap)
- [4. Running and checking](#4-running-and-checking)
- [5. Output files](#5-output-files)
- [6. Task-specific limitations](#6-task-specific-limitations)

---

## 1. Objects

Dimensions are the project owner's requirements (2026-10-23) unless noted otherwise.

| Object | MuJoCo model |
|---|---|
| Sensor station rack | Clear acrylic plate 60 × 60 × 3 mm with a central Ø23 hole, top face at z = 43 mm. Four regular hexagonal standoffs (5 mm across corners), 40 mm tall, flush with the four corners. The plate is assembled from 16 convex pieces around the hole |
| GY-33 module | Box 25 × 25 × 12 mm, centred under the hole, top face touching the underside of the acrylic (z = 40 mm). Mount not modelled |
| Bottle caps | 3 green + 3 red + 3 white. Hollow cylinder Ø30 × 13 mm, 1 mm wall, 2.5 g, placed **upside down**. Plus 2 blue "foreign objects", used only with `--foreign` |
| Target bin | One block with 3 compartments, 3 mm walls and floor. Outside 177 × 66 × 50 mm, each compartment 55 × 60 mm inside. Dimensions chosen by the simulation `[CHON]` |
| Reject area | Flat 80 × 80 mm area drawn on the table `[CHON]` |
| Gripper | Adds **colliding pads** along the clamping faces of both fingers, following the three thickness layers of the finger (`sort_scene.finger_pads`). Other arm and gripper parts still have no collision geometry |

## 2. Layout

Frame: origin on the base rotation axis at table level, x forward, y left, z up. The rack faces the arm, the reject area is on the left, the bin on the right.
Values tagged `[TINH]` are recomputed by `python tools/check_sort_layout.py`.

| Object | Position | Rationale |
|---|---|---|
| Rack | Centre (279; 0) | Gripping needs the "cap" point (cap centre) at z = 59 mm. At this height the arm reaches radii 274.5…283.5 mm; 279 is mid-band |
| 3-compartment bin | Long axis radial, at −30°, centre r = 266 mm. Compartment 3 (white) r = 208, compartment 2 (green) r = 266, compartment 1 (red, farthest) r = 324 | Release at z = 116 mm; the reach band at this height is 193…338.5 mm, leaving ~14 mm at each end. Radial placement means the base turns to a single angle (limit ±45°) |
| Reject area | Centre r = 285 mm, at +30° | Release at z = 66 mm; reach band 267…294.5 mm |

**Why the grip point is at z = 59 mm and `CTRL_ELBOW_DEG` = −62.5°.**
- An upside-down cap on the rack occupies z = 43…56 mm. The finger is 15 mm thick; the "cap" point is at mid-thickness.
- At z = 59 mm the lower finger pad (3.3 mm thick) spans z = 51.5…54.8 mm, fully on the cap body; the upper pad is above the cap.
- With the old bound of −57.3°, the lowest reachable "cap" point is 62.3 mm, where the lower pad overlaps the cap by only ~1.2 mm.
- The project owner chose to widen the bound to −62.5°, still 2.5° from the STL collision limit (`PHI_LIM` = −65°).
- **Must be verified on the real arm:** tilt the horizontal arm about 62.5° below horizontal (main arm at about 43.5°) and confirm there is no jamming or collision.

## 3. Cycle (per cap)

1. Arm at Home (servos 90/90/90), gripper open 40°. After `CAP_PLACE_DELAY_S` = 1 s, a "person" places one cap on the station. Cap order is random, set by `--seed`.
2. The ESP32 reads channel C every `POLL_MS` = 500 ms (firmware `src/robot/sorter.h`).
   - When |C − C_empty| ≥ C_margin (cap present), wait `settle_ms` = 300 ms (≥ 200 ms as required).
   - Then average 5 RGBC readings, 120 ms apart. Total measurement time about 0.78 s.
3. Classify with `color_sim.classify()`, a copy of `colormath::classify()` (`lib/ColorClassifier/ColorMath.h`), computed in float32 like the ESP32.
   - The class is the nearest class centre in the (r, g) plane.
   - d > d_max → reject, printing `# ERR UNKNOWN_COLOR ...` like the firmware.
   - |C − C_empty| < C_margin → no cap, keep waiting.
4. Move to 40 mm above the grip point, descend vertically, grip, lift vertically.
   - The grip command is 15°, below the cap-contact angle 21.27° by more than `SERVO_ERR_SAT_DEG`, so the SG90 clamps at saturated torque.
5. Move to the point where **the cap bottom is 50 mm above the top of the bin** ("cap" point z = 50 + 50 + 16 = 116 mm) and open.
   - For the reject area, the cap bottom is 50 mm above the table (z = 66 mm). The arm first goes above the area at z = 99 mm, then descends vertically.
6. Return Home and wait for the next cap.

Trajectories (command every 20 ms → IK → servos):
- **Between stations:** joint-synchronised double-S (Ruckig, all three joints finish together). Limits 0.536 rad/s, 5.36 rad/s², 107 rad/s³, converted from 150 mm/s, 1500 mm/s², 30000 mm/s³ (firmware `include/arm_config.h`) at a 280 mm radius.
- **Descend and lift:** vertical straight line, double-S along arc length with 150 / 1500 / 30000.

**Sensor in simulation.** MuJoCo does not simulate the TCS34725 optics, so each measurement **replays one real reading** from `data/color/test.csv` with the label of the object currently on the station.
- Caps and foreign objects: sampled without replacement.
- Empty station: sampled with replacement.
- `data/color/model.json` and `test.csv` are copied unchanged from the `eezybot_mk2_robot` repository (commit `a172b69`, calibration 2026-10-23).

## 4. Running and checking

**Check 1 — classifier matches the firmware:**

```bash
python color_sim.py
```

Expected: the matrix matches `reports/color/eval_test.json` in the firmware repository (99/100; one red sample on line 24 is "unknown").

```
  green  [20  0  0  0  0]
  red    [ 0 19  0  1  0]
  white  [ 0  0 20  0  0]
  empty  [ 0  0  0  0 20]
  other  [ 0  0  0 20  0]
Dung 99/100 = 99.0%
```

**Check 2 — run the cycle:**

```bash
python run_sort.py              # 9 caps, seed 0; ~1 min 45 s on a 2-core test machine (78 s simulated time)
python run_sort.py --view       # live view; at most real time (slower machines run slower)
python run_sort.py --foreign 2  # add 2 foreign objects -> must go to the reject area with "# ERR UNKNOWN_COLOR"
```

Expected output (`python run_sort.py`, abridged):

```
Tu the (diem 'cap' = tam nap khi kep, mm) -> goc servo (do), du tru toi gioi han gan nhat:
  tram: diem kep         p = [279.   0.  59.]  servo = [89.8  43.5  28.91]  du tru = 1.00 do
  tram: phia tren        p = [279.   0.  99.]  servo = [89.8  57.94 41.46]  du tru = 13.96 do
  ngan 3 (white): tha    p = [ 180.1 -104.   116. ]  servo = [29.74 87.97 39.35]  du tru = 6.39 do
  ngan 2 (green): tha    p = [ 230.4 -133.   116. ]  servo = [29.79 67.52 45.25]  du tru = 14.90 do
  ngan 1 (red): tha      p = [ 280.6 -162.   116. ]  servo = [29.83 48.09 57.39]  du tru = 5.59 do
  vut bo: phia tren      p = [246.8 142.5  99. ]  servo = [149.81  56.17  42.78]  du tru = 13.67 do
  vut bo: tha            p = [246.8 142.5  66. ]  servo = [149.81  44.79  32.59]  du tru = 2.29 do
Thu tu dat nap (seed 0): red, red, green, white, red, white, white, green, green
[t=   3.28 s] CLS,RED,RED,0.001499,0.461412,0.274220,281.000,167.000,161.000,621.000,0
[t=   3.28 s] # SORT RED (lan 1)  [nap 1: cap_red_1, dong test.csv 31]
           gap: z diem kep (thuc) = 53.5 mm, kep 20.30 do, luc kep 5.38 N, nhac nap 40.3 mm, truot max 0.46 mm; day nap luc tha 99.1 mm; nap nam o: red -> DUNG dich
...
[t=  77.62 s] Tram trong, het nap -> dung.

Toc do servo lon nhat = [1.07 0.97 0.86] rad/s (MG996R khong tai 5.51 rad/s @4.8V) -> OK
Khe ho nho nhat tren quy dao LENH (FK; diem thap nhat = day nap khi mang nap, mep duoi ngon khi kep rong):
  Home -> tren tram (kep rong)           mat ban 91.5 mm, ke + nap 35.5 mm
  tram -> ngan 1 (mang nap)              mat ban 83.0 mm, ke 40.0 mm, hop dich 44.7 mm
  ve Home (kep rong)                     mat ban 108.5 mm, hop dich 58.5 mm, ke 109.3 mm
  tram -> ngan 2 (mang nap)              mat ban 83.0 mm, ke 40.0 mm, hop dich 44.3 mm
  tram -> ngan 3 (mang nap)              mat ban 83.0 mm, ke 40.0 mm, hop dich 47.7 mm
So chu ky lenh co tam kep cham ke/hop/ban: 0
Thoi gian 1 chu ky (dat nap -> tay ve Home): TB 7.35 s, min 7.22, max 7.50
Thoi gian tu luc dat nap toi luc ra quyet dinh: TB 0.78 s
Ma tran: mau that (hang) x noi nap nam sau chu trinh (cot: ngan 1 do, ngan 2 xanh, ngan 3 trang, vut bo, ngoai)
  green  [0 3 0 0 0]
  red    [3 0 0 0 0]
  white  [0 0 3 0 0]
Nap toi dung dich da chon (co hoc gap-tha): 9/9; nap toi dung noi theo mau that (toan chu trinh): 9/9
```

`--seed 1`, `--seed 2` and `--seed 3` were also tested: all give 9/9. `--foreign 2` gives 11/11: the 2 foreign objects produce two `# ERR UNKNOWN_COLOR nearest=WHITE dist=0.08..` lines and end up in the reject area.

Passes when:
- The last line shows `9/9` on both sides.
- `So chu ky lenh co tam kep cham ke/hop/ban: 0`: the fingers never touch the rack, bin or table.
- Every `truot max` (max slip) value is < 1 mm: caps do not slip in the gripper.
- Servo speed shows `-> OK`.

**Layout check** (no MuJoCo needed): `python tools/check_sort_layout.py` reprints every `[TINH]` value:
- cap-contact angle, gap when open;
- which finger layer holds the cap body;
- reach band versus height, for the old and new elbow bounds;
- servo angles and margin to the limits at each pose.

## 5. Output files

Files in `results/sort/`:

| File | Content |
|---|---|
| `summary.txt` | The full output above, including `CLS,...` / `# SORT ...` / `# ERR ...` lines matching the firmware Serial output |
| `caps.csv` | Per cap: true colour, `test.csv` line used, R/G/B/C, r/g, d, label, target, final location, actual grip z, grip angle and force, slip, timestamps |
| `log.csv` | 20 ms commands: phase, 3 servo commands, actual angles, gripper command and angle, "cap" point and cap position, finger-environment contact flag |
| `sort_matrix.png` | Matrix of true colour × final cap location |
| `sort_timeline.png` | Commands of the 3 arm servos and the gripper servo over time |
| `scene_end_top.png`, `scene_end_persp.png` | Final scene renders (need OpenGL) |

## 6. Task-specific limitations

State these in the report.
- **Sag while gripping.** The commanded grip point is at z = 59 mm, but MuJoCo gives an actual z of about 53.5 mm (≈ 5.5 mm sag).
  - Cause: extended pose, finite-gain P servos with `SERVO_ERR_SAT_DEG` = 5° `[GIA-DINH]`.
  - The lower finger edge is then only about 3 mm above the acrylic.
  - Measure the actual z at this pose on the real arm. If the finger touches the acrylic, increase `CAP_GRASP_Z` (up to about 60 mm so the lower pad still holds the cap).
- **Friction coefficient** `CONTACT_FRICTION` = 0.4 is assumed, not measured. The simulated grip force is about 5.4 N (both fingers) while the cap weighs only 0.025 N, so grasp results are insensitive to this value.
- **Cap collision shape** is 12 R2 capsules around the rim + 1 box on the top face (the visual is still a hollow cap).
  - The first version used a single cylinder. When 3 caps fell into the same compartment, cylinder–cylinder contact bounced caps out: 45/54 drops landed in the right compartment, and the simulation was sometimes unstable.
  - With capsules, the same test gives 54/54.
  - The cap keeps the exact mass and inertia of a hollow cylinder.
- The model sets `jacobian="dense"`: with 11 free caps (nv = 82), the `auto` mode of MuJoCo 3.2.3 switches to a sparse Jacobian and stops with `mj_makeConstraint: constraint Jacobian mis-allocation`.
- **The person placing caps** is simulated by instantly placing the cap at rest, exactly at the hole centre. Each `test.csv` row is already a 5-reading average, so the channel-C presence reading is less noisy than a single real reading.
- **Arm collision checking.** Only the fingers have collision geometry.
  - Clearance between the cap and the gripper links, gears and base is checked geometrically (≥ 5 mm, `nap D30` line in [installation.md §4](installation.md#4-build-the-model-from-stl)).
  - Clearance between the lowest point (cap bottom when carrying, lower finger edge when empty) and the rack/bin is computed on the commanded trajectory. The arm body and servos are not checked against objects.
