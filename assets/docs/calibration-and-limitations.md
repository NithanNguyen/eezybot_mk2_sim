# Calibration and model limitations

## Contents

- [Calibrating against the real robot](#calibrating-against-the-real-robot)
- [Re-generating the gripper profile](#re-generating-the-gripper-profile)
- [Checks after every parameter change](#checks-after-every-parameter-change)
- [Model limitations](#model-limitations)

---

## Calibrating against the real robot

This is the most important step for realistic results. Source tags in `params.py`:
- `[GIA-DINH]`: assumed placeholder value.
- `[DO-THAT]`: measured on the real robot with a caliper.
- `[ANH]`: taken from photos.
- `[DO-STL]`: computed from STL; should still be re-measured on the real arm.

Measure in this order, edit `params.py`, then re-run `build_model.py`:

| # | Parameter | How to measure |
|---|---|---|
| 1 | `GRIP_CENTER_Y` | **Currently assumed 0.48 mm** (gripper centreline coincides with the old clamp centre). At home (servos 90/90/90), looking from above, measure the lateral offset (perpendicular to the arm) between the fingertip midpoint and the base rotation axis. Positive means to the left when looking forward along the arm |
| 2 | `PEN_GRIP_TO_TIP`, `PEN_FROM_TIP`, `PEN_LENGTH`, `TOOL_MASS_G` | Distance from mid-finger thickness to the pen tip (currently 90 mm), pen axis position relative to the fingertip (27 mm), pen length; weigh the pen |
| 3 | `GRIP_*` (gripper dimensions) | Already measured with a caliper. Change only if the gripper changes. Afterwards `build_model.py` stops and prints new values of `TCP_W`, `PEN_GRIP_W`, `PEN_GRIP_DEG` to copy into `params.py` |
| 4 | `H_SHOULDER` (via `LOWER_BASE_Z`) | Caliper: from the table to the shoulder axis. The 94.25 mm value is derived from STL, assuming the gearmast rests on the centre face of 013 (Ø6 balls leave a ~0.6 mm gap) |
| 5 | `SERVO_OFFSET`, `SERVO_SIGN` | Command 90/90/90 and check that the main arm is vertical, the horizontal arm level and the base facing straight ahead; record offsets and rotation directions |
| 6 | `THETA2_LIM`, `PHI_LIM`, `REL_LIM`, `THETA1_LIM` | Currently from the collision check (`tools/check_assembly.py --limits`), ≥ 2.5° from the collision boundary. On the real arm: turn each servo slowly until it hits a mechanical stop, then back off 3–5° |
| 7 | `CTRL_BASE_DEG`, `CTRL_SHOULDER_DEG`, `CTRL_ELBOW_DEG` | Real servo command ranges. IK rejects any point needing a command outside them. The `CTRL_ELBOW_DEG` lower bound (−62.5°) sets the lowest cap grip point ([sorting-task.md](sorting-task.md#2-layout)) and R_max on paper ([simulation-guide.md §2](simulation-guide.md#2-kinematics-verification)). **Verify on the real arm first** |
| 7b | `CAP_GRASP_Z`, `CONTACT_FRICTION` | Sorting task: measure the actual z of the finger centre at the cap-grip pose; the cap–finger friction coefficient is not measured yet |
| 8 | `MEASURED_MASS_G` | Weigh each printed part and fill in the dict. The default assumes 100 % infill (upper bound) |
| 9 | `SERVO_ERR_SAT_DEG` | Command a 30° step and film at 240 fps (or read the servo's internal potentiometer, per the Adafruit reference [12] of the proposal); tune until the simulated response matches the measurement |
| 10 | `BACKLASH_DEG` | Hold the servos fixed, gently wiggle the arm end and measure the play at the tool; divide by the lever arm to get the angular backlash |
| 11 | `BASE_GEAR_RATIO` | Count teeth: gearmast 50 / gearservo 25 = 2.0 (the `22DENTI` variant: 50/22) |
| 12 | `PEN_TIP_FRICTION`, `INK_GAP` | Tip–paper friction and the largest gap that still leaves ink; affect straightness error and stroke breaks |

## Re-generating the gripper profile

The finger, base and pin-block profiles are in `gripper_shape.py`, generated from photos by:

```bash
python tools/gripper_from_photos.py --closed "<closed, top view>.png" \
    --open "<open, top view>.png" --bottom "<open, bottom view>.png"
```

The six gripper photos are stored in `assets/images/`. The screw-centre pixel coordinates hard-coded in the script were read
from the original photos; any other photo (including a re-encoded or resized copy) requires re-reading them.

## Checks after every parameter change

Run in this order:

```bash
python build_model.py --stl-dir "$STL_DIR"   # "lech" lines must stay <= 0.05 mm; both "o home" lines geometry ~ FK
python check_kinematics.py                    # [1] and [2] must still pass
python check_kinematics.py --pen              # re-read the R_max table
python run_sim.py --pen                       # reduce --R on IKError
python tools/check_assembly.py --stl-dir "$STL_DIR"   # if gripper dimensions changed
```

After calibration, the expected outputs in the other docs **will change**; that is normal.
Criteria [1] and [2] of the kinematics check do not depend on parameters and must always pass.

To validate the model against reality: run the same trajectory on the real arm, then compare the drawn sheet with `pen_trace.png` and compare the three metrics.

## Model limitations

State these in the report.

General:
- Printed parts are **perfectly rigid bodies**: PLA elasticity is not modelled, so link-bending vibration does not appear.
- Servos are modelled as torque-saturated P controllers plus a DC-motor torque–speed slope (set through `damping`).
  - The real internal controller of the MG996R/SG90 is not published.
  - Dead band and pulse resolution are not modelled.
- **Collisions:**
  - The pen model has only tip–paper contact.
  - The sorting model has contact between the finger pads, caps and objects ([sorting-task.md](sorting-task.md)).
  - Link-to-link collisions are checked offline (`tools/check_assembly.py`) and excluded via joint limits.
- Servos (3 × MG996R, SG90) are not in the STL. They are drawn as datasheet-sized boxes on the correct rotation axes; their axial position is approximate.
- Screws, threaded rods, balls and 606zz bearings are not included in the mass.

Gripper:
- **Shapes are extruded prisms** (2D profile extruded through the confirmed thickness layers). Fillets, screw heads and the servo horn are not drawn.
  - Profiles come from photos. The similarity fit between model screw positions and screw centres in the 3 photos has RMS errors of 1.16 / 1.50 / 0.52 mm.
  - Mechanism dimensions (axis distances, links, W–Q, …) come from caliper measurements, not photos.
- `GRIP_CENTER_Y` = 0.48 mm is **assumed** (table above).
- **Object grasping exists only in the sorting model** (`mk2_sort.xml`).
  - There, the clamping faces carry thin colliding box pads placed along the photo profile.
  - In `mk2.xml` and `mk2_pen.xml` the gripper parts have no collision (`contype = 0`).
- **The gripper TCP is fixed** at the grip point when closed. When opening, the fingers move along a 35 mm-radius arc, so the fingertips retreat toward the wrist (up to 35 mm at 90°). The TCP does not follow the fingertips.
- **The pen is rigidly attached to the wrist** (`--pen` model). The fingers are set to the correct 4.29° grip angle, but grip force, friction and pen slip are not simulated. There is no pen spring. The paper is rigid; paper compliance is not simulated.
