# Troubleshooting

Error messages printed by the scripts are in Vietnamese without diacritics; they are quoted verbatim below.

| Error / symptom | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: No module named 'mujoco'` | Virtual environment not activated | `source ~/venvs/mk2/bin/activate`, check `which python` ([installation.md §2.3](installation.md#23-dedicated-virtual-environment)) |
| `No matching distribution found for mujoco==3.2.3` | Python is not 3.8, or the machine is not x86_64 | Back to [installation.md §1.1](installation.md#11-os-glibc-python) |
| `Khong tim thay .../EBAmk2_xxx.STL` | Wrong `--stl-dir`, or the file extension is lowercase `.stl` | Redo the `STL_DIR` lookup and the 13-file check ([installation.md §3](installation.md#3-get-the-original-stl-files)) |
| `RuntimeError: Hinh hoc khong khop` | Geometry in `params.py` edited incorrectly, or STL differs from the original | Restore `params.py` (`git diff params.py`), use the STL from installation §3 |
| `params.TCP_W/PEN_GRIP_W/PEN_GRIP_DEG khong khop hinh hoc gripper` | `GRIP_*`, `PEN_*` or `gripper_shape.py` was changed | Copy the lines printed by the script into `params.py`, rebuild the model |
| `luoi lang tru khong kin`, `ear clipping that bai`, `tam giac hoa sai` | A profile in `gripper_shape.py` self-intersects, or `GRIP_*` dimensions make layers overlap | Restore `gripper_shape.py` / `params.py` (`git diff`) |
| `run_sim.py la bai VE hinh, can but` | `run_sim.py` run without `--pen` | `python run_sim.py --pen` |
| `Chua co model/mk2.xml` (or `mk2_pen.xml`) | Model not built | Run `build_model.py` |
| `Hinh ve (...) nam ngoai to giay` | `--cx` / `--cy` / `--R` push the drawing off the A4 sheet | Reduce `--R`, or change `PAPER_CENTER` and rebuild |
| `IKError: th2=... ngoai`, `phi=... ngoai`, `phi-th2=... ngoai`, `lenh CTRL_..._DEG=... ngoai`, `diem ngoai tam voi` | Drawing outside the workspace, joint limits or servo command range | Reduce `--R` or change `--cx` per the `R_max` table ([simulation-guide.md §2](simulation-guide.md#2-kinematics-verification), `--pen`) |
| `ngoi cham giay < 100%` or `so net muc > 1` | The tip leaves the paper mid-stroke (broken stroke) | Check `--z` (must equal the 1 mm paper surface), reduce `--vmax` / `--amax` |
| `-> VUOT gioi han, can gian thoi gian` | The trajectory needs more servo speed than the MG996R provides. This is a **warning**: the simulation runs but results are unrealistic | Reduce `--vmax` / `--amax` / `--jmax` |
| `WARNING: Nan, Inf or huge value in QACC` | Simulation unstable (usually `GRIPPER_ARMATURE` set to 0) | Restore `GRIPPER_ARMATURE = 1e-6`, rebuild |
| Fingers pass through the pen in the `mk2_pen.xml` viewer | The viewer starts with `servo_gripper` = 0 (closed) | Simulation panel → **Load key** ([simulation-guide.md §1.2](simulation-guide.md#12-pen-model-drawing)) |
| Viewer does not open / GLFW error / very slow | Graphics driver is `llvmpipe` or not working | Redo [installation.md §1.2](installation.md#12-gpu-and-opengl-only-needed-for-the-viewer-and-rendered-images). No need to install `libglfw3` via apt; the pip `glfw` package bundles the library |
| `khong chup duoc anh ink_M0.png`, `EGL` / `OSMesa` / `GLFWError` errors, or `Exception ignored in: <function Renderer.__del__ ...>` | No OpenGL (e.g. over SSH without a display), or `MUJOCO_GL` is set | `unset MUJOCO_GL` and retry. The script tries to render once, then skips; other files are still written |
| `Khong voi toi '...'` from `run_sort.py` | Layout in `params.py` section 8 (rack, bin, reject positions, `CAP_GRASP_Z`) is outside the workspace, or `CTRL_ELBOW_DEG` was reverted to −57.3° | Run `python tools/check_sort_layout.py` and choose positions within the radius band printed in section [3] |
| `params.CAP_GRIP_W/CAP_GRIP_DEG khong khop hinh hoc gripper` | `GRIP_CAP_GY`, `CAP_DIAMETER`, `GRIP_*` or the finger profile was changed | Copy the 2 lines printed by the script into `params.py`, rebuild the model |
| `mj_makeConstraint: constraint Jacobian mis-allocation` | A model with many DOFs loaded in sparse-Jacobian mode (MuJoCo 3.2.3) | `mk2_sort.xml` already sets `jacobian="dense"`; keep it if you add caps/objects |
| `run_sort.py` reports a cap at `ngoai` (outside) | Cap bounced out of the compartment or slipped from the gripper | Check `slip_max_mm`, `final_pos` in `results/sort/caps.csv`; replay with `--view` |
| Viewer stutters | Full STL meshes are heavy | Disable shadows in the Rendering panel, or run on the NVIDIA GPU ([installation.md §1.2](installation.md#12-gpu-and-opengl-only-needed-for-the-viewer-and-rendered-images), optional) |
