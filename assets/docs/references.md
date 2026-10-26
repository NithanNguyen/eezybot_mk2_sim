# References

## Hardware and geometry

- STL and BOM: daGHIZmo, *EEZYbotARM Mk2*, Thingiverse thing:1454048 (CC BY-NC), https://www.thingiverse.com/thing:1454048. Dimensions measured with `tools/holes.py`.
  Third-party STL mirror used for verification: https://github.com/mchora2/EEZYbotARMMK2
- Real gripper: caliper measurements and design confirmation by the project owner (2026-10-16), plus 6 gripper photos (`assets/images/`).
  - Measured values are tagged `[DO-THAT]` in `params.py`.
  - Profile extraction algorithms (`tools/gripper_from_photos.py`):
    - S. Umeyama, "Least-squares estimation of transformation parameters between two point patterns", *IEEE Trans. PAMI* 13(4), 376–380, 1991 (similarity fit).
    - D. H. Douglas & T. K. Peucker, "Algorithms for the reduction of the number of points required to represent a digitized line or its caricature", *Cartographica* 10(2), 112–122, 1973 (contour simplification).
  - Polygon triangulation (`gripper.py`): D. Eberly, "Triangulation by Ear Clipping", Geometric Tools, https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf
- MG996R: TowerPro, https://towerpro.com.tw/product/mg996r/ (9.4 kgf·cm and 0.19 s/60° at 4.8 V; 11 kgf·cm and 0.15 s/60° at 6 V; 55 g; 40.7 × 19.7 × 42.9 mm). The page lists 0.17 s in another table; this package uses 0.19 s, as in the project proposal.
- SG90: TowerPro SG90 datasheet (archived at https://tams.informatik.uni-hamburg.de/lehre/2023ss/vorlesung/es/doc/sg90.pdf): 1.8 kgf·cm, 0.1 s/60°, 9 g, ~22.2 × 11.8 × 31 mm.
- A4 paper 210 × 297 mm: ISO 216.
- PLA density 1.24 g/cm³: NatureWorks, *Ingeo 4043D Technical Data Sheet* (ASTM D792).

## Sorting task

- Firmware repository https://github.com/NithanNguyen/eezybot_mk2_robot (commit `a172b69`):
  - `data/color/model.json`, `data/color/test.csv`, `reports/color/eval_test.json`;
  - `lib/ColorClassifier/ColorMath.h` (algorithm), `lib/ColorClassifier/ColorClassifier.h` (`READ_INTERVAL_MS`), `src/robot/sorter.h` (`POLL_MS`);
  - `src/robot/color_console.cpp` (`CLS` / `ERR` line format), `include/arm_config.h` (trajectory limits, `DROP_HEIGHT_ABOVE_BIN`).
- Rack, GY-33 module, bottle-cap dimensions, bin wall thickness, the 50 mm release rule and the cycle: project owner's requirements (2026-10-23).

## Software

- MuJoCo XML Reference (`equality/connect`, `equality/joint`, `actuator/position`, `keyframe`, `armature`, geom `margin`/`gap`, `contype`/`conaffinity`) and the Backlash section in Modeling, https://mujoco.readthedocs.io
  - The sorting task also uses: `freejoint`, `option/cone`, `option/jacobian`, `option/flag/multiccd`, geom `solref`/`solimp`, `mesh/vertex` (convex meshes declared in XML).
- MuJoCo Python bindings (`python -m mujoco.viewer --mjcf=...`, `launch_passive`, `user_scn`): https://mujoco.readthedocs.io/en/stable/python.html
- Viewer shortcuts: MuJoCo "Code samples → simulate", https://mujoco.readthedocs.io/en/stable/programming/samples.html
- contourpy (contour tracing on raster images): https://contourpy.readthedocs.io
- Python support per package: each package's PyPI page (e.g. https://pypi.org/project/mujoco/3.2.3/, https://pypi.org/project/ruckig/0.12.2/), "Download files" and "Requires: Python".
- NVIDIA PRIME Render Offload: NVIDIA Linux driver README, https://download.nvidia.com/XFree86/Linux-x86_64/470.82.00/README/primerenderoffload.html
- Ruckig: L. Berscheid & T. Kröger, "Jerk-limited Real-time Trajectory Generation with Arbitrary Target States", RSS 2021 (reference [2] of the project proposal).
