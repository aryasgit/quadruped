# Changelog — newest first

## 2026-06-12 — PyBullet simulation harness (D-008, D-009)

- `stack/sim/`: `world.py` (xacro→URDF expansion, world setup, joint maps,
  ground-truth probes: body state, contacts, COM, support-polygon margin),
  `servo_model.py` (DS3240MG as position control with 3.0 N·m / 6.5 rad/s
  caps), `scenarios.py` (settle, stand_up, pose_sweep, weight_shift),
  `run_sim.py` (CLI; CSV + snapshot artifacts to `~/barq_v1/artifacts/`).
- All four scenarios pass headless; metrics in 05 (2026-06-12 entry).

## 2026-06-12 — IK engine + sim-grade URDF inertials (D-010)

- `stack/barq1/kinematics.py`: analytic leg FK/IK (3-DOF, knee-backward
  branch) + whole-body IK/FK + URDF joint mapping, mirroring the URDF frames
  exactly. `stack/test/test_kinematics.py`: 9 tests (FK∘IK round-trip grid,
  law-of-cosines cross-check, body-pose round-trip, limits envelope,
  left/right mirror) — all passing.
- URDF inertials replaced with analytic box/sphere values (D-010);
  `validate_urdf.py` still passes. requirements: +xacro, numpy, pybullet,
  pytest.

## 2026-06-12 — docs system adopted (D-007)

- `docs/` created at repo root per the BARQ-Rebuild portable reference:
  00–05 + HANDOFF + README; `stack/docs/` content migrated and removed.
- Decisions renumbered D-001…D-006; D-007 (docs), D-008 (PyBullet),
  D-009 (open-loop doctrine) recorded. Q-001…Q-004 opened in 04.

## 2026-06-11 — BARQ v1 URDF (D-006)

- `stack/urdf/barq_v1.urdf.xacro`: spotMicro URDF with shiftx 0.10375
  (axle span 207.5) and body_length 0.1615; upstream cover collision boxes
  sign-fixed; lidar removed; STLs vendored to `stack/urdf/stl/` (MIT).
- `stack/barq1/geometry.py` (kinematic wireframe) and
  `stack/tools/validate_urdf.py` (xacro expand + frame asserts — passing).

## 2026-06-11 — from-scratch stack + calibration GUI (D-001…D-005)

- `stack/barq1/`: truths.py (verbatim port of measured facts), servos.py
  (derived 12-servo tick table), pca9685.py (smbus2 driver: block writes,
  off/free-wheel, all-off safe start, sim bus).
- `stack/tools/calibration_gui.py` + `static/index.html`: web calibration
  GUI on :8035 (slider/nudge in ticks, mech-window clamps, perp/stand
  references, two-point fit per D-004, YAML save, ESC/ALL-OFF).
- Workspace `~/barq_v1/` created (repo + reference spotMicro clone + venv);
  branch `Master` created and pushed.
