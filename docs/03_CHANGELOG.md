# Changelog — newest first

## 2026-06-17 — teleop: 4-quadrant proportional left stick

- `teleop/drive.py`: left stick -> `stick_to_velocity()` dominant-axis
  cardinal mapping (front/back = vx, left/right = strafe vy), proportional
  to push; right stick X = turn. Evofox PS4-clone verified (enumerates as
  "Wireless Controller", standard 0-255 axes / 304-316 buttons; no driver
  change). `test_teleop_map.py` (4 tests). 28 pass.

## 2026-06-17 — gait speed-up: brisk cadence + velocity ramp

- `velocity_gait.py`: cadence swing 16→10 / shift 10→6 (cycle 1.28 s) + a
  velocity ramp (`accel_*`) so walk-start eases in and stays in joint limits.
  Clamps raised (vx 0.024→0.05, vy→0.05, wz→0.22); scenarios at vx 0.045.
- ~3× faster (vel_forward 53 mm/s, p10 +14.6 mm, no fall). 24/24 pass.

## 2026-06-15 — controller FSM completes the gait stack (D-017)

- `barq1/controller.py`: idle/stand/walk FSM, rate-limited height+posture
  transitions, feet ease-to-neutral; one `step(cmd)->frame` entry point.
- `teleop/drive.py` rewritten to emit GaitCommands; `run_sim --teleop` and
  `run_robot --teleop` rewired to the FSM. New `controller_demo` sim scenario
  + `test/test_controller.py` (4 tests). 24/24 pass.
- Validated: stands/walks/sits/rises stable, no fall; stand holds zero drift.

## 2026-06-15 — real masses in the URDF (Q-004 resolved)

- All 8 link masses set to Aryaman's measured values: total **1.76 kg**
  (was 4.88 kg placeholder). base_link 0.522, covers 0.044/0.030, per leg
  shoulder 0.0815 / leg 0.1032 (+cover 0.0199) / foot 0.0807 / toe 0.0063.
- Re-baselined (05, 2026-06-15): velocity gait stability improved
  (vel_forward p10 margin 8.6 → 17.9 mm, neg-frames 9.4% → 0.6%); net
  open-loop motion shifted (mass-dependent slip). 20 tests still pass.

## 2026-06-15 — velocity-commanded gait (D-016)

- `barq1/command.py` (GaitCommand), `barq1/filters.py`
  (RateLimitedFirstOrderFilter, ported from spotMicro), `barq1/velocity_gait.py`
  (VelocityGait) — spotMicro walk port: forward/turn/strafe from one engine,
  8-phase static schedule + body weave. Fixed crawl (`gait.py`) untouched.
- `sim/scenarios.py`: vel_forward / vel_turn / vel_strafe (+ in run_sim/--loop).
  `test/test_velocity_gait.py` (5 tests). 20/20 pass.
- Validated (05, 2026-06-15): all three directions statically stable, none
  fell. Joint-safe speed envelope scanned and set as the clamps (vx 0.024,
  vy 0.022 m/s, wz 0.10 rad/s); thigh upper limit is the binding constraint.

## 2026-06-15 — sim fidelity: hardware actuation boundary (D-015)

- `sim/servo_model.py`: `JointActuator` — travel clamp + PCA9685 tick
  quantization (0.629°/step) + 1-frame transport delay. `SimRobot.command()`
  routes through it (default on); `world.py` builds per-joint actuators from
  URDF limits and resets them on teleport. `run_sim.py --ideal` toggles it off.
- Decomposition (05, 2026-06-15): quantization is the dominant effect, delay
  negligible at quasi-static speed. Refuted the "servo-lag" stride-loss
  hypothesis — it was planted-foot micro-slip; the tick deadband removes it
  (walk efficiency 76 %→98 %). Cost: ±8.6° planted yaw now pushes COM ~3 mm
  outside support (pose_sweep). Re-baselined; all 5 scenarios still pass.

## 2026-06-12 — motion core + hardware bridge (D-011, D-012, D-013)

- `barq1/trajectories.py` + `barq1/gait.py`: 50 Hz frame generators
  (stance ramp, pose sweep, weight-shift, 6-phase crawl); scenarios
  re-expressed on them; new `walk` scenario — **first sim walk passed**
  (90.9 mm / 3 cycles, 0.59° heading drift, never unstable; see 05).
- `barq1/servo_map.py` (+6 tests): IK rad → calibrated ticks, signed
  slope, mech-window clamps. `docs/06_CALIBRATION_PROTOCOL.md` defines the
  angle convention; GUI defaults/help updated to match.
- `barq1/imu.py`: MPU6050 driver, bias calibration, complementary filter.
- `runtime/`: `robot_io.py` (staggered engage, slew-limited apply,
  all-off) + `run_robot.py` (sim's twin: same frames, 50 Hz wall-clock,
  ESTOP, JSONL telemetry; `--dry-run` verified: 600 frames, 0 overruns).
- `teleop/`: PS4 via evdev + shared mapping; `run_sim --teleop` and
  `run_robot --teleop` (sticks = pose, TRIANGLE = crawl, SQUARE = ESTOP).
- requirements: +evdev.

## 2026-06-12 — PyBullet simulation harness (D-008, D-009)

- `stack/sim/`: `world.py` (xacro→URDF expansion, world setup, joint maps,
  ground-truth probes: body state, contacts, COM, support-polygon margin),
  `servo_model.py` (DS3240MG as position control with 3.0 N·m / 6.5 rad/s
  caps), `scenarios.py` (settle, stand_up, pose_sweep, weight_shift),
  `run_sim.py` (CLI; CSV + snapshot artifacts to `~/barq_v1/artifacts/`).
- All four scenarios pass headless; metrics in 05 (2026-06-12 entry).
- Viewing fix: GUI mode now paces realtime by default (the flag was
  previously unwired), holds the final pose 4 s, and `--loop` cycles all
  scenarios in ONE persistent window until it's closed (windows no longer
  blink open/shut per scenario).

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
