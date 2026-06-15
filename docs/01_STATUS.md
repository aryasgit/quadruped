# Status

_Last updated: 2026-06-15_

## Snapshot

Stage C in sim is **done — the robot walks**, and the sim now actuates the
**hardware boundary** (PCA9685 tick quantization + transport delay, D-015),
so its numbers are honest. Re-baselined fidelity-ON: crawl **117.7 mm / 3
cycles, 98.1 % efficiency, never statically unstable**; the old 76 % was a
sim artifact (planted-foot micro-slip the tick deadband removes — 05,
2026-06-15). Full hardware pipeline built + dry-run-verified (calibration →
servo map → slew-limited 50 Hz runtime → telemetry; 600 frames, 0 overruns).
PS4 teleop works in sim. **Two tracks open:** (next) robustness sweeps +
gait control/sequencing in sim; (parallel) hardware day — reassembly,
power-up, channel check, 12-servo calibration per
docs/06_CALIBRATION_PROTOCOL.md. Masses being weighed for the URDF (Q-004).

## Done

- Workspace `~/barq_v1/` (repo `Master` branch, spotMicro reference clone,
  isolated venv). Deps: flask, smbus2, pyyaml, xacro, numpy, pytest,
  pybullet — venv-only.
- Layer 0/1: truths port (bus 7, 0x40/0x68, 106–535 ticks @ 50 Hz over 270°,
  channel map, mech windows, perp/stand), servo table, PCA9685 driver
  (safe start, free-wheel, sim bus).
- Calibration web GUI (:8035), spotMicro two-point method, YAML output.
- URDF `stack/urdf/barq_v1.urdf.xacro` (axle span 207.5) + validator;
  sim-grade inertials (D-010).
- Kinematics: leg FK/IK + body IK (`barq1/kinematics.py`), 9/9 unit tests.
- PyBullet sim (`stack/sim/`): settle / stand_up / pose_sweep / weight_shift
  all passing headless; open-loop leg-lift proven (margin 33.5 mm).
- Docs system (this) per D-007.

## How to run

```bash
V=~/barq_v1/venv/bin/python
cd ~/barq_v1/quadruped
$V stack/tools/calibration_gui.py      # web GUI -> http://barq.local:8035
$V stack/tools/validate_urdf.py        # URDF frame checks
$V -m pytest stack/test -q             # 15 unit tests (kinematics, servo map)
$V stack/sim/run_sim.py                # all 5 scenarios headless + artifacts
DISPLAY=:0 $V stack/sim/run_sim.py --loop          # demo window via VNC
DISPLAY=:0 $V stack/sim/run_sim.py --teleop        # PS4 drives the sim
$V stack/runtime/run_robot.py --dry-run --scenario walk   # hw pipeline, no hw
$V stack/runtime/run_robot.py --scenario stand     # REAL ROBOT (on stand!)
```

## Next (hardware day — checklist in HANDOFF)

1. Reassemble; document the power tree (Q-002: 4S tethered → BEC specs).
2. Power → `i2cdetect -y -r 7` → channel check (Q-001) → calibrate 12
   servos per docs/06_CALIBRATION_PROTOCOL.md → commit the YAML.
3. On the stand: `run_robot --scenario stand` → `pose_sweep` →
   `weight_shift` → `walk`; then ground. Then PS4 teleop.
4. Masses from Q-004 → update xacro → re-run sim baselines.
5. Stage D: IMU posture loop. Stage E: OAK-D Pro perception.
