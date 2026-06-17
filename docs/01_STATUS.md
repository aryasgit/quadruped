# Status

_Last updated: 2026-06-15_

## Snapshot

**Hardware is connected and verified** — first power-up done: I2C bus 7
shows PCA9685 (0x40), MPU6050 (0x68), and bonus OLED (0x3c) + barometer
(0x77); the diagnostic GUI confirmed the channel map and that the stand/perp
poses produce correct physical poses (so a legacy-derived calibration is
viable for hardware testing). The sim actuates the **hardware boundary**
(D-015) so its numbers are honest.

**Gait control underway (D-016):** velocity-commanded gait (spotMicro walk
port) — forward / turn / strafe from one engine, all statically stable in
sim (p10 margin ≥ 9 mm, tilt ≤ 2.6°, none fell). Joint-safe speed envelope
vx 0.024 / vy 0.022 m/s / wz 0.10 rad/s (thigh upper limit binds). Fixed
crawl + full hardware pipeline (dry-run-clean, 0 overruns) + PS4 teleop all
still in place. Masses being weighed for the URDF (Q-004).

**Next:** controller FSM (idle/stand/walk + filter transitions) → wire
teleop + run_robot to it → drive the real robot (legacy-derived calib) to
judge performance. Then robustness sweeps once masses land.

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
