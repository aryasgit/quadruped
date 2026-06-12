# Status

_Last updated: 2026-06-12_

## Snapshot

Stage B (geometry + virtual robot). Hardware layer and calibration tooling
are built and smoke-tested; URDF is adopted and validated (D-006); IK engine
and PyBullet simulation are landing today. The physical robot has not been
powered in this revival yet — servo board absent from the I2C scan, so
everything hardware-side ran in sim mode.

## Done

- Workspace `~/barq_v1/` (repo `Master` branch, spotMicro reference clone,
  isolated venv). Deps: flask, smbus2, pyyaml, xacro, numpy, pytest,
  pybullet — venv-only.
- Layer 0/1: truths port (bus 7, 0x40/0x68, 106–535 ticks @ 50 Hz over 270°,
  channel map, mech windows, perp/stand), servo table, PCA9685 driver
  (safe start, free-wheel, sim bus).
- Calibration web GUI (:8035), spotMicro two-point method, YAML output.
- URDF `stack/urdf/barq_v1.urdf.xacro` (axle span 207.5) + validator.
- Docs system (this) per D-007.

## How to run

```bash
V=~/barq_v1/venv/bin/python
cd ~/barq_v1/quadruped
$V stack/tools/calibration_gui.py      # web GUI -> http://barq.local:8035
$V stack/tools/validate_urdf.py        # URDF frame checks
$V -m pytest stack/test -q             # kinematics unit tests
$V stack/sim/run_sim.py --scenario stand_up        # headless sim + metrics
DISPLAY=:0 $V stack/sim/run_sim.py --gui ...       # via VNC (fix_display.sh)
```

## Next

1. Power the servo rail → `i2cdetect -y -r 7` should show 0x40 (+0x68) →
   verify channel map (Q-001) → calibrate 12 true zeros → commit the YAML.
2. Gait generator (spotMicro-style) in sim, with stability metrics.
3. Servo-map layer: IK angles → calibrated ticks (needs the calibration YAML).
4. IMU driver + posture loop (Stage D).
