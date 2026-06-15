# BARQ v1 — Overview

A 12-DOF quadruped on spotMicro mechanics, revived from a dead 2025 project.
Compute is a Jetson Orin Nano driving everything over one I2C bus: a PCA9685
generates PWM for 12× DS3240MG servos (40 kg·cm, 270°, **no position
feedback**), an HW-290 board provides the MPU6050 IMU. The chassis is a
spotMicro stretched +21.5 mm between the coxa shafts to house the Jetson
(D-006); legs and shoulder modules are stock spotMicro prints.

Source of truth: [mike4192/spotMicro](https://github.com/mike4192/spotMicro)
(read-only clone at `~/barq_v1/reference/spotMicro`) is the **complete design
guide** for geometry, kinematics, gait, and control (D-014). The legacy v1
stack under `src/` contributes ONLY five hardware-interface facts — I2C
bus/addresses, the servo-driving method (PCA9685 PWM), PWM ranges (106–535),
the mechanical limit windows, and the servo channel map (ported to
`barq1/truths.py`). All other legacy code, and its perp/stand poses, are
distrusted / superseded.

## Architecture (new stack, `stack/`)

```
Layer 0    barq1/truths.py     measured electrical/mech facts (ticks canonical, D-001)
           barq1/geometry.py   kinematic wireframe (joint-axis dims, D-006)
Layer 0.5  barq1/servos.py     derived 12-servo table
Layer 1    barq1/pca9685.py    PCA9685 driver, from scratch (safe start, D-003)
Layer 2    barq1/kinematics.py leg FK/IK + body IK (spotMicro-faithful math)
Tools      tools/calibration_gui.py   web GUI, true-zero calibration (D-002/D-004)
           tools/validate_urdf.py     URDF frame checker
Sim        sim/                PyBullet harness (D-008): virtual feedback for an
                               open-loop robot (D-009)
URDF       urdf/barq_v1.urdf.xacro    sim-grade description + vendored STLs
```

## Control doctrine (open-loop servos, D-009)

The servos accept commands and report nothing. Therefore:
- joint-level feedback does not exist on hardware — correctness of commanded
  trajectories is established **in simulation first** (sim = virtual feedback);
- the only runtime feedback is **body-level**: IMU roll/pitch/rates
  (posture control), later optional foot-contact switches;
- controllers may consume only what real hardware provides (commands, IMU,
  time). Sim ground truth is for validation metrics, never for control;
- with no encoders, **servo zero calibration is load-bearing** — the GUI's
  per-servo fits are the only joint-angle truth we get.

## Stage roadmap

- **A — Hardware truth** (done): truths port, driver, calibration GUI.
- **B — Geometry + virtual robot** (current): URDF (done), IK engine, PyBullet
  sim, posture/stability scenarios.
- **C — Gait**: spotMicro-style trot/crawl in sim → metrics → hardware.
- **D — Posture closed-loop**: IMU-based body leveling on hardware.
- **E — Perception revival**: camera stack from `src/main/percept` rebuilt.
