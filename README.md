# Quadruped Robot — Full-Stack Python Controller

> A custom quadruped robot built on a SpotMicro-style frame, running a fully layered Python control stack on a **Jetson Orin Nano**. No ROS. No middleware. Pure Python from IMU to servo.

---

## Demo

> *(Add GIF / video link here once captured)*

---

## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Software Architecture](#software-architecture)
- [Repository Structure](#repository-structure)
- [Setup & Installation](#setup--installation)
- [Running the Robot](#running-the-robot)
- [Modules](#modules)
- [Calibration](#calibration)
- [Tricks & Gaits](#tricks--gaits)
- [Posture Control](#posture-control)
- [Known Issues / TODOs](#known-issues--todos)
- [Credits](#credits)

---

## Overview

This project implements a fully autonomous quadruped robot controller in Python. The stack runs entirely on the robot — no offboard compute, no ROS — and covers:

- **3D Inverse Kinematics** — SpotMicro-style exact geometric IK
- **Gait Generation** — Trot gait with configurable step length, height, and frequency
- **Posture Control** — IMU-based roll + pitch stabilisation via Cartesian foot compensation
- **Joint Space Normalisation** — Converts IK math angles to physical servo angles with direction and zero-offset correction
- **Hardware Abstraction** — PCA9685 PWM driver + MPU6050 IMU driver over I2C (SMBus)
- **Tricks** — Shake, bow, sit, push-ups, rear up, high-five, tilt dance, and more
- **Gamepad Control** — Full Xbox-style controller support via pygame

---

## Hardware

## Hardware

See [`docs/hardware/`](docs/hardware/) for full selection rationale, specs, and integration notes.

### Quick Reference

| Component | Selected | Docs |
|---|---|---|
| Depth camera | Orbbec Gemini 336 | [camera/](docs/hardware/camera/) |
| RGB camera | Arducam IMX219 220° fisheye | [camera/](docs/hardware/camera/) |
| Battery | GenX Premium 5200 (4S 40C) | [battery/](docs/hardware/battery/) |
| Compute | NVIDIA Jetson Orin Nano | — |
| Servo driver | PCA9685 | [hardware/](src/main/hardware/README.md) |
| IMU | MPU6050 | [hardware/](src/main/hardware/README.md) |

| Component | Part |
|---|---|
| Frame | SpotMicro 3D-printed (Thingiverse) |
| Compute | NVIDIA Jetson Orin Nano |
| Servo Driver | PCA9685 (I2C, address 0x40) |
| Servos | 12× MG996R (270° travel) |
| IMU | MPU6050 (I2C, address 0x68) |
| Display | SH1106 OLED 128×64 (I2C, address 0x3C) |
| I2C Bus | Jetson bus 7 |
| Power | 2S LiPo + 5V BEC |

### Body Dimensions

| Parameter | Value |
|---|---|
| Body length | 208.30 mm |
| Body width | 78.90 mm |
| Body height | 74.00 mm |
| Coxa link | 16.05 mm |
| Thigh link | 108.32 mm |
| Shin (wrist) link | 134.76 mm |
| Nominal stand height | 180 mm |

### Wiring Summary

- PCA9685 SDA/SCL → Jetson I2C bus 7
- MPU6050 SDA/SCL → same bus
- SH1106 OLED SDA/SCL → same bus
- Servo channels 0–11 on PCA9685 (see `hardware/absolute_truths.py`)

---

## Software Architecture

The stack is divided into numbered layers. Each layer has exactly one responsibility and communicates only with adjacent layers.

```
┌─────────────────────────────────────────────┐
│  Layer 6 — Gait Generator                   │  gait/generator.py
│  Produces foot targets over time             │
├─────────────────────────────────────────────┤
│  Layer 5 — Posture Controller               │  joints/posture.py
│  IMU feedback → foot target compensation    │
├─────────────────────────────────────────────┤
│  Layer 3 — IK Solver                        │  ik/solver.py
│  Foot targets (hip-local) → joint deltas    │
├─────────────────────────────────────────────┤
│  Layer 2.5 — Joint Conventions              │  joints/conventions.py
│  Sign corrections + zero offsets            │
├─────────────────────────────────────────────┤
│  Layer 2 — Joint Space Normalisation        │  joints/space.py
│  IK deltas → physical servo angles          │
├─────────────────────────────────────────────┤
│  Layer 1.3 — IMU Driver                     │  hardware/imu.py
│  Layer 1.2 — PCA9685 Driver                 │  hardware/pca9685.py
│  Layer 1.1 — I2C Bus                        │  hardware/i2c_bus.py
├─────────────────────────────────────────────┤
│  Layer 0 — Absolute Truths                  │  hardware/absolute_truths.py
│  Channel maps, limits, stand angles         │
└─────────────────────────────────────────────┘
```

### Data Flow (one control loop iteration)

```
Gait Generator
    │  foot targets {FL/FR/RL/RR: (x, y, z)}  [meters, hip-local]
    ▼
Posture Controller   ◄── IMU (roll, pitch, rates)
    │  compensated foot targets
    ▼
IK Solver  (ik/solver.py → ik/kinematics.py)
    │  joint deltas {FL_COXA, FL_THIGH, FL_WRIST, ...}  [degrees]
    ▼
Joint Conventions  (joints/conventions.py)
    │  sign-corrected + offset deltas
    ▼
Joint Space  (joints/space.py)
    │  physical servo angles [degrees, 0–270]
    ▼
PCA9685 Driver  (hardware/pca9685.py)
    │  PWM pulses [106–535 steps]
    ▼
Servos
```

---

## Repository Structure

```
quadruped/
└── src/
    └── main/
        ├── main_controller.py      # Top-level run loop + gamepad input
        │
        ├── gait/
        │   ├── __init__.py
        │   └── generator.py        # Trot gait foot trajectory generator
        │
        ├── hardware/
        │   ├── __init__.py
        │   ├── absolute_truths.py  # ALL electrical/mechanical constants
        │   ├── i2c_bus.py          # Shared SMBus singleton
        │   ├── imu.py              # MPU6050 driver + complementary filter
        │   ├── pca9685.py          # PCA9685 PWM driver
        │   ├── servo_gui.py        # Calibration GUI (tkinter)
        │   └── test_oled.py        # OLED smoke test
        │
        ├── ik/
        │   ├── __init__.py
        │   ├── kinematics.py       # Core 3D IK engine (SpotMicro geometry)
        │   ├── solver.py           # IK interface + stand reference
        │   └── util.py             # Rotation matrix, angle helpers
        │
        ├── joints/
        │   ├── __init__.py
        │   ├── conventions.py      # Joint sign table + offsets
        │   ├── posture.py          # Roll/pitch posture controller
        │   └── space.py            # Delta → physical angle normalisation
        │
        └── stance/
            └── tricks.py           # Shake, bow, sit, pushups, etc.
```

---

## Setup & Installation

### Requirements

- Python 3.10+
- Jetson Orin Nano running Ubuntu 22.04
- I2C bus 7 enabled

### Dependencies

```bash
pip install smbus2 numpy pygame luma.oled pillow
```

### Clone

```bash
git clone <your-repo-url> quadruped
cd quadruped/src/main
```

### I2C Check

Before running, verify devices are visible:

```bash
sudo i2cdetect -y 7
# Expected: 0x40 (PCA9685), 0x68 (MPU6050), 0x3C (OLED)
```

---

## Running the Robot

### Main Controller (gamepad)

```bash
cd quadruped/src/main
python main_controller.py
```

Plug in an Xbox-compatible gamepad before starting.

### Posture Test (static stabilisation)

```bash
python test_posture.py
```

Set `POSTURE_ENABLED = False` first to verify bare stand, then enable.

### Servo Calibration GUI

```bash
python hardware/servo_gui.py
```

Opens a tkinter GUI with sliders for all 12 servos.

### OLED Test

```bash
python hardware/test_oled.py
```

---

## Modules

See individual READMEs for each module:

- [`hardware/README.md`](hardware/README.md) — I2C bus, PCA9685, IMU, constants
- [`ik/README.md`](ik/README.md) — Inverse kinematics engine and solver
- [`joints/README.md`](joints/README.md) — Conventions, normalisation, posture
- [`gait/README.md`](gait/README.md) — Trot gait generator
- [`stance/README.md`](stance/README.md) — Tricks library

---

## Calibration

All mechanical truths live in `hardware/absolute_truths.py`. Nothing else in the stack hardcodes angles or channels.

### Procedure

1. Open `hardware/servo_gui.py`
2. Set each servo to its physical perpendicular (90° from body) position using sliders
3. Record the slider value as `perp` in `WRIST_MECH` / `THIGH_MECH` / `COXA_MECH`
4. Move to standing pose, record as `WRIST_STAND` / `THIGH_STAND` / `COXA_STAND`
5. Record min/max travel angles (0 = one hard stop, max = other)

### Servo Channel Map

| Joint | Leg | PCA Channel |
|---|---|---|
| COXA | FL | 7 |
| COXA | FR | 6 |
| COXA | RL | 1 |
| COXA | RR | 0 |
| THIGH | FL | 9 |
| THIGH | FR | 8 |
| THIGH | RL | 3 |
| THIGH | RR | 2 |
| WRIST | FL | 11 |
| WRIST | FR | 10 |
| WRIST | RL | 5 |
| WRIST | RR | 4 |

---

## Tricks & Gaits

### Gamepad Controls

| Input | Action |
|---|---|
| D-Pad Up | Forward walk |
| D-Pad Down | Backward walk |
| D-Pad Left | Turn left |
| D-Pad Right | Turn right |
| Y button | Cycle height (High/Normal/Low/Crouch) |
| LB | Shake (offer paw) |
| RB | Rear up (bheek) |
| B | Push-ups |
| A | Tilt dance |
| Left Trigger | Wiggle |
| Right Trigger | Bow |
| M1 | Sit |
| M2 | High-five |
| X | Quit |

### Height Modes

| Mode | Z (m) |
|---|---|
| HIGH | -0.19 |
| NORMAL | -0.18 |
| LOW | -0.15 |
| CROUCH | -0.12 |

---

## Posture Control

The posture controller (`joints/posture.py`) runs in the main loop alongside the gait generator. It reads roll and pitch from the MPU6050 and adjusts each leg's Z target to keep the body level.

Key parameters in `joints/posture.py`:

| Parameter | Default | Effect |
|---|---|---|
| `PITCH_GAIN` | 0.8 | Aggressiveness of pitch correction |
| `ROLL_GAIN` | 1.2 | Aggressiveness of roll correction |
| `COXA_ROLL_GAIN` | 8.0 | Lateral coxa splay on roll |
| `PITCH_DAMP` | 0.08 | Derivative damping (pitch) |
| `ROLL_DAMP` | 0.12 | Derivative damping (roll) |
| `DEADBAND_DEG` | 0.7 | Ignore tilt below this threshold |

See [`joints/README.md`](joints/README.md) for full tuning guide.

---

## Known Issues / TODOs

- [ ] Lateral strafe stability needs tuning at NORMAL height
- [ ] Posture controller not yet integrated into walk loop (static stand only)
- [ ] OLED display not showing live telemetry yet
- [ ] No IMU-based terrain adaptation in gait (flat-ground assumption)
- [ ] `stance/tricks.py` uses old import paths — needs update to current module layout

---

## Credits

- IK geometry adapted from the [SpotMicro community](https://github.com/mike4192/spotMicro)
- Frame: SpotMicro by KDY0523 on Thingiverse
- Jetson setup: NVIDIA Developer documentation