# SpotMicro Quadruped — Full-Stack Robot Controller

> **Jetson Orin Nano · PCA9685 · MPU6050 · 12-DOF · Python**
> A layered, hardware-accurate controller stack for a SpotMicro-class quadruped robot.

---

## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Software Architecture](#software-architecture)
  - [Layer 0 — Absolute Truths](#layer-0--absolute-truths)
  - [Layer 1 — Hardware Drivers](#layer-1--hardware-drivers)
  - [Layer 2 — Joint Space](#layer-2--joint-space)
  - [Layer 3 — Inverse Kinematics](#layer-3--inverse-kinematics)
  - [Layer 4 — Single Leg Tests](#layer-4--single-leg-tests)
  - [Layer 5 — Posture Controller](#layer-5--posture-controller)
  - [Layer 6 — Gait Generator](#layer-6--gait-generator)
  - [stance — Tricks & Demo Routines](#stance--tricks--demo-routines)
- [Coordinate Frame Convention](#coordinate-frame-convention)
- [Joint Naming Convention](#joint-naming-convention)
- [Servo Channel Map](#servo-channel-map)
- [Mechanical Calibration](#mechanical-calibration)
- [Getting Started](#getting-started)
- [Running the Robot](#running-the-robot)
  - [Stand Test](#stand-test)
  - [Single Axis Tests](#single-axis-tests)
  - [Walking](#walking)
  - [Tricks](#tricks)
  - [IMU Balance](#imu-balance)
- [Controller Mapping (PS-style gamepad)](#controller-mapping-ps-style-gamepad)
- [Tuning Reference](#tuning-reference)
- [Known Issues & Gotchas](#known-issues--gotchas)
- [File Reference](#file-reference)

---

## Overview

This project implements a **full motion-control stack** for a SpotMicro-class 12-DOF quadruped robot running on a **Jetson Orin Nano**. It is written entirely in Python and designed around a strict layered architecture where each layer has exactly one responsibility and cannot reach "up" or "skip" layers.

Key design goals:
- **Hardware-accurate:** All electrical and mechanical truths live in a single file (`absolute_truths.py`). If the robot is wrong, fix it there.
- **Math stays clean:** IK, gait, and posture code contains zero hardware knowledge. It outputs foot positions or joint deltas in SI units only.
- **One bridge:** `joint_conventions.py` is the *only* place hardware mirroring, sign inversions, and offsets are applied.
- **Incrementally testable:** Every layer can be tested independently before integrating upward.

---

## Hardware

| Component | Spec |
|---|---|
| Compute | NVIDIA Jetson Orin Nano |
| PWM Driver | PCA9685 (I2C, address `0x40`, bus 7) |
| IMU | MPU6050 (I2C, address `0x68`, bus 7) |
| Display (optional) | SH1106 OLED (I2C, address `0x3C`, bus 7) |
| Servos | 12× standard 270° hobby servos |
| Degrees of Freedom | 12 (3 per leg: Coxa / Thigh / Wrist) |
| Frame | SpotMicro-class |
| Body length | 208 mm |
| Body width | 78 mm |
| Coxa length | ~16 mm |
| Thigh length | ~108 mm |
| Shin/Wrist length | ~135 mm |

**I2C wiring note:** All I2C peripherals share bus 7 on the Jetson. The `i2c_bus.py` singleton ensures they share one `SMBus` handle.

---

## Software Architecture

The stack is divided into numbered layers. Each layer **only imports from the layer immediately below it**. Skipping layers is a bug.

```
Layer 6  — Gait Generator          (foot trajectory math, pure geometry)
Layer 5  — Posture Controller      (IMU-based body levelling)
Layer 4  — Integration Tests       (x/y/z axis single-leg tests, stand/squat)
Layer 3  — Inverse Kinematics      (foot XYZ → joint angles, pure math)
Layer 2  — Joint Space             (delta angles → physical servo angles)
 ├─ 2.5  — Joint Conventions       (sign flips, mirroring — THE ONLY PLACE)
Layer 1  — Hardware Drivers
 ├─ 1.1  — I2C Bus (singleton SMBus)
 ├─ 1.2  — PCA9685 driver
 └─ 1.3  — MPU6050 / IMU filter
Layer 0  — Absolute Truths         (channel map, limits, stand angles — READ ONLY)
```

---

### Layer 0 — Absolute Truths

**File:** `hardware/absolute_truths.py`

This file is the **single source of truth for all physical constants**. It has no logic — only dictionaries and constants. If a servo is wired to the wrong channel, fix it here. If a stand angle drifts, fix it here. Nowhere else.

```
COXA    — {leg_name: PCA_channel}
THIGHS  — {leg_name: PCA_channel}
WRISTS  — {leg_name: PCA_channel}

COXA_MECH   — {leg: {min, max, perp}}   # servo angle domain limits
THIGH_MECH  — same
WRIST_MECH  — same

COXA_STAND  — {leg: angle_deg}           # measured stand pose
THIGH_STAND — same
WRIST_STAND — same

PULSE_MIN = 106   # PCA9685 pulse steps
PULSE_MAX = 535
```

**Rule:** Never import `absolute_truths` from above Layer 2. Only hardware drivers and joint normalization touch it.

---

### Layer 1 — Hardware Drivers

#### 1.1 `hardware/i2c_bus.py`
Singleton `SMBus` handle. All drivers call `get_i2c_bus()`. Bus is opened on first call and shared for the process lifetime. Never instantiate `SMBus` directly anywhere else.

#### 1.2 `hardware/pca9685.py`
- `init_pca()` — configure PCA9685 at 50 Hz. Call once at startup.
- `angle_to_pulse(angle_deg)` — maps 0–270° to `PULSE_MIN`–`PULSE_MAX`.
- `set_servo_angle(channel, angle_deg)` — the only output primitive. All servo motion goes through this.

**Servo range:** 0–270° mechanical. Pulse range 106–535 (4096-count PWM). 50 Hz.

#### 1.3 `hardware/imu.py`
- `init_mpu()` — wake MPU6050, configure DLPF ~20 Hz.
- `calibrate(samples=200)` — blocking, robot must be still. Returns `IMUCalibration`.
- `IMUFilter(calib, alpha=0.96, dt=0.02)` — complementary filter. Call `.update()` to get `(roll, pitch, roll_rate, pitch_rate)` in degrees / degrees-per-second.

---

### Layer 2 — Joint Space

#### `layer2/joint_space.py`
Converts **delta angles** (math space, relative to stand) into **physical servo angles** (hardware space, absolute degrees). Clamps to mechanical limits from Layer 0.

- `normalize_joint(joint_name, delta_angle)` → physical angle
- `normalize_all(deltas_dict)` → dict of physical angles

**Canonical joint order (12 joints):**
```
FL_COXA, FL_THIGH, FL_WRIST,
FR_COXA, FR_THIGH, FR_WRIST,
RR_COXA, RR_THIGH, RR_WRIST,
RL_COXA, RL_THIGH, RL_WRIST
```

#### `layer2/joint_conventions.py` ← **THE MIRROR ADAPTER**

This is the **only** place hardware asymmetry is handled. Applied between IK delta output and `normalize_all`.

```python
JOINT_SIGN = {
    # Coxa (yaw): left legs inverted, right legs normal
    "FL_COXA": -1,  "FR_COXA": +1,
    "RL_COXA": -1,  "RR_COXA": +1,

    # Thigh (pitch): same sign for all legs
    "FL_THIGH": +1, "FR_THIGH": +1,
    "RL_THIGH": +1, "RR_THIGH": +1,

    # Wrist (knee): right side is physically mirrored
    "FL_WRIST": +1, "RL_WRIST": +1,
    "FR_WRIST": -1, "RR_WRIST": -1,   # ← hardware inversion
}
```

**Do not add per-leg sign fixes anywhere else. Ever.**

---

### Layer 3 — Inverse Kinematics

**Files:** `layer3/kinematics.py`, `layer3/leg_ik.py`, `layer3/util.py`

Pure geometry. Zero hardware imports. Converts foot positions (hip-local XYZ, meters) into joint angle deltas (degrees, relative to stand pose).

`kinematics.py` implements the full SpotMicro 3D IK model including:
- Coxa: lateral swing (θ₁)
- Thigh: sagittal plane upper link (θ₂)
- Wrist: sagittal plane lower link / knee (θ₃)

**`leg_ik.py` public interface:**
```python
solve_leg_ik(x, y, z, leg)   → (coxa_delta, thigh_delta, wrist_delta) in degrees
solve_all_legs(foot_targets)  → dict {joint_name: delta_deg}
```

Foot targets are **hip-local** (origin at hip joint, not body center). Stand reference is pre-computed once at module load.

**Stand reference used internally:**
```
STAND_X = 0.00 m
STAND_Y = ±0.07 m  (left positive, right negative)
STAND_Z = -0.18 m
```

---

### Layer 4 — Single Leg Tests

**Files:** `layer4/x_axis.py`, `layer4/y_axis.py`, `layer4/z_axis.py`, `layer4/stand_controller.py`

Integration tests and the first layer that moves hardware. Each file runs a sinusoidal single-leg motion to validate the full pipeline: IK → conventions → normalize → servo.

`stand_controller.py` implements the stand-squat test (smooth height sweep all legs simultaneously) and the coxa isolation test (lateral foot sweep with thigh/wrist locked to stand).

**Pipeline in every Layer 4 file:**
```python
coxa, thigh, wrist = solve_leg_ik(x, y, z, leg)
deltas = {"FL_COXA": coxa, "FL_THIGH": thigh, "FL_WRIST": wrist}
deltas = apply_joint_conventions(deltas)
physical = normalize_all(deltas)
set_servo_angle(channel, physical[joint])
```

---

### Layer 5 — Posture Controller

**File:** `layer5/posture_controller.py`

IMU-based roll and pitch compensation. Adjusts per-leg Z targets to keep the body level. Coxa deltas added for roll stabilization.

**Tunable gains:**
```python
PITCH_GAIN     = 0.8    # Z correction amplitude per pitch degree
ROLL_GAIN      = 1.2    # Z correction amplitude per roll degree
COXA_ROLL_GAIN = 8.0    # Coxa degrees per radian of roll
PITCH_DAMP     = 0.080  # Derivative damping on pitch rate
ROLL_DAMP      = 0.120  # Derivative damping on roll rate
DEADBAND_DEG   = 0.7    # Ignore IMU noise below this
```

**Key behavior:** IMU reference is locked on first call. All compensation is relative to boot pose — no absolute gravity dependence, no slow-drift nosedive.

Public functions:
```python
posture_step(nominal_foot_targets, imu)  → joint deltas dict
get_current_imu_state(imu)               → (roll_deg, pitch_deg)
compute_z_compensation(roll, pitch)      → {leg: dz}
compute_coxa_compensation(roll)          → {leg: delta_deg}
```

---

### Layer 6 — Gait Generator

**File:** `layer6/gait_generator.py`

Pure trajectory math. No IK, no hardware. Generates hip-local foot targets as a function of time.

**Trot pattern:** Diagonal pairs (FL+RR) and (FR+RL) alternate with 0.5 phase offset.

**`_leg_trajectory(phase, step_length, step_height, duty)` → `(dx, dz)`**

```
STANCE phase (phase < duty):
  dx = +step/2 → -step/2  (linear pushback)
  dz = 0

SWING phase (phase ≥ duty):
  dx = -step/2 → +step/2  (linear return)
  dz = step_height × sin(π × s)  (half-sine arc)
```

Default parameters:
```python
STEP_LENGTH  = 0.10 m
STEP_HEIGHT  = 0.045 m
DUTY_FACTOR  = 0.60
```

Walk test scripts (`test_walk_minimal_V3.py`, `wireless.py`) override these for stability.

---

### stance — Tricks & Demo Routines

**Files:** `stance/Tricks2.py`, `stance/Trot.py`, `stance/TricksGUI.py`

High-level motion sequences and the production trot controller.

**Available tricks (all use the standard IK pipeline):**

| Key | Trick | Description |
|-----|-------|-------------|
| `1` | `shake` | Weight-shift left, lift FR paw, handshake motion |
| `2` | `bow` | Front body drops, rear stays elevated |
| `3` | `wiggle` | Rear twist side-to-side (excited dog) |
| `4` | `pushups` | Full body squat up/down, N reps |
| `5` | `bheek` | Tip onto rear legs, paw wave, recovery pounce |
| `6` | `high_five` | Rear up + extend FR paw high |
| `7` | `sit` | Rear crouches, front stays up |
| `8` | `stretch` | Front drops forward, rear extends back |
| `9` | `tilt_dance` | Roll side to side rhythmically |
| `0` | `combo` | Full show routine, all tricks chained |

**`Trot.py`** is the production IMU-stabilized trot with Bezier cubic swing trajectories, EMA-filtered IMU, yaw drift trim via differential stride length, and CSV logging.

**`TricksGUI.py`** is a dark-themed tkinter GUI with E-stop, keyboard shortcuts, execution history, and background-threaded trick execution.

---

## Coordinate Frame Convention

**All code above Layer 2.5 uses hip-local right-handed coordinates:**

```
    x : forward  (+)
    y : left     (+)
    z : up       (+)
    Ground is negative Z.
```

Left legs have positive Y. Right legs have negative Y. Stance reference:
```
STANCE_X = 0.0 m
STANCE_Y = +0.07 m  (left legs)  / -0.07 m  (right legs)
STANCE_Z = -0.18 m
```

**Do not flip X or Z for right-side legs in gait code.** Mirroring is handled exclusively in `joint_conventions.py`.

---

## Joint Naming Convention

```
Coxa  — hip yaw joint (lateral swing)
Thigh — hip pitch joint (upper leg, sagittal)
Wrist — knee joint (lower leg / shin)

Legs:
  FL = Front Left
  FR = Front Right
  RL = Rear Left
  RR = Rear Right

Full joint names: {LEG}_{JOINT}
  e.g. FL_COXA, RR_WRIST, FR_THIGH
```

---

## Servo Channel Map

| PCA Channel | Joint | Leg |
|---|---|---|
| 0 | COXA | RR |
| 1 | COXA | RL |
| 2 | THIGH | RR |
| 3 | THIGH | RL |
| 4 | WRIST | RR |
| 5 | WRIST | RL |
| 6 | COXA | FR |
| 7 | COXA | FL |
| 8 | THIGH | FR |
| 9 | THIGH | FL |
| 10 | WRIST | FR |
| 11 | WRIST | FL |

---

## Mechanical Calibration

All calibration data lives in `hardware/absolute_truths.py`. Three dicts per joint group:

- `*_MECH` — `{min, max, perp}` in servo degrees. `min`/`max` define the physical travel direction (note: right-side servos have `min > max` indicating they are mechanically inverted). `perp` is the angle where the limb is perpendicular to the body.
- `*_STAND` — measured stand pose angles (absolute servo degrees, not deltas).

**Calibration workflow:**

1. Run `servo_gui.py` to manually sweep any channel.
2. For coxa: run `test_coxa_calibration.py` — it provides 4 tests:
   - **Test 4 (raw):** Print all current values.
   - **Test 3 (IK zero):** Send neutral stand through full pipeline, check symmetry.
   - **Test 1 (visual):** Move each coxa to its stand angle, confirm perpendicular.
   - **Test 2 (sweep):** Sweep ±15° around stand to find true perpendicular.
3. Update `COXA_STAND` / `THIGH_STAND` / `WRIST_STAND` in `absolute_truths.py`.
4. Re-run Test 3 to confirm left/right symmetry < 3°.

**Critical calibration invariants:**
- `COXA_STAND[leg]` must equal the angle at which the coxa is perpendicular to the body.
- `THIGH_STAND` and `WRIST_STAND` together must produce the desired standing height at the pre-configured `STANCE_Z`.
- Left/right stand angle symmetry in physical space should be within 3°.

---

## Getting Started

### Prerequisites

```bash
pip install smbus2 numpy scipy
pip install luma.oled pillow       # optional — OLED display
pip install pygame                 # optional — gamepad control
```

### Repository Layout

```
hardware/
  absolute_truths.py    # Layer 0 — read only constants
  i2c_bus.py            # Layer 1.1 — I2C singleton
  pca9685.py            # Layer 1.2 — servo driver
  imu.py                # Layer 1.3 — MPU6050 + complementary filter

layer2/
  joint_space.py        # delta → physical angle normalization
  joint_conventions.py  # sign/mirror adapter (the only hardware bridge)

layer3/
  kinematics.py         # 3D IK solver (SpotMicro model)
  leg_ik.py             # public IK interface
  util.py               # rotation matrices, helpers

layer4/
  x_axis.py             # single-leg X-axis test
  y_axis.py             # single-leg Y-axis test
  z_axis.py             # single-leg Z-axis test
  stand_controller.py   # stand-squat + coxa isolation test

layer5/
  posture_controller.py # IMU roll/pitch compensation
  posture_controller_gait_only.py  # gait without IMU

layer6/
  gait_generator.py     # trajectory math (pure functions)

stance/
  Trot.py               # production IMU trot
  Tricks2.py            # show tricks
  Tricks.py             # older tricks (kept for reference)
  TricksGUI.py          # tkinter GUI for tricks

test.py                 # basic stand test (direct servo write)
test_oled.py            # OLED display smoke test
servo_gui.py            # calibration GUI
test_coxa_calibration.py
test_walk_minimal.py    # simplest walking test
test_walk_minimal_V2.py # diagonal arc walk
test_walk_minimal_V3.py # V3 with ramp + logging
test_walk_multiple_dir.py  # multi-directional keyboard control
wireless.py             # gamepad-driven multi-directional walk + tricks
walk_controller_imu_integ.py  # walk + live IMU balance
balance_tester.py       # IMU balance, no walking
main_controller.py      # production gamepad controller
pump.py                 # all-legs XYZ symmetry test
```

### First Boot Checklist

```
[ ] SMBus available: i2cdetect -y 7
[ ] PCA9685 visible at 0x40
[ ] MPU6050 visible at 0x68
[ ] Run: python -m hardware.pca9685   (smoke test, sweeps channel 0)
[ ] Run: python -m hardware.imu       (smoke test, prints live roll/pitch)
[ ] Run: python test.py               (sends stand angles direct, no IK)
[ ] All 12 servos respond
[ ] Robot visually at stand pose
[ ] Run: python servo_gui.py          (calibration GUI)
[ ] Run: python -m test_coxa_calibration --test 3  (IK symmetry check)
```

---

## Running the Robot

### Stand Test

```bash
python test.py
```
Sends hardcoded stand angles directly to all servos. No IK. Use this to verify servo wiring only.

```bash
python servo_gui.py
```
tkinter GUI with sliders for all 12 channels. Use for initial calibration and coxa alignment.

---

### Single Axis Tests

These move one leg through a sinusoidal trajectory. Run them in order to validate the full IK → conventions → hardware pipeline before attempting walking.

```bash
python -m layer4.z_axis    # FL leg bobs up and down
python -m layer4.y_axis    # FL leg sweeps laterally
python -m layer4.x_axis    # FL leg sweeps forward/back
```

Expected result: smooth sinusoidal motion, no jerking, no servo binding, no sign inversions on either side.

```bash
python -m layer4.stand_controller   # stand-squat test (all legs)
```

---

### Walking

**Minimal walk (open-loop, all forward):**
```bash
python test_walk_minimal_V3.py
```
Fastest path to walking. No keyboard input, runs until `Ctrl+C`. Startup ramp included.

**Multi-directional keyboard walk:**
```bash
python test_walk_multiple_dir.py
```
W/A/S/D/Q/E for direction. C to cycle height. 1-9/0 for tricks. X to quit.

**Gamepad walk + tricks:**
```bash
python wireless.py
```
Requires pygame-compatible controller on joystick 0. D-pad for direction, buttons for tricks.

**Production gamepad controller:**
```bash
python main_controller.py
```

---

### Tricks

**Interactive menu:**
```bash
python -m stance.Tricks2
```

**Single trick:**
```bash
python -m stance.Tricks2 --trick shake
python -m stance.Tricks2 --trick combo
```

**Full demo (all tricks):**
```bash
python -m stance.Tricks2 --demo
```

**GUI:**
```bash
python -m stance.TricksGUI
```
Dark cyber-themed GUI. Keyboard shortcuts 1–9, 0=combo, Space=stand, Esc=E-stop.

---

### IMU Balance

**Static balancing only (no walk):**
```bash
python balance_tester.py
```
Robot holds stand pose while actively counteracting tilt. Good for testing IMU gains.

**Walk + live IMU:**
```bash
python walk_controller_imu_integ.py
```
IMU balance runs continuously at idle. Compensation is captured and held (frozen) during walk cycles.

**Production trot with IMU:**
```bash
python -m stance.Trot
python -m stance.Trot --no-imu          # open-loop
python -m stance.Trot --log             # write trot_log.csv
python -m stance.Trot --freq 1.0 --len 0.04 --ht 0.02
```

---

## Controller Mapping (PS-style gamepad)

| Input | Action |
|---|---|
| D-pad Up | Forward |
| D-pad Down | Backward |
| D-pad Left | Turn Left |
| D-pad Right | Turn Right |
| LT (axis 2 > 0.7) | Wiggle |
| RT (axis 5 > 0.7) | Bow |
| LB (button 4) | Shake |
| RB (button 5) | Bheek |
| A (button 0) | Tilt Dance |
| B (button 1) | Pushups |
| X (button 2) | Quit |
| Y (button 3) | Cycle Height |
| M1 (button 6) | Sit |
| M2 (button 7) | High Five |

---

## Tuning Reference

### Walking Stability

| Parameter | File | Safe Range | Effect |
|---|---|---|---|
| `FREQ` | walk scripts | 0.8–2.0 Hz | Gait speed. Lower = more stable. |
| `STEP_LENGTH` | walk scripts | 0.04–0.08 m | Stride length. Smaller = safer. |
| `STEP_HEIGHT` | walk scripts | 0.02–0.05 m | Foot clearance. Higher = slower. |
| `DUTY` | walk scripts | 0.55–0.85 | Stance fraction. Higher = more stable, slower. |
| `STANCE_Y` | walk scripts | 0.065–0.090 m | Lateral foot spread. Wider = more stable. |
| `STANCE_Z` | walk scripts | -0.22 to -0.14 m | Body height. More negative = taller. |
| `COXA_DELTA_BIAS` | walk scripts | ±3.0° | Toe-in preload. Helps prevent slipping. |

### IMU Posture Gains

| Parameter | File | Notes |
|---|---|---|
| `PITCH_GAIN` | `posture_controller.py` | Z correction per pitch degree. Too high → oscillation. |
| `ROLL_GAIN` | `posture_controller.py` | Z correction per roll degree. |
| `COXA_ROLL_GAIN` | `posture_controller.py` | Lateral correction. Too high → yaw drift. |
| `PITCH_DAMP` | `posture_controller.py` | Derivative on pitch rate. Reduces overshoot. |
| `ROLL_DAMP` | `posture_controller.py` | Derivative on roll rate. |
| `DEADBAND_DEG` | `posture_controller.py` | Ignore IMU jitter below this. Increase if hunting. |
| `IMU_EMA_ALPHA` | `Trot.py` | Low-pass on IMU angle. 0=frozen, 1=raw. 0.3 recommended. |

### Trot Yaw Drift Correction

If the robot rotates while walking straight, adjust `YAW_STRIDE_TRIM` in `stance/Trot.py`:
- Robot rotates **right** → increase `YAW_STRIDE_TRIM`
- Robot rotates **left** → decrease `YAW_STRIDE_TRIM`

Start at ±0.005 m steps.

---

## Known Issues & Gotchas

**Right-side wrists:** FR and RR wrist servos are physically mirrored. Their sign is inverted in `joint_conventions.py` (`FR_WRIST: -1`, `RR_WRIST: -1`). If you ever see a right leg extending when it should retract, this is the first place to check.

**Coxa stand angles:** `COXA_STAND` values must reflect where the leg is physically perpendicular to the body — not the servo's electrical center. Measure and calibrate with `test_coxa_calibration.py`. Asymmetric coxa stand angles cause yaw drift during trot.

**IMU drift after reboot:** The IMU reference is locked on the first call to `posture_step`. If the robot is moving during IMU initialization, the reference will be wrong. Keep the robot still during `calibrate()`.

**`bheek` / `high_five` recovery:** After rearing up on hind legs, the recovery pounce sends all legs to a crouch before rising. If servos are under-powered, this recovery can fail. Increase the `crouch` Z offset in `Tricks2.py` if the front legs don't contact the ground cleanly.

**SMBus sharing:** All hardware drivers share a single `SMBus` instance from `i2c_bus.py`. Never open the bus directly. If you see I2C errors, check that no other process has the bus open.

**Layer boundary violations:** The most common debugging mistake is adding a per-leg sign flip outside `joint_conventions.py`. This causes the correct behavior in one test but breaks another. All sign/mirror logic belongs in one place.

**`posture_controller_gait_only.py`:** Contains a commented-out right-leg mirror correction (`deltas[f"{leg}_THIGH"] *= -1`). This was an experiment that duplicates what `joint_conventions.py` already handles. Do not uncomment.

---

## File Reference

| File | Layer | Purpose |
|---|---|---|
| `hardware/absolute_truths.py` | 0 | All physical constants |
| `hardware/i2c_bus.py` | 1.1 | SMBus singleton |
| `hardware/pca9685.py` | 1.2 | Servo PWM driver |
| `hardware/imu.py` | 1.3 | MPU6050 + complementary filter |
| `layer2/joint_space.py` | 2 | Delta → physical angle |
| `layer2/joint_conventions.py` | 2.5 | Sign / mirror adapter |
| `layer3/kinematics.py` | 3 | 3D IK solver |
| `layer3/leg_ik.py` | 3 | IK public interface |
| `layer3/util.py` | 3 | Rotation matrices |
| `layer4/z_axis.py` | 4 | Z-axis single leg test |
| `layer4/y_axis.py` | 4 | Y-axis single leg test |
| `layer4/x_axis.py` | 4 | X-axis single leg test |
| `layer4/stand_controller.py` | 4 | Stand/squat, coxa isolation |
| `layer5/posture_controller.py` | 5 | IMU posture compensation |
| `layer5/posture_controller_gait_only.py` | 5 | Gait without IMU |
| `layer6/gait_generator.py` | 6 | Foot trajectory math |
| `stance/Trot.py` | — | Production IMU trot |
| `stance/Tricks2.py` | — | Show tricks (current) |
| `stance/Tricks.py` | — | Show tricks (legacy) |
| `stance/TricksGUI.py` | — | tkinter tricks GUI |
| `test.py` | — | Direct servo stand test |
| `servo_gui.py` | — | Calibration GUI |
| `test_coxa_calibration.py` | — | Coxa calibration verifier |
| `test_walk_minimal.py` | — | Simplest walk test |
| `test_walk_minimal_V2.py` | — | Diagonal arc walk |
| `test_walk_minimal_V3.py` | — | V3 with ramp + logging |
| `test_walk_multiple_dir.py` | — | Keyboard multi-dir walk |
| `wireless.py` | — | Gamepad walk + tricks |
| `walk_controller_imu_integ.py` | — | Walk + live IMU balance |
| `balance_tester.py` | — | Static IMU balance test |
| `pump.py` | — | All-legs XYZ symmetry test |
| `main_controller.py` | — | Production gamepad controller |
| `test_oled.py` | — | OLED display test |

---
