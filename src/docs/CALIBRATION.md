# Calibration Guide

> How to measure and record the mechanical constants that make the robot walk straight.

---

## What Needs Calibration

The robot requires three types of calibration, in this order:

1. **Servo zero-point** — where is the servo's electrical zero vs the mechanical perpendicular?
2. **Stand pose angles** — what servo angles produce the standing pose?
3. **IMU calibration** — software, runs at boot

All results go into `hardware/absolute_truths.py`. Nowhere else.

---

## Tools Required

- `hardware/servo_gui.py` — GUI with sliders for all 12 servos
- A ruler or caliper
- A flat, level surface
- The robot powered via USB/bench supply (not LiPo — don't stress servos during calibration)

---

## Step 1: Servo Perpendicular (perp)

For each servo, find the slider value that places the associated limb segment perpendicular to its parent segment.

### Coxa (hip abductor)

Perpendicular = leg pointing straight out laterally from the body, no toe-in or toe-out.

1. Open GUI, disconnect all limbs except the coxa
2. Slide until the coxa arm is at 90° to the body side
3. Record as `COXA_MECH["FL"]["perp"]` etc.

### Thigh

Perpendicular = thigh pointing straight down (vertical) with the robot on its side.

1. Lay robot on its side
2. Slide thigh until the thigh link is plumb vertical
3. Record as `THIGH_MECH["TFL"]["perp"]` etc.

### Wrist (shin)

Perpendicular = wrist link pointing straight down from the knee, coplanar with the thigh.

1. With thigh at perpendicular, slide wrist until shin is also vertical (knee at 180°)
2. Record as `WRIST_MECH["WFL"]["perp"]` etc.

---

## Step 2: Mechanical Travel (min / max)

For each servo, find the slider values at the two hard stops.

**Caution:** approach hard stops slowly. Stop before you hear grinding.

| Joint | Travel |
|---|---|
| Coxa | Typically 0–90° mechanical |
| Thigh | Typically 0–270° |
| Wrist | Typically 0–200° |

The **order** of min/max in `absolute_truths.py` matters:
- Record the "retracted / inward" position as `min`
- Record the "extended / outward" position as `max`
- If `min > max`, that is intentional — it encodes inverted servo mount

---

## Step 3: Stand Pose Angles

With the robot assembled, use the GUI to dial in the standing pose:

1. Place robot on flat surface
2. Adjust all 12 servos until the robot stands level, all legs parallel, body at nominal height (~180 mm ground clearance)
3. Read slider values for each servo
4. Record in `COXA_STAND`, `THIGH_STAND`, `WRIST_STAND`

### Checking the Stand Pose via IK

After entering stand angles, verify they are consistent with the IK:

```bash
python joints/space.py
# Should print all joints with delta=0 equal to their stand angles
```

Then verify IK produces near-zero deltas for the nominal foot position:

```python
from ik.solver import solve_all_legs
deltas = solve_all_legs({
    "FL": (0.0,  0.07, -0.18),
    "FR": (0.0, -0.07, -0.18),
    "RL": (0.0,  0.07, -0.18),
    "RR": (0.0, -0.07, -0.18),
})
print(deltas)
# All values should be close to 0.0
```

If they are not, the stand angles in `absolute_truths.py` do not match the IK geometry.

---

## Step 4: IMU Calibration (Runtime)

The IMU calibrates at boot by averaging 200 samples with the robot still.

```python
from hardware.imu import init_mpu, calibrate, IMUFilter

init_mpu()
calib = calibrate(samples=200)   # ~4 seconds, robot must be flat and still
imu = IMUFilter(calib)
```

The calibration removes DC bias from the accelerometer and gyro. It does not save to disk — repeats every boot.

**To verify:** after calibration, print `imu.update()` with robot stationary. Roll and pitch should be within ±1° of zero.

---

## Step 5: Posture Controller Reference Lock

The posture controller locks its zero reference on the first call to `posture_step()`. This happens automatically when the main loop starts.

If the robot is not on level ground at startup, call `reset_reference()` after placing it on flat ground:

```python
from joints.posture import reset_reference
reset_reference()
# Then call posture_step() once to lock the new reference
```

---

## Troubleshooting

### Robot leans forward/backward at stand

Check `THIGH_STAND` values. If all four legs are off by the same amount, adjust `_STAND_Z` in `ik/solver.py`.

### Robot leans left/right at stand

Check `COXA_STAND` values. May also be a `JOINT_OFFSET` issue in `joints/conventions.py`.

### One leg is shorter/longer than others

Check that leg's `THIGH_STAND` and `WRIST_STAND` are consistent with the measured perpendicular.

### Servos hunt/oscillate at stand

Increase `DEADBAND_DEG` in `joints/posture.py`. Check that IMU is not picking up servo vibration (mount the MPU6050 on foam padding).

### IK returns large deltas for stand pose

The stand angles in `absolute_truths.py` do not agree with the IK geometry. Re-measure the stand pose with the GUI and update.