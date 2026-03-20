# joints/ — Layers 2, 2.5, 5: Joint Space, Conventions, Posture

> The bridge between math and metal. Everything above this layer is dimensionless. Everything below is physical degrees.

---

## Files

| File | Layer | Responsibility |
|---|---|---|
| `conventions.py` | 2.5 | Sign corrections + static offsets |
| `space.py` | 2 | Delta → physical servo angle + clamping |
| `posture.py` | 5 | IMU roll/pitch → foot target compensation |

---

## The Pipeline

Every control cycle passes through this sequence:

```
IK deltas (degrees, math space)
    │
    ▼  joints/conventions.py  →  apply_joint_conventions()
    │
sign-corrected + offset deltas
    │
    ▼  joints/space.py  →  normalize_all()
    │
physical servo angles (0–270°)
    │
    ▼  hardware/pca9685.py  →  set_servo_angle()
```

**Never skip a step. Never reorder.**

---

## conventions.py — Layer 2.5

### Why This Exists

The IK engine (`kinematics.py`) outputs angles in a consistent mathematical frame. But the physical robot has:
1. Servo mounts in different orientations on left vs right legs
2. `angle_corrector` in `kinematics.py` that already mirrors the thigh for left/right symmetry

`conventions.py` is the single place that reconciles these two facts so the geometric result is symmetric across all four legs.

### Sign Table

```python
JOINT_SIGN = {
    "FL_COXA": -1,   # left servo normal mount → flip to match right
    "FR_COXA": +1,
    "RL_COXA": -1,
    "RR_COXA": +1,

    "FL_THIGH": +1,  # angle_corrector already mirrors — preserve it
    "FR_THIGH": +1,
    "RL_THIGH": +1,
    "RR_THIGH": +1,

    "FL_WRIST": +1,  # angle_corrector outputs same sign both sides
    "RL_WRIST": +1,  # right mount is inverted → flip here
    "FR_WRIST": -1,
    "RR_WRIST": -1,
}
```

**Do not change any sign without re-verifying the full physical pipeline.** Each sign accounts for two compounding factors: `angle_corrector` output sign and servo mount direction. Changing one without the other doubles the error.

### Offsets

```python
JOINT_OFFSET = { "FL_COXA": 0.0, ... }   # all zero by default
```

Used to correct a static lean that cannot be fixed by recalibrating `absolute_truths.py`. Applied after sign flip, in signed-delta space.

### Usage

```python
from joints.conventions import apply_joint_conventions

corrected = apply_joint_conventions(deltas)
```

---

## space.py — Layer 2

Converts signed IK deltas (degrees, math space) into physical servo angles (degrees, hardware space).

### Formula

```
physical_angle = stand_angle + delta
physical_angle = clamp(physical_angle, min(mech_min, mech_max), max(mech_min, mech_max))
```

Stand angles and mechanical limits come directly from `hardware/absolute_truths.py` — `space.py` never hardcodes them.

### Canonical Joint Order

```python
JOINT_ORDER = [
    "FL_COXA", "FL_THIGH", "FL_WRIST",
    "FR_COXA", "FR_THIGH", "FR_WRIST",
    "RR_COXA", "RR_THIGH", "RR_WRIST",
    "RL_COXA", "RL_THIGH", "RL_WRIST",
]
```

### Usage

```python
from joints.space import normalize_all

physical = normalize_all(corrected_deltas)
# Returns dict: { "FL_COXA": 39.0, "FL_THIGH": 168.0, ... }
```

`normalize_joint(joint_name, delta)` is also available for single joints.

### Sanity Check

Run directly to verify all joints at zero delta equal their stand angles:

```bash
python joints/space.py
# FL_COXA   ->  39.0°
# FL_THIGH  -> 168.0°
# ...
```

---

## posture.py — Layer 5

Roll + pitch stabilisation using IMU feedback. Adjusts foot Z targets to keep the body level.

### How It Works

On each control cycle:

1. Read IMU → get roll, pitch, roll_rate, pitch_rate
2. Subtract boot-time reference (so flat ground = zero error)
3. Apply deadband (ignore small vibration noise)
4. Clamp to safe limits
5. Apply derivative damping using raw gyro rate
6. For each leg, compute a Z offset:
   ```
   dz = -PITCH_GAIN × leg_X × sin(pitch) − ROLL_GAIN × leg_Y × sin(roll)
   ```
7. Apply coxa splay correction for roll (abducts legs on the falling side)
8. Solve IK on the compensated foot targets

### Reference Lock

On the first call, the current IMU angles are saved as the reference. All subsequent corrections are relative to this, not to absolute zero. This means:

- Boot the robot on a slope → it will hold that slope as level
- To re-lock: call `reset_reference()` with the robot on the desired "level" surface

### Usage

```python
from joints.posture import posture_step, reset_reference

# In your control loop:
deltas = posture_step(nominal_feet, imu, swing_leg="FL")
corrected = apply_joint_conventions(deltas)
physical = normalize_all(corrected)
send_to_servos(physical)
```

`swing_leg` is the leg currently in the air (skip compensation for it). Pass `None` for static standing.

### Tuning Guide

Start with the defaults. Tune one parameter at a time, physically.

| Parameter | Default | Increase if... | Decrease if... |
|---|---|---|---|
| `PITCH_GAIN` | 0.8 | Robot nose-dives on incline | Legs oscillate fore/aft |
| `ROLL_GAIN` | 1.2 | Robot rolls on cross-slope | Legs oscillate laterally |
| `COXA_ROLL_GAIN` | 8.0 | Robot slides sideways on roll | Coxas hit limits |
| `PITCH_DAMP` | 0.08 | Pitching oscillates | Response is sluggish |
| `ROLL_DAMP` | 0.12 | Rolling oscillates | Response is sluggish |
| `DEADBAND_DEG` | 0.7 | Legs jitter at rest | Robot doesn't react to small tilts |

### Physical Test Sequence

1. Run `test_posture.py` with `POSTURE_ENABLED = False` — confirm robot stands still, terminal shows near-zero angles
2. Enable posture — tilt robot by hand ~10°, watch `dz_FL/FR/RL/RR` in terminal
3. **Roll right:** FL and RL `dz` should go negative (legs extend), FR and RR positive (legs retract). If reversed, flip `ROLL_GAIN` sign
4. **Pitch nose-down:** FL and FR `dz` should go negative (extend front), RL and RR positive. If reversed, remove the `pitch = -pitch` line
5. If oscillating at rest → raise `DEADBAND_DEG` or raise damp values
6. If too slow to react → lower damp values

### Geometry Constants

```python
BODY_LENGTH = 0.20830   # meters
BODY_WIDTH  = 0.07890

LEG_X = { "FL": +0.104, "FR": +0.104, "RL": -0.104, "RR": -0.104 }
LEG_Y = { "FL": +0.039, "FR": -0.039, "RL": +0.039, "RR": -0.039 }
```

These must match `kinematics.py` body dimensions.