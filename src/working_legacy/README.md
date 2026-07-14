# main_controller.py — Top-Level Run Loop

<img width="628" height="537" alt="Screenshot 2026-03-21 at 2 01 40 AM" src="https://github.com/user-attachments/assets/d82a24b4-e011-4329-a96b-c1ef19c95fbb" />


> Gamepad input → direction commands → gait execution → servo output. The only file that talks to both the gait layer and the hardware layer.

---

## Overview

`main_controller.py` is the top of the stack. It:

1. Initialises hardware (PCA9685 + IMU)
2. Connects to a gamepad via pygame
3. Reads gamepad input (blocking — waits for button press)
4. Executes one complete gait cycle or trick per button press
5. Returns to stand after each action
6. Handles height mode cycling

---

## Running

```bash
cd quadruped/src/main
python main_controller.py
```

Plug in gamepad before starting.

---

## Gamepad Map

### Movement (D-Pad)

| Button | Direction | Cycles | Notes |
|---|---|---|---|
| D-Pad Up | Forward | 1 | Standard trot |
| D-Pad Down | Backward | 1 | Reverses X trajectory |
| D-Pad Left | Turn Left | 1 | In-place rotation |
| D-Pad Right | Turn Right | 1 | In-place rotation |

### Movement (Triggers / Bumpers)

| Button | Direction | Cycles |
|---|---|---|
| (reserved) | Left strafe | 2 |
| (reserved) | Right strafe | 2 |

### Height (Y Button)

Cycles through: `HIGH (-0.19m)` → `NORMAL (-0.18m)` → `LOW (-0.15m)` → `CROUCH (-0.12m)` → repeat.

Height transitions are smoothly interpolated at `DT` rate over `0.3 s`.

### Tricks

| Button | Trick |
|---|---|
| LB | Shake |
| RB | Bheek (rear up) |
| B | Push-ups |
| A | Tilt dance |
| Left Trigger (>0.7) | Wiggle |
| Right Trigger (>0.7) | Bow |
| M1 | Sit |
| M2 | High-five |

### System

| Button | Action |
|---|---|
| X | Quit (returns to stand, exits) |

---

## Gait Configuration

```python
FREQ        = 1.5    # gait cycles/second (forward/backward)
DT          = 0.02   # control loop period (50 Hz)

STEP_LENGTH = 0.06   # meters (forward/backward)
STEP_HEIGHT = 0.030  # meters
DUTY        = 0.80   # stance fraction

LATERAL_STEP_LENGTH = 0.04
LATERAL_STEP_HEIGHT = 0.020
LATERAL_FREQ        = 1.5

TURN_STEP_LENGTH = 0.04
TURN_STEP_HEIGHT = 0.025
TURN_FREQ        = 1.5
```

---

## Key Functions

### `execute_step(feet) → bool`

Runs the full pipeline for one control frame:

```
solve_all_legs(feet)
    → apply_coxa_bias()
    → apply_joint_conventions()
    → normalize_all()
    → set_servo_angle() × 12
```

Returns `False` on pipeline error (logs warning, does not crash).

### `execute_single_cycle(direction, stance_z)`

Runs a full gait cycle (or 2 cycles for lateral). Prints a live progress bar. Returns to stand at current `stance_z` on completion.

### `compute_feet_directional(phase, direction, stance_z) → dict`

Routes to the correct trajectory function:

| Direction | Function |
|---|---|
| `"forward"`, `"backward"` | `compute_feet_forward_backward()` |
| `"left"`, `"right"` | `compute_feet_lateral()` |
| `"turn_left"`, `"turn_right"` | `compute_feet_turn()` |

### `apply_coxa_bias(deltas) → dict`

Applies a static toe-in bias (default `+1.5°` on all coxas). Applied in IK delta space before sign correction, so it is geometrically consistent across all legs.

---

## Diagonal Pairs

```python
DIAG_A = ("FL", "RR")   # phase offset 0.0
DIAG_B = ("FR", "RL")   # phase offset 0.5
```

Legs in the same diagonal group are always in phase. This implements the trot gait.

---

## Height Mode

```python
HEIGHT_MODES = [
    ("HIGH",   -0.19),
    ("NORMAL", -0.18),   # default
    ("LOW",    -0.15),
    ("CROUCH", -0.12),
]
```

Current height is tracked in `current_z` and passed to all trajectory functions. All foot targets use `stance_z` instead of the hardcoded constant, so height mode affects all movements.

---

## Error Handling

- Pipeline errors in `execute_step` are caught and logged — the loop continues
- Trick errors are caught and logged — `stand()` is called after
- `KeyboardInterrupt` (Ctrl+C) triggers clean shutdown: return to stand, exit

---

## Integrating Posture Control

The posture controller is currently not wired into the walk loop. To integrate:

1. Init IMU at startup:
   ```python
   from hardware.imu import init_mpu, calibrate, IMUFilter
   init_mpu()
   calib = calibrate()
   imu = IMUFilter(calib)
   ```

2. Replace `solve_all_legs(feet)` in `execute_step` with:
   ```python
   from joints.posture import posture_step
   deltas = posture_step(feet, imu, swing_leg=None)
   ```

3. Detect swing leg from `feet` dict (any foot with `z > STANCE_Z + 1e-4`) and pass it.

See `joints/README.md` for posture tuning guide.
