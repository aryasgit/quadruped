# stance/ — Tricks Library

> Crowd-pleasing scripted moves. Uses the same full pipeline as the walk controller.

---

## Files

| File | Responsibility |
|---|---|
| `tricks.py` | All trick implementations + interactive menu |

---

## Pipeline

Every trick drives the robot through the same pipeline used by the walk controller:

```python
# In _send(feet):
deltas  = solve_all_legs(feet)
conv    = apply_joint_conventions(deltas)
phys    = normalize_all(conv)
for joint, angle in phys.items():
    set_servo_angle(SERVO_MAP[joint], angle)
```

Tricks never write to servos directly. They only compute foot target dicts and call `_send()`.

---

## Tricks

| Name | Description | Duration |
|---|---|---|
| `shake` | Shift weight left, lift FR paw, 3× up-down shake | ~5 s |
| `bow` | Front legs retract (chest down), rear stays up | ~3 s |
| `wiggle` | Rear alternates RL forward / RR forward (butt wag) | ~2 s |
| `pushups` | All legs squat low then extend high, N reps | ~3 s |
| `bheek` | Tip backward onto hind legs, front paws pump, pan | ~8 s |
| `high_five` | Rear up + extend FR paw, 3× wave, recovery | ~8 s |
| `sit` | Rear legs retract, front stays extended | ~4 s |
| `stretch` | Front drops forward, rear pushes back | ~4 s |
| `tilt_dance` | Roll left-right rhythmically | ~3 s |
| `combo` | Chain: bheek → bow → wiggle → pushups → stretch → tilt → sit → shake | ~40 s |

---

## Utility Functions

### `_neutral_feet()`

Returns the nominal standing foot positions (uses `STANCE_Y` and `STANCE_Z` from `leg_ik.py`).

### `_send(feet)`

Runs the full pipeline for a given foot target dict. Any trick that moves the robot calls this.

### `_transition(start_feet, end_feet, duration=0.5)`

Smooth cubic interpolation between two foot-position dicts. Uses `_smooth(t) = t²(3 − 2t)` for ease in/out. Update rate: 40 Hz (`DT = 0.025 s`).

### `_lerp_feet(a, b, t)`

Linear interpolation between two foot dicts. Used internally by `_transition`. `t` is clamped to `[0, 1]`.

### `stand()`

Drives all feet to neutral, waits 0.3 s.

---

## Running

### Interactive Menu

```bash
python -m stance.tricks
```

Prints a numbered menu. Enter number or name. Enter `0` or `q` to quit.

### Single Trick

```bash
python -m stance.tricks --trick shake
```

### Demo (all tricks in sequence)

```bash
python -m stance.tricks --demo
```

Runs `stand()` then `combo()`.

### List Available Tricks

```bash
python -m stance.tricks --list
```

---

## Trick Design Notes

### Weight Shifting Before Lifting

Tricks that lift a foot always shift the centre of mass first. Example from `shake`:

```
Step 1: shift weight LEFT (FL out + down, FR in, rear left)
Step 2: lift FR paw
Step 3: shake
Step 4: return to weight-shifted position
Step 5: return to neutral
```

Skipping step 1 causes the robot to tip when the foot leaves the ground.

### `bheek` Recovery

After rearing up on hind legs, the recovery sequence is:
1. Snap to crouch (all legs retracted, `z + 0.05`)
2. Smooth transition to half-height (`z + 0.025`)
3. Smooth transition to full stand

This avoids the torque spike of going directly from rear-up to full stand.

---

## Adding a New Trick

1. Define a function `def my_trick():` in `tricks.py`
2. Build foot target dicts using `_neutral_feet()` as base
3. Use `_transition(from, to, duration)` for smooth motion
4. Use `_send(feet)` for instant snaps (rare — only for controlled drops)
5. Register in `TRICKS` dict:
   ```python
   TRICKS["my_trick"] = (my_trick, "Description of what it does")
   ```
6. If it needs a gamepad button, add the key mapping in `main_controller.py`

---

## Known Issues

- `tricks.py` currently imports from old paths (`IK.leg_ik`, `layer2.joint_conventions`, etc.). Update to current paths (`ik.solver`, `joints.conventions`, `joints.space`) before running.
- `bheek` recovery is sensitive to battery voltage — low voltage causes the snap-to-crouch to fail silently.