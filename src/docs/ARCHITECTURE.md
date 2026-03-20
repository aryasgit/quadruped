# Architecture Reference

> Layer-by-layer breakdown of the control stack. Read this before modifying any layer.

---

## Design Principles

1. **Each layer has one job.** A layer that does IK does not touch hardware. A layer that writes to servos does not know what a foot is.

2. **Dependencies flow downward only.** Layer 6 may call Layer 5. Layer 5 may call Layer 3. Layer 3 never calls Layer 6.

3. **All physical constants live in Layer 0.** No angle, channel number, or limit appears in any other file. If it does, that is a bug.

4. **Math space is unitless.** Above Layer 2, all angles are deltas from stand. Zero means "same as standing." The physical world starts at Layer 2.

5. **Layers are independently testable.** You can run `joints/space.py` directly. You can import `ik/solver.py` in a notebook. You can swap the gait generator without touching IK.

---

## Layer Map

```
Layer 6  ─── gait/generator.py
              "Where should the feet be at time t?"
              Output: foot targets {leg: (x, y, z)} in meters, hip-local

Layer 5  ─── joints/posture.py
              "Adjust foot targets to compensate for tilt"
              Input: foot targets + IMU angles
              Output: compensated foot targets (same format)

Layer 3  ─── ik/solver.py  →  ik/kinematics.py
              "What joint angles reach this foot position?"
              Input: hip-local (x, y, z)
              Output: joint deltas {FL_COXA: deg, FL_THIGH: deg, ...}

Layer 2.5 ── joints/conventions.py
              "Correct for servo mount direction"
              Input: IK deltas
              Output: sign-corrected + offset deltas

Layer 2  ─── joints/space.py
              "Convert math deltas to physical servo angles"
              Input: corrected deltas
              Output: servo angles {joint: degrees, 0–270}

Layer 1.3 ── hardware/imu.py
              MPU6050 read, complementary filter

Layer 1.2 ── hardware/pca9685.py
              Angle → PWM → I2C write

Layer 1.1 ── hardware/i2c_bus.py
              Shared SMBus singleton

Layer 0  ─── hardware/absolute_truths.py
              ALL physical constants
```

---

## Data Types at Each Boundary

| Boundary | Type | Units | Notes |
|---|---|---|---|
| Layer 6 → 5 | `dict[str, tuple[float,float,float]]` | meters | `{"FL": (x,y,z), ...}` |
| Layer 5 → 3 | same | meters | modified z values |
| Layer 3 → 2.5 | `dict[str, float]` | degrees | keys like `"FL_COXA"` |
| Layer 2.5 → 2 | `dict[str, float]` | degrees | same keys, signs flipped |
| Layer 2 → 1.2 | `dict[str, float]` | degrees (0–270) | physical servo space |
| Layer 1.2 → servo | int | PWM counts (106–535) | PCA9685 12-bit |

---

## The Sign Problem

The hardest thing to understand in this codebase is why `joints/conventions.py` has the signs it does. Here is the full explanation.

### The two sources of asymmetry

**Source 1: `kinematics.py` `angle_corrector`**

For the thigh joint, `angle_corrector` outputs **opposite signs** for left vs right legs for the same foot target. That is, if you ask FL and FR to go to the same foot position, the thigh delta for FL is `+X` and for FR is `-X`. This is intentional — it accounts for the physical mirror symmetry of the leg geometry.

**Source 2: Servo mount direction**

Right-side servos are mounted in the opposite orientation to left-side servos. This is encoded in `absolute_truths.py` as `min > max` for right-side joints. The `normalize_joint` function in `space.py` applies the delta directly to the stand angle — it does not know or care about mount direction.

### How conventions.py resolves this

For **thighs**: both sources cancel. `angle_corrector` already mirrors the output, and `space.py` adds the delta directly to stand — the servo mount inversion (encoded in min/max order) then makes the physical motion symmetric. So all thigh signs are `+1`. Do not change them to `-1` — that would double-invert.

For **coxas**: `angle_corrector` outputs the same sign for both sides, but the servo mounts are opposite. So left coxas need a sign flip (`-1`) to match right coxas (`+1`).

For **wrists**: `angle_corrector` outputs the same sign for both sides, and the right wrist servo is inverted (min > max). Flipping the sign here (`-1` for right wrists) compensates.

### Rule of thumb

If a correction is applied twice — once in `angle_corrector` and once in `conventions.py` — the net effect is zero (double inversion). The table was built by physically verifying each joint moves in the correct geometric direction. Do not modify without re-verifying on hardware.

---

## IMU Coordinate Frame

The MPU6050 is mounted flat inside the body. The filter outputs:

- **roll** — rotation about the X (forward) axis. Positive = tilt left (left side up)
- **pitch** — rotation about the Y (lateral) axis. The sign convention depends on mounting orientation and is corrected by `pitch = -pitch` in `posture.py`

The posture controller applies a sign convention correction internally. If after tuning the compensation is in the wrong direction, flip the appropriate sign in `posture.py` — not in `imu.py`.

---

## Loop Timing

The main controller runs at `DT = 0.02 s` (50 Hz). Each iteration:

1. Read gamepad (~0 ms, non-blocking poll)
2. Compute foot targets (~0.1 ms)
3. Solve IK — 4 legs (~2 ms)
4. Apply conventions + normalize (~0.1 ms)
5. Write 12 servo channels via I2C (~8 ms at 400 kHz bus)

The I2C writes are the bottleneck. At 400 kHz, each write is ~0.5 ms × 4 bytes × 12 channels = ~24 ms theoretical. In practice SMBus2 overhead brings this to ~8–12 ms. This means 50 Hz is achievable but tight.

If the loop runs slow, reduce logging and remove any debug prints from the hot path.

---

## Adding a New Feature

### New gait direction

→ Add trajectory function in `gait/generator.py` or `main_controller.py`, add to `compute_feet_directional()`, add key binding in `KEY_MAP`.

### New trick

→ Add function in `stance/tricks.py`, register in `TRICKS`, add button in `main_controller.py`.

### New sensor

→ Add driver in `hardware/` (new file, no changes to existing hardware files). If it affects foot targets, wire it into Layer 5 or 6 — not into IK.

### Change body geometry

→ Update `hardware/absolute_truths.py` for link lengths and `kinematics.py` for `link_1/2/3` and `leg_origins`. Re-run stand reference computation.

### Change a stand angle

→ Update `COXA_STAND` / `THIGH_STAND` / `WRIST_STAND` in `absolute_truths.py`. The IK solver recomputes `_STAND_REF` at import time — no other changes needed.