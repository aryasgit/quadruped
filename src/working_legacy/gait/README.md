# gait/ — Layer 6: Gait Generator

> Produces foot targets as a function of time. No IK. No hardware. Pure trajectory math.

---

## Files

| File | Responsibility |
|---|---|
| `generator.py` | Trot gait trajectory — foot positions over time |

---

## Overview

The gait generator takes a timestamp `t` and returns a dict of four foot positions `{FL, FR, RL, RR}` in hip-local coordinates. It knows nothing about IK or servos — it only answers: *"where should each foot be at time t?"*

---

## Gait: Trot

The current implementation uses a **trot** — diagonal pairs (FL+RR, FR+RL) move in anti-phase.

```
Phase offset:
  FL, RR: 0.0    (in phase)
  FR, RL: 0.5    (anti-phase)
```

Each leg cycles through **stance** (foot on ground, body pushed forward) and **swing** (foot in the air, repositioning).

### Duty Factor

```python
DUTY_FACTOR = 0.60   # 60% of cycle is stance, 40% is swing
```

Higher duty = more stable, slower. Lower duty = faster, less stable.

---

## Trajectory

### Stance Phase (`phase < duty`)

Foot drags backward relative to body (body moves forward):

```
dx = +step_length/2 − (phase/duty) × step_length
dz = 0
```

### Swing Phase (`phase >= duty`)

Foot lifts and swings forward in a half-sine arc:

```
s  = (phase − duty) / (1 − duty)
dx = −step_length/2 + s × step_length
dz = step_height × sin(π × s)
```

---

## Parameters

Defined at the top of `generator.py`. Tune here first, not in the controller.

| Parameter | Default | Notes |
|---|---|---|
| `STANCE_X` | 0.0 m | Nominal fore/aft foot position |
| `STANCE_Y` | 0.07 m | Nominal lateral foot offset |
| `STANCE_Z` | -0.18 m | Nominal standing height |
| `STEP_LENGTH` | 0.10 m | How far each foot travels per cycle |
| `STEP_HEIGHT` | 0.045 m | Peak foot lift height |
| `DUTY_FACTOR` | 0.60 | Fraction of cycle in stance |

In `main_controller.py`, these are overridden per-direction:

| Direction | Step Length | Step Height | Duty |
|---|---|---|---|
| Forward/Backward | 0.06 m | 0.030 m | 0.80 |
| Lateral | 0.04 m | 0.020 m | 0.80 |
| Turn | 0.04 m | 0.025 m | 0.80 |

---

## Usage

```python
from gait.generator import generate_foot_targets

# t: elapsed time in seconds
# frequency: gait cycles per second (0 = stand still)
foot_targets = generate_foot_targets(
    t=1.5,
    frequency=1.5,
    step_length=0.06,
    step_height=0.030,
    duty=0.80,
)

# Returns:
# {
#   "FL": (dx, +0.07, -0.18 + dz),
#   "FR": (dx, -0.07, -0.18 + dz),
#   "RL": (dx, +0.07, -0.18 + dz),
#   "RR": (dx, -0.07, -0.18 + dz),
# }
```

If `frequency <= 0`, returns the static stand pose.

---

## Directional Variants (main_controller.py)

The `main_controller.py` implements direction-specific generators that are not in `generator.py`:

### Forward / Backward

Uses X-axis trajectory. Backward multiplies `dx` by -1.

### Lateral (Strafe)

Uses Y-axis trajectory. Left strafe reverses the phase: `leg_phase = 1.0 - leg_phase`. Right legs get opposite Y sign so all feet push outward (body moves laterally).

### Turn in Place

Left legs get `-dx × turn_mult`, right legs get `+dx × turn_mult`. Creates rotation about the body centre.

---

## Timing

The main loop runs at `DT = 0.02 s` (50 Hz). Phase is computed as:

```python
phase = (elapsed / cycle_duration) % 1.0
```

For multi-cycle moves (lateral strafe runs 2 cycles for stability), the loop accumulates phase across cycles before returning to stand.

---

## Adding a New Gait

1. Add a new `_leg_trajectory_xxx` function in `generator.py` (returns `dx, dz` or `dy, dz`)
2. Add a new `compute_feet_xxx(phase, direction, stance_z)` function in `main_controller.py`
3. Add the direction string to `KEY_MAP` and the routing logic in `compute_feet_directional()`
4. Test with `POSTURE_ENABLED = False` first on flat ground