# Robot Profile & Gait Model — Rebuild Reference

Complete first-hand read of the robot's physical/electrical truths and its walking
system, extracted from `src/working_legacy/`. This is the spec to rebuild against.

---

## 1. What the robot physically is

A 12-DOF quadruped (4 legs × 3 joints: coxa / thigh / wrist), SpotMicro-style
geometry, driven by a PCA9685 over I2C from a Jetson.

### Body (from `ik/kinematics.py:21-29`)
| Dim | Value | Meaning |
|---|---|---|
| `length` | **0.2083 m** | front↔back hip spacing (X) |
| `width`  | **0.0789 m** | left↔right hip spacing (Y) |
| `hight` *(sic)* | 0.074 m | body height (declared, unused in IK) |

Leg hip origins (`kinematics.py:32-36`), body frame, from center:
`LF (+L/2,+W/2)`, `LB (-L/2,+W/2)`, `RB (-L/2,-W/2)`, `RF (+L/2,-W/2)`.
⚠️ These rows are ordered `LF,LB,RB,RF` but `LEG_ID` is `FL=0,RL=1,FR=2,RR=3`
(`solver.py:31-36`) → right-leg origins are index-swapped front↔back. Harmless for
pure translation (origin added then subtracted), **wrong under body rotation** — fix
before any posture/rotation work.

### Leg links (`kinematics.py:21-23`)
| Link | Value | Joint |
|---|---|---|
| `link_1` | **0.01605 m** | coxa (hip lateral offset) |
| `link_2` | **0.10832 m** | thigh (femur) |
| `link_3` | **0.13476 m** | wrist (tibia) |

Max straight-leg reach = `link_2 + link_3` = **0.24308 m**. At nominal stand
(Z = −0.18) the leg is well inside reach (~74% extended) — healthy workspace margin.

---

## 2. Actuation (from `hardware/absolute_truths.py`)

### I2C / electrical
- Bus **7**, PCA9685 @ **0x40**, MPU6050 @ **0x68**.
- PWM: prescale `int(25e6/(4096·50)-1)=121` → **50 Hz** servo frame (`pca9685.py`).
- Pulse range **106 … 535** counts mapped over an **assumed 270° travel**
  → resolution ≈ **1.59 counts/°** (≈0.63°/count). ⚠️ 270° is assumed for *all*
  joints, but wrist mech range is 0–200° and coxa 0–90° (below) — only correct if
  every servo is physically the same 270° unit. **Verify against datasheets.**

### Servo channel map (12 channels, all of 0–11 used)
| Joint | FR | FL | RR | RL |
|---|---|---|---|---|
| **Coxa**  | 6 | 7 | 0 | 1 |
| **Thigh** | 8 | 9 | 2 | 3 |
| **Wrist** | 10 | 11 | 4 | 5 |

### Mechanical limits + "perp" (measured absolute servo degrees)
Left and right sides are mounted **mirrored** — right-side `min>max` (e.g. wrist R
`200→0`). The pipeline sorts before clamping (`space.py`), so inversion is handled.
`perp` = the servo angle at which that link is mechanically perpendicular (reference).

| Joint | Left min/max/perp | Right min/max/perp |
|---|---|---|
| Wrist | 0 / 200 / 153 | 200 / 0 / 47 |
| Thigh | 0 / 270 / 128–135 | 270 / 0 / 138–140 |
| Coxa  | 0 / 90 / 39–50 | 90 / 0 / 44–47 |

### Stand pose (measured absolute servo degrees) — the calibration anchor
| Joint | FR | FL | RR | RL |
|---|---|---|---|---|
| Wrist | 122 | 78 | 122 | 78 |
| Thigh | 98 | 168 | 100 | 175 |
| Coxa  | 47 | 39 | 44 | 50 |

These are the physical "home" angles. Everything downstream is expressed as a
**delta from stand**: IK is computed once at stand (`solver._STAND_REF`), and live
targets output `(angle − stand_angle)` in degrees, which `space.normalize_joint`
adds back onto `*_STAND` and clamps to mech limits. Clean, drift-free model — keep it.

---

## 3. Coordinate & convention model

- **Foot targets are hip-local** `(x, y, z)` in meters. `x` = fore/aft (+forward),
  `y` = lateral (+left; legs use `±STANCE_Y`), `z` = vertical (**negative down**).
- Nominal stance foot: `X=0, Y=±0.07, Z=−0.18`.
- Pipeline: `foot (x,y,z) → solve_all_legs (IK, Δdeg) → apply_coxa_bias →
  apply_joint_conventions (sign/offset) → normalize_all (stand+Δ, clamp) →
  set_servo_angle`.
- ⚠️ **The stance constants are triplicated and must stay identical:**
  `generator.py:8-10`, `solver.py:43-45`, `web_control.py:99-101` all independently
  define `Z=−0.18, Y=0.07, X=0`. If they ever diverge, IK deltas silently break.
  **Single source of truth in the rebuild.**

---

## 4. The gait model (this is the important part)

### 4a. Trajectory shape — `_leg_trajectory` (`gait/generator.py:34-56`)
One planar foot path, parameterized by `(phase∈[0,1), step_length, step_height, duty)`:
- **Stance** (`phase < duty`): `dx = +L/2 − s·L`, `dz = 0` — foot drags backward
  linearly (body moves forward). `s = phase/duty`.
- **Swing** (`phase ≥ duty`): `dx = −L/2 + s·L`, `dz = H·sin(π·s)` — foot returns
  forward with a half-sine lift. `s = (phase−duty)/(1−duty)`.
- Position is C0-continuous across the boundary; **velocity is not** (instant drag
  reversal at touchdown → impact/scrub). A cycloid/smoother swind-down fixes this in
  the rebuild.

### 4b. Phasing — the LIVE gait is a proper diagonal TROT
In `compute_feet_analog` (`web_control.py:497-501`): `DIAG_A=(FL,RR)`. Default order:
`FL,RR → phase`; `FR,RL → phase+0.5`. Diagonal pairs move together, 50% out of phase
= textbook trot. `should_mirror_diagonal_order` swaps which diagonal swings first for
left/turn-left commands (cosmetic swing-order symmetry).

> ⚠️ **Do not confuse with `generator.py`'s `PHASE_OFFSET` (`FL=FR=0, RL=RR=0.5`) —
> that's a BOUND and it is DEAD CODE** (only used by `generate_foot_targets`, which
> the live controller never calls). The live controller imports only the *shape*
> function `_leg_trajectory` and does its own (correct) trot phasing. Carry the trot
> logic forward; delete `generate_foot_targets`.

### 4c. Gait parameters (LIVE values, `web_control.py`)
| Param | Value | Notes |
|---|---|---|
| `FREQ` (fwd/back) | **1.3 Hz** | nominal cycle rate |
| `DT` | **0.0134 s** | loop period. ⚠️ comment says "50 Hz" but 1/0.0134 ≈ **74.6 Hz** — comment is wrong |
| `STEP_LENGTH` (fwd) | **0.080 m** | stride in X |
| `STEP_HEIGHT` (fwd) | **0.030 m** | swing lift |
| `DUTY` | **0.70** | 70% stance / 30% swing (statically safer than 0.6) |
| Backward length scale | **0.86** | shorter strides in reverse |
| Backward height scale | **1.08** | slightly higher lift in reverse |
| `LATERAL_STEP_LENGTH` | 0.03 m | strafe stride (Y) |
| `LATERAL_STEP_HEIGHT` | 0.018 m | |
| `LATERAL_FREQ` | 1.2 Hz | |
| `TURN_STEP_LENGTH` | 0.07 m | yaw stride (X, opposite L/R) |
| `TURN_STEP_HEIGHT` | 0.032 m | |
| `TURN_FREQ` | 1.2 Hz | |

> ⚠️ **Second divergent constant set:** `gait/generator.py:15-17` defines
> `STEP_LENGTH=0.10, STEP_HEIGHT=0.045, DUTY=0.60` — **different** from the live
> values above and used only by the dead `generate_foot_targets`. Two gait configs
> that disagree. Collapse to one in the rebuild.

### 4d. How strafe & turn compose (`compute_feet_analog:505-535`)
`fwd`, `strafe`, `turn` each contribute a trajectory; they **sum** into `dx_total`/
`dy_total`, and `dz` takes the **max** lift across active components (so a leg lifts
once even for combined motion):
- **fwd**: `_leg_trajectory` in X, sign-flipped for backward.
- **strafe**: `_lateral_trajectory` in Y (`web_control.py`), side sign per leg,
  phase reversed for left.
- **turn**: `_leg_trajectory` in X but **left legs +, right legs −** → yaw.

### 4e. Analog stick → gait mapping (continuous drive)
Stick magnitude scales both cadence and stride:
- Frequency: `ANALOG_FREQ_MIN 0.5 … ANALOG_FREQ_MAX 2.2 Hz`.
- Step scale: `ANALOG_STEP_MIN 0.20 … ANALOG_STEP_MAX 0.8`.
- Deadzone `0.15`; command ramp-up `5.0`, ramp-down `4.0` (units/s of the ramp env).
- Axes: `LX=0` (strafe), `LY=1` (fwd), `RX=3` (turn).

### 4f. Single-cycle (button) execution (`execute_single_cycle:567`)
One discrete cycle per press: fwd/back 1 cycle @ 1.3 Hz, lateral **2** cycles @ 1.2 Hz,
turn 1 cycle @ 1.2 Hz. Wrapped in `ramp_factor` (sine ease-in/out over `RAMP_TIME
0.15 s`, floor `RAMP_MIN 0.05`) so it accelerates from and settles to stand.

---

## 5. Stabilizing/aesthetic overlays on the gait

| Term | Value | Effect |
|---|---|---|
| `BODY_SHIFT_FWD` | 0.010 m | shifts body **back** when driving forward (CoM over support) |
| `BODY_SHIFT_LAT` | 0.008 m | lateral CoM shift for strafe |
| `BODY_SHIFT_TURN` | 0.012 m | lateral CoM shift for yaw |
| `MOTION_PITCH_BIAS` | 0.006 m | nose-down pitch scaled by motion magnitude |
| `BACKWARD_PITCH_BIAS` | +0.010 m | extra pitch when reversing |
| `COXA_DELTA_BIAS` | +1.5° all | static toe-in |
| `LATERAL_COXA_OPPOSE_GAIN` | 2.5° | dynamic coxa tilt opposing strafe |

## 6. Height modes (`HEIGHT_MODES`)
`HIGH −0.19` · **`NORMAL −0.18` (default, index 1)** · `LOW −0.15` · `CROUCH −0.12`
(meters, foot Z). Transitions interpolate over `HEIGHT_TRANSITION 0.3 s`. All gait
math takes `stance_z` as a parameter, so height changes apply to every motion.

---

## 7. Rebuild guidance for the gait/robot layer

**Keep:** the trajectory-shape + trot-phasing design, the stand-relative delta IK
model, the `stance_z`-parameterized gait, the analog stick→(freq, stride) mapping,
the body-shift/pitch-bias overlays (they visibly help stability), `absolute_truths`
as the hardware spec.

**Fix / consolidate:**
1. **One config, one source of truth.** Collapse the two divergent gait-constant sets
   (`generator.py` vs `web_control.py`) and the **triplicated stance constants**
   (`generator`/`solver`/`web_control`) into a single config module. IK's stand
   reference must read the *same* stance the gait uses.
2. **Delete dead gait code:** `generate_foot_targets` (bound gait) and its
   `PHASE_OFFSET`; keep only `_leg_trajectory` (renamed/relocated).
3. **Velocity-continuous swing** (cycloid or smootherstep touchdown) to kill the
   drag-reversal impact at foot strike.
4. **Correct `DT`/frequency truth** — decide the real loop rate; the "50 Hz" comment
   is wrong (0.0134 s ≈ 74.6 Hz). Drive timing from a measured monotonic clock.
5. **Fix leg-origin ordering** (`kinematics.py` rows vs `LEG_ID`) before enabling body
   rotation, and re-verify the 270° servo-travel assumption against real datasheets.
6. Make the gait a pure function of `(t, cmd, params)` with no I/O, so it can be unit
   tested against expected foot paths — the current logic is sound but buried inside
   the control loop.

---

## 8. Quick numeric reference card
```
Body:    L=0.2083  W=0.0789  H=0.074 m
Links:   coxa=0.01605  thigh=0.10832  wrist=0.13476 m   (reach=0.24308)
Stance:  X=0  Y=±0.07  Z=-0.18 m
Trot:    FREQ=1.3Hz  STEP_L=0.080  STEP_H=0.030  DUTY=0.70
Lateral: FREQ=1.2Hz  STEP_L=0.030  STEP_H=0.018
Turn:    FREQ=1.2Hz  STEP_L=0.070  STEP_H=0.032
Analog:  freq 0.5-2.2Hz  step 0.20-0.8  deadzone 0.15
Heights: HIGH -0.19 | NORMAL -0.18 | LOW -0.15 | CROUCH -0.12
Servo:   50Hz PWM  pulse 106-535 / 270deg  (1.59 counts/deg)
I2C:     bus7  PCA=0x40  MPU=0x68
```
