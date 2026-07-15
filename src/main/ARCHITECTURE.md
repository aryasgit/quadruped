# Rebuild Architecture — Layer Boundaries & Contracts

Goal: a strictly layered stack where each concern lives in exactly one place, so the
redundancy that plagued the old code (triplicated constants, duplicated deadzone,
three copies of the control loop, two divergent gait configs) is structurally
impossible. **Dependencies point downward only. No layer imports a layer above it.
No constant is defined in more than one place.**

```
        ┌─────────────────────────────────────────────┐
  L6    │  io/         web server + UI + headless CLI   │  (command source)
        ├─────────────────────────────────────────────┤
  L5    │  control/    the ONE loop: modes, timing,     │  (orchestration)
        │              watchdog, safety                 │
        ├───────────────┬──────────────┬───────────────┤
  L4    │  gait/ (pure) │ estimation/  │  telemetry/    │
  L3    │ kinematics/   │  (IMU state) │  (logging)     │
        │   (pure)      │              │                │
        ├───────────────┴──────────────┴───────────────┤
  L1    │  hardware/    locked I2C bus, PCA9685, IMU    │  (only stateful/threaded)
        ├─────────────────────────────────────────────┤
  L0    │  config/      robot_spec + params (pure data) │  (single source of truth)
        └─────────────────────────────────────────────┘
```

Rule of thumb: **L3–L4 pure layers do zero I/O** (fully unit-testable), **L1 is the
only place that touches hardware or holds threads**, **L0 is the only place constants
live**.

---

## L0 — `config/` — Single Source of Truth (pure data)

**Owns:** every constant. Geometry (links, body dims, leg origins), servo channel map,
mechanical limits, stand pose, I2C addresses/registers, pulse range, AND all gait /
analog / height / body-shift parameters.

**Interface:** plain immutable data (dataclasses / frozen constants). No logic beyond
trivial derived values.

**Must NOT:** import anything above; contain control flow, math pipelines, or I/O.

**Kills these old redundancies:**
- Triplicated `STANCE_Z/Y/X` (`generator`+`solver`+`web_control`) → one `STANCE`.
- Two divergent gait-constant sets (`generator.py` vs `web_control.py`) → one set.
- `ANALOG_DEADZONE`/`apply_deadzone` duplicated in `web_control`+`web_remote` → one.
- `absolute_truths.py` grows into this layer; nothing else defines hardware facts.

> Contract: IK's stand reference and the gait's stance MUST read the *same* `STANCE`
> from here. That coupling bug is now impossible.

---

## L1 — `hardware/` — Devices (the only stateful, threaded, I/O layer)

**Owns:** the one I2C bus (with a `threading.Lock`), the PCA9685 servo driver, the
MPU6050 raw driver. All bus access is serialized and uniformly error-handled.

**Interface (down→config only):**
- `bus`: `read_byte(addr,reg)`, `write_byte(addr,reg,val)` — **lock-guarded, retry +
  catch** on transient EREMOTEIO. One policy for everyone.
- `servos.set_angle(channel, deg)` / `set_pose(dict)` — angle→pulse→write.
- `imu.read_raw()` → raw accel+gyro counts (retry+catch, same policy as servos).

**Must NOT:** know about legs, gait, IK, modes, or web. It moves bytes and converts
angle↔pulse. No trajectory math.

**Kills:** the asymmetric error handling (servos retried, IMU didn't), the unlocked
shared bus (root cause of the Errno-121 storms), and the unguarded trick writes —
because **every** write goes through one guarded path.

---

## L2 (inside L1) — angle/pulse conversion + joint conventions

Lives with hardware but is pure: `angle_to_pulse` (uses config pulse range + per-servo
travel), and the **sign/offset conventions + normalize** (stand+Δ, clamp to sorted mech
limits). This is the math→servo adapter. Kept adjacent to the driver because it's
hardware-specific, but it holds no state.

---

## L3 — `kinematics/` — Inverse Kinematics (PURE)

**Owns:** `foot (x,y,z) hip-local → joint angles (Δ from stand)`. Exact 3-DOF solve.

**Interface:** `solve_leg(x,y,z,leg) → (coxaΔ, thighΔ, wristΔ)` and
`solve_all(foot_targets) → joint dict`. Geometry injected from config.

**Must NOT:** touch hardware, IMU, config globals, or print. Reachability/`acos`/`asin`
domain-clamped and returned as a status, never crashing or spamming stdout.

**Kills:** the `acos` crash, the print-per-tick warning spam, and the leg-origin
ordering bug (one canonical leg-id↔origin table in config, consumed here).

---

## L4 — `gait/` — Gait Generator (PURE)

**Owns:** foot-path shapes and phasing. **`gait(t, command, params) → foot_targets`**
with no I/O and no hidden state — a deterministic function of time + command.

**Interface:**
- `trajectory(phase, step_len, step_h, duty) → (dx, dz)` — stance/swing shape,
  **velocity-continuous** touchdown (cycloid/smoother than the old linear drag).
- `gait(t, Command, params) → {leg: (x,y,z)}` — diagonal trot phasing (carry forward
  the *live* `(FL,RR)` vs `(FR,RL)` logic), composes fwd/strafe/turn, applies
  body-shift & pitch-bias overlays, parameterized by `stance_z` (height modes).

**Must NOT:** call servos, read IMU, or own the loop/clock. `t` is passed in.

**Kills:** the dead bound-gait (`generate_foot_targets`/`PHASE_OFFSET` deleted), the
gait logic being buried and untestable inside the control loop, and the
velocity-discontinuous swing (foot-slip contributor).

> This is the layer we'll iterate on hardest for the walk-quality fixes. Because it's
> pure, I can validate foot paths offline before ever moving a servo.

---

## L3.5 — `estimation/` — IMU State (my sensing layer)

**Owns:** turning raw IMU into usable state. Complementary filter with **measured
monotonic dt**, outputs `roll, pitch, yaw_rate, accel[xyz]`, plus derived debug
signals (tilt amplitude, lateral lean, cadence, impact events).

**Interface:** `update() → RobotState`. Consumes `hardware.imu.read_raw()`.

**Must NOT:** command servos or own modes. It observes.

**Kills:** hardcoded IMU `dt`, uncaught IMU reads in the loop. **Enables** the
IMU-as-eyes debugging.

---

## L4.5 — `telemetry/` — Logging & Analysis (my eyes' recording)

**Owns:** structured per-tick logging (t, command, foot targets, RobotState, loop dt)
to disk, and **offline analysis** (tilt mean/amplitude, yaw-drift rate, cadence FFT,
motion-onset timing, left/right symmetry).

**Interface:** `log(tick_record)` (cheap, non-blocking) + analysis scripts run after.

**Must NOT:** affect control. Pure sink + offline reader. This is the instrument I use
to debug gait against real IMU traces + Krish's ground-truth answers.

---

## L5 — `control/` — Orchestration (the ONE loop)

**Owns:** the single control loop and everything stateful about *running*: the
monotonic clock/timing, the mode state machine (walk / idle-brace / stability), the
watchdog, safe-pose on error, and wiring: `command → gait → IK → hardware`, with
`estimation` feeding balance/telemetry.

**Interface:** consumes a `Command` (from L6), pulls `RobotState` (L3.5), calls
`gait` (L4) → `kinematics` (L3) → `hardware` (L1), emits telemetry (L4.5).

**Must NOT:** define constants (imports L0), contain gait/IK math, or talk WebSockets.
Exactly one loop body — no gamepad/keyboard/web triplication.

**Kills:** the ~40% dead code and triplicated loop in `web_control.py`; the watchdog
vs trick-timeout mismatch; unguarded post-hook `stand()`.

---

## L6 — `io/` — Command Sources (web + headless)

**Owns:** translating the outside world into a `Command` struct. FastAPI + WebSocket
hub + `index.html` (keep the thread-safe hub design), AND a **headless CLI / script
source** so gait can be exercised without a phone (essential for repeatable debugging).

**Interface:** produces `Command(fwd, strafe, turn, height, mode)`; nothing else.
Deadzone/staleness handled once here.

**Must NOT:** contain robot/gait logic. It's a transport that yields commands.

**Kills:** robot logic leaking into the web layer; enables scripted, repeatable test
runs (same command every time) — critical for isolating the "random/delayed" walk bug.

---

## The `Command` contract (the spine)

One dataclass flows L6 → L5 → L4:
```
Command:
  fwd:    float  # -1..1
  strafe: float  # -1..1
  turn:   float  # -1..1
  height: enum   # HIGH/NORMAL/LOW/CROUCH
  mode:   enum   # WALK / STABILITY / IDLE
```
Every command source produces it; the loop consumes it; gait is a function of it.
No other cross-layer data shape exists.

---

## Redundancy-prevention checklist (maps old sin → new rule)

| Old problem | New structural rule |
|---|---|
| Stance consts in 3 files | Only `config/` defines constants; everyone imports |
| Two gait configs disagree | One `gait_params` in `config/` |
| Deadzone in 2 files | Handled once in `io/` |
| 3× control loop | One loop in `control/`; command sources are pluggable |
| Gait logic un-testable in loop | `gait/` is pure `f(t, cmd) → feet` |
| Unlocked bus, mixed error handling | One locked, uniformly-guarded `hardware/bus` |
| IMU dt hardcoded, reads uncaught | `estimation/` owns dt; `hardware` guards reads |
| `acos` crash, print spam | `kinematics/` pure, clamps, returns status |
| Dead gamepad/keyboard/bound code | Deleted; not ported |

---

## Proposed folder layout (`src/main/`)
```
config/        robot_spec.py  gait_params.py
hardware/      bus.py  pca9685.py  imu.py  conventions.py
kinematics/    ik.py
gait/          trajectory.py  gait.py
estimation/    imu_state.py
telemetry/     logger.py  analysis.py
control/       loop.py  command.py  modes.py  watchdog.py
io/            web_server.py  index.html  cli.py
```

Import direction is enforced by this order: a module may import only from layers
listed below it, never above. If that rule is ever hard to follow, the code is in the
wrong layer.
```
