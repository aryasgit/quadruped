# Working-Legacy Web-Control Stack — Full Engineering Insights

Deep-read analysis of the verified web-control codebase at `src/working_legacy/`
(the old `src/main`). Scope = the 17-file runtime stack that `web_control.py` uses.
Line references are `file:line` within `src/working_legacy/`.

---

## 0. Executive summary

The system is a **phone/browser-driven quadruped controller**: FastAPI + WebSocket
hub → a 50 Hz control loop → IK pipeline → PCA9685 servos, with an MPU6050 IMU
feeding a "brace" (reactive) and a "stability" (posture-hold) mode.

**Overall verdict:** the *architecture* is genuinely good — clean layering, a
single-source-of-truth constants module, a thread-safe web hub, and a correctly
centralized servo pipeline. The *robustness* is where it hurts: almost every real
problem traces back to **one shared, unlocked I2C bus** and **inconsistent error
handling around it**. Servo writes retry; IMU reads and trick writes do not; a
"stopped" trick thread keeps writing. Layered on top are a handful of real
math/control defects (an `acos` that can crash, a gait that isn't actually a trot,
a stability integrator that can wind up) and a large amount of dead/duplicated code
(~40% of `web_control.py` is unreachable).

**If you rebuild from here, the three things to carry forward** are the layering,
the `absolute_truths` pattern, and the `WebControlHub`. **The three things to fix
at the foundation** are: lock the I2C bus, make error handling uniform across all
bus users, and drive all timing/`dt` from a real monotonic clock.

---

## 1. Architecture map (how it actually runs)

- **Entry:** `main()` (`web_control.py:1324`) → inits `init_pca`/`init_mpu`/`calibrate`,
  builds `BraceController`, starts `WebControlHub` server thread + watchdog thread,
  then runs **`main_web`** (the only live loop; `main_gamepad`/`main_keyboard` are dead).
- **Threads:** main control loop (~50 Hz) · watchdog (1 Hz daemon) · uvicorn server
  (daemon) · transient per-trick daemon threads.
- **Input flow:** browser `index.html` → WS `/ws` → `RemoteState` (under a `Lock`)
  → control loop pulls `poll_command()` (buttons, FIFO) + `read_sticks()` (analog).
- **Modes:** *Normal* (analog gait + button tricks; idle → brace hold) · *Brace*
  (IMU shove-reaction while idle) · *Stability* (exclusive IMU posture hold, `'p'`).
- **Pipeline:** foot targets → `solve_all_legs` → `apply_coxa_bias` →
  `apply_joint_conventions` → `normalize_all` → `set_servo_angle ×12`, centralized in
  `execute_step` (`web_control.py:317`).

Layering (clean): `absolute_truths` (L0 constants) → `i2c_bus` (bus owner) →
`pca9685`/`imu` (drivers) → `ik/*` (math) → `joints/*` (math→servo adapter).

---

## 2. What's working well (keep these)

- **Thread-safe web hub.** `WebControlHub` (`web_remote.py:47-117`) guards all
  `RemoteState` access under one lock — clean producer/consumer split between the
  async WS thread and the sync loop. Strongest-engineered part of the system.
- **Centralized, guarded servo pipeline.** `execute_step` (`web_control.py:317-330`)
  wraps the whole IK→servo chain in one try/except and returns `False` on failure —
  the *correct* place for the Errno-121 guard (walk/brace paths are safe because of it).
- **Watchdog does in-place recovery, not `os.execv`.** (`web_control.py:204-215`) —
  far safer for a live robot; heartbeat stamped inside stability loops too.
- **Jerk-free stability exit.** `smooth_stand` (`:342-363`) lerps live `_sc._dz` to zero.
- **`BraceSuspended` context manager** (`:444-459`) guarantees brace reset via `finally`.
- **Single-source-of-truth L0** (`absolute_truths.py`) — channel maps, mech limits,
  stand poses, registers as pure constants, no logic. Exactly right.
- **Bus singleton** (`i2c_bus.py:23-33`) — correct lazy init, one handle shared.
- **Bounded Errno-121 retry that re-raises on persistent fault** (`pca9685.py:45-54`) —
  well-reasoned; absorbs transient NAKs without hiding real hardware failures.
- **IK numerical guards (partial):** `asin` clamp + `len_A` floor (`kinematics.py:64,71-73`),
  reachability soft-clamp (`:101-104`), non-mutating `angle_corrector` (`:181-182`),
  inverted-limit-safe clamp via sort (`space.py:109-115`).
- **`brace_controller.py` is reference-quality:** dt clamping, alpha clamping,
  hysteresis (`:264-278`), deadband snap-to-zero, final safety clamp, immutable
  telemetry, a real `reset()`. Model the rebuild's controllers on this file.
- **Stand-relative delta posture model** (`solver.py`) — compute IK once at stand,
  output deltas: clean and drift-free.

---

## 3. Critical issues (crash / unsafe-robot)

### C1 — Tricks bypass the Errno-121 guard and can crash the whole loop
`stance/tricks.py:73-81` (`_send`) writes servos in a raw loop with **no try/except**;
every trick funnels through it (`_transition:109`, `stand:115`, recovery crouches
`:320,:388`). A single I2C NAK mid-trick propagates out and, because
`TrickRunner.run`'s **post-hook `stand()` runs unguarded on the main thread**
(`web_control.py:434`), it crashes `main_web` (whose try only catches
`KeyboardInterrupt`, `:1040`). `send_to_servos` (`:312`) is only safe when reached via
`execute_step`. **Worst case:** an error during a reared-up trick (`bheek`, `high_five`,
`stretch`) leaves the robot balanced on two legs with the control loop dead — there is
no `try/finally` to command a safe pose. *(This is the historical fragility; the
driver-level retry reduces its frequency but does not remove the class.)*

### C2 — "Stopped" trick thread keeps writing the bus (race reopened)
`run_trick_with_timeout` (`web_control.py:366-397`) on timeout sets `_trick_stop`,
sleeps `DT*2`, then **clears `_trick_stop` and returns** while the timed-out daemon
thread is *still running* `trick_func`. Clearing the gate re-enables that zombie
thread's `execute_step` writes concurrently with the main thread → the exact I2C race
the Event was meant to prevent, now open for the whole remaining life of the runaway trick.

### C3 — Unclamped `acos` in the IK solve (hard crash on marginal targets)
`kinematics.py:110-111` feeds `acos` with no domain clamp (contrast the careful `asin`
clamp at `:71-73`). For a target with `len_B < |l2−l3|`, the argument exceeds 1 →
`ValueError: math domain error`, which propagates into the gait loop. Most likely
runtime crash for out-of-workspace / near-singular foot targets.

### C4 — Shared I2C bus has no lock (corruption under mixed load)
`smbus2.SMBus` is not thread-safe, yet IMU reads and servo writes interleave on one
handle from different threads (`i2c_bus.py` has no `Lock`; the retry comment at
`pca9685.py:36-40` even acknowledges "rapid mixed IMU-read/servo-write load"). This is
the **root cause** of the Errno-121 NAK storms: concurrent transactions tear each
other, including `_safe_read_word`'s two non-atomic byte reads. Retries treat the
*symptom*; the missing lock is the *cause*.

---

## 4. High-priority correctness issues

### H1 — IMU error handling is asymmetric and can stall the loop
`_safe_read_word` raises bare `IOError` on EREMOTEIO with **no retry** (`imu.py:49-50`);
`calibrate` catches it but `IMUFilter.update` does **not** — a single transient NAK
during normal operation throws straight out of the control loop. Servo writes get 3
retries; IMU reads get zero. (Pairs with C4.)

### H2 — All `dt` is hardcoded, decoupled from real loop rate
IMU complementary filter uses fixed `dt=0.02` (`imu.py:129,155-156`); the stability
controller uses fixed `dt=0.02` (`stability_controller.py:187`). Actual loop period
varies (Jetson load, blocking sleeps). Gyro integration and correction gain are
therefore mis-scaled whenever the loop isn't exactly at the assumed rate → biased
fused attitude and timing-dependent control gain. `brace_controller` does it right
(measures dt at `:236-239`) — copy that everywhere.

### H3 — Stability controller can wind up and self-oscillate
`stability_controller.py` is a **sign-based integrator with no leak/decay** — `_dz`
only accumulates by the sign of static tilt (`:228-241`). With sensor bias or a
persistent small tilt just above the 0.5° deadband, `_dz` ramps to `±MAX_DZ` and pins
there; near the deadband it limit-cycles (bang-bang hunting). Worst case it tilts the
body and *creates* the tilt it's correcting → sustained oscillation. Also uses
**global mutable module state** (`:129-134`) — not reentrant, not thread-safe. Brace's
decay-to-zero (`:311`) is the missing ingredient.

### H4 — The gait is a bound, not a trot; touchdown is velocity-discontinuous
`gait/generator.py:22-27` phase offsets are `FL=FR=0.0, RL=RR=0.5` → front pair and
rear pair each move together (a **bound/pronk**), not a diagonal trot (docstring `:20`
admits "demo: all legs same arc"). Statically unstable at low speed. Separately, X
position is continuous across stance↔swing but **X velocity is not** — the foot
instantly reverses drag direction at touchdown → impact/jerk and foot scrub. And
**no turn or lateral gait exists** — `y` is a fixed per-leg constant (`:81`); there is
no yaw term. (The web UI has a turn axis with no gait behind it.)

### H5 — Leg-origin ordering is swapped for the right legs
`kinematics.py:32-36` `leg_origins` are ordered `(LF, LB, RB, RF)` but `LEG_ID` uses
`FR=2, RR=3`, so `legID=2` (FR) indexes right-back and `legID=3` (RR) indexes
right-front. Cancels out for translation-only motion (origin added then subtracted),
so straight-line gaits are fine — but **any body roll/pitch/yaw or center offset
applies the wrong lever arm to the right legs.** Latent; will bite the moment posture
uses body rotation.

### H6 — Browser: releasing one joystick zeroes *all* axes
`index.html:243-248` — both joysticks share one `axes` object and `stop()` resets
`{fwd, strafe, turn}` wholesale. Hold left stick (walk forward) + tap-release the right
stick → the robot stops. Very user-visible.

### H7 — No WebSocket reconnection in the browser
`index.html:170,179` — on `ws.onclose` the UI shows "disconnected" and never retries. A
transient Wi-Fi hiccup permanently kills control until manual page reload — serious for
a phone-driven robot.

---

## 5. Medium issues

- **M1 — Watchdog (5 s) is shorter than trick timeout (10 s).** Long tricks/combos
  don't stamp the heartbeat during `t.join(timeout)` (`web_control.py:387`), so they
  trip `recovery_requested` mid-trick → a spurious `smooth_stand` recovery on return.
  `execute_single_cycle` (`:567`) and `transition_height` (`:604`) also don't stamp
  (safe only while their durations stay < 5 s).
- **M2 — Uvicorn startup failures are swallowed.** `start` (`web_remote.py:119-136`)
  runs `server.run` in a daemon thread with a `sleep(0.5)` "readiness" race; a bind
  failure dies silently and `main()` prints the URL as if up.
- **M3 — Unguarded hardware init in `main()`** (`web_control.py:1330-1337`) — a startup
  hiccup crashes before the server/watchdog exist.
- **M4 — Calibration assumes perfectly flat.** `imu.py:100-114` subtracts full mean of
  `ax`/`ay` as bias (only `az` keeps gravity via `-16384`); calibrating on a tilt bakes
  that tilt in as "level."
- **M5 — `sleep`-based trick timing drifts and blocks.** `_transition` uses
  `time.sleep(DT)` with `steps = int(duration/DT)` (`tricks.py:104-110`) — truncation +
  per-call overhead makes every trick run long and stall any shared thread.
- **M6 — Mode-transition state bleed.** `stability._dz` and `brace._dz` are independent
  globals with no coordinated handoff; switching modes without `reset_reference()`/
  `brace.reset()` carries stale offsets in.
- **M7 — No auth/origin check on `/ws`** (`web_remote.py:152`) and plaintext `ws://` —
  anyone on the LAN can drive the robot. (Acceptable for a lab LAN; note it.)
- **M8 — Stability confirm window drops balancing.** During the 5 s exit-confirm loop
  (`web_control.py:859-865`) it runs plain `execute_step(stand)` with no posture
  correction — up to 5 s of no active balancing.
- **M9 — GYRO/ACCEL full-scale never configured.** `init_mpu` never writes
  `GYRO_CONFIG`/`ACCEL_CONFIG` (`imu.py:58-67`); the `131`/`16384` scale factors only
  hold because they match power-on defaults. Fragile against a warm reset.
- **M10 — 270° travel assumption.** `angle_to_pulse` maps every joint over 0–270°
  (`pca9685.py:100`) but wrists are 0–200° and coxa 0–90° — correct only if all servos
  are physically identical 270° units. Verify against datasheets.
- **M11 — `print`-based fault spam.** Far-target warning (`kinematics.py:104`) prints
  every control tick when a foot is over-extended.

---

## 6. Maintainability / smells

- **~40% of `web_control.py` is dead code:** `main_gamepad` (`:1048`), `main_keyboard`
  (`:1209`), non-web `run_stability_mode` (`:732`), `poll_buttons`/`poll_dpad`, keyboard
  `get_key_*` termios paths (`:693-719`), the `pygame` import. The control-loop body is
  **triplicated** across the three `main_*` functions — every fix must be applied 3×.
- **Wrong-file docstrings / stale provenance:** `web_control.py:2` says
  `super_controller_v1.py`; `tricks.py`/`generator.py`/`stability_controller.py` headers
  cite `Parth.Tricks`, `layer6/…`, `joints/stability_controller.py` paths that don't
  match reality. "FIX #3/#4/#5" comments reference an absent changelog.
- **Duplicated constants:** `ANALOG_DEADZONE`/`apply_deadzone` defined in *both*
  `web_control.py` and `web_remote.py`; `MECH_LIMITS`/`STAND_ANGLES` re-keyed in
  `space.py`; **three different leg orderings coexist** (root of H5).
- **Docstring/constant drift in `brace_controller`:** header cites gains/thresholds
  (`0.0012`, `8 deg/s`) that don't match the actual constants (`0.0005`, `20/15`).
- **Massive pose-dict copy-paste in `tricks.py`** (uniform crouch/half poses duplicated
  verbatim across `pushups`/`bheek`/`high_five`); magic numbers everywhere, no config.
- **Dead geometry:** `stability_controller.py:72-89` (`LEG_X`/`LEG_Y`/`BODY_LENGTH`/
  `BASE_Z`) declared, never used.
- **Deprecated `np.matrix`** throughout `kinematics.py`/`util.py` — fragile vs NumPy
  upgrades; relies on `*` meaning matmul.
- **`watchdog_last_heartbeat = [time.time()]`** — 1-element list as a mutable cell to
  dodge `global` (works via GIL, reads badly).

---

## 7. Prioritized fix roadmap

**Foundation (do first — most bugs collapse into these):**
1. **Add a `threading.Lock` around every I2C transaction** in `i2c_bus.py` (or serialize
   IMU+servo access). Fixes the root cause of the Errno-121 storms (C4) and the
   `_safe_read_word` tearing.
2. **Make bus error handling uniform:** same retry/catch on IMU reads as servo writes
   (H1); guard `_send`/tricks and never let a bus error escape the loop (C1); add
   `try/finally` safe-pose recovery to reared-up tricks.
3. **Drive all `dt` from `time.monotonic()`** (H2) — IMU filter and stability controller.

**Correctness:**
4. Clamp `acos` args in `kinematics.py:110-111` (C3, crash).
5. Fix the trick-thread stop protocol so a timed-out thread truly stops writing (C2).
6. Add integral leak/decay + make state instance-level in the stability controller (H3).
7. Fix `leg_origins` vs `LEG_ID` ordering before enabling body-rotation posture (H5).
8. Rework the gait into a real diagonal trot with velocity-continuous touchdown, and
   implement actual turn/lateral gaits (H4).

**UX / reliability:**
9. Browser: per-stick axis reset (H6) + WebSocket auto-reconnect (H7).
10. Align watchdog timeout with trick durations / stamp heartbeat in blocking sections (M1).
11. Surface uvicorn/hardware-init failures instead of swallowing them (M2, M3).

**Cleanup (once behavior is settled):**
12. Delete the dead gamepad/keyboard paths and de-triplicate the control loop.
13. Single-source the duplicated constants and leg orderings; fix stale docstrings.

---

## 8. One-paragraph takeaway for the rebuild

The bones are good — keep the layering, `absolute_truths`, `WebControlHub`, the
`execute_step` guard pattern, and `brace_controller` as your quality bar. Treat the
**I2C bus as a single locked resource with uniform error handling** and drive
**everything off a measured monotonic clock**; doing just those two things eliminates
the majority of the crash/instability surface. Then the remaining work is
well-contained: a correct trot + real turn/lateral gaits, a non-winding posture
controller, the IK `acos`/leg-ordering fixes, and browser reconnect + per-stick axes.
