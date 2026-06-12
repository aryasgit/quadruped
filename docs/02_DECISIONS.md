# Decisions (ADR log) — newest first

Format: `D-NNN (date) — the call`. Context → decision → why. Overridden
decisions are superseded here and narrated in 05, never erased.

---

## D-009 (2026-06-12) — Open-loop control doctrine

**Context:** DS3240MG servos have no position/velocity/torque feedback; the
Jetson sees nothing back from a joint.
**Call:** (1) Simulation is the substitute for joint feedback — every
trajectory is validated in PyBullet before hardware. (2) Runtime feedback is
body-level only: IMU roll/pitch/rates (+ optional contact switches later).
(3) Hard interface rule: controllers may consume only signals real hardware
provides (commands, IMU, time); sim ground truth feeds metrics, never
controllers. (4) Servo zero calibration is treated as load-bearing truth.
**Why:** anything else silently builds a controller the robot cannot run.

## D-008 (2026-06-12) — PyBullet is the simulation engine

**Context:** v1 died without a sim or IK validation; need contact dynamics +
URDF loading on an aarch64 headless Jetson, deps venv-only.
**Call:** PyBullet (pip, built from source on aarch64) in DIRECT mode for
tests; GUI via the existing VNC display when wanted. Servos modeled as
position-controlled motors with torque ≈ 3.0 N·m cont. (3.92 stall) and
max velocity ≈ 6.5 rad/s (~0.15 s/60°).
**Why:** native URDF, proven on spotMicro-class robots (spotmicroai
community), no ROS dependency, runs headless. Gazebo = v2 territory; MuJoCo
kept as fallback (has aarch64 wheels) if the PyBullet build misbehaves.

## D-007 (2026-06-12) — Adopt the BARQ-Rebuild docs system

**Context:** Aryaman supplied the v2 docs system as a portable reference.
**Call:** `docs/` at repo root with numbered files (00–05, HANDOFF, README),
D-NNN/Q-NNN cross-referencing, artifacts outside the repo
(`~/barq_v1/artifacts/`), docs updated in the same commit as the change.
Prior `stack/docs/{RESEARCH_LOG,HANDOFF}.md` migrated here; decisions D1–D6
renumbered D-001…D-006.
**Why:** v2 proved the rhythm; identical structure makes the two repos
mutually navigable and the v1 log publication-ready.

## D-006 (2026-06-11) — URDF = spotMicro stretched to the measured axle span

**Context:** Aryaman's CAD shows coxa-shaft span 207.5 mm (rear shaft centre →
front shaft centre) vs spotMicro's 186; lateral 78 and height 70 match
exactly; CAD leg links read 113.92/134.76 vs stock 107.5/130.
**Call (Aryaman):** trust the spotMicro URDF for everything except length:
`shiftx` 0.093→0.10375, `body_length` 0.140→0.1615. Legs/shoulders treated as
stock prints; CAD leg numbers treated as remodel artifacts (residual check =
Q-003).
**Why:** the difference lives in the chassis section extended outward from
the main components; spotMicro's numbers are the more precise source for the
stock parts.

## D-005 (2026-06-11) — Mechanical windows enforced at the tool layer

The calibration GUI clamps commands to each servo's measured mech window
(truths); the full electrical range (106–535) is an explicit per-servo
unlock. **Why:** hard stops + 40 kg·cm servos grind gears; the windows were
measured for exactly this reason.

## D-004 (2026-06-11) — Calibration method = spotMicro two-point linear fit

Per servo: record (angle, ticks) at the joint's 0° reference (TRUE ZERO) and
one more known angle (±90° links 2–3, ±45° link 1); least-squares
`ticks = zero + slope·angle`; output `stack/config/servo_calibration.yaml`.
Refinement over upstream: the fit is anchored at the joint's true zero, not a
fixed center pulse. **Why:** stick to the spotMicro research; it walked.

## D-003 (2026-06-11) — Safe start: all outputs off

Driver init forces ALL PCA9685 outputs off; servos stay limp until explicitly
enabled; ESC / red button = all-off. **Why:** the legacy GUI snapped all 12
servos to stand pose at import time — a known hazard, eliminated.

## D-002 (2026-06-11) — Web GUI, not Tkinter/VNC

Calibration tool is a Flask app on `http://barq.local:8035`. **Why:** the
Jetson is headless; a browser on the Mac (or phone, hands at the robot) beats
X11/VNC plumbing for a hands-on workflow.

## D-001 (2026-06-11) — PCA9685 ticks are the canonical unit

All measured truths (106–535 over 270°, perp/stand/mech windows) were taken
in ticks against this board's uncalibrated oscillator and the legacy prescale
(121 @ 50 Hz). The new driver reproduces both exactly (including the legacy
int() truncation); µs and degrees are display-only views. **Why:** re-basing
units would silently invalidate every measured number.
