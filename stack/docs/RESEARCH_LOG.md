# BARQ v1 — Research Log

Publication-record log for the v1 revival. One entry per experiment,
measurement, or decision that overrides prior practice. Newest first.

---

## 2026-06-11 — Revival kickoff: workspace, truths recovery, URDF confirmation, calibration GUI

**Context.** v1 was a working-ish Python/I2C stack (Jetson Orin Nano → PCA9685
→ 12× DS3240MG; HW-290 10-DOF IMU) on spotMicro mechanics. Code quality made it
unmaintainable; project declared dead. Decision: rebuild the stack from scratch
on the same hardware, anchored to the spotMicro research, keeping the legacy
repo only as a source of *measured facts*.

**Workspace.** `~/barq_v1/` (fully isolated from `~/barq_ws` v2): `quadruped/`
(this repo, new work on branch `Master`), `reference/spotMicro` (read-only
clone), `venv/` (all Python deps; host `python3-venv` was missing and apt needs
a password, so pip was bootstrapped into the venv via `get-pip.py`).

**Truths recovered** (from legacy `src/main/hardware/absolute_truths.py`,
ported verbatim to `stack/barq1/truths.py`):

- I2C bus 7; PCA9685 @ 0x40; MPU6050 @ 0x68.
- DS3240MG electrical pulse range **106–535 PCA9685 ticks @ 50 Hz**
  (≈ 518–2612 µs, nominal oscillator) across 270° travel.
- Channel map, per-joint mechanical windows, perpendicular references, and a
  measured stand pose. Internal consistency check passed: every coxa's stand
  angle equals its perpendicular (legs vertical at stand) — strong evidence
  these numbers are genuine measurements, not copy-paste.

**URDF chassis confirmation** (spotMicro `spot_micro.urdf.xacro`):
collision body = three boxes sharing a 110 × 70 mm cross-section: center
**140 mm** + front **58 mm** + rear **40 mm** → **238 × 110 × 70 mm** overall.
Kinematic wireframe (`spot_micro_motion_cmd.yaml`): shoulder-to-shoulder
**186 mm** (length), hip-to-hip **78 mm** (width); links: hip 55 mm, upper leg
107.5 mm, lower leg 130 mm. Awaiting Aryaman's tape-measure confirmation
against the physical chassis before we adopt the URDF wholesale.

**Hardware scan.** `i2cdetect -y -r 7` showed no devices — servo board was
unpowered at the time. Not a fault finding; re-scan with electronics powered.

**Built.** `stack/` package (`truths.py`, `servos.py`, `pca9685.py`) and the
web calibration GUI (`tools/calibration_gui.py`, Flask, port 8035).

**Decisions.**
- **D1 — ticks are canonical.** All measured truths are in PCA9685 ticks taken
  against this exact board's (uncalibrated ±%) oscillator and the legacy
  prescale formula (121 @ 50 Hz). The new driver reproduces both exactly,
  including the legacy int() truncation in angle→tick conversion, so every
  recorded number keeps meaning. µs and degrees are display-only views.
- **D2 — web GUI, not Tkinter/VNC.** Jetson is headless; a browser GUI on
  http://barq.local:8035 beats X11/VNC plumbing for a hand-on-robot workflow
  (phone/Mac both work).
- **D3 — safe start.** Driver init forces ALL outputs off; servos stay limp
  until explicitly enabled. (Legacy GUI snapped all 12 servos to stand pose at
  import time — known hazard, eliminated.)
- **D4 — calibration method = spotMicro's.** Two-point linear fit per servo
  (`docs/servo_calibration.md` + their spreadsheet), with our refinement that
  the fit is anchored at the joint's TRUE ZERO instead of a fixed center pulse.
  Output: `stack/config/servo_calibration.yaml`.
- **D5 — mechanical windows are enforced.** GUI clamps commands to each
  servo's measured mech window by default; full electrical range is an
  explicit per-servo unlock.

**Next.** Power servo rail → re-scan bus 7 → calibrate all 12 true zeros →
then IK layer (spotMicro `spot_micro_kinematics`-faithful) against the
calibration file.
