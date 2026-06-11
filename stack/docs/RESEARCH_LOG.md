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

**URDF chassis dimensions** (spotMicro `spot_micro.urdf.xacro`) — corrected
same-day; an earlier reading summed the body boxes as if contiguous (238 mm)
and missed that the two hip-servo modules sit between them. Actual length
composition, nose→tail, all sections 110 wide × 70 tall:
nose cover **58** | hip module **44** (coxa shaft at its center) | center
shell **140** | hip module **44** | tail cover **40** → **overall ≈ 326–329 mm
including both covers** (±2 mm box-placement slop in the upstream model; the
upstream front/rear collision boxes also carry swapped x-signs — lengths are
trustworthy, placements sloppy). Coxa shafts at x = ±93 → **shaft-to-shaft
186 mm**; that, not any shell number, is the kinematic `body_length`.
Wireframe (`spot_micro_motion_cmd.yaml`): body 186 × 78 (hip-to-hip lateral);
links: hip 55, upper leg 107.5, lower leg 130.

**URDF vs BARQ CAD cross-check** (assets/*.jpeg, Aryaman's annotated CAD).
Exact matches: hip↔hip lateral **78 = 78**, shell height **70 = 70**;
near match: width 114 vs 110–116. Mismatches: shoulder-axle span **207.5 vs
186** (+21.5), overall **345.6 vs ≈327** (+19), mounting plate **148 vs 140**
(+8), upper leg **113.92 vs 107.5** (+6.4), lower leg **134.76 vs 130** (+4.8).
Front/rear overhang beyond the axles is ~equal in both (≈138 vs ≈141 mm) —
the stretch is concentrated between the axles. Conclusion: **same
architecture, NOT dimensionally identical**; the v1 chassis appears to be a
longitudinally stretched spotMicro with slightly longer leg links. The URDF
is adoptable as structure but ~5 parameters must be re-fit to the as-built
robot (xacro: shiftx, body/front/rear lengths, leg_length, foot_length;
kinematics: body_length, upper/lower link lengths, hip offset). Running
spotMicro's stock numbers on this geometry would bias every foot target by
roughly the half-stretch (~10 mm) plus link errors. Pending: physical
tape-measure of the four decisive numbers and confirmation of what the CAD's
207.5 dimension actually spans.

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
