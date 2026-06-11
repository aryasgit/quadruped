# BARQ v1 — Research Log

Publication-record log for the v1 revival. One entry per experiment,
measurement, or decision that overrides prior practice. Newest first.

---

## 2026-06-11 — D6: BARQ v1 URDF adopted (spotMicro stretched to the measured axle span)

**Decision (Aryaman).** The CAD's 207.5 mm spans rear coxa shaft centre to
front coxa shaft centre. The geometry difference vs spotMicro lives in the
chassis section extended outward from the main components; spotMicro's URDF
is trusted as the more precise source for everything else. Therefore: keep
the spotMicro URDF wholesale and grow only the length so the axle span is
**207.5 mm**.

**Implementation** (`stack/urdf/barq_v1.urdf.xacro`, STLs vendored to
`stack/urdf/stl/`): `shiftx` 0.093 → **0.10375**; `body_length` 0.140 →
**0.1615** (center section carries the whole +21.5 mm). Unchanged stock:
lateral `shifty` 0.039 (hip span 78), `shift` 0.055 (hip link), legs
0.1075 / 0.130, shoulder 44 × 38, shell 110 × 70, covers 58 / 40. Visuals:
mainbody mesh (spans both hip housings per STL bbox analysis) stretched
×1.15357 in x — printed hip modules are stock, so the stretched visual
housings sit ~3.5 mm off shaft centre, cosmetic only; cover meshes shifted
outward ±10.75 mm. Upstream's swapped-sign front/rear collision boxes fixed
(front +0.15475, rear −0.14575). Lidar backpack removed (no lidar on v1).
Mesh paths via `$(arg mesh_prefix)` so the file works in pybullet (relative)
and RViz (`file://` absolute).

**Validation** (`stack/tools/validate_urdf.py`, standalone pip `xacro` in the
workspace venv): 23 links / 22 joints; axle span 0.20750, hip span 0.07800,
hip link 0.05500, upper 0.10750, lower 0.13000; four-way shoulder symmetry;
all mesh refs resolve. Consistency: rendered cover-to-cover length from STL
bounding boxes = **342.4 mm** vs 345.631 mm measured on the CAD (≤3 mm slop,
same order as upstream's own box placement slop).

**Kinematic truth for the IK layer**: `barq1/geometry.py` — body 0.2075 ×
0.078, hip link 0.055, upper 0.1075, lower 0.130. Residual risk, accepted:
CAD showed legs 113.92 / 134.76; per D6 these are treated as remodel
artifacts of stock prints. A caliper check of one thigh (axis-to-axis,
expect ~107.5) would retire the risk entirely.

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
