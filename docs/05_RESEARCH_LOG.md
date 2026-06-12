# BARQ v1 — Research Log

Publication-record log: why each change was an improvement — metrics,
overridden decisions, lessons. One entry per experiment/measurement/override.
Newest first. (Migrated from `stack/docs/RESEARCH_LOG.md` on 2026-06-12,
D-007; decision refs renumbered D-001…D-006.)

---

## 2026-06-12 — First simulation results: the robot stands, poses, and lifts a leg open-loop

**What was built.** `barq1/kinematics.py` (analytic 3-DOF leg IK, knee-back
branch, + whole-body IK — frames identical to the URDF by construction) and
the PyBullet harness (`stack/sim/`), driving the robot exactly as hardware
will be driven: position commands only, torque capped 3.0 N·m, velocity
capped 6.5 rad/s (D-008/D-009). URDF upgraded to physical inertials first
(D-010) — upstream placeholders were off by 4–6 orders of magnitude.

**Kinematics verification** (`pytest stack/test -q`, 9/9):
- FK∘IK round-trip < 1e-9 m over a 100+-point workspace grid, both sides.
- IK cross-checked against an independent law-of-cosines derivation at
  stand height: q2 = 0.97564 rad, q3 = −1.73004 rad — match < 1e-9.
- Body-pose round-trip (IK→FK through body transforms) exact on translations
  + rotations; operating envelope h ∈ 0.12–0.19 m with ±3 cm shifts stays
  inside URDF joint limits. Boundary documented: a 0.10 m crouch + 4 cm
  shift exceeds the 1.548 rad thigh limit (lie-down needs foot x-offsets,
  as spotMicro's motion config also does).

**Sim scenarios** (`run_sim.py`, headless, artifacts
`~/barq_v1/artifacts/sim-20260612-153854/`):

| scenario | result |
|---|---|
| settle (drop at stance) | stands: roll 0.01°, pitch −0.02°, 4 contacts, support margin **94.0 mm** = predicted stance-rectangle half-width (93.99 computed) |
| stand_up (0.125→0.155 m ramp, 2 s) | max tilt **0.04°**, height err +5.4 mm |
| pose_sweep (roll ±8.6°, pitch ±6.9°, yaw ±8.6°, h ±2 cm) | open-loop RMS tracking err: roll 3.08°, pitch 2.43°, yaw 5.30°; min margin 3.2 mm; toe scrub ≤ 13.5 mm |
| weight_shift (shift → lift FL 4 cm → hold 1 s → return) | **survived**, min 3-leg margin 33.5 mm, max tilt 0.73° |

**Findings.**
1. The static-walk primitive (shift weight onto a tripod, lift the free leg)
   works open-loop with a 33 mm COM margin — static/quasi-static gait is
   viable on command-only servos. This is the green light for a crawl gait.
2. Planted-foot yaw scrubs feet (13.5 mm) and eats nearly the whole support
   margin at ±8.6° — posture control should keep yaw amplitudes small or
   accept scrub. Roll/pitch posturing is cheap and safe.
3. Constant +5.4 mm height bias at stand (contact/controller steady state) —
   absorb into calibration later, not worth modeling.
4. RMS pose-tracking error of 2.4–5.3° is the open-loop floor (servo lag
   included); the IMU loop (Stage D) is what will close this on hardware.

**Transfer caveats** (kept honest): masses are upstream guesses (Q-004), no
gear backlash/deadband modeled yet, servo torque/speed from nameplate
(Q-002). Visual check: `settle.png` in the artifacts dir — robot standing,
level, correct knee direction.

---

## 2026-06-11 — D-006: BARQ v1 URDF adopted (spotMicro stretched to the measured axle span)

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
CAD showed legs 113.92 / 134.76; per D-006 these are treated as remodel
artifacts of stock prints (Q-003).

---

## 2026-06-11 — Revival kickoff: workspace, truths recovery, URDF correction, calibration GUI

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
  angle equals its perpendicular (legs vertical at stand), and thigh/wrist
  stand poses sit at perp ±40° / ∓75° symmetrically across mirrored mounts —
  strong evidence these numbers are genuine measurements.

**URDF chassis dimensions** (spotMicro `spot_micro.urdf.xacro`) — corrected
same-day; an earlier reading summed the body boxes as if contiguous (238 mm)
and missed that the two hip-servo modules sit between them. Actual length
composition, nose→tail, all sections 110 wide × 70 tall:
nose cover **58** | hip module **44** (coxa shaft at its center) | center
shell **140** | hip module **44** | tail cover **40** → **overall ≈ 326–329 mm
including both covers**. Coxa shafts at x = ±93 → **shaft-to-shaft 186 mm**;
that, not any shell number, is the kinematic `body_length`.

**URDF vs BARQ CAD cross-check** (assets/*.jpeg, Aryaman's annotated CAD).
Exact matches: hip↔hip lateral **78 = 78**, shell height **70 = 70**;
near match: width 114 vs 110–116. Mismatches: shoulder-axle span **207.5 vs
186**, overall **345.6 vs ≈327**, mounting plate **148 vs 140**, upper leg
**113.92 vs 107.5**, lower leg **134.76 vs 130**. Front/rear overhang beyond
the axles ~equal in both (≈138 vs ≈141 mm) — the stretch is concentrated
between the axles. → resolved next day by D-006.

**Hardware scan.** `i2cdetect -y -r 7` showed no devices — servo board was
unpowered at the time. Not a fault finding; re-scan with electronics powered.

**Built.** `stack/` package (`truths.py`, `servos.py`, `pca9685.py`) and the
web calibration GUI (`tools/calibration_gui.py`, Flask, port 8035), smoke-
tested end-to-end in sim mode (state/command/clamp/fit/save/all-off).

**Decisions made:** D-001 ticks canonical · D-002 web GUI · D-003 safe start ·
D-004 spotMicro calibration method · D-005 mech windows enforced (see
`02_DECISIONS.md`).
