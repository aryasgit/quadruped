# BARQ v1 — Research Log

Publication-record log: why each change was an improvement — metrics,
overridden decisions, lessons. One entry per experiment/measurement/override.
Newest first. (Migrated from `stack/docs/RESEARCH_LOG.md` on 2026-06-12,
D-007; decision refs renumbered D-001…D-006.)

---

## 2026-06-15 — Real masses measured: robot is 1.76 kg, not 4.88 (Q-004 RESOLVED)

**Measured** (Aryaman, per part, servos attached as-built — hip servo in the
shoulder, one servo per leg segment, none in the body): center body 522 g,
front cover 44, rear cover 30; per leg shoulder 81.5, upper leg 103.2
(+ cover 19.9), shin 80.7, toe 6.3 → 291.6 g/leg × 4. **Total 1.76 kg**
(battery tethered/excluded) — the placeholder was 4.88 kg, i.e. **2.8× too
heavy** (body alone 2.8 kg guessed vs 0.52 measured). URDF masses updated;
D-010 analytic inertias recompute from them. Frame + 20 tests still pass.

**Re-baseline (old 4.88 kg → new 1.76 kg, fidelity-on):**

| scenario | old | new |
|---|---|---|
| settle margin | 94.0 mm | 94.0 mm (geometric, unchanged) |
| weight_shift 3-leg margin / tilt | 22.4 mm / 0.92° | 26.7 mm / 0.77° |
| crawl distance / eff / min-margin | 117.7 mm / 98% / 12.4 mm | 122 mm / 102% / 4.3 mm |
| vel_forward p10 margin / neg% | 8.6 mm / 9.4% | **17.9 mm / 0.6%** |
| vel_forward distance | 132 mm | 160 mm |
| vel_turn yaw (wz 0.10, 8 s) | +20.5° | +14° |

**Findings.**
1. **Velocity gait is markedly MORE stable with real mass** — forward p10
   margin doubled (8.6 → 17.9 mm), negative-margin frames 9.4% → 0.6%. A
   lighter robot is easier to keep statically balanced.
2. **Servo torque headroom grew** — 3.0 N·m continuous now drives 1.76 kg;
   actuator torque is not a transfer risk.
3. **Open-loop net motion is mass-sensitive** — turn yaws less, forward
   travels more, coupling shifted. Expected with no feedback (slip/grip ∝
   mass); the IMU/heading loop (Stage D) nulls it. Velocity clamps unchanged
   (joint-limit-derived, mass-independent).

**Caveat (minor):** the 60 g servos dominate each leg segment, but the
analytic box inertias still place each link's COM at its geometric centre;
real COM is biased toward the servo. Mass totals are now correct; per-link
COM offset is a future refinement (second-order for quasi-static gait).

---

## 2026-06-15 — Gait control: velocity-commanded gait ported from spotMicro (D-016)

**Why:** the diagnostic GUI confirmed the channel map + stand/perp poses on
real hardware, so the next high-ROI step is gait control ("can only tell
performance once it's moving"). Our crawl was fixed/pre-baked — no turn,
strafe, or velocity command.

**Built** (per the audit's Phase-2): `command.py` (GaitCommand), `filters.py`
(RateLimitedFirstOrderFilter, ported verbatim from spotMicro), and
`velocity_gait.py` (VelocityGait) — a faithful port of spotMicro's
`spot_micro_walk.cpp` into our frame (x-fwd/y-left/z-up vs their
x/y-up/z-left): 8-phase static schedule, stance controller (planted feet
sweep back at cmd velocity + yaw), swing controller (triangular-height arc
to a velocity-scaled touchdown), body weave to the support tripod. Fixed
crawl (`gait.py`) left intact.

**Validation** (sim, fidelity-on, 8 s; new scenarios + 5 unit tests, 20/20):

| command | result | median margin | p10 | tilt |
|---|---|---|---|---|
| forward 0.024 m/s | +132 mm (drift −22 mm, yaw +0.3°) | 18 mm | 9 mm | 2.5° |
| turn-left 0.10 rad/s | **+20.5° yaw** | 30 mm | 10 mm | 2.6° |
| strafe-left 0.022 m/s | **+163 mm** (yaw +4°) | 30 mm | 10 mm | 2.4° |

All statically stable (p10 ≥ 9 mm = 90% of frames), none fell. Yaw sign
needed flipping after the axis remap; linear signs were correct first try.

**Joint-safe envelope (finding).** Scanned the largest single-axis velocity
keeping every joint inside its URDF limit (1.7° margin, both directions,
full cycle): **vx 0.024, vy 0.022 m/s, wz 0.105 rad/s** — clamps set there.
Binding constraint = the thigh's tight upper limit (1.548 rad); larger or
backward strides drive it past, where sim (and the hardware mech-window)
saturate it silently. Earlier ad-hoc vx=0.04 runs were over-driving into
that saturation — the honest envelope is ~3–4× slower. Raising it needs a
faster cadence or different stance height (future tuning).

**Residual:** open-loop lateral coupling (forward drifts ~22 mm / 132 mm;
turning translates slightly) — expected; the IMU/heading loop (Stage D)
nulls it on hardware.

**Next:** controller FSM (idle/stand/walk + transitions via the ported
filter), then wire teleop + run_robot to it.

---

## 2026-06-15 — Fidelity-first: the sim now actuates the hardware boundary (D-015)

**Why:** the readiness audit found the sim fed continuous float radians to
PyBullet, bypassing the PCA9685 tick grid + command latency that bind the
real (feedback-less) robot — so sim success didn't guarantee a hardware-
reachable angle. Highest-ROI de-risking step before more gait.

**Built.** `JointActuator` (`sim/servo_model.py`): per-joint travel clamp +
tick quantization (270° / 429 ticks = **0.629°/step**, from truths) +
one-frame (20 ms) command transport delay, inserted in `SimRobot.command()`;
`--ideal` toggles it off. 15/15 tests pass; all 5 scenarios run clean and
**none fall** under the honest boundary.

**Decomposition** (walk, 3 cycles; one variable at a time):

| config | distance | efficiency | min margin |
|---|---|---|---|
| ideal (old float path) | 90.9 mm | 75.8 % | 12.8 mm |
| delay-only (1–2 frames) | 91.0 mm | 75.8 % | 12.8 mm |
| quantize-only | 118.0 mm | 98.3 % | 12.4 mm |
| both (new default) | 117.7 mm | 98.1 % | 12.4 mm |

**Findings.**
1. **Quantization is the whole effect; transport delay is negligible at
   quasi-static speed** (a uniform 20–40 ms delay just time-shifts a
   quasi-static gait — no geometry change). Delay kept anyway: faithful, and
   it will matter for the Stage-D IMU loop and any dynamic motion.
2. **Refuted hypothesis (audit said "servo lag" caused the 24 % stride
   loss — it does not).** The loss was **planted-foot micro-slip**: the
   ideal sim issued sub-tick (<0.63°) per-frame nudges to STANCE feet that
   PyBullet's contact let micro-slip, bleeding forward progress. The tick
   grid is a **deadband** that zeroes those nudges → stance feet stay
   planted → efficiency 76 % → ~98 %. The real servo has the same deadband,
   so 76 % was a sim artifact. (How much of ~98 % survives real foot
   friction is bounded in the robustness phase + hardware — not over-claimed
   yet.)
3. **Same deadband makes fine posturing coarse.** pose_sweep at ±8.6 °
   planted-foot yaw: yaw-tracking RMS **0.11° → 1.18°**, min support margin
   **3.2 mm → −3.2 mm** (COM transiently leaves the 4-foot polygon — held up
   only because all four feet are down; a single tripod would tip).
   Operational/teleop yaw posture must stay well under ±8.6° (teleop
   `YAW_AMP` is currently 8.6° — flag for the gait-control phase).

**New canonical baselines (fidelity ON; `--ideal` reproduces the old ones):**
settle margin 94.0 mm, level; stand_up tilt 0.05°; pose_sweep RMS
0.31/0.17/1.18° (r/p/y); weight_shift survived, 3-leg margin **22.4 mm**
(was 33.5 ideal — coarser body-shift placement, still healthy); walk
**117.7 mm / 98.1 % / margin 12.4 mm / no fall**.

**Still pending (Q-002):** servo internal response-lag calibration
(PyBullet positionGain/maxVelocity vs the real DS3240MG step response). The
robustness sweep (next phase) will bound it; the datasheet will pin it.

---

## 2026-06-15 — URDF length verification; spotMicro adopted as the complete guide (D-014)

**URDF length confirmed** (`/tmp/measure_len.py`, two independent methods —
own STL-bbox transform + Bullet AABB via the real sim load path; stock
spotMicro measured the same way for comparison). Body shell, nose-to-tail:

| measure | ours (updated sim URDF) | stock spotMicro |
|---|---|---|
| visual mesh extent (X) | **342.5 mm** | 321.0 mm |
| collision extent (Bullet AABB) | **349.5 mm** | — |
| coxa axle span (shaft→shaft) | **207.5 mm** (exact) | 186 mm |

Ours − stock = **21.5 mm**, exactly the D-006 stretch. CAD physical length
345.6 mm sits between our visual (342.5) and collision (349.5) extents; the
~3 mm visual gap is cosmetic — the IK uses the exact 207.5 mm axle span.
Confirms the running total length is **~345 mm, not the stock ~327 mm**.

**Doctrine (D-014).** Legacy `src/` now contributes ONLY five
hardware-interface facts (I2C bus/addresses, servo-driving method, PWM
ranges 106–535, mechanical limit windows, servo channel map). spotMicro is
the complete guide for everything else. Legacy perp/stand poses are
superseded by phase-2 calibration and remain only as GUI references.

---

## 2026-06-12 — IT WALKS (in sim): 6-phase crawl, and the full hardware pipeline dry-run

**Built** (while the robot is disassembled): trajectory layer (D-013),
crawl gait, servo map, IMU driver, hardware runtime, PS4 teleop — the
complete sim→hardware pipeline, every part testable without the robot.

**Crawl gait** (`barq1/gait.py`): 6-phase static creep on the proven
weight-shift primitive — shift right (+advance) | swing RL | swing FL |
shift left (+advance) | swing RR | swing FR. Params: step 40 mm,
clearance 35 mm, shift ±35 mm lateral / −30 mm aft, 0.9 s shifts,
0.7 s swings → 4.6 s/cycle. Open-loop, zero feedback consumed.

**Walk results** (3 cycles, headless, deterministic across runs;
artifacts sim-20260612-173430):

| metric | value |
|---|---|
| fell | **no** (max tilt 1.56°) |
| distance | 90.9 mm of 120 commanded → **75.8 % open-loop efficiency** |
| lateral drift / heading drift | 2.75 mm / **0.59°** over 13.8 s |
| min support margin | **12.8 mm** (always statically stable) |
| avg speed | 6.6 mm/s |

Findings: the 24 % stride loss is toe scrub + servo lag during swings —
the open-loop tax; levers are slower swings, stiffer position gain, and
stride calibration on hardware. Heading hold of half a degree over three
cycles open-loop is better than expected.

**Re-baseline note:** scenarios re-expressed on the 50 Hz trajectory layer.
pose_sweep tracking RMS is now 0.31/0.15/0.11° (was 3.08/2.43/5.30 at the
old 30 Hz cadence — mostly a sampling artifact of the old executor, plus
genuinely smaller per-frame steps at 50 Hz). All other metrics match the
2026-06-12 baselines (margins 94.0/33.5 mm, tilt 0.04°).

**Hardware pipeline dry-run** (`runtime/run_robot.py --dry-run`, synthetic
calibration, simulated I2C bus): engage(staggered) → ramp → 1 walk cycle →
ramp down → all-off; **600 frames @ 50 Hz, 0 overruns**, 12 joints/frame,
JSONL telemetry written. The Orin holds the loop rate trivially.

**Servo map** (`barq1/servo_map.py`, 6 new unit tests, 15/15 total):
ticks = zero + signed_slope·deg, clamped to measured mech windows; sanity
gates on slope band and zero placement. The calibration GUI's angle
convention is now formally specified in docs/06_CALIBRATION_PROTOCOL.md —
the legacy conventions.py sign tables are fully superseded.

**Decisions:** D-011 (no middleware — pure Python, escape hatches kept),
D-012 (PS4 teleop via evdev, shared sim/hw mapping), D-013 (trajectory
layer as single motion source). New facts: 4S 6200 mAh battery, tethered
(Q-002 power-tree documentation required before power-up); masses being
weighed (Q-004); INA260 ordered, integration planned (Q-005).

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
