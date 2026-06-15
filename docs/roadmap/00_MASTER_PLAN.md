# BARQ v1 — MASTER PLAN (the doomsday guide)

_Written 2026-06-12. Premise: you may have NO LLM assistance from here on.
These documents + the repo are everything needed to go from the current
state (sim walks, hardware disassembled) to a fully functioning quadruped —
and beyond, to RL policies. Every phase has fallbacks: if plan A fails, B;
if B fails, C._

## How to use this guide

1. Read `docs/01_STATUS.md` to find where the project actually is.
2. Open the phase folder you're in. Phases are strictly ordered unless
   marked parallel-safe. Do not skip **GATE** checks — they exist because
   skipping them historically killed this project once already.
3. After every session: update `docs/01_STATUS.md`, `03_CHANGELOG.md`,
   `05_RESEARCH_LOG.md` (numbers or it didn't happen), commit, push.
   The docs rhythm (D-007) is what makes the project survivable.
4. When something fails: check the phase's FAILURE section, then
   `appendices/B_failure_matrix.md`. Change ONE variable at a time.
5. Code that doesn't exist yet is fully specified in
   `appendices/C_code_specs.md` — API, algorithm, integration point, and
   an acceptance test for each. Implement to the spec, test to the test.

## Phase map

| phase | folder | one-line goal | needs | status 2026-06-12 |
|---|---|---|---|---|
| 0 | `phase-0-recovery/` | rebuild environment from bare metal | repo access | done (env live) — doc is for disaster |
| 1 | `phase-1-hardware-bringup/` | robot powered, I2C alive, channels verified | reassembled robot | **next** |
| 2 | `phase-2-calibration/` | 12 servo fits committed | phase 1 | tooling ready |
| 3 | `phase-3-first-stand/` | stands level on hardware, on stand then ground | phase 2 | runtime ready |
| 4 | `phase-4-walking/` | crawls on hardware, straight, repeatably | phase 3 | gait proven in sim |
| 5 | `phase-5-posture-loop/` | IMU closes the loop: stays level on slopes | phase 3 | driver written, spec'd |
| 6 | `phase-6-teleop-ux/` | PS4 + fallback teleop, OLED, e-stops | phase 3 | PS4 done, rest spec'd |
| 7 | `phase-7-perception/` | OAK-D: obstacle stop, person follow | phase 4 | spec'd |
| 8 | `phase-8-rl/` | learned policies that transfer | phases 4+5, masses | spec'd, gated |
| 9 | `phase-9-integration/` | boot-to-demo product: supervisor, battery onboard | phases 4–6 | spec'd |

Parallel-safe: 5 alongside 4; 6 alongside 4–5; 7 after 4 anytime.
8 is HARD-GATED (see its README) — do not start it early, it will eat
months and return nothing without its prerequisites.

## The robot, on one page

- **Mechanics**: spotMicro (KDY0523 frame family) stretched +21.5 mm
  between coxa shafts. Kinematic truth (`stack/barq1/geometry.py`):
  body 0.2075 × 0.078 m (axle spans), hip link 0.055, upper leg 0.1075,
  lower leg 0.130. URDF: `stack/urdf/barq_v1.urdf.xacro` (validated).
- **Electronics**: Jetson Orin Nano (Ubuntu 22.04, JetPack 6) → I2C **bus 7**
  → PCA9685 @ `0x40` (12× DS3240MG, 40 kg·cm, 270°, **no feedback**) and
  MPU6050 @ `0x68` (HW-290). PWM 50 Hz, prescale 121; servo range
  **106–535 ticks** (ticks are canonical, D-001). Power: 4S 6200 mAh LiPo
  **tethered off-board** → step-down to servo rail (document it! Q-002).
- **Software** (`stack/`, pure Python in `~/barq_v1/venv`, D-011): truths →
  driver → kinematics → trajectories/gait → sim (PyBullet) and hardware
  runtime (`runtime/run_robot.py`) consuming IDENTICAL 50 Hz frames (D-013).
  Teleop: PS4/evdev (D-012). Calibration GUI on :8035.
- **Doctrine** (D-009): no joint feedback exists → sim validates every
  trajectory before hardware; IMU is the only runtime feedback; servo zero
  calibration is load-bearing; controllers may consume only signals real
  hardware has.

## Proven numbers (your regression baselines)

| what | value | where proven |
|---|---|---|
| kinematics FK∘IK | <1e-9 m round-trip, 15/15 tests | `pytest stack/test` |
| sim stance margin | 94.0 mm (= hand calc) | 05, 2026-06-12 |
| single-leg lift margin | 33.5 mm, tilt 0.73° | 05, 2026-06-12 |
| crawl, 3 cycles | 90.9 mm, drift 0.59°, margin ≥12.8 mm, no fall | 05, 2026-06-12 |
| open-loop stride efficiency | 75.8 % (toe scrub tax) | 05, 2026-06-12 |
| hw loop timing | 600 frames @ 50 Hz, 0 overruns (dry-run) | 05, 2026-06-12 |

If after any change these regress in sim, the change is wrong, not the
baseline.

## Working rules without an LLM

1. **One variable at a time.** Two changes = unexplainable result.
2. **Sim first, always.** If it fails in sim it WILL fail on hardware.
3. **Gates are sacred.** Each phase README ends with a GATE checklist;
   all boxes or you don't advance.
4. **Never edit `barq1/truths.py` without a physical re-measurement.**
5. **Safety envelope is permanent**: mech-window clamps + slew limiter +
   all-off ESTOP stay in the command path forever, including under RL.
6. **Commit + push per session.** The repo (github.com/aryasgit/quadruped,
   branch `Master`) is the only durable artifact. SSH over 443 is already
   configured (`~/.ssh/config`).
7. When stuck >2 hours: write the symptom into `04_OPEN_QUESTIONS.md`,
   move to a parallel-safe task, come back fresh.

## Reading order for a total cold start

`phase-0-recovery/README.md` → `docs/01_STATUS.md` → `docs/00_OVERVIEW.md`
→ `docs/02_DECISIONS.md` → the current phase folder →
`appendices/A_command_reference.md` (keep open while working).
