# Phase 2 — Calibration: 12 servo fits, committed

_Goal: `stack/config/servo_calibration.yaml` with a trustworthy
`zero_ticks` + signed `slope_ticks_per_deg` for all 12 servos, in the
URDF/IK angle convention. This file is LOAD-BEARING (D-009): with no
encoders, it is the only joint-angle truth the robot ever gets._

**The full procedure and the angle convention live in
`docs/06_CALIBRATION_PROTOCOL.md` — that is plan A. This document adds the
fallback ladder and recovery paths.**

## 2.1 Plan A — the web GUI (primary)

Per 06: free-wheel → hand-pose at TRUE ZERO → enable near → nudge → "Set
TRUE ZERO" → pose at the 2nd reference (coxa +45°, thigh −90°, wrist −90°)
→ "Record @" → fit appears. Save → commit.

Quality gates before trusting (also in 06):
- |slope| ≈ **1.589 ticks/°** for every servo. This is GEOMETRY (270° over
  429 ticks), not a preference — a slope of 1.3 or 2.0 means your two poses
  weren't at the angles you thought. Re-do that servo.
- Left/right pairs mirror in slope sign for thigh & wrist; coxa signs: the
  +q1=foot-tips-LEFT rule decides, verify by nudging.
- `python stack/barq1/servo_map.py` prints the table + warnings — zero
  warnings allowed at this gate.

## 2.2 Plan B — GUI unavailable (browser/Flask broken): curl or REPL

The GUI is just a client of HTTP endpoints; calibrate with curl:

```bash
B=http://localhost:8035/api
curl -s -X POST $B/servo/FL_thigh/command -H 'Content-Type: application/json' -d '{"ticks":372}'
curl -s -X POST $B/servo/FL_thigh/record  -H 'Content-Type: application/json' -d '{"angle_deg":0}'
curl -s -X POST $B/servo/FL_thigh/off; curl -s -X POST $B/api/save
```

Flask itself broken → bare REPL with the driver, write YAML by hand:

```python
import sys; sys.path.insert(0,"stack")
from barq1.pca9685 import PCA9685
from barq1.servos import SERVOS
p = PCA9685()
s = SERVOS["FL_thigh"]; p.set_ticks(s.channel, 340)   # nudge until at pose
# note ticks at q=0 and at the ref angle; slope=(t_ref-t_zero)/ref_deg
p.all_off()
```

YAML schema servo_map accepts (write all 12):

```yaml
servos:
  FL_thigh: {channel: 9, zero_ticks: 309.0, slope_ticks_per_deg: 1.589}
```

## 2.3 Plan C — angle references without instruments

- ±45°: fold a sheet of paper corner-to-corner = exact 45° jig.
- −90°: any carpenter square / phone against the link axis.
- Phone inclinometer app on a flat face of the link (spotMicro's own
  method, `reference/spotMicro/docs/servo_calibration.md`).
- Their spreadsheet (`docs/servo_calibration_spreadsheet.ods` in the
  reference clone) is the same two-point fit if you prefer it.

## 2.4 Plan D — EMERGENCY calibration from legacy truths (no measuring at all)

Good enough to stand carefully; NOT good enough to walk well. Derivation:
slope magnitude is geometric (±1.589); sign comes from the legacy mount
direction; zero comes from the legacy measured references:

- **slope** = +1.589 for normal mounts, −1.589 for inverted mounts
  (inverted = legacy mech min>max; flags are in `barq1/servos.py` table:
  FR/RR coxa, TFR/TRR thigh, WFR/WRR wrist are inverted).
- **zero_ticks**:
  - coxa: legacy perp ticks (FL 167, FR 180, RL 185, RR 175) — legacy
    stand==perp and stance has q1=0, so perp IS q1 zero.
  - thigh: legacy perp = thigh vertical = q2 zero
    (TFL 309, TRL 320, TFR 325, TRR 328).
  - wrist: legacy perp = shin in line with thigh = q3 zero
    (WFL 349, WRL 349, WFR 180, WRR 180).
- **verify** against the legacy stand pose: at body height ~0.18 m the IK
  says q2≈+45.6°, q3≈−81.9°; predicted thigh ticks = zero+1.589·45.6 ≈
  perp+72; legacy measured stand−perp = +63/64 (left) — i.e. legacy
  eyeballing carried ~10–15 % error. THAT is why plan D is coarse: expect
  the same error band, keep heights ≥0.14, speeds low, and redo plan A at
  the first opportunity.
- Sign check after writing the YAML, per servo: command zero, then
  zero+16 ticks (= +10°): coxa foot must tip LEFT, thigh knee must swing
  BACKWARD, wrist toe must fold FORWARD. Wrong way → negate that slope.

## 2.5 Recovery & maintenance

- The YAML is committed — `git log -- stack/config/servo_calibration.yaml`
  is your history; never overwrite without committing the predecessor.
- A servo was swapped / a horn re-seated → recalibrate THAT servo only
  (GUI overwrites per-servo points; others persist).
- Drift symptoms (robot leans that calibration used to fix): horn screw
  slipped a spline. Re-zero that servo, log it in 05.

## GATE (phase 2 complete when)

- [ ] all 12 fits present; every |slope| within 1.45–1.75 t/° (plan A/B/C)
      or explicitly logged as plan-D coarse
- [ ] sign check passed on all 12 (the 3 motion rules above)
- [ ] `python stack/barq1/servo_map.py` → no warnings
- [ ] YAML committed + pushed
- [ ] `run_robot.py --dry-run --scenario walk` runs against the REAL file
