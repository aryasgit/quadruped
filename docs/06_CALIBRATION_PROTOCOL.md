# 06 — Servo Calibration Protocol

The angles you record in the calibration GUI **define** the map from IK
output to PWM ticks (`barq1/servo_map.py`). They must be measured in the
URDF/IK convention below — nothing else in the pipeline corrects signs
(the legacy `conventions.py` sign tables are superseded by the signed
slope of each servo's fit, D-004).

## Prerequisites

- Robot **on the stand**, legs hanging free. Bench supply / servo rail on.
- `i2cdetect -y -r 7` shows `0x40`.
- GUI: `~/barq_v1/venv/bin/python stack/tools/calibration_gui.py`
  → `http://barq.local:8035`. Channel sanity first (Q-001): enable one
  card at a time, nudge ±10 ticks, confirm the right joint moves.

## Angle convention (URDF/IK — memorize the three zeros)

Body axes: **x forward, y left, z up**.

| joint | q = 0 (TRUE ZERO) | positive direction | record 2nd point at |
|---|---|---|---|
| coxa (q1) | leg's side plane vertical — foot hangs straight below the hip, zero splay | foot tips toward the robot's **LEFT** (same rule both sides) | **+45°** |
| thigh (q2) | thigh link plumb **vertical** | knee swings **BACKWARD** | **−90°** (thigh horizontal, knee pointing forward) |
| wrist/knee (q3) | shin **in line** with the thigh (leg straight) | (operation uses q3 ≤ 0) | **−90°** (shin perpendicular to thigh, folded toward the thigh's front) |

Eyeball placement is fine (spotMicro's research used by-eye + a phone
inclinometer for ±45°); **consistency matters more than accuracy**. Use
link axis-to-axis lines, not plastic edges.

## Per-servo procedure (×12)

1. Card **Off (free-wheel)** → hand-place the link at its TRUE ZERO pose.
2. **Enable @ slider** after moving the slider near where the servo already
   is (so it doesn't snap), then nudge ±1/±10 until the link is exactly at
   the zero pose → **Set TRUE ZERO here (0°)**.
3. Hand-place / drive the link to the 2nd reference (table above), set the
   angle field accordingly (+45 / −90) → **Record @**.
4. The card shows the fit: `zero … slope … dir …`. Move on.

## Sanity gates (before trusting it)

- All 12 |slopes| similar, ≈ **1.59 ticks/°** (270° over 106–535); left vs
  right sides mirror in sign for thigh/wrist. Outliers → re-measure
  (spotMicro doc says the same).
- **Save calibration** → commit `stack/config/servo_calibration.yaml`
  (it's a truth).
- `~/barq_v1/venv/bin/python stack/barq1/servo_map.py` → prints the table +
  warnings (slope band, zero inside mech window).
- Dry-run the whole pipeline, no hardware risk:
  `python stack/runtime/run_robot.py --dry-run --scenario walk`.
- First powered run, ON THE STAND:
  `python stack/runtime/run_robot.py --scenario stand` — staggered engage
  at crouch, slow ramp to stand, slew-limited throughout, Ctrl-C = all off.
