# HANDOFF — BARQ v1 revival frontier

_Last updated: 2026-06-11_

## Where we are

- Workspace `~/barq_v1/` is live: this repo (branch **`Master`** — all new work
  lands here), `reference/spotMicro` (read-only research), `venv/` (all deps).
- `stack/` holds the new from-scratch code: truths (ported, measured), servo
  table, PCA9685 driver, and the **servo calibration web GUI** (port 8035).
- Legacy code under `src/` is reference only. Trust its
  `hardware/absolute_truths.py` (ported already); distrust everything else.

## Frontier (do next)

1. Power the servo rail + logic, `i2cdetect -y -r 7` → expect `0x40` (+ `0x68`).
2. Run the GUI, calibrate TRUE ZERO + one reference point for all 12 servos,
   Save → commit `stack/config/servo_calibration.yaml`.
3. Sanity-check fit slopes: all 12 should be similar (~1.59 ticks/° for 270°
   over 106–535). Outliers ⇒ re-measure (spotMicro doc says the same).
4. Then: kinematics layer faithful to spotMicro's, consuming the calibration
   YAML (not the legacy stand/perp constants).

## Open questions for Aryaman

- **Chassis is NOT dimensionally identical to the spotMicro URDF** (see
  RESEARCH_LOG 2026-06-11): lateral 78 and height 70 match exactly, but CAD
  says axle span 207.5 vs 186, overall 345.6 vs ≈327, legs 113.92/134.76 vs
  107.5/130. Tape-measure on the real robot, joint-axis to joint-axis:
  1. front coxa shaft → rear coxa shaft (CAD claims 207.5 — confirm endpoints)
  2. thigh axis → knee axis (CAD 113.92)
  3. knee axis → foot ground-contact center (CAD 134.76)
  4. coxa axis → thigh axis lateral offset (URDF hip_link 55 — not in CAD shots)
  Also: is the CAD the as-built robot, or a stylized remodel of it?
- Are all 12 servos still mounted per the legacy channel map (truths.py)?
- DS3240MG variant: 270° assumed (legacy stack + measured range agree); any
  datasheet/spec sheet on hand to confirm pulse spec (commonly 500–2500 µs)?

## How to run things

```bash
# calibration GUI (Jetson side)
cd ~/barq_v1/quadruped && ~/barq_v1/venv/bin/python stack/tools/calibration_gui.py
# Mac: http://barq.local:8035   (ESC = all outputs off)
```
