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

- ~~Chassis vs URDF~~ → **resolved by D6** (RESEARCH_LOG 2026-06-11): URDF is
  spotMicro stretched to the measured 207.5 mm axle span; legs/shoulders
  stock. `stack/urdf/barq_v1.urdf.xacro` + `barq1/geometry.py` are now the
  geometric truth. Residual (low priority): caliper one thigh axis-to-axis —
  ~107.5 mm confirms stock prints; ~114 would mean custom legs and a 2-line
  geometry fix.
- Are all 12 servos still mounted per the legacy channel map (truths.py)?
- DS3240MG variant: 270° assumed (legacy stack + measured range agree); any
  datasheet/spec sheet on hand to confirm pulse spec (commonly 500–2500 µs)?

## How to run things

```bash
# calibration GUI (Jetson side)
cd ~/barq_v1/quadruped && ~/barq_v1/venv/bin/python stack/tools/calibration_gui.py
# Mac: http://barq.local:8035   (ESC = all outputs off)
```
