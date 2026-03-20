# Battery Technical Reference

> Complete specification and selection analysis for the robot's power system. Chassis internal cavity: 150 × 97 × 48 mm.

---

## Selected Battery: GenX Premium 5200 (Option A)

**Full spec:**

| Parameter | Value |
|---|---|
| Model | GenX Premium 5200 |
| Chemistry | LiPo |
| Cell config | 4S1P |
| Nominal voltage | 14.8 V |
| Max charge voltage | — |
| Min discharge voltage | — |
| Capacity | 5200 mAh |
| Energy | 76.9 Wh |
| Internal resistance | — |
| Nominal C-rate | 1C |
| Nominal discharge current | 5.2 A |
| Continuous C-rate | **40C** |
| Max continuous current | **208 A** |
| Burst C-rate | **80C** |
| Max burst current | **416 A (10 s)** |
| Discharge connector | XT-60 |
| Balance connector | — |
| BMS included | No |
| Standard charge rate | 1–3C |
| Max charge rate | — |
| Cycle life | Higher than standard |
| Dimensions | 138 × 43 × 33 mm |
| Weight | 512 g |
| Volume | 245,388 mm³ |
| Chassis volume used | 33% |
| Free space alongside (top view) | 138 × 54 mm (7,452 mm²) |
| Free space above (side view) | 138 × 15 mm (2,070 mm²) |
| End gap (top view) | 12 × 97 mm (1,164 mm²) |
| Height clearance | 15 mm above battery |

---

## Full Candidate Comparison

Chassis cavity: 150 × 97 × 48 mm. All candidates evaluated for electrical performance and physical fit.

| Parameter | Old battery | **Option A** | Option B | Option C |
|---|---|---|---|---|
| Model | Pro-Range 6200 | **GenX Premium 5200** | GenX Molicel 15000 | Unnamed 8400 |
| Chemistry | LiPo | LiPo | Li-ion (4S3P) | LiPo |
| Nominal voltage | 14.8 V | 14.8 V | **14.4 V** | 14.8 V |
| Capacity | 6200 mAh | 5200 mAh | 15000 mAh | 8400 mAh |
| Energy | 91.8 Wh | 76.9 Wh | 216 Wh | 124.3 Wh |
| Cont. C-rate | 35C | **40C** | 12C | 11C |
| Max cont. current | 110 A | **208 A** | 180 A | 92 A |
| Burst C-rate | 70C | **80C** | 15C | — |
| Max burst current | 220 A | **416 A** | 225 A | not rated |
| Dimensions | 157×50×30 | **138×43×33** | 142×80×39 | 145×45×45 |
| Weight | 630 g | **512 g** | 972 g | ~650 g |
| Chassis fit | **✗ No fit** | **✓ Fits easily** | ✓ Fits (rotated) | ⚠ Tight (3mm H) |
| Chassis vol. used | does not fit | **33%** | 63% | 42% |
| BMS included | — | No | Yes (Li-ion) | No |
| Recommendation | ✗ ref only | **★ Top pick** | ✓ If runtime priority | ⚠ Verify dims first |

**Notes:**
- Old battery: 7 mm over chassis length. Eliminated.
- Option B: Li-ion at 14.4 V nominal — verify all 4S electronics accept this before ordering. Rotated 80 mm sits along 97 mm axis. 9 mm height clearance.
- Option C: 3 mm height margin is a critical tolerance — must physically verify before purchasing. 11C continuous may be insufficient under simultaneous multi-leg load.

---

## Runtime Estimates

Based on 80% depth of discharge (industry standard for LiPo health). Constant draw assumption.

| Scenario | Current draw | Single battery | Dual battery |
|---|---|---|---|
| Idle / standing | ~20 A | 15.6 min | 31.2 min |
| Slow walk | ~40 A | 7.8 min | 15.6 min |
| Normal gait | ~80 A | 3.9 min | 7.8 min |
| Aggressive / fast | ~160 A | 1.95 min | 3.9 min |

Real-world runtime varies with gait profile, terrain, servo load, and ambient temperature.

---

## Dual Battery Configuration

Two Option A batteries in parallel. Same 14.8 V, XT-60 parallel harness (wired parallel = same voltage, doubled capacity and halved internal resistance).

### Electrical gains

| Parameter | Single | Dual |
|---|---|---|
| Capacity | 5200 mAh | 10400 mAh (+100%) |
| Energy | 76.9 Wh | 153.8 Wh (+100%) |
| Max cont. current | 208 A | 416 A (+100%) |
| Burst current | 416 A | 832 A (+100%) |
| Internal resistance (est.) | ~6 mΩ | **~3 mΩ (-50%)** |

Internal resistance halving is the key electrical advantage. At 160 A draw, single battery voltage sag approaches the 11.1 V cutoff; dual battery holds approximately 0.7 V higher at the same current — more stable power delivery to servos and Jetson.

### Physical layout — rotated pair

Rotating each battery (swap W↔H: 43 mm becomes height, 33 mm becomes width):

- Each battery footprint becomes 138 × 33 mm (W)
- Two side by side: 138 × 66 mm combined
- Combined + chassis: 138×66×43 mm — fits within 150×97×48 mm chassis ✓
- Free strip alongside: **31 mm × 138 mm** (4,278 mm²) — fits ESP32, PCA9685, buck converter in a row
- Height clearance: **5 mm** — flat ribbon cables only, no tall capacitors

### When to use dual

Upgrade from single to dual if:
- Runtime at normal gait (<4 min) is too short for mission
- Voltage sag is causing servo brownout at aggressive manoeuvres
- Jetson or ESP32 are resetting under load (brownout to 5V rail)

### Wiring (parallel)

```
Battery 1 (+) ──┬── XT-60 Main (+) → Robot
Battery 2 (+) ──┘

Battery 1 (-) ──┬── XT-60 Main (-) → Robot
Battery 2 (-) ──┘
```

Always use equal-length wires from each battery to the junction. Charge each battery individually (not through the parallel harness) or use a parallel charging board rated for 2× the single battery's max charge current.

---

## Charging

| Parameter | Value |
|---|---|
| Standard rate | 1–3C (5.2–15.6 A for 5200 mAh) |
| Recommended | 1C = 5.2 A (safest, best cycle life) |
| Max | Not rated — treat 3C as ceiling |
| Chemistry | LiPo 4S — charge to 16.8 V (4.20 V/cell) |
| Storage voltage | 15.2 V (3.80 V/cell) |
| Cutoff voltage | 11.1 V under load (2.78 V/cell) |
| Balance charging | Required — use balance lead |

Never discharge below 3.5 V/cell under load. Store at ~3.8 V/cell if not using for >1 week.

---

## Free Space Summary (Option A, single battery)

| Zone | Dimensions | Area | Best use |
|---|---|---|---|
| Alongside battery (top view) | 138 × 54 mm | 7,452 mm² | Full electronics board — ESP32, PCA9685, buck converter |
| Above battery (side view) | 138 × 15 mm | 2,070 mm² | Cables, flat ribbon connectors |
| End gap (top view) | 12 × 97 mm | 1,164 mm² | Power switch, XT-60 plug |
| End gap (side view) | 12 × 48 mm | 576 mm² | Cable routing |