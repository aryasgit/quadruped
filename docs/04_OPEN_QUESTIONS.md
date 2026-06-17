# Open questions — Q-NNN

On resolution the heading gains `— RESOLVED <date> -> D-NNN`; entries stay.

---

## Q-001 — Are all 12 servos still wired per the legacy channel map?

`barq1/truths.py` carries the legacy map (coxa FR=6 FL=7 RR=0 RL=1, thighs
8/9/2/3, wrists 10/4/11/5). The robot sat dead for months. Verify with the
calibration GUI on first power-up (wiggle one channel at a time, ±10 ticks).

## Q-002 — DS3240MG variant & pulse spec; servo-rail power tree

270° travel assumed (legacy driver + measured 106–535 tick range agree, and
the mech windows only make sense on 270°). A datasheet/spec sheet would also
confirm the electrical pulse limits (commonly 500–2500 µs) and stall vs
continuous torque for the sim servo model (D-008 uses 3.92/3.0 N·m).

**Power tree (added 2026-06-12):** battery is a **4S 6200 mAh LiPo, kept
off-board/tethered** — 14.8–16.8 V, far above servo spec, so a step-down
regulator must sit between. Document during reassembly: regulator
make/model, output voltage, continuous & burst current rating, and how the
Jetson is fed. Many DS3240MG are spec'd 4.8–6.8 V — the rail voltage
matters before first power-up.

## Q-003 — Are the leg links really stock spotMicro prints?

D-006 assumes yes (107.5/130 mm axis-to-axis); Aryaman's CAD reads
113.92/134.76. One caliper measurement of a thigh, axis-to-axis, settles it:
~107.5 → stock confirmed; ~114 → custom legs, 2-line fix in
`barq1/geometry.py` + xacro and re-run `tools/validate_urdf.py` + tests.

## Q-004 — Real masses — RESOLVED 2026-06-15

URDF masses were upstream guesses (~4.9 kg). Aryaman measured per-part:
**total 1.76 kg** (body 522 g, covers 44/30 g, 291.6 g/leg incl. one 60 g
servo per segment, hip servo in the shoulder). URDF updated; re-baselined
(05, 2026-06-15) — gait stability *improved* (forward p10 margin 8.6 →
17.9 mm). Residual: per-link COM still at geometric centre though servos
dominate — a second-order refinement, not blocking.

## Q-005 — INA260 integration (opened 2026-06-12)

An INA260 (V/I monitor, integrated shunt) is ordered for v2 and not yet on
hand. When it arrives, put it on the v1 servo rail (I2C bus 7 has room):
rail current is the only stall/collision proxy an encoder-less robot can
get (D-009). Plan: driver + telemetry field + a current-spike alarm that
triggers all-off.
