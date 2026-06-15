# Phase 1 — Hardware bring-up: power, I2C, channels

_Goal: robot reassembled, electronics alive, every servo answering on the
channel the software thinks it owns. NOTHING moves under power until 1.4._

## 1.1 Power tree — document before energizing (Q-002)

Battery is a **4S 6200 mAh LiPo (14.8–16.8 V), tethered off-board**. That
voltage would destroy the servos; a step-down sits between. While wiring,
fill this into `docs/02_DECISIONS.md` as a new D-entry:

- regulator/BEC make+model, rated continuous & burst amps
- measured output voltage UNDER NO LOAD and under 4-servo load
- how the Jetson is powered (its own supply? same pack via second reg?)
- wire gauge to the rail (12×3 A stall bursts ⇒ ≥14 AWG to the PCA9685 V+)
- **common ground** between servo rail, PCA9685 logic, and Jetson — mandatory

Rules: DS3240MG-class servos are typically rated 4.8–6.8 V (HV variants
7.4 V) — verify the regulator output is ≤6.8 V unless the datasheet (Q-002)
proves HV. Add an **inline fuse (15–20 A)** on the rail and a **physical
rail switch** — these are the only e-stops that work when software is dead.

Pre-power continuity checks (multimeter, battery DISCONNECTED):
V+↔GND not shorted; logic 3V3/5V↔GND not shorted; every servo plug
orientation (signal/V+/GND) against the PCA9685 silk screen.

## 1.2 First energize (logic before muscle)

1. Jetson on, servo rail OFF. `i2cdetect -y -r 7` → expect `0x40` and
   `0x68`. (`-r` uses read probes — safe.)
2. Servo rail ON, **no horns attached if servos were removed** (else: robot
   on the stand, legs free). Servos will twitch ≤ a few degrees at power —
   normal. They hold nothing until commanded (driver init forces all-off).
3. Register sanity:
   `~/barq_v1/venv/bin/python - <<'EOF'`
   `import sys; sys.path.insert(0,"stack")`
   `from barq1.pca9685 import PCA9685; p=PCA9685(); print(hex(p.mode1()))`
   `EOF`
   → expect `0xa0` (RESTART|AI). MPU check: read reg 0x75 (WHO_AM_I) → 0x68.

### Failure ladder — nothing at 0x40

a. Rail/logic V present? PCA9685 needs VCC (logic, from Jetson 3V3/5V) even
   if V+ (servo) is off — check both.
b. Common ground present? (No ground = ghost devices / nothing.)
c. Try other buses: `for b in 1 7; do i2cdetect -y -r $b; done` (pins 27/28
   are bus 1). If found on 1, set `I2C_BUS = 1` in truths? NO — move the
   wires to pins 3/5, the truths stay.
d. Cable/solder joints; try a short known-good cable.
e. Address jumpers: A0–A5 solder bridges change 0x40 → 0x41…; scan shows
   where it landed; either clear the bridge or (last resort) change
   PCA_ADDR in truths with a re-measurement note.
f. Board dead → any PCA9685 clone is drop-in (same registers). Worst case
   emergency: Jetson hardware PWM pins can drive ~2 servos directly for
   bench tests, but that's diagnostics, not a path.

### Failure ladder — bus errors / EREMOTEIO under load

Usually power sag or wiring, not software: shorten leads, fatten gauge,
add 470–1000 µF electrolytic across the rail at the PCA9685, ferrite on
the I2C lines, lower I2C clock (add `i2c-` overlay only if desperate —
record everything you change).

## 1.3 Servo health audit (one at a time, channel-by-channel)

For each channel 0–11, with the calibration GUI (`http://barq.local:8035`)
or the REPL snippet in `appendices/A_command_reference.md`:

1. Free-wheel: with output off, move the link by hand — smooth, no grinding.
2. Enable at the slider's current position (place slider near the physical
   pose first — the card shows ticks; eyeball from the mech window center).
3. Nudge ±10 ticks: motion must be small, smooth, single-joint.
4. Listen: buzzing at rest is normal-ish under load; grinding/clicking =
   stripped gears → swap servo (spares!), re-audit.

Record the result per channel in `04_OPEN_QUESTIONS.md` Q-001 and resolve
it. **The legacy channel map (truths.py) is INNOCENT UNTIL PROVEN GUILTY —
if a nudge moves the wrong joint, fix the WIRE to match truths**, don't
edit truths (all mech windows/perp/stand data is keyed to that map).

## 1.4 IMU quick check

```
python stack/barq1/imu.py-less REPL: from barq1.imu import IMU; m=IMU();
m.calibrate(); print(m.update())
```
Flat & still → roll/pitch within ±2°, gyro |bias-corrected| < 1 °/s.
Tilt the chassis by hand → signs follow the convention (+roll = right side
down… verify against URDF: +roll about +x tips LEFT side UP). Note the
observed signs in the research log — phase 5 needs them.

## GATE (phase 1 complete when)

- [ ] Power tree documented (D-entry), fuse + physical switch installed
- [ ] `0x40` + `0x68` on bus 7; MODE1 = 0xa0; WHO_AM_I = 0x68
- [ ] all 12 channels audited; Q-001 resolved (wires match truths)
- [ ] IMU reads sane, signs recorded
- [ ] nothing edited in truths.py
