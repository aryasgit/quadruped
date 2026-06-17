# HANDOFF — BARQ v1 cold-start bootstrap

_Keep the frontier fresh. Last updated: 2026-06-12_

## Read order

`docs/01_STATUS.md` → `00_OVERVIEW.md` → `02_DECISIONS.md` →
`04_OPEN_QUESTIONS.md` → `05_RESEARCH_LOG.md`.

## Working agreements

1. Docs updated after **every** change/decision — same commit (D-007).
2. Commit per milestone on `Master`, push after each; author = Aryaman Gupta.
3. Numbers or it didn't happen: claims in 05 carry the metric and the script.
4. Supersede decisions in 02, narrate deltas in 05 — never erase.
5. All deps live in `~/barq_v1/venv`; nothing system-wide; never touch
   `~/barq_ws` (v2).
6. Controllers may consume only hardware-available signals (D-009).

## PROVEN — do not re-verify

- Truths port is faithful (joint-space symmetry of mirrored mounts, see 05
  2026-06-11). Bus 7, PCA 0x40, MPU 0x68, 106–535 ticks @ 50 Hz / 270°.
- URDF frame: axle span 0.20750, hip span 0.078, hip link 0.055, legs
  0.1075/0.130, shoulder symmetry — `stack/tools/validate_urdf.py` passes.
- Calibration GUI API: state/command/clamp/two-point-fit/save/all-off —
  exercised end-to-end in sim mode.
- SSH-over-443 push to `aryasgit/quadruped` works; branch `Master` tracks
  origin.

## Current frontier — gait control (D-016) → drive real robot

Hardware is connected + verified (channel map + stand/perp poses good via the
diagnostic GUI). Velocity gait ported from spotMicro and validated in sim
(forward/turn/strafe, statically stable). Immediate next:
1. **Controller FSM** (`barq1/controller.py`, to build): idle/stand/walk +
   transition states, smoothed by the ported RateLimitedFirstOrderFilter
   (`barq1/filters.py`); one `step(cmd)->frame` entry point. Stand state =
   hold stance (no leg cycling → no idle drift); walk state = VelocityGait.
2. Wire teleop (`teleop/drive.py`) + `runtime/run_robot.py` to the FSM.
3. Drive the real robot: derive a calibration from the legacy truths (perp =
   joint-zero anchor + ±1.589 ticks/° slope, signs from inv-mount flags;
   see roadmap phase-2 "Plan D") since stand/perp poses are confirmed good —
   lets us move the robot now without the full 12-servo 2-point recal.
4. Robustness sweeps once Aryaman's measured masses land (Q-004).

## (Superseded) HARDWARE DAY checklist

The sim walks (05, 2026-06-12) and the hardware pipeline is dry-run-clean.
Robot is being reassembled. The day-one sequence, in order:

1. **While reassembling**: record the power tree (Q-002 — 4S 6200 tethered
   → which BEC/regulator, output V/A; DS3240MG voltage spec) and part
   masses (Q-004).
2. Power logic+rail → `i2cdetect -y -r 7` → expect `0x40` (+ `0x68`).
3. Calibration GUI → one-card-at-a-time channel sanity (Q-001).
4. Calibrate all 12 per **docs/06_CALIBRATION_PROTOCOL.md** → Save →
   sanity gates (slope band, `python stack/barq1/servo_map.py`) →
   **commit the YAML**.
5. `run_robot.py --dry-run --scenario walk` (pipeline against the file),
   then ON THE STAND: `--scenario stand` → `pose_sweep` → `weight_shift`
   → `walk --cycles 1`. Telemetry lands in `~/barq_v1/artifacts/`.
6. Ground. Then `--teleop` (PS4).

PS4 one-time setup (needs Aryaman):
- `sudo usermod -aG input barq` then log out/in (evdev read permission).
- Bluetooth: hold SHARE+PS until the bar double-flashes →
  `bluetoothctl` → `scan on` → `pair <MAC>` → `trust <MAC>` →
  `connect <MAC>` ("Wireless Controller"). USB cable works with zero setup.

Open questions: Q-001 channels, Q-002 datasheet+power tree, Q-003 leg-link
caliper, Q-004 masses (incoming), Q-005 INA260 when it arrives.

## How to run things

See `01_STATUS.md` § How to run. GUIs from the Mac: calibration at
`http://barq.local:8035`; sim GUI via VNC (`sudo ~/fix_display.sh`,
`~/setup_vnc.sh`, then `DISPLAY=:0 ...--gui`).
