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

## Current frontier

- **Hardware**: robot not yet powered this revival. First power-up:
  `i2cdetect -y -r 7` → calibration GUI → Q-001 channel check → 12 true
  zeros → commit `stack/config/servo_calibration.yaml`.
- **Sim**: Stage B done — IK (9/9 tests) + PyBullet scenarios all pass; the
  open-loop leg lift works (05, 2026-06-12). Next: **crawl gait generator**
  in sim (spotMicro 8-phase as the template), then the servo-map layer
  (IK angles → calibrated ticks) once the calibration YAML exists.
- Open questions: Q-001 channel map, Q-002 servo datasheet, Q-003 leg-link
  caliper check, Q-004 mass audit.

## How to run things

See `01_STATUS.md` § How to run. GUIs from the Mac: calibration at
`http://barq.local:8035`; sim GUI via VNC (`sudo ~/fix_display.sh`,
`~/setup_vnc.sh`, then `DISPLAY=:0 ...--gui`).
