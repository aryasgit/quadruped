#!/usr/bin/env python3
"""Derive a PROVISIONAL servo calibration from the legacy truths (roadmap Plan D).

Anchors each servo at the VERIFIED stand pose: the stand-pose IK angle maps to
the legacy stand tick (the calibration GUI confirmed those ticks produce a
correct physical stand), using the geometric slope (+-1.5889 ticks/deg, sign
from each servo's inverted-mount flag).

=> At the stand pose this reproduces the verified-good ticks EXACTLY, regardless
of slope sign, so STANDING is safe. The slope SIGN is a guess and only affects
motion away from stand; verify signs on the stand (small nudges) before walking.
Writes stack/config/servo_calibration.yaml.
"""
import sys, math, datetime
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import yaml
from barq1.kinematics import body_ik, stance_feet_world
from barq1.servos import SERVOS
from barq1.trajectories import STAND_H
from barq1 import truths

SLOPE = (truths.PULSE_MAX - truths.PULSE_MIN) / truths.SERVO_TRAVEL_DEG  # 1.5889
JOINTS = ("coxa", "thigh", "wrist")
OUT = Path(__file__).resolve().parents[1] / "config" / "servo_calibration.yaml"

angles = body_ik(stance_feet_world(STAND_H), body_xyz=(0, 0, STAND_H))
servos_out = {}
for leg, qs in angles.items():
    for joint, q in zip(JOINTS, qs):
        name = f"{leg}_{joint}"
        s = SERVOS[name]
        slope = -SLOPE if s.inverted else SLOPE
        zero = s.stand - slope * math.degrees(q)
        servos_out[name] = {"channel": s.channel,
                            "zero_ticks": round(zero, 2),
                            "slope_ticks_per_deg": round(slope, 4)}
doc = {"meta": {"generated": datetime.date(2026, 6, 17).isoformat(),
                "method": "legacy-derived, stand-anchored (Plan D) — PROVISIONAL; "
                          "slope signs unverified, safe for STAND only",
                "units": "PCA9685 ticks @ 50 Hz"},
       "servos": servos_out}
OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(yaml.safe_dump(doc, sort_keys=False))
print(f"wrote {OUT}")

# Verify: at the stand pose, the map must reproduce the legacy stand ticks EXACTLY
from barq1.servo_map import ServoMap
m = ServoMap.from_yaml(OUT)
print(f"{'servo':10} {'q_stand°':>8} {'cmd_tick':>8} {'legacy_stand':>12}  match")
ok = True
for leg, qs in angles.items():
    for joint, q in zip(JOINTS, qs):
        name = f"{leg}_{joint}"
        cmd = m.ticks_for(name, q)
        legacy = SERVOS[name].stand
        good = cmd == legacy
        ok = ok and good
        print(f"{name:10} {math.degrees(q):8.1f} {cmd:8} {legacy:12}  {'OK' if good else 'MISMATCH'}")
print("\nALL STAND TICKS MATCH VERIFIED VALUES" if ok else "\n*** MISMATCH ***")
print("warnings:", m.warnings or "none")
