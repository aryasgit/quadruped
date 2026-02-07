"""
TEST — Layer 6 PERFECT ARC (ALL LEGS, SAME TRAJECTORY)
=====================================================

Purpose:
- All four legs trace IDENTICAL arcs
- No mirroring
- No hacks
- Pure Layer 6 + IK validation

This is a VISUAL DEMO TEST ONLY.
"""

import time

# ---------------- Layer 6 ----------------
from layer6.gait_generator import _leg_trajectory

# ---------------- Layer 3 ----------------
from layer3.leg_ik import solve_all_legs

# ---------------- Layer 2 ----------------
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

# ---------------- Hardware ----------------
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# -------------------------------------------------
# CONFIG
# -------------------------------------------------

FREQ = 0.4
DT = 0.03

STEP_LENGTH = 0.10
STEP_HEIGHT = 0.045
DUTY = 0.60

STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18


# -------------------------------------------------
# Servo map
# -------------------------------------------------

CHANNELS = {
    "FL_COXA":  COXA["FL"],
    "FL_THIGH": THIGHS["TFL"],
    "FL_WRIST": WRISTS["WFL"],

    "FR_COXA":  COXA["FR"],
    "FR_THIGH": THIGHS["TFR"],
    "FR_WRIST": WRISTS["WFR"],

    "RL_COXA":  COXA["RL"],
    "RL_THIGH": THIGHS["TRL"],
    "RL_WRIST": WRISTS["WRL"],

    "RR_COXA":  COXA["RR"],
    "RR_THIGH": THIGHS["TRR"],
    "RR_WRIST": WRISTS["WRR"],
}


# -------------------------------------------------
# MAIN
# -------------------------------------------------

print("[TEST] Layer 6 — PERFECT ARC (ALL LEGS SAME)")
print("No mirroring. No hacks. CTRL+C to stop.")

t0 = time.time()

try:
    while True:
        t = time.time() - t0
        phase = (t * FREQ) % 1.0

        dx, dz = _leg_trajectory(
            phase,
            STEP_LENGTH,
            STEP_HEIGHT,
            DUTY,
        )

        # SAME target for all legs (only Y differs)
        feet = {
            "FL": (STANCE_X + dx,  STANCE_Y, STANCE_Z + dz),
            "FR": (STANCE_X + dx, -STANCE_Y, STANCE_Z + dz),
            "RL": (STANCE_X + dx,  STANCE_Y, STANCE_Z + dz),
            "RR": (STANCE_X + dx, -STANCE_Y, STANCE_Z + dz),
        }

        deltas = solve_all_legs(feet)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)

        for joint, ch in CHANNELS.items():
            set_servo_angle(ch, physical[joint])

        time.sleep(DT)

except KeyboardInterrupt:
    print("\n[TEST] Stopped cleanly 🫡")
