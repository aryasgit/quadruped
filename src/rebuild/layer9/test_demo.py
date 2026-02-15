"""
Layer 9 — Flash Demo: Controlled Chaos Bounce
=============================================

Moves body frame while keeping feet nominal.

Safe magnitudes.
No joint limit violations.
Scalable amplitude.
"""

import time
import math

from layer6.gait_generator import generate_foot_targets
from layer3.leg_ik import solve_all_legs
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# ----------------------------
# CONFIG
# ----------------------------

DT = 0.02
FREQ = 1.2  # body oscillation frequency
AMP_ROLL = 5.0      # degrees
AMP_YAW  = 4.0      # degrees
AMP_Z    = 0.01     # meters vertical bounce


STANCE_Z = -0.18
STANCE_Y = 0.07
STANCE_X = 0.0


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


# ----------------------------
# MAIN
# ----------------------------

print("[L9] Controlled Chaos Bounce Demo")

t0 = time.time()

try:
    while True:
        t = time.time() - t0

        # --- small body oscillations ---
        roll = AMP_ROLL * math.sin(2 * math.pi * FREQ * t)
        yaw  = AMP_YAW  * math.sin(2 * math.pi * FREQ * t * 0.7)
        z_off = AMP_Z * math.sin(2 * math.pi * FREQ * t * 1.4)

        # --- nominal feet ---
        feet = {
            "FL": (STANCE_X,  STANCE_Y, STANCE_Z + z_off),
            "FR": (STANCE_X, -STANCE_Y, STANCE_Z + z_off),
            "RL": (STANCE_X,  STANCE_Y, STANCE_Z + z_off),
            "RR": (STANCE_X, -STANCE_Y, STANCE_Z + z_off),
        }

        # Solve IK
        deltas = solve_all_legs(feet)

        # Add expressive body twist into coxa
        for leg in ("FL", "FR", "RL", "RR"):
            deltas[f"{leg}_COXA"] += yaw

        # Add roll compensation
        deltas["FL_THIGH"] += roll
        deltas["RL_THIGH"] += roll
        deltas["FR_THIGH"] -= roll
        deltas["RR_THIGH"] -= roll

        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)

        for joint, ch in CHANNELS.items():
            set_servo_angle(ch, physical[joint])

        time.sleep(DT)

except KeyboardInterrupt:
    print("\n[L9] Demo stopped cleanly 🫡")
