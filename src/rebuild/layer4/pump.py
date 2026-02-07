"""
All-legs XYZ symmetry test (POST-CONVENTIONS)

Behavior:
 - ALL legs receive the SAME hip-local XYZ offsets
 - Left/right differences handled ONLY by:
     • base Y sign
     • Layer 2.5 joint conventions
 - NO mirroring hacks
 - NO per-leg sign flips

This test VALIDATES that:
 - Layer 6/3 math is symmetric
 - Layer 2.5 fixes real hardware mirroring
"""

import time
import math

from layer3.leg_ik import solve_leg_ik
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS

LEGS = ["FL", "FR", "RL", "RR"]

THIGH_MAP = {"FL": "TFL", "FR": "TFR", "RL": "TRL", "RR": "TRR"}
WRIST_MAP = {"FL": "WFL", "FR": "WFR", "RL": "WRL", "RR": "WRR"}


def send_leg_xyz(leg, x, y, z):
    # Solve IK (returns deltas in degrees)
    coxa, thigh, wrist = solve_leg_ik(x, y, z, leg)

    deltas = {
        f"{leg}_COXA":  coxa,
        f"{leg}_THIGH": thigh,
        f"{leg}_WRIST": wrist,
    }

    # Apply canonical conventions
    deltas = apply_joint_conventions(deltas)
    physical = normalize_all(deltas)

    # Send to hardware
    set_servo_angle(COXA[leg],  physical[f"{leg}_COXA"])
    set_servo_angle(THIGHS[THIGH_MAP[leg]], physical[f"{leg}_THIGH"])
    set_servo_angle(WRISTS[WRIST_MAP[leg]], physical[f"{leg}_WRIST"])


def main():
    # Base hip-local positions (meters)
    BASE_X = 0.0
    BASE_Z = -0.18

    BASE_Y = {
        "FL":  0.06,
        "FR": -0.06,
        "RL":  0.06,
        "RR": -0.06,
    }

    # Offset amplitudes (meters)
    AMP_X = 0.02
    AMP_Y = 0.01
    AMP_Z = 0.05

    FREQ = 0.25
    DT = 0.03

    t = 0.0
    print("Running ALL-LEGS XYZ symmetry test — CTRL+C to stop")

    try:
        while True:
            # SAME offsets for ALL legs
            off_x = AMP_X * math.sin(2 * math.pi * FREQ * t)
            off_y = AMP_Y * math.sin(2 * math.pi * FREQ * t)
            off_z = AMP_Z * math.sin(2 * math.pi * FREQ * t)

            for leg in LEGS:
                x = BASE_X + off_x
                y = BASE_Y[leg] + off_y
                z = BASE_Z + off_z

                send_leg_xyz(leg, x, y, z)

            t += DT
            time.sleep(DT)

    except KeyboardInterrupt:
        print("\nStopped cleanly.")


if __name__ == "__main__":
    main()
