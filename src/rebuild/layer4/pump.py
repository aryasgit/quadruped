"""
All-legs mirrored XYZ test

Behavior:
 - Left legs (FL, RL) follow the base + offset
 - Right legs (FR, RR) follow the base - offset (exact inverse)
 - Offsets applied to X, Y, Z components (mirrored across the body)
 - Uses solve_leg_ik(...) -> conventions -> normalize_all -> set_servo_angle
 - Slow sine motion, safe defaults
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

# Sign multiplier: left = +1, right = -1
SIDE_SIGN = {
    "FL": +1,
    "RL": +1,
    "FR": -1,
    "RR": -1,
}

def send_leg_xyz(leg, x, y, z):
    # Solve IK (returns deltas in degrees)
    coxa, thigh, wrist = solve_leg_ik(x, y, z, leg)

    deltas = {
        f"{leg}_COXA":  coxa,
        f"{leg}_THIGH": thigh,
        f"{leg}_WRIST": wrist,
    }

    # Conventions + normalize
    deltas = apply_joint_conventions(deltas)
    physical = normalize_all(deltas)

    # Send to hardware
    set_servo_angle(COXA[leg],  physical[f"{leg}_COXA"])
    set_servo_angle(THIGHS[THIGH_MAP[leg]], physical[f"{leg}_THIGH"])
    set_servo_angle(WRISTS[WRIST_MAP[leg]], physical[f"{leg}_WRIST"])


def main():
    # Base hip-local positions (meters)
    BASE_X = 0.0
    BASE_Y = {
        "FL":  0.06,
        "FR": -0.06,
        "RL":  0.06,
        "RR": -0.06,
    }
    BASE_Z = -0.18

    # Offsets amplitude (meters) for each axis
    AMP_X = 0.02   # forward/back
    AMP_Y = 0.01   # left/right
    AMP_Z = 0.02   # up/down

    FREQ = 0.25    # Hz (slow and safe)
    DT = 0.03      # control loop step

    t = 0.0
    print("Running ALL-LEGS MIRRORED XYZ test — CTRL+C to stop")

    try:
        while True:
            # base sinusoidal offsets (same for all legs, mirrored by SIDE_SIGN)
            off_x = AMP_X * math.sin(2 * math.pi * FREQ * t)
            off_y = AMP_Y * math.sin(2 * math.pi * FREQ * t)
            off_z = AMP_Z * math.sin(2 * math.pi * FREQ * t)

            for leg in LEGS:
                sign = SIDE_SIGN[leg]

                # Apply inverse for right legs by multiplying offset by sign
                x = BASE_X + sign * off_x
                y = BASE_Y[leg] + sign * off_y
                z = BASE_Z + sign * off_z

                send_leg_xyz(leg, x, y, z)

            t += DT
            time.sleep(DT)

    except KeyboardInterrupt:
        print("\nStopped cleanly.")


if __name__ == "__main__":
    main()
