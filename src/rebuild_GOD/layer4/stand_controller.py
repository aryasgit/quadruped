"""
Layer 4 — STAND GEOMETRY & FIRST MOTION BRIDGE
=============================================

This is the FIRST layer where math is allowed to touch hardware.

Pipeline implemented here:

    Stand foot geometry
        ↓
    Layer 3 (IK) → joint deltas
        ↓
    Layer 2 (joint normalization) → physical angles
        ↓
    Layer 1 (PCA9685) → servo motion

This file exists for ONE purpose:
    Make the robot stand using math, not hardcoded angles.

No gait. No balance. No IMU.
"""
from layer2.joint_conventions import apply_joint_conventions
from layer3.leg_ik import solve_all_legs
from layer2.joint_space import normalize_all
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import WRISTS, THIGHS, COXA
STAND_Z = -0.18


# -------------------------------------------------
# Servo channel lookup (joint_name → PCA channel)
# -------------------------------------------------

SERVO_CHANNELS = {
    # Front Left
    "FL_COXA":  COXA["FL"],
    "FL_THIGH": THIGHS["TFL"],
    "FL_WRIST": WRISTS["WFL"],

    # Front Right
    "FR_COXA":  COXA["FR"],
    "FR_THIGH": THIGHS["TFR"],
    "FR_WRIST": WRISTS["WFR"],

    # Rear Right
    "RR_COXA":  COXA["RR"],
    "RR_THIGH": THIGHS["TRR"],
    "RR_WRIST": WRISTS["WRR"],

    # Rear Left
    "RL_COXA":  COXA["RL"],
    "RL_THIGH": THIGHS["TRL"],
    "RL_WRIST": WRISTS["WRL"],
}

# -------------------------------------------------
# Stand foot geometry (meters, BODY FRAME)
# -------------------------------------------------
# These values define WHAT "standing" means geometrically.
# Adjust Z only for height tuning later.

STAND_FEET = {
    "FL": ( 0.15,  0.05, -0.22),
    "FR": ( 0.15, -0.05, -0.22),
    "RR": (-0.15, -0.05, -0.22),
    "RL": (-0.15,  0.05, -0.22),
}

# -------------------------------------------------
# Stand execution
# -------------------------------------------------

import time

def stand_squat_test():
    """
    Layer 4 integration test:
    Smooth stand → squat → stand
    """

    zs = [z * 0.001 for z in range(-180, -221, -2)]
    zs += [z * 0.001 for z in range(-220, -179, 2)]


    for z in zs:
        foot_targets = {
            "FL": ( 0.15,  0.05, z),
            "FR": ( 0.15, -0.05, z),
            "RR": (-0.15, -0.05, z),
            "RL": (-0.15,  0.05, z),
        }

        deltas = solve_all_legs(foot_targets)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)

        for joint, angle in physical.items():
            ch = SERVO_CHANNELS[joint]
            set_servo_angle(ch, angle)

        time.sleep(0.01)

import time

from hardware.absolute_truths import (
    COXA_STAND,
    THIGH_STAND,
    WRIST_STAND,
)

def coxa_isolation_test():
    ys = [y * 0.001 for y in range(30, 121, 5)]
    ys += [y * 0.001 for y in range(120, 29, -5)]

    for y in ys:
        foot_targets = {
            "FL": ( 0.15,  y, -0.18),
            "FR": ( 0.15, -y, -0.18),
            "RR": (-0.15, -y, -0.18),
            "RL": (-0.15,  y, -0.18),
        }

        deltas = solve_all_legs(foot_targets)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)

        # 🔒 HARD OVERRIDE — ABSOLUTE ANGLES
        physical["FL_THIGH"] = THIGH_STAND["TFL"]
        physical["FR_THIGH"] = THIGH_STAND["TFR"]
        physical["RL_THIGH"] = THIGH_STAND["TRL"]
        physical["RR_THIGH"] = THIGH_STAND["TRR"]

        physical["FL_WRIST"] = WRIST_STAND["WFL"]
        physical["FR_WRIST"] = WRIST_STAND["WFR"]
        physical["RL_WRIST"] = WRIST_STAND["WRL"]
        physical["RR_WRIST"] = WRIST_STAND["WRR"]

        for joint, angle in physical.items():
            ch = SERVO_CHANNELS[joint]
            set_servo_angle(ch, angle)

        time.sleep(0.05)


# -------------------------------------------------
# Smoke test — THIS WILL MOVE THE ROBOT
# -------------------------------------------------

if __name__ == "__main__":
    #print("[L4] Running stand–squat integration test")
    stand_squat_test()
    print("[L4] coxa_isolation_test")
    #coxa_isolation_test()

