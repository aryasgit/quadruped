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

def stand():
    """
    Compute stand pose from geometry and command servos.
    """
    # 1. IK — geometry → joint deltas
    deltas = solve_all_legs(STAND_FEET)
    deltas = apply_joint_conventions(deltas)
    physical = normalize_all(deltas)


    # 3. Command servos
    for joint, angle in physical.items():
        ch = SERVO_CHANNELS[joint]
        set_servo_angle(ch, angle)



# -------------------------------------------------
# Smoke test — THIS WILL MOVE THE ROBOT
# -------------------------------------------------

if __name__ == "__main__":
    print("[L4] Executing stand() — robot WILL MOVE")
    stand()
