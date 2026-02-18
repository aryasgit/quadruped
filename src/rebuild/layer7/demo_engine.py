# layer7/demo_engine.py

import math
import time

from layer3.leg_ik import solve_all_legs
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# -------------------------------------------------
# Safe geometry
# -------------------------------------------------

STANCE_Y = 0.07
STANCE_Z = -0.18

SAFE_MIN_Z = -0.22
SAFE_MAX_Z = -0.14

DT = 0.02


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
# Base stance generator
# -------------------------------------------------

def base_stance():
    return {
        "FL": (0.0,  STANCE_Y, STANCE_Z),
        "FR": (0.0, -STANCE_Y, STANCE_Z),
        "RL": (0.0,  STANCE_Y, STANCE_Z),
        "RR": (0.0, -STANCE_Y, STANCE_Z),
    }


# -------------------------------------------------
# Safe executor
# -------------------------------------------------

def execute(feet):
    deltas = solve_all_legs(feet)
    deltas = apply_joint_conventions(deltas)
    physical = normalize_all(deltas)

    for joint, ch in CHANNELS.items():
        set_servo_angle(ch, physical[joint])


# -------------------------------------------------
# MODE 1 — BOW (balanced)
# -------------------------------------------------

def bow(t):
    depth = 0.035 * (0.5 + 0.5 * math.sin(2 * math.pi * 0.5 * t))

    z_front = max(SAFE_MIN_Z, STANCE_Z - depth)
    z_rear  = STANCE_Z + depth * 0.15  # slight rear lift

    return {
        "FL": (0.0,  STANCE_Y, z_front),
        "FR": (0.0, -STANCE_Y, z_front),
        "RL": (0.0,  STANCE_Y, z_rear),
        "RR": (0.0, -STANCE_Y, z_rear),
    }


# -------------------------------------------------
# MODE 2 — BOUNCE
# -------------------------------------------------

def bounce(t):
    amp = 0.018
    z = STANCE_Z + amp * math.sin(2 * math.pi * 2.0 * t)
    z = max(SAFE_MIN_Z, min(SAFE_MAX_Z, z))

    feet = base_stance()
    for leg in feet:
        x, y, _ = feet[leg]
        feet[leg] = (x, y, z)

    return feet


# -------------------------------------------------
# MODE 3 — CORRECT TAIL WAG
# -------------------------------------------------

def tail_wag(t):

    amp = 12.0 * math.sin(2 * math.pi * 1.2 * t)

    feet = base_stance()

    deltas = solve_all_legs(feet)
    deltas = apply_joint_conventions(deltas)

    # CORRECT MIRROR
    deltas["RL_COXA"] += amp
    deltas["RR_COXA"] -= amp

    physical = normalize_all(deltas)

    for joint, ch in CHANNELS.items():
        set_servo_angle(ch, physical[joint])


# -------------------------------------------------
# MODE 4 — HANDSHAKE
# -------------------------------------------------

def handshake(t):
    lift = 0.04 * (0.5 + 0.5 * math.sin(2 * math.pi * 1.5 * t))

    z_lift = min(SAFE_MAX_Z, STANCE_Z + lift)

    return {
        "FL": (0.0,  STANCE_Y, z_lift),
        "FR": (0.0, -STANCE_Y, STANCE_Z),
        "RL": (0.0,  STANCE_Y, STANCE_Z),
        "RR": (0.0, -STANCE_Y, STANCE_Z),
    }


# -------------------------------------------------
# MODE 5 — SIT (correct mirrored rear)
# -------------------------------------------------

def sit(progress):

    drop = 0.045 * progress
    back = 0.035 * progress

    z_rear = max(SAFE_MIN_Z, STANCE_Z - drop)

    return {
        "FL": (0.0,  STANCE_Y, STANCE_Z),
        "FR": (0.0, -STANCE_Y, STANCE_Z),
        "RL": (-back,  STANCE_Y, z_rear),
        "RR": (-back, -STANCE_Y, z_rear),
    }


# -------------------------------------------------
# TEST LOOP
# -------------------------------------------------

if __name__ == "__main__":

    print("=== Layer 7 Demo Engine ===")

    t0 = time.time()

    mode = 3
    modes = ["bow", "bounce", "tail", "handshake"]

    while True:

        t = time.time() - t0

        if modes[mode] == "bow":
            feet = bow(t)
            execute(feet)

        elif modes[mode] == "bounce":
            feet = bounce(t)
            execute(feet)

        elif modes[mode] == "tail":
            tail_wag(t)

        elif modes[mode] == "handshake":
            feet = handshake(t)
            execute(feet)

        time.sleep(DT)
