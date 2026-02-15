#!/usr/bin/env python3
"""
Layer 9 — Flashy Demo CLI
Safe, scalable, IK-based demo motions
"""

import time
import math
import argparse

# --- Your stack ---
from layer3.leg_ik import solve_all_legs
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS


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
# Nominal stance
# -------------------------------------------------

STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18


# -------------------------------------------------
# Servo writer
# -------------------------------------------------

def send_to_servos(deltas):
    deltas = apply_joint_conventions(deltas)
    physical = normalize_all(deltas)

    for joint, ch in CHANNELS.items():
        set_servo_angle(ch, physical[joint])


# -------------------------------------------------
# Smooth stand
# -------------------------------------------------

def smooth_stand(duration=1.5):
    print("Standing up smoothly...")
    t0 = time.time()

    while True:
        t = time.time() - t0
        s = min(t / duration, 1.0)

        z = -0.10 - 0.08 * s  # interpolate to -0.18

        feet = {
            "FL": (0,  STANCE_Y, z),
            "FR": (0, -STANCE_Y, z),
            "RL": (0,  STANCE_Y, z),
            "RR": (0, -STANCE_Y, z),
        }

        deltas = solve_all_legs(feet)
        send_to_servos(deltas)

        if s >= 1.0:
            break

        time.sleep(0.02)


# -------------------------------------------------
# POSES
# -------------------------------------------------

def pose_sway(scale):
    """Side to side body sway"""
    t = time.time()
    amp = 0.02 * scale

    shift = amp * math.sin(t * 1.5)

    return {
        "FL": (0,  STANCE_Y + shift, STANCE_Z),
        "FR": (0, -STANCE_Y + shift, STANCE_Z),
        "RL": (0,  STANCE_Y + shift, STANCE_Z),
        "RR": (0, -STANCE_Y + shift, STANCE_Z),
    }


def pose_twist(scale):
    """Diagonal torque twist using coxa"""
    t = time.time()
    amp = 0.03 * scale

    dx = amp * math.sin(t * 2.0)

    return {
        "FL": ( dx,  STANCE_Y, STANCE_Z),
        "FR": (-dx, -STANCE_Y, STANCE_Z),
        "RL": (-dx,  STANCE_Y, STANCE_Z),
        "RR": ( dx, -STANCE_Y, STANCE_Z),
    }


def pose_bounce(scale):
    """Energetic vertical bounce"""
    t = time.time()
    amp = 0.03 * scale

    dz = amp * abs(math.sin(t * 2.5))

    return {
        "FL": (0,  STANCE_Y, STANCE_Z + dz),
        "FR": (0, -STANCE_Y, STANCE_Z + dz),
        "RL": (0,  STANCE_Y, STANCE_Z + dz),
        "RR": (0, -STANCE_Y, STANCE_Z + dz),
    }


def pose_party(scale):
    """Flashy alternating diagonal lift + twist"""
    t = time.time()
    amp_xy = 0.03 * scale
    amp_z  = 0.04 * scale

    phase = math.sin(t * 3.0)

    if phase > 0:
        return {
            "FL": ( amp_xy,  STANCE_Y, STANCE_Z + amp_z),
            "RR": ( amp_xy, -STANCE_Y, STANCE_Z + amp_z),
            "FR": (-amp_xy, -STANCE_Y, STANCE_Z),
            "RL": (-amp_xy,  STANCE_Y, STANCE_Z),
        }
    else:
        return {
            "FL": (-amp_xy,  STANCE_Y, STANCE_Z),
            "RR": (-amp_xy, -STANCE_Y, STANCE_Z),
            "FR": ( amp_xy, -STANCE_Y, STANCE_Z + amp_z),
            "RL": ( amp_xy,  STANCE_Y, STANCE_Z + amp_z),
        }


POSES = {
    "sway": pose_sway,
    "twist": pose_twist,
    "bounce": pose_bounce,
    "party": pose_party,
}


# -------------------------------------------------
# MAIN
# -------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", type=str, default="sway")
    parser.add_argument("--scale", type=float, default=1.0)
    args = parser.parse_args()

    if args.pose not in POSES:
        print("Available poses:", list(POSES.keys()))
        return

    smooth_stand()

    print(f"Running pose: {args.pose}")
    fn = POSES[args.pose]

    try:
        while True:
            feet = fn(args.scale)
            deltas = solve_all_legs(feet)
            send_to_servos(deltas)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping cleanly.")


if __name__ == "__main__":
    main()
