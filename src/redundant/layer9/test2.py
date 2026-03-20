#!/usr/bin/env python3

"""
Layer 9.5 — Cinematic Pose Sequencer
=====================================

Flashy demo poses with:
- Smooth transitions
- Coxa involvement
- Body attitude control
- CG-friendly low amplitudes
- CLI selectable mode

SAFE DEFAULT MAGNITUDES
"""

import time
import math
import argparse

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


STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18


# -------------------------------------------------
# Utilities
# -------------------------------------------------

def smoothstep(t):
    return t * t * (3 - 2 * t)


def interpolate_pose(p0, p1, t):
    s = smoothstep(t)
    return {
        leg: (
            p0[leg][0] + (p1[leg][0] - p0[leg][0]) * s,
            p0[leg][1] + (p1[leg][1] - p0[leg][1]) * s,
            p0[leg][2] + (p1[leg][2] - p0[leg][2]) * s,
        )
        for leg in p0
    }


def send_pose(feet):
    deltas = solve_all_legs(feet)
    deltas = apply_joint_conventions(deltas)
    physical = normalize_all(deltas)

    for joint, ch in CHANNELS.items():
        set_servo_angle(ch, physical[joint])


# -------------------------------------------------
# Base pose
# -------------------------------------------------

def stand_pose():
    return {
        "FL": (STANCE_X,  STANCE_Y, STANCE_Z),
        "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
        "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
        "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
    }


# -------------------------------------------------
# Poses
# -------------------------------------------------

def sit_pose():
    z = -0.13
    return {
        "FL": (0.02,  STANCE_Y, z),
        "FR": (0.02, -STANCE_Y, z),
        "RL": (-0.02, STANCE_Y, z),
        "RR": (-0.02,-STANCE_Y, z),
    }


def kneel_pose():
    z = -0.14
    return {
        "FL": (0.03,  STANCE_Y, z),
        "FR": (0.03, -STANCE_Y, z),
        "RL": (0.00,  STANCE_Y, STANCE_Z),
        "RR": (0.00, -STANCE_Y, STANCE_Z),
    }


def lean_left_pose():
    return {
        "FL": (0.00,  STANCE_Y + 0.02, STANCE_Z),
        "FR": (0.00, -STANCE_Y + 0.02, STANCE_Z),
        "RL": (0.00,  STANCE_Y + 0.02, STANCE_Z),
        "RR": (0.00, -STANCE_Y + 0.02, STANCE_Z),
    }


def twist_pose():
    return {
        "FL": (0.03,  STANCE_Y, STANCE_Z),
        "FR": (-0.03,-STANCE_Y, STANCE_Z),
        "RL": (-0.03, STANCE_Y, STANCE_Z),
        "RR": (0.03, -STANCE_Y, STANCE_Z),
    }


def bow_pose():
    return {
        "FL": (0.05,  STANCE_Y, -0.15),
        "FR": (0.05, -STANCE_Y, -0.15),
        "RL": (0.00,  STANCE_Y, STANCE_Z),
        "RR": (0.00, -STANCE_Y, STANCE_Z),
    }


def wave_pose():
    pose = stand_pose()
    pose["FL"] = (0.05, STANCE_Y + 0.03, -0.10)
    return pose


# -------------------------------------------------
# Sequencer
# -------------------------------------------------

def run_sequence(poses, duration=1.2):
    for i in range(len(poses) - 1):
        p0 = poses[i]
        p1 = poses[i + 1]

        t0 = time.time()
        while True:
            t = (time.time() - t0) / duration
            if t >= 1.0:
                break
            feet = interpolate_pose(p0, p1, t)
            send_pose(feet)
            time.sleep(0.02)

        send_pose(p1)
        time.sleep(0.3)


# -------------------------------------------------
# CLI
# -------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        type=str,
        required=True,
        help="sit, kneel, lean, twist, bow, wave, sitwave"
    )

    args = parser.parse_args()

    base = stand_pose()

    if args.mode == "sit":
        run_sequence([base, sit_pose(), base])

    elif args.mode == "kneel":
        run_sequence([base, kneel_pose(), base])

    elif args.mode == "lean":
        run_sequence([base, lean_left_pose(), base])

    elif args.mode == "twist":
        run_sequence([base, twist_pose(), base])

    elif args.mode == "bow":
        run_sequence([base, bow_pose(), base])

    elif args.mode == "wave":
        run_sequence([base, wave_pose(), base])

    elif args.mode == "sitwave":
        run_sequence([base, sit_pose(), wave_pose(), sit_pose(), base])

    else:
        print("Unknown mode.")
