"""
Layer 3 — LEG INVERSE KINEMATICS (IK)
===================================

Pure math. Zero hardware. Zero servos. Zero balance.

This file is a CLEAN extraction of the SpotMicro leg IK logic.
It answers ONE question:

    Given a desired foot position relative to the hip,
    what are the joint angle DELTAS (coxa, thigh, wrist)?

Outputs are:
- degrees
- delta angles (0 = stand reference)
- directly consumable by Layer 2 (joint_space)

Coordinate convention (matches SpotMicro):
- x : forward (+)
- y : left (+)
- z : up (+)
"""

import math

# -------------------------------------------------
# Geometry constants (SpotMicro dimensions, meters)
# -------------------------------------------------
# These values are taken directly from the SpotMicro
# inverse kinematics reference implementation.

L0 = 0.055   # Coxa length (hip offset)
L1 = 0.107  # Thigh length
L2 = 0.130  # Wrist / shin length

# -------------------------------------------------
# Core leg IK solver
# -------------------------------------------------

def solve_leg_ik(x: float, y: float, z: float, side: str):
    """
    Solve inverse kinematics for a single leg.

    Inputs:
        x, y, z : foot position relative to hip (meters)
        side    : 'L' or 'R'

    Returns:
        (d_coxa, d_thigh, d_wrist) in DEGREES
        These are DELTA angles relative to stand pose.
    """

    # Side correction (right legs mirror Y)
    if side == 'R':
        y = -y

    # ----------------------------
    # Coxa (hip yaw)
    # ----------------------------
    theta_coxa = math.atan2(y, x)

    # Effective horizontal distance from hip after coxa
    r = math.sqrt(x*x + y*y) - L0

    # Distance from hip joint to foot
    d = math.sqrt(r*r + z*z)

    # Safety clamp
    d = max(min(d, L1 + L2 - 1e-6), abs(L1 - L2) + 1e-6)

    # ----------------------------
    # Knee (wrist)
    # ----------------------------
    cos_knee = (L1*L1 + L2*L2 - d*d) / (2 * L1 * L2)
    theta_wrist = math.pi - math.acos(cos_knee)

    # ----------------------------
    # Thigh (hip pitch)
    # ----------------------------
    alpha = math.atan2(z, r)
    cos_beta = (L1*L1 + d*d - L2*L2) / (2 * L1 * d)
    beta = math.acos(cos_beta)
    theta_thigh = alpha + beta

    # ----------------------------
    # Convert to DEGREES
    # ----------------------------
    d_coxa  = math.degrees(theta_coxa)
    d_thigh = math.degrees(theta_thigh)
    d_wrist = math.degrees(theta_wrist)

    return d_coxa, d_thigh, d_wrist


# -------------------------------------------------
# Convenience: solve all 4 legs
# -------------------------------------------------

def solve_all_legs(foot_targets: dict):
    """
    foot_targets example:
    {
        'FL': (x, y, z),
        'FR': (x, y, z),
        'RR': (x, y, z),
        'RL': (x, y, z),
    }

    Returns:
        dict mapping joint_name -> delta_angle
    """
    out = {}

    for leg, (x, y, z) in foot_targets.items():
        side = 'L' if leg in ('FL', 'RL') else 'R'
        d_coxa, d_thigh, d_wrist = solve_leg_ik(x, y, z, side)

        out[f"{leg}_COXA"]  = d_coxa
        out[f"{leg}_THIGH"] = d_thigh
        out[f"{leg}_WRIST"] = d_wrist

    return out


# -------------------------------------------------
# Smoke test (math only)
# -------------------------------------------------

if __name__ == "__main__":
    # Simple neutral test: foot straight down
    test_targets = {
        'FL': (0.15,  0.05, -0.18),
        'FR': (0.15, -0.05, -0.18),
        'RR': (-0.15, -0.05, -0.18),
        'RL': (-0.15,  0.05, -0.18),
    }

    res = solve_all_legs(test_targets)
    print("[L3] IK delta outputs:")
    for k, v in res.items():
        print(f"{k:10s} -> {v:+7.2f}°")
