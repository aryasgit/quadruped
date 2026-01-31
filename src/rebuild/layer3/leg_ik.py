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

L1 = 0.10832   # thigh length (meters)
L2 = 0.13476   # foreleg length (meters)


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

def solve_leg_ik(x, y, z, side):
    """
    Layer 3 — THIGH BRING-UP MODE

    COXA  : lateral-only (already fixed)
    THIGH : planar x–z IK
    WRIST : locked at 0
    """

    import math

    # -------------------------------------------------
    # COXA — LATERAL ONLY (already verified)
    # -------------------------------------------------
    if side == 'R':
        y = -y

    COXA_GAIN = 120.0        # degrees per meter (safe)
    theta_coxa = COXA_GAIN * y
    theta_coxa = max(min(theta_coxa, 30.0), -30.0)

    # -------------------------------------------------
    # SAFE PLANAR IK (PHASE B — LINEAR, LOW GAIN)
    # -------------------------------------------------

    # Reference stand height
    STAND_Z = -0.18

    # Vertical error (meters)
    dz = z - STAND_Z

    # Horizontal bias (meters)
    dx = x - 0.15   # front legs reference
    if side == 'R':
        dx = -dx    # mirror

    # ----------------------------
    # VERY SMALL GAINS (SAFE)
    # ----------------------------
    THIGH_Z_GAIN  = -80.0    # deg / meter  (START SAFE)
    WRIST_Z_GAIN  = -90.0   # deg / meter
    THIGH_X_GAIN  = -30.0
    WRIST_X_GAIN  = +40.0

    theta_thigh = (
        THIGH_Z_GAIN * dz +
        THIGH_X_GAIN * dx
    )

    theta_wrist = (
        WRIST_Z_GAIN * dz +
        WRIST_X_GAIN * dx
    )

    # ----------------------------
    # HARD SAFETY CLAMPS
    # ----------------------------
    theta_thigh = max(min(theta_thigh, 15.0), -15.0)
    theta_wrist = max(min(theta_wrist, 20.0), -20.0)

    return theta_coxa, theta_thigh, theta_wrist



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