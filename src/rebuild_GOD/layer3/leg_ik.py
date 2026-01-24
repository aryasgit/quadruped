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
    # THIGH — PLANAR IK (x–z)
    # -------------------------------------------------
    # Link lengths (meters)
    L1 = 0.108   # thigh
    L2 = 0.138   # shin (unused for now)

    # Effective reach in sagittal plane
    r = math.sqrt(x*x + z*z)

    # Safety clamp (geometry only)
    r = max(min(r, L1 + L2 - 1e-3), abs(L1 - L2) + 1e-3)

    # Thigh angle: point leg toward foot in x–z plane
    # delta z from stand (positive = want to go down)
    STAND_Z = -0.18
    dz = z - STAND_Z

    THIGH_GAIN = -220.0   # deg per meter (now safe)
    theta_thigh = THIGH_GAIN * dz
    theta_thigh = max(min(theta_thigh, 45.0), -45.0)



    # -------------------------------------------------
    # WRIST — LOCKED
    # -------------------------------------------------
    # -------------------------------------------------
    # WRIST — COMPLEMENT THIGH (TEMPORARY)
    # -------------------------------------------------
    # -------------------------------------------------
    # WRIST — KNEE COMPENSATION FOR VERTICAL MOTION
    # -------------------------------------------------

    # Wrist bends forward more than thigh bends back
    WRIST_GAIN = -1.6    # stronger than thigh
    theta_wrist = WRIST_GAIN * abs(theta_thigh)

    theta_wrist = max(min(theta_wrist, 60.0), -60.0)



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
