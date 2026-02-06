"""
Layer 3 — LEG INVERSE KINEMATICS (IK)
===================================

Pure math. Zero hardware. Zero servos. Zero balance.

Contract:
- Input foot positions are HIP-LOCAL (not body-frame)
- Output angles are DELTAS (0 = stand pose)
- Units: degrees

Coordinate convention:
- x : forward (+)
- y : left (+)
- z : up (+)
"""

import math

# -------------------------------------------------
# Geometry constants (SpotMicro, meters)
# -------------------------------------------------

L0 = 0.055   # Coxa length (hip offset)
L1 = 0.107   # Thigh length
L2 = 0.130   # Wrist / shin length

# -------------------------------------------------
# Stand pose (HIP-LOCAL)
# -------------------------------------------------
# This MUST match your controller STANCE Z
STAND_Z = -0.18
STAND_DX = 0.0   # foot directly under hip in X at stand

# -------------------------------------------------
# Precompute stand reference angles (ONCE)
# -------------------------------------------------

def _compute_stand_angles():
    dx = STAND_DX
    dz = STAND_Z

    r = math.sqrt(dx*dx + dz*dz)
    r = max(min(r, L1 + L2 - 1e-6), abs(L1 - L2) + 1e-6)

    cos_knee = (L1*L1 + L2*L2 - r*r) / (2*L1*L2)
    knee = math.acos(cos_knee)

    alpha = math.atan2(dz, dx)
    cos_beta = (L1*L1 + r*r - L2*L2) / (2*L1*r)
    beta = math.acos(cos_beta)

    thigh = alpha - beta
    wrist = math.pi - knee

    return thigh, wrist


THIGH_0_RAD, WRIST_0_RAD = _compute_stand_angles()

# -------------------------------------------------
# Core leg IK solver
# -------------------------------------------------

def solve_leg_ik(x, y, z, side):
    """
    Inputs:
        x, y, z : HIP-LOCAL foot target (meters)
        side    : 'L' or 'R'

    Returns:
        (theta_coxa, theta_thigh, theta_wrist) in DEGREES
        All are DELTAS relative to stand pose
    """

    # ---------------- COXA (lateral only) ----------------
    if side == 'R':
        y = -y

    theta_coxa = 120.0 * y
    theta_coxa = max(min(theta_coxa, 30.0), -30.0)

    # ---------------- PLANAR IK (x–z) ----------------
    dx = x
    dz = z

    if side == 'R':
        dx = -dx

    r = math.sqrt(dx*dx + dz*dz)
    r = max(min(r, L1 + L2 - 1e-6), abs(L1 - L2) + 1e-6)

    cos_knee = (L1*L1 + L2*L2 - r*r) / (2*L1*L2)
    knee = math.acos(cos_knee)

    alpha = math.atan2(dz, dx)
    cos_beta = (L1*L1 + r*r - L2*L2) / (2*L1*r)
    beta = math.acos(cos_beta)

    thigh = alpha - beta
    wrist = math.pi - knee

    # ---------------- DELTA FROM STAND ----------------
    theta_thigh = math.degrees(thigh - THIGH_0_RAD)
    theta_wrist = math.degrees(wrist - WRIST_0_RAD)

    # ---------------- SAFETY CLAMPS ----------------
    theta_thigh = max(min(theta_thigh, 45.0), -45.0)
    theta_wrist = max(min(theta_wrist, 70.0), -70.0)

    return theta_coxa, theta_thigh, theta_wrist


# -------------------------------------------------
# Solve all legs
# -------------------------------------------------

def solve_all_legs(foot_targets: dict):
    """
    foot_targets:
    {
        'FL': (x, y, z),
        'FR': (x, y, z),
        'RL': (x, y, z),
        'RR': (x, y, z),
    }
    """
    out = {}

    for leg, (x, y, z) in foot_targets.items():
        side = 'L' if leg in ('FL', 'RL') else 'R'
        coxa, thigh, wrist = solve_leg_ik(x, y, z, side)

        out[f"{leg}_COXA"]  = coxa
        out[f"{leg}_THIGH"] = thigh
        out[f"{leg}_WRIST"] = wrist

    return out


# -------------------------------------------------
# Smoke test
# -------------------------------------------------

if __name__ == "__main__":

    # Hip-local neutral stance
    test_targets = {
        'FL': (0.0,  0.07, -0.18),
        'FR': (0.0, -0.07, -0.18),
        'RL': (0.0,  0.07, -0.18),
        'RR': (0.0, -0.07, -0.18),
    }

    res = solve_all_legs(test_targets)
    print("[L3] IK delta outputs (stand):")
    for k, v in res.items():
        print(f"{k:10s} -> {v:+7.3f}°")
