"""
Layer 3 — LEG INVERSE KINEMATICS (IK)
===================================

Pure geometry. No hardware. No balance.

Uses exact 3D IK (SpotMicro-style).

Contract:
- Input foot positions are HIP-LOCAL
- Output angles are DELTAS from stand
- Units: degrees
"""

import math
from layer3.kinematics import kinematics   # your new solver


# -------------------------------------------------
# Global IK engine (stateless geometry)
# -------------------------------------------------

_IK = kinematics()


# -------------------------------------------------
# Leg ID mapping (consistent across stack)
# -------------------------------------------------

LEG_ID = {
    'FL': 0,
    'RL': 1,
    'FR': 2,
    'RR': 3,
}


# -------------------------------------------------
# Stand pose reference (computed ONCE)
# -------------------------------------------------

_STAND_Z = -0.18
_STAND_Y = 0.07
_STAND_X = 0.0

_STAND_REF = {}

def _compute_stand_reference():
    for leg, leg_id in LEG_ID.items():
        y = _STAND_Y if leg in ('FL', 'RL') else -_STAND_Y
        t1, t2, t3, *_ = _IK.leg_IK(
            xyz=[_STAND_X, y, _STAND_Z],
            legID=leg_id,
            is_radians=True
        )

        _STAND_REF[leg] = (t1, t2, t3)




_compute_stand_reference()


# -------------------------------------------------
# Core solver
# -------------------------------------------------

def solve_leg_ik(x, y, z, leg):
    leg_id = LEG_ID[leg]

    t1, t2, t3, *_ = _IK.leg_IK(
        xyz=[x, y, z],
        legID=leg_id,
        is_radians=True
    )

    # Convert to degrees
    deg = [
        math.degrees(t1),
        math.degrees(t2),
        math.degrees(t3),
    ]

    # Delta from stand
    ref = _STAND_REF[leg]
    delta = (
        deg[0] - math.degrees(ref[0]),
        deg[1] - math.degrees(ref[1]),
        deg[2] - math.degrees(ref[2]),
    )

    return delta


# -------------------------------------------------
# Solve all legs
# -------------------------------------------------

def solve_all_legs(foot_targets):
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
        c, t, w = solve_leg_ik(x, y, z, leg)
        out[f"{leg}_COXA"]  = c
        out[f"{leg}_THIGH"] = t
        out[f"{leg}_WRIST"] = w

    return out
