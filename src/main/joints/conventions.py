#joints/conventions.py

"""
Layer 2.5 — JOINT CONVENTION MAPPING
===================================

Adapter between IK math space and real servo space.
This is the ONLY place where sign corrections are applied.

Sign table encodes the net correction needed so that a positive IK delta
always produces geometrically consistent motion across all legs.

The signs account for TWO physical realities:
  1. kinematics.py angle_corrector already mirrors left/right for thigh
     (left theta_2 = -val, right theta_2 = +val for same foot target).
  2. Servo mount direction encoded in MECH min/max order in absolute_truths.py.

Together these two factors make the end-to-end pipeline geometrically symmetric.
Do not change any sign without re-verifying both factors.
"""

# =================================================
# JOINT SIGN CONVENTIONS
# =================================================
# +1 → pass IK delta through unchanged
# -1 → invert IK delta
#
# COXA: left=-1, right=+1
#   IK outputs same sign for both sides.
#   Left servo mount is normal (min<max), right is inverted (min>max).
#   Sign flip on left compensates so both sides toe the same geometric direction.
#
# THIGH: all +1
#   angle_corrector already outputs OPPOSITE signs for left vs right thighs
#   for the same foot target. The +1 here preserves that. The servo mount
#   inversion (min>max on right) then makes the geometric result symmetric.
#   DO NOT change right thighs to -1 — that would double-invert and reverse them.
#
# WRIST: left=+1, right=-1
#   angle_corrector outputs SAME sign for both sides.
#   Right servo mount is inverted (min>max), so we flip the sign here
#   to compensate and keep geometric motion symmetric.

JOINT_SIGN = {
    "FL_COXA": -1,
    "FR_COXA": +1,
    "RL_COXA": -1,
    "RR_COXA": +1,

    "FL_THIGH": +1,
    "FR_THIGH": +1,
    "RL_THIGH": +1,
    "RR_THIGH": +1,

    "FL_WRIST": +1,
    "RL_WRIST": +1,
    "FR_WRIST": -1,
    "RR_WRIST": -1,
}


# =================================================
# JOINT ZERO OFFSETS (degrees)
# =================================================
# Applied AFTER sign flip, in signed-IK-delta space — NOT in physical servo degrees.
# Use these only to correct a static lean at stand that cannot be fixed by
# recalibrating COXA_STAND / THIGH_STAND / WRIST_STAND in absolute_truths.py.
# Positive offset moves in the same geometric direction as a positive IK delta
# for that joint, after sign correction above has been applied.

JOINT_OFFSET = {
    "FL_COXA":  0.0,
    "FR_COXA":  0.0,
    "RL_COXA":  0.0,
    "RR_COXA":  0.0,

    "FL_THIGH": 0.0,
    "FR_THIGH": 0.0,
    "RL_THIGH": 0.0,
    "RR_THIGH": 0.0,

    "FL_WRIST": 0.0,
    "FR_WRIST": 0.0,
    "RL_WRIST": 0.0,
    "RR_WRIST": 0.0,
}


# =================================================
# APPLY CONVENTIONS
# =================================================

def apply_joint_conventions(deltas: dict) -> dict:
    """
    Apply sign + offset corrections to IK deltas.
    """
    corrected = {}

    for joint, delta in deltas.items():
        if joint not in JOINT_SIGN:
            raise KeyError(f"Unknown joint '{joint}'")

        corrected[joint] = (
            JOINT_SIGN[joint] * delta
            + JOINT_OFFSET[joint]
        )

    return corrected
