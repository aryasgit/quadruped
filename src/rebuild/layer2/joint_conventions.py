"""
Layer 2.5 — JOINT CONVENTION MAPPING
===================================

PURPOSE
-------
This layer is the *adapter* between:

- Mathematical joint angles produced by IK (repo coordinate system)
- Physical joint angles expected by your real servos

Why this exists:
- SpotMicro IK assumes ideal, symmetric joints
- Your real robot has mirrored servos, flipped axes, and measured stand zeros
- Sending IK output directly to servos will ALWAYS look broken

This layer fixes ONLY conventions — nothing else.

RESPONSIBILITIES
----------------
- Apply per-joint sign corrections (axis direction)
- Apply per-joint zero-offset corrections (IK-zero → stand reference)

NON-RESPONSIBILITIES
--------------------
- No hardware I/O
- No IK math
- No limit clamping
- No gait or balance logic

If something is wrong here, symptoms look like:
- one side collapsing
- rear legs swinging wildly
- left/right asymmetry
"""

# =================================================
# JOINT SIGN CONVENTIONS
# =================================================
# Meaning:
#   +1 → IK delta added as-is
#   -1 → IK delta inverted
#
# These are PURE DIRECTION FIXES.
# They do NOT encode offsets or limits.

JOINT_SIGN = {
    # Coxa (yaw) — physical mirroring ONLY
    "FL_COXA": -1,
    "FR_COXA": +1,
    "RL_COXA": -1,
    "RR_COXA": +1,

    # Thigh (pitch) — SAME SIGN FOR ALL
    "FL_THIGH": +1,
    "FR_THIGH": +1,
    "RL_THIGH": +1,
    "RR_THIGH": +1,

    # Wrist (knee) — SAME SIGN FOR ALL
    "FL_WRIST": +1,
    "FR_WRIST": +1,
    "RL_WRIST": +1,
    "RR_WRIST": +1,
}


# =================================================
# JOINT ZERO-OFFSET CORRECTIONS (degrees)
# =================================================
# Meaning:
#   corrected_delta = sign * ik_delta + offset
#
# These offsets reconcile:
#   IK zero configuration
#   vs
#   your MEASURED stand pose (absolute_truths)
#
# Start with ZERO. Tune ONLY if a joint is
# consistently biased after signs are correct.

JOINT_OFFSET = {
    "FL_COXA":  0.0,
    "FR_COXA":  0.0,

    "RL_COXA": 0.0,
    "RR_COXA": 0.0,

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
    Apply sign and offset corrections to IK deltas.

    Parameters
    ----------
    deltas : dict
        joint_name -> IK delta angle (degrees)

    Returns
    -------
    dict
        joint_name -> corrected delta angle (degrees)
    """
    corrected = {}

    for joint, delta in deltas.items():
        if joint not in JOINT_SIGN:
            raise KeyError(f"Unknown joint '{joint}' in joint conventions")

        corrected[joint] = (
            JOINT_SIGN[joint] * delta
            + JOINT_OFFSET[joint]
        )
    return corrected
