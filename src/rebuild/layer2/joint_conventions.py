"""
Layer 2.5 — JOINT CONVENTION MAPPING
===================================

Adapter between IK math space and real servo space.
This layer is the ONLY place where mirroring is handled.
"""

# =================================================
# JOINT SIGN CONVENTIONS
# =================================================
# +1 → use IK delta as-is
# -1 → invert IK delta

JOINT_SIGN = {
    # Coxa (yaw) — mirrored left/right
    "FL_COXA": -1,
    "FR_COXA": +1,
    "RL_COXA": -1,
    "RR_COXA": +1,

    # Thigh (pitch) — SAME for all legs
    "FL_THIGH": +1,
    "FR_THIGH": +1,
    "RL_THIGH": +1,
    "RR_THIGH": +1,

    # Wrist (knee) — RIGHT SIDE IS MIRRORED ❗
    "FL_WRIST": +1,
    "RL_WRIST": +1,
    "FR_WRIST": -1,   # ← FIX
    "RR_WRIST": -1,   # ← FIX
}


# =================================================
# JOINT ZERO OFFSETS (degrees)
# =================================================
# Leave at zero unless you see static bias at stand

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
