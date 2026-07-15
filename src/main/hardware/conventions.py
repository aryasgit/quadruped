"""
L2 — JOINT CONVENTIONS + NORMALIZATION (pure)
=============================================

The single bridge between math space (IK deltas) and physical servo degrees.

    IK Δ  --sign/offset-->  corrected Δ  --stand + Δ, clamp-->  servo degrees

Sign table and stand/mech tables come from config.robot_spec. This is the ONLY
place sign corrections and mechanical clamping happen. No I/O.
"""

from config.robot_spec import (
    JOINT_ORDER, JOINT_SIGN, JOINT_OFFSET, STAND_ANGLE, MECH_LIMITS,
)


def apply_conventions(deltas):
    """Sign-correct and offset raw IK deltas (still in delta space)."""
    out = {}
    for joint, delta in deltas.items():
        sign = JOINT_SIGN.get(joint)
        if sign is None:
            raise KeyError(f"Unknown joint '{joint}'")
        out[joint] = sign * delta + JOINT_OFFSET[joint]
    return out


def _clamp_to_mech(joint, physical):
    mech = MECH_LIMITS[joint]
    lo = min(mech["min"], mech["max"])
    hi = max(mech["min"], mech["max"])
    return lo if physical < lo else hi if physical > hi else physical


def to_servo_degrees(corrected_deltas):
    """
    corrected_deltas: {joint: Δdeg (post-conventions)}
    Returns {joint: physical servo degrees}, clamped to mechanical limits.
    Missing joints default to their stand angle (Δ=0).
    """
    out = {}
    for joint in JOINT_ORDER:
        delta = corrected_deltas.get(joint, 0.0)
        out[joint] = _clamp_to_mech(joint, STAND_ANGLE[joint] + delta)
    return out


def joints_to_servo(ik_deltas):
    """Full adapter: raw IK deltas -> clamped physical servo degrees."""
    return to_servo_degrees(apply_conventions(ik_deltas))
