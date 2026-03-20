# joints/posture_controller.py
"""
Layer 5 — POSTURE CONTROLLER (Roll + Pitch Stabilisation)
==========================================================

Drop-in replacement. Works with the current pipeline:

    posture_step(nominal_feet, imu)
        → joint deltas (degrees)
        → apply_joint_conventions()
        → normalize_all()
        → servos

Key design decisions vs old versions:
  - Reference-locked at first call (no drift from boot angle)
  - Cartesian Z correction per leg (pitch + roll)
  - Coxa splay correction for roll (keeps feet flat under body)
  - Derivative damping from raw IMU rate (no numerical diff)
  - NO wrist sign patch here — conventions.py handles that already
  - Swing leg detection: skip compensation on airborne leg
  - Full pipeline NOT called here — caller does conventions+normalize

Gains are conservative. Tune PITCH_GAIN, ROLL_GAIN, COXA_ROLL_GAIN first.
"""

import math
from typing import Dict, Tuple, Optional

from hardware.imu import IMUFilter
from ik.solver import solve_all_legs


# -------------------------------------------------
# Body geometry (meters) — must match kinematics.py
# -------------------------------------------------

BODY_LENGTH = 0.20830
BODY_WIDTH  = 0.07890

LEG_X = {
    "FL": +BODY_LENGTH / 2,
    "FR": +BODY_LENGTH / 2,
    "RL": -BODY_LENGTH / 2,
    "RR": -BODY_LENGTH / 2,
}

LEG_Y = {
    "FL": +BODY_WIDTH / 2,
    "FR": -BODY_WIDTH / 2,
    "RL": +BODY_WIDTH / 2,
    "RR": -BODY_WIDTH / 2,
}

STANCE_Z = -0.18   # nominal stand height (meters)


# -------------------------------------------------
# Gains — start conservative, tune up physically
# -------------------------------------------------

# Cartesian Z correction per radian of tilt
# Larger = more leg extension to fight tilt, but risks oscillation
PITCH_GAIN     = 0.8   # (meters of Z per radian of pitch per meter of leg X offset)
ROLL_GAIN      = 1.2   # (meters of Z per radian of roll per meter of leg Y offset)

# Coxa splay correction for roll
# Larger = more toe-in/out to fight lateral tilt
COXA_ROLL_GAIN = 8.0   # (degrees of coxa delta per radian of roll)

# Derivative damping — uses raw IMU gyro rate
# Larger = more opposition to fast tilts (prevents overshoot)
PITCH_DAMP = 0.08
ROLL_DAMP  = 0.12


# -------------------------------------------------
# Limits / deadbands
# -------------------------------------------------

MAX_PITCH_DEG = 20.0
MAX_ROLL_DEG  = 20.0
DEADBAND_DEG  = 0.7    # ignore tilt below this — prevents jitter at rest


# -------------------------------------------------
# Module-level state (reference lock)
# -------------------------------------------------

_ref_roll:  Optional[float] = None
_ref_pitch: Optional[float] = None


def reset_reference():
    """Call this to re-lock reference (e.g. after robot is repositioned)."""
    global _ref_roll, _ref_pitch
    _ref_roll  = None
    _ref_pitch = None


# -------------------------------------------------
# Core posture step
# -------------------------------------------------

def posture_step(
    nominal_feet: Dict[str, Tuple[float, float, float]],
    imu: IMUFilter,
    swing_leg: Optional[str] = None,
) -> dict:
    """
    Compute joint deltas with roll+pitch compensation.

    Args:
        nominal_feet:  foot targets from gait/stand (hip-local, meters)
        imu:           live IMUFilter instance (already calibrated)
        swing_leg:     leg currently in the air — skips compensation for it

    Returns:
        joint deltas dict  →  feed directly into apply_joint_conventions()

    Call pattern in your loop:
        deltas  = posture_step(feet, imu, swing_leg)
        deltas  = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)
        send_to_servos(physical)
    """

    global _ref_roll, _ref_pitch

    # --- read IMU ---
    roll, pitch, roll_rate, pitch_rate = imu.update()

    # --- lock reference on first call ---
    if _ref_roll is None:
        _ref_roll  = roll
        _ref_pitch = pitch
        # Return uncompensated deltas on first frame
        return solve_all_legs(nominal_feet)

    # --- relative angles ---
    roll  = roll  - _ref_roll
    pitch = pitch - _ref_pitch

    # Sign convention: front-down = positive pitch correction needed
    # Flip pitch so nose-down gives positive error (pushes front feet down)
    pitch      = -pitch
    pitch_rate = -pitch_rate

    # --- deadband ---
    if abs(roll)  < DEADBAND_DEG: roll  = 0.0
    if abs(pitch) < DEADBAND_DEG: pitch = 0.0

    # --- clamp ---
    roll  = max(min(roll,  MAX_ROLL_DEG),  -MAX_ROLL_DEG)
    pitch = max(min(pitch, MAX_PITCH_DEG), -MAX_PITCH_DEG)

    # --- derivative damping (uses raw gyro, no numerical diff needed) ---
    roll_damped  = roll  - ROLL_DAMP  * roll_rate
    pitch_damped = pitch - PITCH_DAMP * pitch_rate

    roll_r  = math.radians(roll_damped)
    pitch_r = math.radians(pitch_damped)

    # --- Cartesian Z compensation ---
    compensated_feet = {}
    for leg, (x, y, z) in nominal_feet.items():
        if leg == swing_leg:
            compensated_feet[leg] = (x, y, z)   # don't move the airborne foot
            continue

        dz_pitch = -PITCH_GAIN * LEG_X[leg] * math.sin(pitch_r)
        dz_roll  = -ROLL_GAIN  * LEG_Y[leg] * math.sin(roll_r)
        compensated_feet[leg] = (x, y, z + dz_pitch + dz_roll)

    # --- solve IK ---
    deltas = solve_all_legs(compensated_feet)

    # --- coxa splay for roll ---
    # Left legs splay out when rolling right (pushes body back up)
    # Right legs splay in — opposite direction
    for leg in ("FL", "RL"):
        deltas[f"{leg}_COXA"] += COXA_ROLL_GAIN * roll_r
    for leg in ("FR", "RR"):
        deltas[f"{leg}_COXA"] -= COXA_ROLL_GAIN * roll_r

    return deltas