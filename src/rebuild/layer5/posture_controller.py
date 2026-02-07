"""
Layer 5 — Roll + Pitch Posture Controller (REFERENCE-STABLE)
============================================================

Key properties:
- Uses IMU RELATIVE angles (boot = zero)
- NO slow drift / nose dive
- Cartesian Z-only leg motion
- Roll adds coxa stabilization
- Right wrist mirror FIXED locally
- Outputs JOINT DELTAS (degrees)
"""

import math
from typing import Dict, Tuple

from hardware.imu import IMUFilter
from layer3.leg_ik import solve_all_legs


# -------------------------------------------------
# Geometry (meters)
# -------------------------------------------------

BODY_LENGTH = 0.208
BODY_WIDTH  = 0.078

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


# -------------------------------------------------
# Gains (tuned for REAL robot)
# -------------------------------------------------

PITCH_GAIN = 2.0     # stronger than before
ROLL_GAIN  = 2.5
COXA_ROLL_GAIN = 18.0  # degrees per rad


# -------------------------------------------------
# Limits / deadbands
# -------------------------------------------------

MAX_PITCH_DEG = 20.0
MAX_ROLL_DEG  = 20.0

DEADBAND_DEG = 0.4    # kills slow drift


# -------------------------------------------------
# Internal state (REFERENCE LOCK)
# -------------------------------------------------

_ref_roll  = None
_ref_pitch = None


# -------------------------------------------------
# Internal: right wrist mirror fix
# -------------------------------------------------

RIGHT_LEGS = ("FR", "RR")

def _fix_right_wrist_mirroring(deltas: dict) -> dict:
    out = deltas.copy()
    for leg in RIGHT_LEGS:
        k = f"{leg}_WRIST"
        if k in out:
            out[k] = -out[k]
    return out


# -------------------------------------------------
# Core posture controller
# -------------------------------------------------

def posture_step(
    nominal_foot_targets: Dict[str, Tuple[float, float, float]],
    imu: IMUFilter,
) -> dict:
    """
    Returns:
        joint deltas (degrees)
    """

    global _ref_roll, _ref_pitch

    # ---------------- IMU ----------------
    roll, pitch, _, _ = imu.update()

    # Lock reference on first call
    if _ref_roll is None:
        _ref_roll  = roll
        _ref_pitch = pitch
        return solve_all_legs(nominal_foot_targets)

    # Relative angles
    roll  = roll  - _ref_roll
    pitch = pitch - _ref_pitch

    # IMU sign conventions (your robot)
    # left roll positive, front pitch negative
    pitch = -pitch

    # Deadband (kills drift)
    if abs(roll)  < DEADBAND_DEG: roll  = 0.0
    if abs(pitch) < DEADBAND_DEG: pitch = 0.0

    # Clamp
    roll  = max(min(roll,  MAX_ROLL_DEG),  -MAX_ROLL_DEG)
    pitch = max(min(pitch, MAX_PITCH_DEG), -MAX_PITCH_DEG)

    roll_r  = math.radians(roll)
    pitch_r = math.radians(pitch)

    # ---------------- Cartesian compensation ----------------
    foot_targets = {}

    for leg, (x, y, z) in nominal_foot_targets.items():
        lx = LEG_X[leg]
        ly = LEG_Y[leg]

        dz_pitch = -PITCH_GAIN * lx * math.sin(pitch_r)
        dz_roll  = -ROLL_GAIN  * ly * math.sin(roll_r)

        foot_targets[leg] = (x, y, z + dz_pitch + dz_roll)

    # ---------------- IK ----------------
    deltas = solve_all_legs(foot_targets)

    # ---------------- Coxa roll stabilization ----------------
    for leg in ("FL", "RL"):
        deltas[f"{leg}_COXA"] +=  COXA_ROLL_GAIN * roll_r
    for leg in ("FR", "RR"):
        deltas[f"{leg}_COXA"] -=  COXA_ROLL_GAIN * roll_r

    # ---------------- Wrist mirror fix ----------------
    deltas = _fix_right_wrist_mirroring(deltas)

    return deltas
