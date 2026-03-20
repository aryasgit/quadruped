# layer5/posture_controller.py
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

PITCH_GAIN = 0.8      # Pitch: less aggressive
ROLL_GAIN  = 1.2      # Roll: less aggressive
COXA_ROLL_GAIN = 8.0  # Coxa: less aggressive

# Derivative damping (opposes rapid changes)
PITCH_DAMP = 0.080    # Pitch: much more damping
ROLL_DAMP  = 0.120    # Roll: much more damping


# -------------------------------------------------
# Limits / deadbands
# -------------------------------------------------

MAX_PITCH_DEG = 20.0
MAX_ROLL_DEG  = 20.0

DEADBAND_DEG = 0.7    # Higher deadband for less jitter


# -------------------------------------------------
# Internal state (REFERENCE LOCK)
# -------------------------------------------------

_ref_roll  = None
_ref_pitch = None
_last_roll = 0.0
_last_pitch = 0.0


# -------------------------------------------------
# Get current IMU state (for external use)
# -------------------------------------------------

def get_current_imu_state(imu: "IMUFilter") -> tuple:
    """
    Returns current (roll_deg, pitch_deg) relative to reference.
    Call this to capture IMU state before starting walk.
    """
    global _ref_roll, _ref_pitch, _last_roll, _last_pitch
    
    roll, pitch, _, _ = imu.update()
    
    if _ref_roll is None:
        return 0.0, 0.0
    
    # Relative angles
    rel_roll = roll - _ref_roll
    rel_pitch = -(pitch - _ref_pitch)  # Sign convention
    
    # Deadband
    if abs(rel_roll) < DEADBAND_DEG:
        rel_roll = 0.0
    if abs(rel_pitch) < DEADBAND_DEG:
        rel_pitch = 0.0
    
    # Clamp
    rel_roll = max(min(rel_roll, MAX_ROLL_DEG), -MAX_ROLL_DEG)
    rel_pitch = max(min(rel_pitch, MAX_PITCH_DEG), -MAX_PITCH_DEG)
    
    return rel_roll, rel_pitch


def compute_z_compensation(roll_deg: float, pitch_deg: float) -> dict:
    """
    Compute per-leg Z compensation for given roll/pitch.
    Returns dict: {"FL": dz, "FR": dz, "RL": dz, "RR": dz}
    """
    roll_r = math.radians(roll_deg)
    pitch_r = math.radians(pitch_deg)
    
    compensation = {}
    for leg in ("FL", "FR", "RL", "RR"):
        lx = LEG_X[leg]
        ly = LEG_Y[leg]
        dz_pitch = -PITCH_GAIN * lx * math.sin(pitch_r)
        dz_roll = -ROLL_GAIN * ly * math.sin(roll_r)
        compensation[leg] = dz_pitch + dz_roll
    
    return compensation


def compute_coxa_compensation(roll_deg: float) -> dict:
    """
    Compute coxa compensation for given roll.
    Returns dict: {"FL": delta, "FR": delta, ...}
    """
    roll_r = math.radians(roll_deg)
    return {
        "FL": +COXA_ROLL_GAIN * roll_r,
        "FR": -COXA_ROLL_GAIN * roll_r,
        "RL": +COXA_ROLL_GAIN * roll_r,
        "RR": -COXA_ROLL_GAIN * roll_r,
    }


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
    roll, pitch, roll_rate, pitch_rate = imu.update()

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
    pitch_rate = -pitch_rate

    # Deadband (kills drift)
    if abs(roll)  < DEADBAND_DEG: roll  = 0.0
    if abs(pitch) < DEADBAND_DEG: pitch = 0.0

    # Clamp
    roll  = max(min(roll,  MAX_ROLL_DEG),  -MAX_ROLL_DEG)
    pitch = max(min(pitch, MAX_PITCH_DEG), -MAX_PITCH_DEG)

    # Apply derivative damping (opposes rapid changes)
    # This reduces overshoot and oscillation
    roll_damped  = roll  - ROLL_DAMP  * roll_rate
    pitch_damped = pitch - PITCH_DAMP * pitch_rate

    roll_r  = math.radians(roll_damped)
    pitch_r = math.radians(pitch_damped)

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

    # Note: No wrist mirroring needed for balance - both sides move same direction

    return deltas

