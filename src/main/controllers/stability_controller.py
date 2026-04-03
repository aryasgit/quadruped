# joints/stability_controller.py
"""
Layer 5 — STABILITY CONTROLLER (Incremental / Rate-Limited)
=============================================================

WHY NOT PID:
  Servos already contain an internal position PID running at ~1-5 kHz.
  An outer PID at 50 Hz moves the target faster than the servo can settle,
  causing the two loops to fight each other endlessly — exactly the
  oscillation / jitter observed with the PID approach.

THIS APPROACH — Incremental Z nudging:
  Instead of computing an absolute foot target from a PID output, we
  maintain a small per-leg Z offset (dz) and nudge it by a tiny amount
  each tick based only on the SIGN (and magnitude) of the tilt error.

  The servo always receives a small, reachable position change.
  It settles before the next nudge arrives.  No fighting.

  Per tick (50 Hz, dt = 0.02 s):
    if pitch > DEADBAND  → nudge dz_front up,   dz_rear  down
    if pitch < -DEADBAND → nudge dz_front down,  dz_rear  up
    if roll  > DEADBAND  → nudge dz_left  down,  dz_right up
    if roll  < -DEADBAND → nudge dz_left  up,    dz_right down
    clamp each dz to ±MAX_DZ
    apply dz to foot targets → IK → conventions → servos

TUNING (only 3 parameters):
  PITCH_RATE   how fast pitch is corrected (m/s of Z at max error)
               too low  → slow to react
               too high → overshoots (but much more forgiving than PID KP)

  ROLL_RATE    same but for roll axis

  DEADBAND_DEG stop nudging when tilt is within this band (prevents jitter at rest)
               typical: 0.8 – 1.5°

  MAX_DZ       maximum Z offset per leg (metres)
               limits how far legs can extend/retract for balance
               typical: 0.04 m (robot still stable in workspace)

  Scale factor (optional):
    SCALE_BY_ERROR = True  → step size ∝ error magnitude (faster at large tilts)
    SCALE_BY_ERROR = False → fixed step size regardless of error size
    Start with True — it gives faster gross correction and finer near-zero.

Pipeline contract:
    physical = posture_step(foot_targets, imu)
    → returns fully-processed servo angles (0-270°), write directly.
    Do NOT pass through apply_joint_conventions / normalize_all again.

Sign convention (from imu.py):
    pitch > 0  = nose UP    (atan2(-ax, sqrt(ay²+az²)))
    roll  > 0  = tilt LEFT  (atan2(ay, az))
"""

import math
from typing import Dict, Tuple, Optional

from hardware.imu import IMUFilter
from hardware.absolute_truths import COXA_STAND as COXA_STAND_MAP

from ik.solver          import solve_all_legs
from joints.conventions import apply_joint_conventions
from joints.space       import normalize_all


# =====================================================================
# BODY GEOMETRY  (must match kinematics.py)
# =====================================================================

BODY_LENGTH = 0.20830
BODY_WIDTH  = 0.07890

LEG_X = {
    "FL": +BODY_LENGTH / 2,    # front (+)
    "FR": +BODY_LENGTH / 2,
    "RL": -BODY_LENGTH / 2,    # rear  (-)
    "RR": -BODY_LENGTH / 2,
}

LEG_Y = {
    "FL": +BODY_WIDTH / 2,     # left  (+)
    "RL": +BODY_WIDTH / 2,
    "FR": -BODY_WIDTH / 2,     # right (-)
    "RR": -BODY_WIDTH / 2,
}

BASE_Z = -0.18   # nominal stand height (metres)


# =====================================================================
# TUNING PARAMETERS
# =====================================================================

# Correction speed (metres of Z per second at maximum error)
# At 50 Hz each tick moves dz by at most: RATE / 50 = RATE * dt
PITCH_RATE = 0.08    # start here, raise if too slow to react
ROLL_RATE  = 0.10    # roll usually needs slightly faster correction

# Deadband — don't nudge if tilt is within this many degrees
# Prevents constant micro-jitter at rest
DEADBAND_DEG = 1.0

# Maximum Z correction per leg (metres)
# Keeps feet within safe IK workspace
MAX_DZ = 0.04

# When True: step size scales linearly with error magnitude
# When False: fixed step size (RATE * dt) regardless of error size
# True gives faster gross correction and finer near-zero — recommended
SCALE_BY_ERROR = True

# Maximum error angle used for scaling (errors above this saturate to full rate)
# Equivalent to the old PID CMD_LIMIT
MAX_ERROR_DEG = 15.0

# =====================================================================
# CONFIG
# =====================================================================
POSTURE_ENABLED = True


# =====================================================================
# STATE — per-leg Z offsets, persists across ticks
# =====================================================================
_dz = {
    "FL": 0.0,
    "FR": 0.0,
    "RL": 0.0,
    "RR": 0.0,
}


# =====================================================================
# RESET
# =====================================================================

def reset_pid():
    """Zero all compensation state. Call before a new run."""
    global _dz
    _dz = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}


def reset_reference():
    """Alias for test.py compatibility."""
    reset_pid()


# =====================================================================
# CORE POSTURE STEP
# =====================================================================

def posture_step(
    foot_targets: Dict[str, Tuple[float, float, float]],
    imu: IMUFilter,
    swing_leg: Optional[str] = None,
) -> dict:
    """
    Compute and return physical servo angles with roll+pitch compensation.

    Args:
        foot_targets : hip-local foot positions {FL/FR/RL/RR: (x, y, z)}
        imu          : calibrated IMUFilter instance
        swing_leg    : leg in the air — freeze its dz (None = standing)

    Returns:
        dict  joint_name → physical servo angle (degrees, 0-270)
        Write directly to set_servo_angle(). DO NOT re-apply conventions.
    """
    global _dz

    # ------------------------------------------------------------------
    # Passthrough when disabled
    # ------------------------------------------------------------------
    if not POSTURE_ENABLED:
        deltas = solve_all_legs(foot_targets)
        conv   = apply_joint_conventions(deltas)
        return normalize_all(conv)

    # ------------------------------------------------------------------
    # Read IMU
    # ------------------------------------------------------------------
    roll, pitch, _, _ = imu.update()
    dt = 0.02

    # ------------------------------------------------------------------
    # Compute per-axis step size this tick
    #
    # If SCALE_BY_ERROR:
    #   step = RATE * dt * clamp(|error| / MAX_ERROR_DEG, 0, 1)
    #   → tiny step near zero, full rate at large errors
    # Else:
    #   step = RATE * dt  (fixed)
    # ------------------------------------------------------------------
    if SCALE_BY_ERROR:
        pitch_scale = min(abs(pitch) / MAX_ERROR_DEG, 1.0)
        roll_scale  = min(abs(roll)  / MAX_ERROR_DEG, 1.0)
    else:
        pitch_scale = 1.0
        roll_scale  = 1.0

    pitch_step = PITCH_RATE * dt * pitch_scale
    roll_step  = ROLL_RATE  * dt * roll_scale

    # ------------------------------------------------------------------
    # Nudge dz per leg based on error sign
    #
    # PITCH sign trace:
    #   pitch > 0 = nose UP → front too high → retract front, extend rear
    #   retract front: dz_front increases (foot moves up)
    #   extend rear:   dz_rear  decreases (foot moves down)
    #   → dz[FL/FR] += step,  dz[RL/RR] -= step   when pitch > 0
    #
    # ROLL sign trace:
    #   roll > 0 = tilt LEFT → left side low → extend left, retract right
    #   extend left:   dz_left  decreases (foot moves down)
    #   retract right: dz_right increases (foot moves up)
    #   → dz[FL/RL] -= step,  dz[FR/RR] += step   when roll > 0
    # ------------------------------------------------------------------
    for leg in ("FL", "FR", "RL", "RR"):
        if leg == swing_leg:
            continue  # don't move airborne leg

        # ── pitch contribution ──
        if pitch > DEADBAND_DEG:
            # nose up: retract front (+dz), extend rear (-dz)
            _dz[leg] += pitch_step * (1.0 if leg in ("FL", "FR") else -1.0)
        elif pitch < -DEADBAND_DEG:
            # nose down: extend front (-dz), retract rear (+dz)
            _dz[leg] += pitch_step * (-1.0 if leg in ("FL", "FR") else 1.0)

        # ── roll contribution ──
        if roll > DEADBAND_DEG:
            # tilt left: extend left (-dz), retract right (+dz)
            _dz[leg] += roll_step * (-1.0 if leg in ("FL", "RL") else 1.0)
        elif roll < -DEADBAND_DEG:
            # tilt right: retract left (+dz), extend right (-dz)
            _dz[leg] += roll_step * (1.0 if leg in ("FL", "RL") else -1.0)

        # ── clamp to workspace ──
        _dz[leg] = max(min(_dz[leg], MAX_DZ), -MAX_DZ)

    # ------------------------------------------------------------------
    # Apply current dz offsets to foot targets
    # ------------------------------------------------------------------
    compensated_feet = {}
    for leg, (x, y, z) in foot_targets.items():
        compensated_feet[leg] = (x, y, z + _dz[leg])

    # ------------------------------------------------------------------
    # IK → conventions → physical angles
    # ------------------------------------------------------------------
    deltas   = solve_all_legs(compensated_feet)
    conv     = apply_joint_conventions(deltas)
    physical = normalize_all(conv)

    # ------------------------------------------------------------------
    # Lock coxas to calibrated stand angles
    # ------------------------------------------------------------------
    for leg_key, stand_angle in COXA_STAND_MAP.items():
        joint_key = f"{leg_key}_COXA"
        if joint_key in physical:
            physical[joint_key] = stand_angle

    return physical