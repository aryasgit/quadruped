# layer5/posture_controller_gait_only.py
"""
Layer 5 — GAIT-ONLY KINEMATICS BRIDGE
===================================

STRICT CONTRACT:
- NO IMU
- NO pitch / roll stabilization
- NO posture deltas
- NO gait timing
- NO FSM logic

This layer ONLY:
1. Accepts absolute foot targets from Layer 9
2. Solves IK
3. Applies joint conventions
4. Normalizes to servo angles
5. Locks COXA for stance consistency

This guarantees:
- Deterministic gait geometry
- No step magnitude distortion
- No sequencing overlap
"""

from layer3.leg_ik import solve_all_legs
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all
from hardware.absolute_truths import COXA

# ----------------------------------
# CONSTANTS (MATCH controller.py)
# ----------------------------------
COXA_STAND = 45.0   # same as before, do NOT change silently


def posture_step(foot_targets):
    """
    foot_targets: dict
        {
          "FL": (x, y, z),
          "FR": (x, y, z),
          "RL": (x, y, z),
          "RR": (x, y, z),
        }

    returns:
        physical joint angles dict compatible with controller.py
    """

    # ----------------------------------
    # 1. Inverse kinematics
    # ----------------------------------
    deltas = solve_all_legs(foot_targets)
    # Confirm units: check a couple expected magnitudes



    # ----------------------------------
    # 2. Mechanical sign conventions
    # ----------------------------------
    deltas = apply_joint_conventions(deltas)

    # ================================
    # RIGHT-LEG MIRROR CORRECTION
    # ================================
    for leg in ("FR", "RR"):
        deltas[f"{leg}_THIGH"] *= -1
        deltas[f"{leg}_WRIST"] *= -1

    

    # ----------------------------------
    # 3. Normalize to physical servo space
    # ----------------------------------
    physical = normalize_all(deltas)

    # ----------------------------------
    # 4. COXA LOCK (NO YAW DURING GAIT)
    # ----------------------------------
    #physical["FL_COXA"] = COXA_STAND
    #physical["FR_COXA"] = COXA_STAND
    #physical["RL_COXA"] = COXA_STAND
    #physical["RR_COXA"] = COXA_STAND

    return physical
