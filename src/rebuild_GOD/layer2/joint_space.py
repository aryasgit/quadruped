"""
Layer 2 — JOINT SPACE NORMALIZATION
=================================

This layer is the ONLY bridge between:
- mathematical joint angles (IK / posture / gait)
- physical servo angles (Layer 0 truths)

Responsibilities:
- Define canonical joint ordering
- Apply direction flips
- Apply mechanical zero offsets
- Clamp to mechanical limits

NON-RESPONSIBILITIES:
- No IK math
- No balance
- No gait timing
- No hardware I/O

Everything above this layer is dimensionless math.
Everything below this layer is physical metal.
"""

from hardware.absolute_truths import (
    WRISTS,
    THIGHS,
    COXA,
    WRIST_MECH,
    THIGH_MECH,
    COXA_MECH,
    WRIST_STAND,
    THIGH_STAND,
    COXA_STAND,
)

# -------------------------------------------------
# Canonical joint order (MATCHES SPOTMICRO REPO)
# -------------------------------------------------
# Order: LF, RF, RR, LR
# Each leg: [coxa, thigh, wrist]

JOINT_ORDER = [
    "FL_COXA", "FL_THIGH", "FL_WRIST",
    "FR_COXA", "FR_THIGH", "FR_WRIST",
    "RR_COXA", "RR_THIGH", "RR_WRIST",
    "RL_COXA", "RL_THIGH", "RL_WRIST",
]

# -------------------------------------------------
# Mechanical truth tables (flattened)
# -------------------------------------------------

MECH_LIMITS = {
    "FL_COXA":  COXA_MECH["FL"],
    "FR_COXA":  COXA_MECH["FR"],
    "RR_COXA":  COXA_MECH["RR"],
    "RL_COXA":  COXA_MECH["RL"],

    "FL_THIGH": THIGH_MECH["TFL"],
    "FR_THIGH": THIGH_MECH["TFR"],
    "RR_THIGH": THIGH_MECH["TRR"],
    "RL_THIGH": THIGH_MECH["TRL"],

    "FL_WRIST": WRIST_MECH["WFL"],
    "FR_WRIST": WRIST_MECH["WFR"],
    "RR_WRIST": WRIST_MECH["WRR"],
    "RL_WRIST": WRIST_MECH["WRL"],
}

STAND_ANGLES = {
    "FL_COXA":  COXA_STAND["FL"],
    "FR_COXA":  COXA_STAND["FR"],
    "RR_COXA":  COXA_STAND["RR"],
    "RL_COXA":  COXA_STAND["RL"],

    "FL_THIGH": THIGH_STAND["TFL"],
    "FR_THIGH": THIGH_STAND["TFR"],
    "RR_THIGH": THIGH_STAND["TRR"],
    "RL_THIGH": THIGH_STAND["TRL"],

    "FL_WRIST": WRIST_STAND["WFL"],
    "FR_WRIST": WRIST_STAND["WFR"],
    "RR_WRIST": WRIST_STAND["WRR"],
    "RL_WRIST": WRIST_STAND["WRL"],
}

# -------------------------------------------------
# Normalization core
# -------------------------------------------------

def normalize_joint(joint_name: str, delta_angle: float) -> float:
    """
    Convert a math-space delta angle into a physical servo angle.

    delta_angle:
        +ve means forward in math space
        0 means stand pose

    Returns:
        physical servo angle (degrees), clamped
    """
    stand = STAND_ANGLES[joint_name]
    mech = MECH_LIMITS[joint_name]

    physical = stand + delta_angle

    lo = min(mech["min"], mech["max"])
    hi = max(mech["min"], mech["max"])

    if physical < lo:
        physical = lo
    elif physical > hi:
        physical = hi

    return physical


def normalize_all(deltas: dict) -> dict:
    """
    Normalize all joints.

    deltas:
        dict mapping joint_name -> delta_angle

    Returns:
        dict mapping joint_name -> physical_angle
    """
    out = {}
    for j in JOINT_ORDER:
        out[j] = normalize_joint(j, deltas.get(j, 0.0))
    return out


# -------------------------------------------------
# Stand sanity check
# -------------------------------------------------

if __name__ == "__main__":
    print("[L2] Stand pose sanity check")
    zeros = {j: 0.0 for j in JOINT_ORDER}
    phys = normalize_all(zeros)
    for j in JOINT_ORDER:
        print(f"{j:10s} -> {phys[j]:6.1f}°")
