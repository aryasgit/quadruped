# com_mapper.py
#
# COM → LEG XYZ mapper
# PURE GEOMETRY
# No gains, no control, no actuation

from typing import Dict

# MUST MATCH state.py
FOOT_POSITIONS = {
    "FL": (-0.12, -0.08),
    "FR": (-0.12, +0.08),
    "RL": (+0.11, -0.08),
    "RR": (+0.11, +0.08),
}

# === VISUAL SCALE (TEMP, FOR BRING-UP) ===
Z_GAIN = 0.6      # meters per meter COM error
Z_MAX  = 0.06     # meters (6 cm hard clamp)

def com_to_leg_xyz(dx: float, dy: float) -> Dict[str, dict]:
    """
    dx, dy : desired COM correction (meters)
    returns per-leg xyz delta (meters, body frame)
    """

    cmd = {}

    for leg, (fx, fy) in FOOT_POSITIONS.items():
        # how much this leg should support COM shift
        influence = dx * fx + dy * fy

        dz = -Z_GAIN * influence
        dz = max(-Z_MAX, min(Z_MAX, dz))

        cmd[leg] = {
            "x": 0.0,   # forward handled later
            "y": 0.0,   # lateral handled later
            "z": dz,    # vertical support
        }

    return cmd
