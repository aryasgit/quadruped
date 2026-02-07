# layer6/gait_generator.py
import math
from typing import Dict, Tuple

# -------------------------------------------------
# Nominal stance (meters)
# -------------------------------------------------
STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

# -------------------------------------------------
# Gait parameters
# -------------------------------------------------
STEP_LENGTH = 0.10
STEP_HEIGHT = 0.045
DUTY_FACTOR = 0.60

# -------------------------------------------------
# Phase offsets (demo: all legs same arc)
# -------------------------------------------------
PHASE_OFFSET = {
    "FL": 0.0,
    "FR": 0.0,
    "RL": 0.0,
    "RR": 0.0,
}

# -------------------------------------------------
def _wrap_phase(p: float) -> float:
    return p % 1.0

# -------------------------------------------------
def _leg_trajectory(
    phase: float,
    step_length: float,
    step_height: float,
    duty: float,
) -> Tuple[float, float]:
    """
    Returns dx, dz relative to stance
    """

    # ---- STANCE ----
    if phase < duty:
        s = phase / duty
        dx = +step_length / 2 - s * step_length
        dz = 0.0
        return dx, dz

    # ---- SWING ----
    s = (phase - duty) / (1.0 - duty)
    dx = -step_length / 2 + s * step_length
    dz = step_height * math.sin(math.pi * s)

    return dx, dz

# -------------------------------------------------
def generate_foot_targets(
    t: float,
    frequency: float,
    step_length: float = STEP_LENGTH,
    step_height: float = STEP_HEIGHT,
    duty: float = DUTY_FACTOR,
) -> Dict[str, Tuple[float, float, float]]:

    if frequency <= 0.0:
        return {
            "FL": (STANCE_X,  STANCE_Y, STANCE_Z),
            "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
            "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
            "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
        }

    foot_targets = {}
    base_phase = t * frequency

    for leg, offset in PHASE_OFFSET.items():
        phase = _wrap_phase(base_phase + offset)
        dx, dz = _leg_trajectory(phase, step_length, step_height, duty)
        y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y

        foot_targets[leg] = (
            STANCE_X + dx,
            y,
            STANCE_Z + dz,
        )

    return foot_targets
