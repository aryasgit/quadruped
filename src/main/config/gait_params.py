"""
L0 — GAIT PARAMETERS (single source of truth for motion tuning)
===============================================================

All gait / analog / height tuning in one place. The old stack had two
divergent sets (gait/generator.py vs web_control.py); this is the one set.

Values start from the verified LIVE web_control.py numbers. They are the
knobs we will tune during IMU-instrumented debugging.

Pure data only.
"""

from dataclasses import dataclass
from config.robot_spec import STANCE_Z

# =====================================================================
# Trot (forward / backward)
# =====================================================================
FREQ_HZ      = 1.3        # nominal cycle rate
STEP_LENGTH  = 0.080      # m, stride in X
STEP_HEIGHT  = 0.050      # m, swing lift (raised: feet must clear despite body rock)
DUTY         = 0.70       # stance fraction

# Backward asymmetry (reverse tends to need shorter, higher steps)
BACKWARD_LENGTH_SCALE = 0.86
BACKWARD_HEIGHT_SCALE = 1.08

# =====================================================================
# Lateral (strafe) and yaw (turn)
# =====================================================================
LATERAL_STEP_LENGTH = 0.030
LATERAL_STEP_HEIGHT = 0.018
TURN_STEP_LENGTH    = 0.070
TURN_STEP_HEIGHT    = 0.032

# =====================================================================
# Analog stick -> gait mapping
# =====================================================================
DEADZONE     = 0.15
FREQ_MIN     = 0.5
FREQ_MAX     = 2.2
STEP_SCALE_MIN = 0.20
STEP_SCALE_MAX = 0.80
RAMP_UP_RATE   = 5.0      # command envelope units/s
RAMP_DOWN_RATE = 4.0

# =====================================================================
# Stability / aesthetic overlays (CoM shift + pitch bias)
# =====================================================================
BODY_SHIFT_FWD  = 0.010
BODY_SHIFT_LAT  = 0.008
BODY_SHIFT_TURN = 0.012
MOTION_PITCH_BIAS   = 0.006
BACKWARD_PITCH_BIAS = 0.010
COXA_TOE_IN_DEG = 1.5     # static toe-in bias on all coxas

# Constant body-tilt trim (via leg heights — no coxa) to counter the rear-left
# CoM's dynamic lean during walking. +roll raises left feet (leans body right).
# Calibrated on hardware: trim 0 -> ~-11deg lean, +0.9cm -> ~level.
TRIM_ROLL  = 0.009
TRIM_PITCH = 0.008   # shift CoM forward so the front legs bear load (were floating)
STRAFE_TRIM = 0.2    # constant lateral bias to cancel crab-walk drift (+right)

# =====================================================================
# Trot phasing — diagonal pairs (this is the LIVE trot, not the dead bound)
# =====================================================================
DIAG_A = ("FL", "RR")     # phase = t
DIAG_B = ("FR", "RL")     # phase = t + 0.5

# =====================================================================
# Crawl / creep gait (statically stable: one leg swings at a time)
# =====================================================================
# Duty 0.75 with four evenly-staggered legs -> exactly one leg airborne at any
# instant, the other three form a support triangle.
CRAWL_DUTY        = 0.80    # more overlap -> longer support, shorter swing window
CRAWL_STEP_LENGTH = 0.050    # m, forward stride per leg
CRAWL_STEP_HEIGHT = 0.040    # m, higher lift than trot so feet clearly clear
# Deliberately SLOW: a static crawl must settle onto the support triangle before
# each lift. At 0.7 Hz it just rocks; ~0.15-0.30 Hz is watchable and stable.
CRAWL_FREQ_MIN    = 0.12     # Hz, full 4-leg cycle
CRAWL_FREQ_MAX    = 0.30
# Phase offsets -> time order of lifts is FR, RR, RL, FL (a smoothly
# circulating support pattern). Each leg swings for 25% of the cycle.
CRAWL_OFFSET = {"FL": 0.00, "RL": 0.25, "RR": 0.50, "FR": 0.75}
# Body CoM lean over the support triangle (opposite the swinging leg).
# Lateral lean matters most; x helps the front/rear triangles.
CRAWL_SHIFT_X   = 0.012      # m
CRAWL_SHIFT_Y   = 0.028      # m
CRAWL_SHIFT_LEAD = 0.06      # cycles: lean slightly BEFORE the leg lifts

# =====================================================================
# Height modes (name -> foot Z, meters)
# =====================================================================
HEIGHT_MODES = {
    "HIGH":   -0.19,
    "NORMAL": -0.18,
    "LOW":    -0.15,
    "CROUCH": -0.12,
}
DEFAULT_HEIGHT = "NORMAL"

# =====================================================================
# Control loop
# =====================================================================
LOOP_HZ = 60.0            # real target rate, driven by monotonic clock
LOOP_DT = 1.0 / LOOP_HZ


@dataclass(frozen=True)
class GaitParams:
    """Bundle passed to the pure gait function; defaults = the tuning above."""
    freq_hz: float = FREQ_HZ
    step_length: float = STEP_LENGTH
    step_height: float = STEP_HEIGHT
    duty: float = DUTY
    lateral_step_length: float = LATERAL_STEP_LENGTH
    lateral_step_height: float = LATERAL_STEP_HEIGHT
    turn_step_length: float = TURN_STEP_LENGTH
    turn_step_height: float = TURN_STEP_HEIGHT
    backward_length_scale: float = BACKWARD_LENGTH_SCALE
    backward_height_scale: float = BACKWARD_HEIGHT_SCALE
    body_shift_fwd: float = BODY_SHIFT_FWD
    body_shift_lat: float = BODY_SHIFT_LAT
    body_shift_turn: float = BODY_SHIFT_TURN
    motion_pitch_bias: float = MOTION_PITCH_BIAS
    backward_pitch_bias: float = BACKWARD_PITCH_BIAS
    stance_z: float = STANCE_Z


DEFAULT_GAIT = GaitParams()
