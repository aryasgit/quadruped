"""
L4 — CRAWL / CREEP GAIT (pure, statically stable)
=================================================

One leg swings at a time; the other three form a support triangle. Before each
lift the body CoM leans over that triangle so the swing leg unloads and clears
the ground cleanly — the fix for "swing feet stay stuck, body tilts".

Reuses trot_profile for the per-leg foot path; the difference from the trot is:
  * four evenly-staggered legs (duty 0.75 -> one airborne at a time),
  * a phase-synced body lean toward the diagonal-opposite of the swinging leg.

CoM schedule (with the default CRAWL_OFFSET, lift order in time = FR,RR,RL,FL):
    body leans to the corner OPPOSITE the swinging leg, tracing a smooth loop
    com_x ~ -cos(2pi*phase),  com_y ~ +sin(2pi*phase)
The body offset applied to feet is the NEGATIVE of the CoM shift (to move the
body toward +x, all feet move -x).

Pure function of (phase, command, params, height_z). No I/O.
Currently forward/backward + yaw; strafe can be added later.
"""

import math
from config.robot_spec import STANCE_X, STANCE_Y, LEGS
from config import gait_params as GP
from gait.trajectory import trot_profile, wrap_phase

_EPS = 0.01


def foot_targets(phase, command, params=None, height_z=None):
    fwd, turn = command.fwd, command.turn
    z0 = GP.HEIGHT_MODES[GP.DEFAULT_HEIGHT] if height_z is None else height_z

    # --- body CoM lean over the support triangle (opposite the swing leg) ---
    ph = 2 * math.pi * wrap_phase(phase + GP.CRAWL_SHIFT_LEAD)
    com_x = GP.CRAWL_SHIFT_X * (-math.cos(ph))
    com_y = GP.CRAWL_SHIFT_Y * (math.sin(ph))

    feet = {}
    for leg in LEGS:
        lp = wrap_phase(phase + GP.CRAWL_OFFSET[leg])
        frac, lift = trot_profile(lp, GP.CRAWL_DUTY)

        left_side = leg in ("FL", "RL")
        turn_x = (-1.0 if left_side else +1.0) * turn * GP.CRAWL_STEP_LENGTH
        stride_x = fwd * GP.CRAWL_STEP_LENGTH + turn_x

        dx = frac * stride_x
        dz = lift * GP.CRAWL_STEP_HEIGHT

        base_y = STANCE_Y if left_side else -STANCE_Y
        # body moves toward +com  =>  feet move by -com
        feet[leg] = (
            STANCE_X + dx - com_x,
            base_y - com_y,
            z0 + dz,
        )
    return feet


def swing_leg_at(phase):
    """Debug helper: which leg (if any) is airborne at this phase."""
    for leg in LEGS:
        lp = wrap_phase(phase + GP.CRAWL_OFFSET[leg])
        if lp >= GP.CRAWL_DUTY:
            return leg
    return None
