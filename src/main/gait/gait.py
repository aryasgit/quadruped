"""
L4 — GAIT GENERATOR (pure)
==========================

foot_targets(phase, command, params, height_z) -> {leg: (x, y, z)}  hip-local.

Design goals vs the old stack:
  * PURE function of (phase, command, params) — no I/O, no hidden state,
    fully unit-testable against expected foot paths.
  * SYMMETRIC by construction — fwd/back, left/right strafe, and left/right turn
    are all sign flips of one vector formulation. No per-direction branches.
    (Fixes "strafe right never works" and the systematic left-drift class.)
  * PHASE is an input (accumulated by the control loop as phase += freq*dt), so
    changing gait frequency never causes a position jump.
  * Diagonal trot phasing: DIAG_A legs at `phase`, DIAG_B at `phase + 0.5`.

Command convention (all in [-1, 1]):
    fwd    > 0 forward,     < 0 backward
    strafe > 0 right,       < 0 left
    turn   > 0 turn-left (CCW), < 0 turn-right

Foot frame: x forward, y left (+), z up (negative down).
"""

import math
from config.robot_spec import STANCE_X, STANCE_Y, LEGS
from config import gait_params as GP
from gait.trajectory import trot_profile, wrap_phase

_EPS = 0.01


def _leg_phase(leg, phase):
    return phase if leg in GP.DIAG_A else wrap_phase(phase + 0.5)


def foot_targets(phase, command, params=GP.DEFAULT_GAIT, height_z=None):
    """
    phase:    float in [0,1) accumulated by the control loop
    command:  object/namedtuple with .fwd, .strafe, .turn in [-1,1]
    params:   GaitParams
    height_z: foot Z (defaults to params.stance_z)
    """
    fwd, strafe, turn = command.fwd, command.strafe, command.turn
    z0 = params.stance_z if height_z is None else height_z

    # --- stride magnitudes & blended lift height (symmetric, branch-free) ---
    backward = fwd < 0.0
    len_fwd = params.step_length * (params.backward_length_scale if backward else 1.0)
    h_fwd = params.step_height * (params.backward_height_scale if backward else 1.0)

    active = abs(fwd) + abs(strafe) + abs(turn)
    if active > _EPS:
        lift_height = (
            abs(fwd) * h_fwd
            + abs(strafe) * params.lateral_step_height
            + abs(turn) * params.turn_step_height
        ) / active
    else:
        lift_height = 0.0

    # --- body-shift overlay (CoM toward the support side) ---
    shift_x = -fwd * params.body_shift_fwd
    shift_y = -strafe * params.body_shift_lat - turn * params.body_shift_turn

    # --- pitch bias (nose-down while moving; extra in reverse) ---
    mag = min(1.0, math.sqrt(fwd * fwd + strafe * strafe + turn * turn))
    pitch = params.motion_pitch_bias * mag
    if fwd < -_EPS:
        pitch += params.backward_pitch_bias * min(1.0, abs(fwd))

    feet = {}
    for leg in LEGS:
        lp = _leg_phase(leg, phase)
        frac, lift = trot_profile(lp, params.duty)

        # Stride vector = desired body-velocity direction (foot drags opposite).
        left_side = leg in ("FL", "RL")
        turn_x = (-1.0 if left_side else +1.0) * turn * params.turn_step_length
        stride_x = fwd * len_fwd + turn_x
        stride_y = -strafe * params.lateral_step_length

        dx = frac * stride_x
        dy = frac * stride_y
        dz = lift * lift_height

        base_y = STANCE_Y if left_side else -STANCE_Y
        pitch_z = pitch if leg in ("FL", "FR") else -pitch

        feet[leg] = (
            STANCE_X + dx + shift_x,
            base_y + dy + shift_y,
            z0 + dz + pitch_z,
        )

    return feet
