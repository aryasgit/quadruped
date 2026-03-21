# arya/perception/planner.py
"""
Layer 3b — AVOIDANCE PLANNER
============================
Maps FSM state + world state → gait command.

Outputs GaitCommand which modifies step_length, step_height,
frequency, and direction override in the main controller.

Gait command is injected into controller.py's execute_single_cycle.
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from arya.perception.world import WorldState
from arya.perception.fsm import BehaviourState, FSMOutput


# ── Direction override enum ───────────────────────────────────────

class DirectionOverride(Enum):
    NONE        = auto()   # use operator command
    STOP        = auto()   # zero motion
    TURN_LEFT   = auto()
    TURN_RIGHT  = auto()
    PASS_LEFT   = auto()   # strafe left
    PASS_RIGHT  = auto()   # strafe right


# ── Gait command ─────────────────────────────────────────────────

@dataclass
class GaitCommand:
    direction_override: DirectionOverride
    step_length_scale:  float   # multiplier on nominal STEP_LENGTH (0..1)
    step_height_scale:  float   # multiplier on nominal STEP_HEIGHT
    freq_scale:         float   # multiplier on nominal FREQ
    allow_operator:     bool    # if False, ignore operator input entirely


# ── Nominal gait (must match controller.py) ──────────────────────

NOMINAL_STEP  = 0.06
NOMINAL_H     = 0.030
NOMINAL_FREQ  = 1.5


# ── Planner rules ────────────────────────────────────────────────

class AvoidancePlanner:
    """
    Single call per control tick.
    Returns GaitCommand based on current FSM state and world geometry.
    """

    def plan(self, fsm: FSMOutput, world: WorldState) -> GaitCommand:
        state = fsm.state

        # ── FREE — full operator control ─────────────────────────
        if state == BehaviourState.FREE:
            return GaitCommand(
                direction_override = DirectionOverride.NONE,
                step_length_scale  = 1.0,
                step_height_scale  = 1.0,
                freq_scale         = 1.0,
                allow_operator     = True,
            )

        # ── CAUTIOUS — slow down, operator still in control ──────
        if state == BehaviourState.CAUTIOUS:
            return GaitCommand(
                direction_override = DirectionOverride.NONE,
                step_length_scale  = 0.40,
                step_height_scale  = 0.80,
                freq_scale         = 0.70,
                allow_operator     = True,
            )

        # ── STOPPING — reduce speed aggressively ─────────────────
        if state == BehaviourState.STOPPING:
            return GaitCommand(
                direction_override = DirectionOverride.STOP,
                step_length_scale  = 0.0,
                step_height_scale  = 1.0,
                freq_scale         = 0.0,
                allow_operator     = False,
            )

        # ── STOPPED — zero motion ─────────────────────────────────
        if state == BehaviourState.STOPPED:
            return GaitCommand(
                direction_override = DirectionOverride.STOP,
                step_length_scale  = 0.0,
                step_height_scale  = 1.0,
                freq_scale         = 0.0,
                allow_operator     = False,
            )

        # ── EVADING — choose turn direction ──────────────────────
        if state == BehaviourState.EVADING:
            # Prefer side with more room
            override = self._choose_evasion(world)
            return GaitCommand(
                direction_override = override,
                step_length_scale  = 0.60,
                step_height_scale  = 1.0,
                freq_scale         = 0.80,
                allow_operator     = False,
            )

        # Fallback
        return GaitCommand(
            direction_override = DirectionOverride.STOP,
            step_length_scale  = 0.0,
            step_height_scale  = 1.0,
            freq_scale         = 0.0,
            allow_operator     = False,
        )

    # ── evasion direction selector ────────────────────────────────

    def _choose_evasion(self, world: WorldState) -> DirectionOverride:
        """
        Use grid spatial awareness to pick evasion direction.

        Logic:
          - If only one side is clear → go that way
          - If object is in left half of frame → pass right
          - If object is in right half → pass left
          - Default: turn right (arbitrary tiebreak)
        """
        left_ok  = world.left_clear
        right_ok = world.right_clear

        if left_ok and not right_ok:
            return DirectionOverride.PASS_LEFT
        if right_ok and not left_ok:
            return DirectionOverride.PASS_RIGHT

        # Both clear or both blocked — check track positions
        if world.predictions:
            # Average y of all tracked objects
            avg_y = sum(p.y_now for p in world.predictions) / len(world.predictions)
            if avg_y > 0:
                # Objects on left (+Y) → pass right
                return DirectionOverride.PASS_RIGHT
            else:
                return DirectionOverride.PASS_LEFT

        return DirectionOverride.TURN_RIGHT


# ── Integration helper ────────────────────────────────────────────

def apply_gait_command(
    cmd: GaitCommand,
    operator_direction: Optional[str],
    step_length: float,
    step_height: float,
    freq: float,
) -> tuple:
    """
    Returns (effective_direction, step_length, step_height, freq).

    Merge operator intent with planner command.
    controller.py calls this each cycle before execute_single_cycle.
    """
    from arya.perception.planner import DirectionOverride

    # Map override enum to controller direction strings
    _OVERRIDE_MAP = {
        DirectionOverride.NONE:       operator_direction,
        DirectionOverride.STOP:       None,
        DirectionOverride.TURN_LEFT:  "turn_left",
        DirectionOverride.TURN_RIGHT: "turn_right",
        DirectionOverride.PASS_LEFT:  "left",
        DirectionOverride.PASS_RIGHT: "right",
    }

    if not cmd.allow_operator:
        direction = _OVERRIDE_MAP[cmd.direction_override]
    else:
        direction = operator_direction

    return (
        direction,
        step_length * cmd.step_length_scale,
        step_height * cmd.step_height_scale,
        freq        * cmd.freq_scale,
    )