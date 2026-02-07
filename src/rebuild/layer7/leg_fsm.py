"""
Layer 7 — Leg Finite State Machine (FSM)
=======================================

Responsibilities:
- Decide per-leg STANCE / SWING states
- Enforce gait sequencing
- No geometry, no IK, no posture, no hardware

States:
- STANCE : foot must stay on ground
- SWING  : foot allowed to follow swing trajectory

Output:
{
  "FL": LegState,
  "FR": LegState,
  "RL": LegState,
  "RR": LegState,
}
"""

from enum import Enum
from typing import Dict


# -------------------------------------------------
# Leg states
# -------------------------------------------------

class LegState(Enum):
    STANCE = 0
    SWING = 1


# -------------------------------------------------
# Gait definition (WALK – SpotMicro canonical)
# -------------------------------------------------
# Order in which legs are allowed to swing

WALK_ORDER = ["FL", "RR", "FR", "RL"]


# -------------------------------------------------
# FSM class
# -------------------------------------------------

class LegFSM:
    def __init__(self, swing_time: float = 0.25):
        """
        Args:
            swing_time : seconds per leg swing
        """
        self.swing_time = swing_time
        self.legs = WALK_ORDER
        self.index = 0
        self.timer = 0.0

        # Initialize all legs in STANCE
        self.state = {leg: LegState.STANCE for leg in self.legs}

        # First leg starts swinging
        self.state[self.legs[self.index]] = LegState.SWING

    # -------------------------------------------------

    def reset(self):
        """Reset FSM to initial state."""
        self.index = 0
        self.timer = 0.0
        self.state = {leg: LegState.STANCE for leg in self.legs}
        self.state[self.legs[0]] = LegState.SWING

    # -------------------------------------------------

    def update(self, dt: float) -> Dict[str, LegState]:
        """
        Advance FSM by dt seconds.

        Args:
            dt : timestep (seconds)

        Returns:
            dict mapping leg -> LegState
        """
        self.timer += dt

        if self.timer >= self.swing_time:
            # End current swing
            current_leg = self.legs[self.index]
            self.state[current_leg] = LegState.STANCE

            # Advance to next leg
            self.index = (self.index + 1) % len(self.legs)
            next_leg = self.legs[self.index]
            self.state[next_leg] = LegState.SWING

            self.timer = 0.0

        return self.state.copy()
