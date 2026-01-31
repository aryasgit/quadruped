# layer7/leg_fsm.py

from enum import Enum
from typing import Dict, Optional


class LegState(Enum):
    STANCE = 0
    UNLOAD = 1
    SWING = 2
    LOAD = 3


LEG_ORDER_WALK = ["FL", "RR", "FR", "RL"]   # crawl / walk-safe diagonal order


class LegFSM:
    """
    Layer 7 — Leg Finite State Machine

    Authoritative responsibilities:
    - Which leg may lift
    - Enforce single-leg swing (for walk)
    - Respect Layer 6 balance authority
    """

    def __init__(self):
        self.state: Dict[str, LegState] = {
            "FL": LegState.STANCE,
            "FR": LegState.STANCE,
            "RL": LegState.STANCE,
            "RR": LegState.STANCE,
        }

        self.active_leg: Optional[str] = None
        self.sequence = LEG_ORDER_WALK.copy()
        self.seq_idx = 0

    # --------------------------------------------------
    # PUBLIC UPDATE
    # --------------------------------------------------
    def update(
        self,
        allow_leg_lift: bool,
        foot_contact: Dict[str, bool],
        swing_done: bool,
        load_done: bool,
    ):
        """
        allow_leg_lift  : from Layer 6
        foot_contact    : per-leg contact (True = on ground)
        swing_done      : signal from Layer 9 later
        load_done       : signal from Layer 9 later
        """

        # No leg currently moving → try to start one
        if self.active_leg is None:
            if not allow_leg_lift:
                return

            next_leg = self._next_leg()
            if foot_contact.get(next_leg, True):
                self._enter_unload(next_leg)
            return

        # One leg is active → advance its state
        leg = self.active_leg
        st = self.state[leg]

        if st == LegState.UNLOAD:
            # Minimal rule: once contact breaks, go SWING
            if not foot_contact.get(leg, True):
                self.state[leg] = LegState.SWING

        elif st == LegState.SWING:
            if swing_done:
                self.state[leg] = LegState.LOAD

        elif st == LegState.LOAD:
            if load_done:
                self.state[leg] = LegState.STANCE
                self.active_leg = None
                self._advance_sequence()

    # --------------------------------------------------
    # QUERY HELPERS (used by higher layers)
    # --------------------------------------------------
    def is_swing_leg(self, leg: str) -> bool:
        return self.state[leg] == LegState.SWING

    def stance_legs(self):
        return [l for l, s in self.state.items() if s == LegState.STANCE]

    # --------------------------------------------------
    # INTERNALS
    # --------------------------------------------------
    def _enter_unload(self, leg: str):
        self.active_leg = leg
        self.state[leg] = LegState.UNLOAD

    def _next_leg(self) -> str:
        return self.sequence[self.seq_idx]

    def _advance_sequence(self):
        self.seq_idx = (self.seq_idx + 1) % len(self.sequence)
