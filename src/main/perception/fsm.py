# arya/perception/fsm.py
"""
Layer 3a — BEHAVIOUR FSM
========================
Five states with hysteresis-gated transitions.
Minimum dwell time per state prevents flicker.

States:
    FREE       — no obstacles detected, full gait
    CAUTIOUS   — obstacle present but distant, slowed gait
    STOPPING   — imminent collision, decelerating
    STOPPED    — zero motion, waiting
    EVADING    — active avoidance manoeuvre underway

Input:  WorldState (from world.py)
Output: BehaviourState enum
"""

import time
from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional

from arya.perception.world import WorldState


# ── Thresholds ────────────────────────────────────────────────────

ZONE_STOP    = 0.35   # m — stop below this
ZONE_CAUTION = 0.80   # m — caution below this

# Minimum dwell time in each state (seconds)
DWELL = {
    "FREE":     0.20,
    "CAUTIOUS": 0.30,
    "STOPPING": 0.10,
    "STOPPED":  0.40,
    "EVADING":  0.80,
}

# Clear-distance required before leaving STOPPED/EVADING
CLEAR_HYSTERESIS_M = 1.00


# ── State ─────────────────────────────────────────────────────────

class BehaviourState(Enum):
    FREE     = auto()
    CAUTIOUS = auto()
    STOPPING = auto()
    STOPPED  = auto()
    EVADING  = auto()


@dataclass
class FSMOutput:
    state:        BehaviourState
    prev_state:   BehaviourState
    state_age_s:  float          # seconds in current state
    trigger:      str            # human-readable reason for last transition


# ── FSM ───────────────────────────────────────────────────────────

class BehaviourFSM:

    def __init__(self):
        self.state      = BehaviourState.FREE
        self.prev_state = BehaviourState.FREE
        self._enter_t   = time.monotonic()
        self.trigger    = "init"

    # ── dwell check ──────────────────────────────────────────────

    def _dwell_ok(self) -> bool:
        min_dwell = DWELL.get(self.state.name, 0.0)
        return (time.monotonic() - self._enter_t) >= min_dwell

    def _transition(self, new_state: BehaviourState, reason: str):
        if new_state != self.state:
            self.prev_state = self.state
            self.state      = new_state
            self._enter_t   = time.monotonic()
            self.trigger    = reason

    # ── main update ──────────────────────────────────────────────

    def update(self, world: WorldState) -> FSMOutput:
        nearest = world.nearest_m
        no_obstacle = nearest is None

        # ── FREE → anything ──────────────────────────────────────
        if self.state == BehaviourState.FREE:
            if world.closing_threat or (nearest is not None and nearest < ZONE_STOP):
                self._transition(BehaviourState.STOPPING, "closing_threat or nearest < stop_zone")
            elif world.person_present:
                self._transition(BehaviourState.CAUTIOUS, "person_within_1m")
            elif nearest is not None and nearest < ZONE_CAUTION:
                self._transition(BehaviourState.CAUTIOUS, f"nearest={nearest:.2f}m < {ZONE_CAUTION}m")

        # ── CAUTIOUS → FREE / STOPPING ───────────────────────────
        elif self.state == BehaviourState.CAUTIOUS:
            if not self._dwell_ok():
                pass
            elif world.closing_threat or (nearest is not None and nearest < ZONE_STOP):
                self._transition(BehaviourState.STOPPING, "closing_threat or entered_stop_zone")
            elif no_obstacle and not world.person_present:
                self._transition(BehaviourState.FREE, "obstacle_cleared")

        # ── STOPPING → STOPPED ───────────────────────────────────
        elif self.state == BehaviourState.STOPPING:
            if not self._dwell_ok():
                pass
            else:
                self._transition(BehaviourState.STOPPED, "stopped")

        # ── STOPPED → EVADING / FREE ─────────────────────────────
        elif self.state == BehaviourState.STOPPED:
            if not self._dwell_ok():
                pass
            elif no_obstacle or (nearest is not None and nearest > CLEAR_HYSTERESIS_M):
                # Path cleared on its own
                self._transition(BehaviourState.FREE, "path_cleared_while_stopped")
            elif world.left_clear or world.right_clear:
                # Evade around obstacle
                self._transition(BehaviourState.EVADING, "evasion_path_available")
            # Otherwise stay STOPPED

        # ── EVADING → FREE / STOPPED ─────────────────────────────
        elif self.state == BehaviourState.EVADING:
            if not self._dwell_ok():
                pass
            elif no_obstacle or (nearest is not None and nearest > CLEAR_HYSTERESIS_M):
                self._transition(BehaviourState.FREE, "evasion_complete")
            elif world.closing_threat or (nearest is not None and nearest < ZONE_STOP):
                # Obstacle still blocking even after evasion attempt
                self._transition(BehaviourState.STOPPED, "evasion_blocked")

        age = time.monotonic() - self._enter_t
        return FSMOutput(
            state       = self.state,
            prev_state  = self.prev_state,
            state_age_s = age,
            trigger     = self.trigger,
        )

    def reset(self):
        self.state      = BehaviourState.FREE
        self.prev_state = BehaviourState.FREE
        self._enter_t   = time.monotonic()
        self.trigger    = "reset"