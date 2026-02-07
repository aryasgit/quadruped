"""
Layer 8 — Gait Phase Supervisor
===============================

Responsibilities:
- Track global gait phase
- Determine per-leg STANCE / SWING
- Freeze SWING legs at nominal trajectory
- Allow posture ONLY on STANCE legs

This layer does:
- NO IK
- NO servo output
- NO trajectory math

Policy:
- STANCE legs → posture allowed
- SWING legs  → posture FROZEN (Option A)
"""

from typing import Dict, Tuple


STANCE = "STANCE"
SWING  = "SWING"


class GaitPhaseSupervisor:
    def __init__(
        self,
        phase_offset: Dict[str, float],
        duty_factor: float,
    ):
        self.phase_offset = phase_offset
        self.duty = duty_factor

    # -------------------------------------------------
    # Phase utilities
    # -------------------------------------------------

    @staticmethod
    def _wrap_phase(p: float) -> float:
        return p % 1.0

    # -------------------------------------------------
    # Per-leg phase classification
    # -------------------------------------------------

    def leg_state(self, base_phase: float, leg: str) -> str:
        phase = self._wrap_phase(base_phase + self.phase_offset[leg])
        return STANCE if phase < self.duty else SWING

    # -------------------------------------------------
    # Main supervision step
    # -------------------------------------------------

    def step(
        self,
        t: float,
        frequency: float,
        nominal_targets: Dict[str, Tuple[float, float, float]],
        posture_targets: Dict[str, Tuple[float, float, float]],
    ) -> Tuple[
        Dict[str, Tuple[float, float, float]],
        Dict[str, str]
    ]:
        """
        Args:
            t               : time (seconds)
            frequency       : gait frequency (Hz)
            nominal_targets : from Layer 6
            posture_targets : from Layer 5

        Returns:
            (final_targets, leg_states)
        """

        # Standing still → posture applies everywhere
        if frequency <= 0.0:
            return posture_targets, {
                leg: STANCE for leg in nominal_targets
            }

        base_phase = t * frequency
        final_targets = {}
        states = {}

        for leg in nominal_targets:
            state = self.leg_state(base_phase, leg)
            states[leg] = state

            if state == SWING:
                # Freeze swing leg at nominal trajectory
                final_targets[leg] = nominal_targets[leg]
            else:
                # Allow posture on stance legs
                final_targets[leg] = posture_targets[leg]

        return final_targets, states
