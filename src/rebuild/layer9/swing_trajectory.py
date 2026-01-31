# layer9/swing_trajectory.py
import math


class SwingTrajectory:
    """
    Layer 9 — Step-in-place swing trajectory
    """

    def __init__(self, step_length=0.04, step_height=0.035):
        self.step_length = step_length
        self.step_height = step_height

    def sample(self, phase: float):
        """
        phase ∈ [0, 1]
        returns (dx, dz) in meters
        """

        # Step-in-place forward/back motion
        dx = self.step_length * math.sin(2.0 * math.pi * phase)

        # Vertical lift
        dz = self.step_height * math.sin(math.pi * phase)

        return dx, dz
