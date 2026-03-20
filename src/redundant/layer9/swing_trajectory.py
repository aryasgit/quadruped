import math

class SwingTrajectory:
    """
    Layer 9 — Cartesian foot swing trajectory
    Produces (dx, dz) offsets in meters
    """

    def __init__(self, step_length=0.05, step_height=0.035):
        self.step_length = step_length
        self.step_height = step_height

    def sample(self, phase: float):
        """
        phase ∈ [0, 1]
        returns (dx, dz)
        """

        # Forward/back component
        dx = self.step_length * math.sin(math.pi * phase)

        # Vertical lift (true semicircle)
        dz = self.step_height * math.sin(math.pi * phase)

        return dx, dz
