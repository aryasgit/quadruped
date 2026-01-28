# balance_controller.py
#
# COM-authoritative balance controller
# NO actuation
# NO geometry
# NO leg logic
# Outputs desired COM correction only

from dataclasses import dataclass
from typing import Dict

# ==================================================
# PUBLIC COMMAND TYPE (AUTHORITATIVE)
# ==================================================

@dataclass
class COMCommand:
    dx: float   # forward (+X)
    dy: float   # left (+Y)
    dz: float   # height (unused for now, keep = 0.0)


# ==================================================
# BALANCE CONTROLLER
# ==================================================

class BalanceController:
    """
    Converts body orientation → desired COM correction.
    This is the ONLY balance authority in the system.
    """

    # ---------- TUNABLE GAINS ----------
    K_ROLL  = 0.006    # meters / degree
    K_PITCH = 0.006

    # ---------- HARD LIMITS ----------
    MAX_DX = 0.05      # meters
    MAX_DY = 0.05

    def __init__(self):
        pass

    # ==================================================
    # CORE API
    # ==================================================
    def compute(self, state) -> COMCommand:
        """
        Input:
            state.roll   (deg)
            state.pitch  (deg)
            state.stability (0–1)

        Output:
            COMCommand(dx, dy, dz)

        Sign conventions:
            +roll  → robot tilts left  → COM must move right (−y)
            +pitch → robot tilts forward → COM must move backward (−x)
        """

        # ----- raw proportional correction -----
        dx = -self.K_PITCH * state.pitch
        dy = -self.K_ROLL  * state.roll

        # ----- stability-aware soft clamp -----
        scale = max(0.2, min(1.0, state.stability))

        dx *= scale
        dy *= scale

        # ----- hard safety clamp -----
        dx = max(-self.MAX_DX, min(self.MAX_DX, dx))
        dy = max(-self.MAX_DY, min(self.MAX_DY, dy))

        return COMCommand(
            dx=dx,
            dy=dy,
            dz=0.0
        )
