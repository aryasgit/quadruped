# layer6/balance_supervisor.py

import math
from typing import Dict, Tuple

class BalanceSupervisor:
    """
    Layer 6 — Balance Supervisor

    Responsibilities:
    - Evaluate static stability
    - Gate posture reference adaptation
    - Authorize / forbid leg lifting

    This layer:
    - DOES NOT move joints
    - DOES NOT compute IK
    - DOES NOT touch hardware
    """

    def __init__(
        self,
        min_margin: float = 0.015,   # meters
        critical_margin: float = 0.005
    ):
        self.min_margin = min_margin
        self.critical_margin = critical_margin

        self.balance_ok = True
        self.allow_leg_lift = False
        self.allow_posture_ref_adapt = True

    # --------------------------------------------------
    # PUBLIC UPDATE
    # --------------------------------------------------
    def update(
        self,
        foot_positions: Dict[str, Tuple[float, float]],
        foot_contact: Dict[str, bool],
        cg_xy: Tuple[float, float]
    ):
        """
        foot_positions:
            {
              "FL": (x, y),
              "FR": (x, y),
              "RL": (x, y),
              "RR": (x, y)
            }

        foot_contact:
            {
              "FL": True/False,
              ...
            }

        cg_xy:
            (x, y) center of gravity projection
        """

        support_polygon = self._compute_support_polygon(
            foot_positions, foot_contact
        )

        if len(support_polygon) < 3:
            # Cannot define a polygon → unstable by definition
            self.balance_ok = False
            self.allow_leg_lift = False
            self.allow_posture_ref_adapt = False
            return

        margin = self._distance_to_polygon_edge(
            cg_xy, support_polygon
        )

        # -----------------------------
        # DECISION LOGIC (AUTHORITATIVE)
        # -----------------------------
        self.balance_ok = margin > 0.0

        self.allow_leg_lift = margin > self.min_margin

        self.allow_posture_ref_adapt = margin > self.critical_margin

    # --------------------------------------------------
    # INTERNAL GEOMETRY
    # --------------------------------------------------
    def _compute_support_polygon(
        self,
        foot_positions: Dict[str, Tuple[float, float]],
        foot_contact: Dict[str, bool]
    ):
        # Order is IMPORTANT (convex hull not needed yet)
        order = ["FL", "FR", "RR", "RL"]

        polygon = []
        for leg in order:
            if foot_contact.get(leg, False):
                polygon.append(foot_positions[leg])

        return polygon

    def _distance_to_polygon_edge(
        self,
        point: Tuple[float, float],
        polygon
    ):
        """
        Signed distance to closest polygon edge.
        Positive = inside
        Negative = outside
        """

        min_dist = float("inf")

        px, py = point

        for i in range(len(polygon)):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % len(polygon)]

            dx = x2 - x1
            dy = y2 - y1

            length = math.hypot(dx, dy)
            if length == 0:
                continue

            # outward normal
            nx = dy / length
            ny = -dx / length

            dist = (px - x1) * nx + (py - y1) * ny
            min_dist = min(min_dist, dist)

        return min_dist
