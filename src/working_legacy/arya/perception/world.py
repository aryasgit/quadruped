# arya/perception/world.py
"""
Layer 2b — WORLD MODEL
======================
Persistent environment understanding beyond single-frame detections.

Components:
  OccupancyGrid   — 10cm cells, 3m forward × 2m wide
  TrajectoryEKF   — per-track extended Kalman filter predicting position in 0.5s / 1.0s
  WorldModel      — combines tracker output into actionable world state

Coordinate convention (robot-centric):
  +X = forward
  +Y = left
  Z  = up (not used here — floor plane assumed flat)

OAK-D gives depth (Z camera = X robot) and horizontal pixel offset → Y robot.
"""

import math
import time
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from arya.perception.tracker import Track


# ── Grid config ──────────────────────────────────────────────────

GRID_CELL_M   = 0.10    # 10 cm resolution
GRID_DEPTH_M  = 3.00    # 3m forward
GRID_WIDTH_M  = 2.00    # 2m wide (1m each side)

GRID_ROWS = int(GRID_DEPTH_M / GRID_CELL_M)   # 30
GRID_COLS = int(GRID_WIDTH_M / GRID_CELL_M)   # 20

GRID_DECAY = 0.85       # per-frame decay factor (cells fade if not reinforced)
GRID_HIT   = 0.90       # confidence added per detection hit
GRID_THRESH = 0.5       # above this = occupied


# ── Camera → robot transform ─────────────────────────────────────
# OAK-D: depth along Z, horizontal pixel offset → Y via FOV

CAMERA_FOV_H_DEG = 73.0   # OAK-D horizontal FOV
FRAME_W_PX       = 640    # detection frame width


def pixel_to_robot_y(pixel_x: float, depth_m: float) -> float:
    """Convert pixel x offset to robot Y (left positive)."""
    fov_rad = math.radians(CAMERA_FOV_H_DEG)
    # Normalised offset: -0.5 (right edge) to +0.5 (left edge)
    norm_x = (FRAME_W_PX / 2 - pixel_x) / FRAME_W_PX
    angle = norm_x * fov_rad
    return depth_m * math.tan(angle)


# ── EKF per track ─────────────────────────────────────────────────

@dataclass
class PredictedPath:
    track_id: int
    label: str
    x_now: float       # robot forward (m)
    y_now: float       # robot lateral (m)
    x_05s: float       # predicted at +0.5s
    y_05s: float
    x_10s: float       # predicted at +1.0s
    y_10s: float
    speed_ms: float    # magnitude of velocity (m/s)


class TrackEKF:
    """
    Simple constant-velocity EKF for one track in robot frame.
    State: [x, y, vx, vy]  (meters, m/s)
    """

    def __init__(self, x: float, y: float, dt: float = 0.033):
        self.dt = dt
        self.x = np.array([x, y, 0.0, 0.0], dtype=float)
        self.P = np.diag([0.1, 0.1, 0.5, 0.5])

        self.F = np.array([[1,0,dt,0],
                           [0,1,0,dt],
                           [0,0,1, 0],
                           [0,0,0, 1]], dtype=float)
        self.H = np.array([[1,0,0,0],
                           [0,1,0,0]], dtype=float)
        self.Q = np.diag([0.01, 0.01, 0.1, 0.1])
        self.R = np.diag([0.04, 0.04])   # ≈ 20cm measurement noise

    def update(self, x_meas: float, y_meas: float):
        self.F[0,2] = self.dt
        self.F[1,3] = self.dt

        # Predict
        xp = self.F @ self.x
        Pp = self.F @ self.P @ self.F.T + self.Q

        # Update
        z = np.array([x_meas, y_meas])
        y = z - self.H @ xp
        S = self.H @ Pp @ self.H.T + self.R
        K = Pp @ self.H.T @ np.linalg.inv(S)
        self.x = xp + K @ y
        self.P = (np.eye(4) - K @ self.H) @ Pp

    def predict_at(self, t_seconds: float) -> Tuple[float, float]:
        """Predict position at t_seconds in the future."""
        steps = int(round(t_seconds / self.dt))
        x = self.x.copy()
        for _ in range(steps):
            x = self.F @ x
        return float(x[0]), float(x[1])

    @property
    def speed_ms(self) -> float:
        return float(math.sqrt(self.x[2]**2 + self.x[3]**2))


# ── Occupancy grid ────────────────────────────────────────────────

class OccupancyGrid:
    """
    Probability grid in front of the robot.
    Row 0 = closest (0-10cm), Row N = furthest.
    Col 0 = far right, Col N = far left.
    """

    def __init__(self):
        self.grid = np.zeros((GRID_ROWS, GRID_COLS), dtype=float)

    def decay(self):
        self.grid *= GRID_DECAY

    def mark(self, x_m: float, y_m: float, confidence: float = GRID_HIT):
        """Mark a cell occupied given robot-frame coordinates."""
        row = int(x_m / GRID_CELL_M)
        col = int((y_m + GRID_WIDTH_M / 2) / GRID_CELL_M)

        if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
            self.grid[row, col] = min(1.0, self.grid[row, col] + confidence * (1.0 - GRID_DECAY))

    def nearest_occupied_m(self, y_band_m: float = 0.5) -> Optional[float]:
        """
        Return distance in meters to nearest occupied cell
        within ±y_band_m of center.
        Returns None if nothing occupied.
        """
        col_center = GRID_COLS // 2
        half_cols  = int(y_band_m / GRID_CELL_M)
        col_lo = max(0, col_center - half_cols)
        col_hi = min(GRID_COLS, col_center + half_cols + 1)

        for row in range(GRID_ROWS):
            if self.grid[row, col_lo:col_hi].max() >= GRID_THRESH:
                return (row + 0.5) * GRID_CELL_M
        return None

    def is_clear(self, x_m: float, y_m: float, radius_m: float = 0.15) -> bool:
        """Check if a robot-frame region is clear."""
        r0 = int((x_m - radius_m) / GRID_CELL_M)
        r1 = int((x_m + radius_m) / GRID_CELL_M) + 1
        c0 = int((y_m - radius_m + GRID_WIDTH_M/2) / GRID_CELL_M)
        c1 = int((y_m + radius_m + GRID_WIDTH_M/2) / GRID_CELL_M) + 1
        r0, r1 = max(0,r0), min(GRID_ROWS, r1)
        c0, c1 = max(0,c0), min(GRID_COLS, c1)
        return self.grid[r0:r1, c0:c1].max() < GRID_THRESH

    def reset(self):
        self.grid[:] = 0.0


# ── World model ───────────────────────────────────────────────────

@dataclass
class WorldState:
    tracks:           List[Track]
    predictions:      List[PredictedPath]
    nearest_m:        Optional[float]   # closest object on-axis
    left_clear:       bool              # left lane clear at 1m
    right_clear:      bool              # right lane clear at 1m
    person_present:   bool              # any person within 1m
    closing_threat:   bool              # any tracked object predicted <0.6m in 1s


class WorldModel:
    """
    Aggregates tracker output into persistent world understanding.
    Call update(tracks) every frame.
    """

    def __init__(self, dt: float = 0.033):
        self.dt = dt
        self.grid  = OccupancyGrid()
        self._ekfs: Dict[int, TrackEKF] = {}

    def update(self, tracks: List[Track]) -> WorldState:
        # 1 — decay grid
        self.grid.decay()

        # 2 — sync EKF pool
        active_ids = {t.id for t in tracks}
        dead = [tid for tid in self._ekfs if tid not in active_ids]
        for tid in dead:
            del self._ekfs[tid]

        # 3 — update EKFs + mark grid
        predictions = []
        person_present = False
        closing_threat = False

        for track in tracks:
            x_m = track.z_m                                     # depth = forward
            y_m = pixel_to_robot_y(track.x, track.z_m)         # pixel → lateral

            # EKF
            if track.id not in self._ekfs:
                self._ekfs[track.id] = TrackEKF(x_m, y_m, self.dt)
            ekf = self._ekfs[track.id]
            ekf.update(x_m, y_m)

            # Grid
            self.grid.mark(x_m, y_m)

            # Trajectory prediction
            x05, y05 = ekf.predict_at(0.5)
            x10, y10 = ekf.predict_at(1.0)

            pred = PredictedPath(
                track_id  = track.id,
                label     = track.label,
                x_now     = x_m,
                y_now     = y_m,
                x_05s     = x05,
                y_05s     = y05,
                x_10s     = x10,
                y_10s     = y10,
                speed_ms  = ekf.speed_ms,
            )
            predictions.append(pred)

            # Flags
            if track.label == "person" and x_m < 1.0:
                person_present = True

            # Closing threat: moving toward robot AND predicted to be <0.6m in 1s
            if ekf.speed_ms > 0.3 and x10 < 0.60:
                closing_threat = True

        # 4 — spatial awareness
        nearest_m    = self.grid.nearest_occupied_m(y_band_m=0.30)
        left_clear   = self.grid.is_clear(1.0,  0.35, radius_m=0.20)
        right_clear  = self.grid.is_clear(1.0, -0.35, radius_m=0.20)

        return WorldState(
            tracks         = tracks,
            predictions    = predictions,
            nearest_m      = nearest_m,
            left_clear     = left_clear,
            right_clear    = right_clear,
            person_present = person_present,
            closing_threat = closing_threat,
        )

    def reset(self):
        self.grid.reset()
        self._ekfs.clear()