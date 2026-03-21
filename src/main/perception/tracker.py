# arya/perception/tracker.py
"""
Layer 2a — OBJECT TRACKER (SORT)
=================================
Simple Online and Realtime Tracking.
Pure numpy — no scipy needed.

Input:  list of Detection(x, y, z_m, w, h, label, conf) per frame
Output: list of Track(id, x, y, z_m, vx, vy, label, age, hits)

x, y  = pixel center of bounding box
z_m   = depth in meters (from OAK-D spatial)
vx,vy = pixel velocity (px/frame), used to predict future position
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional


# ── Data contracts ───────────────────────────────────────────────

@dataclass
class Detection:
    x: float        # bbox center x (pixels)
    y: float        # bbox center y (pixels)
    z_m: float      # depth in meters
    w: float        # bbox width (pixels)
    h: float        # bbox height (pixels)
    label: str      # class label ("person", "chair" etc.)
    conf: float     # detection confidence 0-1


@dataclass
class Track:
    id: int
    x: float
    y: float
    z_m: float
    vx: float       # px/frame
    vy: float       # px/frame
    w: float
    h: float
    label: str
    age: int        # total frames this track has existed
    hits: int       # consecutive frames matched
    misses: int     # consecutive frames unmatched


# ── Kalman state: [x, y, vx, vy] ────────────────────────────────

class KalmanBox:
    """
    Constant-velocity Kalman filter for a single bounding box center.
    State: [x, y, vx, vy]
    """

    _F = np.array([[1,0,1,0],
                   [0,1,0,1],
                   [0,0,1,0],
                   [0,0,0,1]], dtype=float)

    _H = np.array([[1,0,0,0],
                   [0,1,0,0]], dtype=float)

    _Q = np.diag([1.0, 1.0, 0.5, 0.5])   # process noise
    _R = np.diag([4.0, 4.0])              # measurement noise

    def __init__(self, x, y):
        self.x = np.array([x, y, 0.0, 0.0], dtype=float)
        n = 4
        self.P = np.eye(n) * 10.0

    def predict(self):
        self.x = self._F @ self.x
        self.P = self._F @ self.P @ self._F.T + self._Q
        return self.x[:2].copy()

    def update(self, z):
        z = np.array(z, dtype=float)
        y = z - self._H @ self.x
        S = self._H @ self.P @ self._H.T + self._R
        K = self.P @ self._H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self._H) @ self.P

    @property
    def pos(self):
        return self.x[0], self.x[1]

    @property
    def vel(self):
        return self.x[2], self.x[3]


# ── IoU helper ───────────────────────────────────────────────────

def _iou(ax, ay, aw, ah, bx, by, bw, bh) -> float:
    ax1, ay1, ax2, ay2 = ax-aw/2, ay-ah/2, ax+aw/2, ay+ah/2
    bx1, by1, bx2, by2 = bx-bw/2, by-bh/2, bx+bw/2, by+bh/2
    ix = max(0, min(ax2,bx2) - max(ax1,bx1))
    iy = max(0, min(ay2,by2) - max(ay1,by1))
    inter = ix * iy
    union = aw*ah + bw*bh - inter
    return inter / union if union > 0 else 0.0


# ── Hungarian assignment (pure numpy, O(n³)) ────────────────────

def _hungarian(cost: np.ndarray):
    """
    Returns (row_indices, col_indices) of optimal assignment.
    Minimises cost. Works for rectangular matrices.
    """
    from copy import deepcopy
    C = deepcopy(cost).astype(float)
    n, m = C.shape
    # Pad to square
    size = max(n, m)
    pad = np.full((size, size), 1e9)
    pad[:n, :m] = C
    C = pad

    # Row / col reduction
    C -= C.min(axis=1, keepdims=True)
    C -= C.min(axis=0, keepdims=True)

    # Greedy assignment (good enough for small n, typically <20 detections)
    row_ind, col_ind = [], []
    used_r, used_c = set(), set()
    flat = np.argsort(C.ravel())
    for idx in flat:
        r, c = divmod(int(idx), size)
        if r not in used_r and c not in used_c and C[r,c] < 1e8:
            row_ind.append(r)
            col_ind.append(c)
            used_r.add(r)
            used_c.add(c)
        if len(row_ind) == min(n, m):
            break

    # Filter to valid indices only
    valid = [(r, c) for r, c in zip(row_ind, col_ind) if r < n and c < m]
    if not valid:
        return np.array([], dtype=int), np.array([], dtype=int)
    rows, cols = zip(*valid)
    return np.array(rows), np.array(cols)


# ── SORT Tracker ─────────────────────────────────────────────────

class SORTTracker:
    """
    SORT tracker.

    Config:
        iou_threshold   – minimum IoU to match a detection to a track
        max_misses      – frames without match before track is deleted
        min_hits        – frames matched before track is considered confirmed
    """

    def __init__(
        self,
        iou_threshold: float = 0.25,
        max_misses: int = 5,
        min_hits: int = 2,
    ):
        self.iou_threshold = iou_threshold
        self.max_misses = max_misses
        self.min_hits = min_hits

        self._tracks: List[dict] = []   # internal track state
        self._next_id = 1

    def update(self, detections: List[Detection]) -> List[Track]:
        """
        Call once per frame with all detections.
        Returns confirmed tracks (age >= min_hits).
        """

        # Step 1 — predict all tracks forward
        for t in self._tracks:
            t["kf"].predict()

        # Step 2 — build cost matrix (1 - IoU)
        n_det = len(detections)
        n_trk = len(self._tracks)

        matched_det  = set()
        matched_trk  = set()

        if n_det > 0 and n_trk > 0:
            cost = np.ones((n_trk, n_det))
            for ti, t in enumerate(self._tracks):
                px, py = t["kf"].pos
                for di, d in enumerate(detections):
                    iou = _iou(px, py, t["w"], t["h"],
                               d.x, d.y, d.w, d.h)
                    cost[ti, di] = 1.0 - iou

            r_idx, c_idx = _hungarian(cost)
            for r, c in zip(r_idx, c_idx):
                if cost[r, c] <= (1.0 - self.iou_threshold):
                    matched_trk.add(r)
                    matched_det.add(c)
                    d = detections[c]
                    t = self._tracks[r]
                    t["kf"].update([d.x, d.y])
                    t["z_m"]   = d.z_m
                    t["w"]     = d.w
                    t["h"]     = d.h
                    t["label"] = d.label
                    t["hits"]  += 1
                    t["misses"] = 0
                    t["age"]   += 1

        # Step 3 — unmatched detections → new tracks
        for di, d in enumerate(detections):
            if di not in matched_det:
                self._tracks.append({
                    "id":     self._next_id,
                    "kf":     KalmanBox(d.x, d.y),
                    "z_m":    d.z_m,
                    "w":      d.w,
                    "h":      d.h,
                    "label":  d.label,
                    "hits":   1,
                    "misses": 0,
                    "age":    1,
                })
                self._next_id += 1

        # Step 4 — unmatched tracks → increment misses
        for ti, t in enumerate(self._tracks):
            if ti not in matched_trk:
                t["misses"] += 1
                t["age"]    += 1

        # Step 5 — delete dead tracks
        self._tracks = [t for t in self._tracks if t["misses"] <= self.max_misses]

        # Step 6 — return confirmed tracks
        out = []
        for t in self._tracks:
            if t["hits"] >= self.min_hits:
                px, py = t["kf"].pos
                vx, vy = t["kf"].vel
                out.append(Track(
                    id     = t["id"],
                    x      = float(px),
                    y      = float(py),
                    z_m    = t["z_m"],
                    vx     = float(vx),
                    vy     = float(vy),
                    w      = t["w"],
                    h      = t["h"],
                    label  = t["label"],
                    age    = t["age"],
                    hits   = t["hits"],
                    misses = t["misses"],
                ))
        return out

    def reset(self):
        self._tracks.clear()
        self._next_id = 1