"""
Detection post-processing.
Converts raw DepthAI spatial detections and tracklets into structured
Detection objects with 3D coordinates in meters.
"""

from dataclasses import dataclass, field
from typing import List, Optional
from .pipeline import LABELS, NAV_LABELS


@dataclass
class Detection:
    label: str
    label_id: int
    confidence: float
    bbox: tuple                    # (x1, y1, x2, y2) normalized 0→1
    spatial_x: float               # meters, right = +
    spatial_y: float               # meters, down  = +
    spatial_z: float               # meters, forward = +
    distance: float                # euclidean distance (m)
    track_id: Optional[int] = None
    status: Optional[str] = None   # NEW / TRACKED / LOST / REMOVED

    @property
    def is_nav_relevant(self) -> bool:
        return self.label in NAV_LABELS

    @property
    def bearing_deg(self) -> float:
        """Horizontal bearing angle from camera centre (degrees). Left = negative."""
        import math
        if self.spatial_z <= 0:
            return 0.0
        return math.degrees(math.atan2(self.spatial_x, self.spatial_z))


# DepthAI tracker status enum → string
_STATUS_MAP = {0: "NEW", 1: "TRACKED", 2: "LOST", 3: "REMOVED"}


def _spatial_to_meters(spatial_coords):
    """Convert depthai SpatialCoordinates (mm) → (x, y, z, dist) in meters."""
    x = spatial_coords.x / 1000.0
    y = spatial_coords.y / 1000.0
    z = spatial_coords.z / 1000.0
    d = (x * x + y * y + z * z) ** 0.5
    return x, y, z, d


def process_detections(raw_detections, min_confidence: float = 0.5) -> List[Detection]:
    """Process raw SpatialImgDetections from the NN output queue."""
    results = []
    for det in raw_detections:
        if det.confidence < min_confidence:
            continue
        label = LABELS[det.label] if det.label < len(LABELS) else f"id_{det.label}"
        sx, sy, sz, dist = _spatial_to_meters(det.spatialCoordinates)

        results.append(Detection(
            label=label,
            label_id=det.label,
            confidence=det.confidence,
            bbox=(det.xmin, det.ymin, det.xmax, det.ymax),
            spatial_x=sx, spatial_y=sy, spatial_z=sz,
            distance=dist,
        ))
    return results


def process_tracklets(raw_tracklets, min_confidence: float = 0.5) -> List[Detection]:
    """Process raw Tracklets from the on-chip tracker output queue."""
    results = []
    for t in raw_tracklets:
        det = t.srcImgDetection
        if det.confidence < min_confidence:
            continue
        label = LABELS[det.label] if det.label < len(LABELS) else f"id_{det.label}"
        sx, sy, sz, dist = _spatial_to_meters(t.spatialCoordinates)

        results.append(Detection(
            label=label,
            label_id=det.label,
            confidence=det.confidence,
            bbox=(det.xmin, det.ymin, det.xmax, det.ymax),
            spatial_x=sx, spatial_y=sy, spatial_z=sz,
            distance=dist,
            track_id=t.id,
            status=_STATUS_MAP.get(t.status.value, "UNKNOWN"),
        ))
    return results


def filter_threats(detections: List[Detection], max_range: float = 3.0) -> List[Detection]:
    """Return only navigation-relevant objects within range, sorted nearest-first."""
    threats = [d for d in detections if d.is_nav_relevant and d.distance <= max_range]
    threats.sort(key=lambda d: d.distance)
    return threats