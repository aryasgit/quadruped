"""
state.py — SHARED STATE DEFINITIONS
=====================================
All inter-module data contracts live here.
These dataclasses are passed through queue.Queue instances.
No module imports another module's internals — only these contracts.

Thread safety: These are immutable snapshots. Each producer creates
a new instance per tick and puts it on the queue. Consumers read
the latest snapshot. No locks needed on the data itself.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import Enum, auto
import time


# ── Hardware boot mode ───────────────────────────────────────────

class BootMode(Enum):
    CAMERA_ONLY = 1
    ROBOT_ONLY  = 2
    FULL        = 3


# ── Robot proximity state machine ────────────────────────────────

class ProximityState(Enum):
    SAFE    = auto()   # > 80 cm — full speed
    CAUTION = auto()   # 60–80 cm — reduced speed
    SLOW    = auto()   # 40–60 cm — reduced speed
    STOP    = auto()   # < 40 cm — halt forward motion


# ── Control authority ────────────────────────────────────────────

class ControlMode(Enum):
    MANUAL     = auto()   # Xbox controller has full authority
    AUTO_AVOID = auto()   # Autonomous avoidance in progress


# ── Detection snapshot (from perception) ─────────────────────────

@dataclass(frozen=True)
class DetectionSnapshot:
    label: str
    confidence: float
    distance_m: float          # euclidean distance in meters
    bearing_deg: float         # horizontal angle from centerline (+ = right, - = left)
    spatial_x: float           # meters, right = +
    spatial_y: float           # meters, down = +
    spatial_z: float           # meters, forward = +
    bbox: Tuple[float, float, float, float]  # normalized (x1, y1, x2, y2)
    track_id: Optional[int] = None
    status: Optional[str] = None


# ── Perception output (one per frame) ────────────────────────────

@dataclass
class PerceptionFrame:
    timestamp: float
    rgb_jpeg: Optional[bytes] = None          # JPEG-encoded RGB frame with overlays
    depth_jpeg: Optional[bytes] = None        # JPEG-encoded colorized depth
    detections: List[DetectionSnapshot] = field(default_factory=list)
    nearest_distance_m: Optional[float] = None
    nearest_bearing_deg: Optional[float] = None
    proximity_state: ProximityState = ProximityState.SAFE
    fps: float = 0.0
    loop_ms: float = 0.0


# ── IMU snapshot ─────────────────────────────────────────────────

@dataclass(frozen=True)
class IMUSnapshot:
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    timestamp: float = 0.0


# ── Step/gait state ─────────────────────────────────────────────

@dataclass(frozen=True)
class StepState:
    is_stepping: bool = False
    active_leg: str = ""          # "FL", "FR", "RL", "RR" or ""
    phase: float = 0.0            # 0.0 → 1.0 within current step
    direction: str = ""           # "forward", "backward", "turn_left", etc.
    cycle_progress: float = 0.0   # 0.0 → 1.0 within full gait cycle


# ── Robot state (published by controller) ────────────────────────

@dataclass
class RobotState:
    timestamp: float = 0.0
    proximity: ProximityState = ProximityState.SAFE
    control_mode: ControlMode = ControlMode.MANUAL
    imu: IMUSnapshot = field(default_factory=IMUSnapshot)
    step: StepState = field(default_factory=StepState)
    height_mode: str = "NORMAL"
    stance_z: float = -0.18
    last_command: str = "idle"


# ── Dashboard aggregate (everything the dashboard needs) ─────────

@dataclass
class DashboardData:
    """
    Single object containing everything the dashboard renders.
    Built by main.py from perception + robot state queues.
    """
    # Perception
    rgb_jpeg: Optional[bytes] = None
    depth_jpeg: Optional[bytes] = None
    detections: List[DetectionSnapshot] = field(default_factory=list)
    nearest_distance_m: Optional[float] = None
    nearest_bearing_deg: Optional[float] = None
    proximity_state: ProximityState = ProximityState.SAFE
    perception_fps: float = 0.0

    # Robot
    control_mode: ControlMode = ControlMode.MANUAL
    imu: IMUSnapshot = field(default_factory=IMUSnapshot)
    step: StepState = field(default_factory=StepState)
    height_mode: str = "NORMAL"
    last_command: str = "idle"

    # Meta
    timestamp: float = 0.0
    boot_mode: BootMode = BootMode.FULL
    uptime_s: float = 0.0


# ── Zone threshold constants (meters) ───────────────────────────
# These are the authoritative thresholds. perception.py and
# controller.py both import from here.

ZONE_STOP_M    = 0.40    # < 40 cm  → STOP
ZONE_SLOW_M    = 0.60    # 40–60 cm → SLOW
ZONE_CAUTION_M = 0.80    # 60–80 cm → CAUTION
ZONE_SAFE_M    = 0.80    # > 80 cm  → SAFE


def classify_distance(distance_m: Optional[float]) -> ProximityState:
    """Map a distance reading to a ProximityState."""
    if distance_m is None:
        return ProximityState.SAFE
    if distance_m < ZONE_STOP_M:
        return ProximityState.STOP
    if distance_m < ZONE_SLOW_M:
        return ProximityState.SLOW
    if distance_m < ZONE_CAUTION_M:
        return ProximityState.CAUTION
    return ProximityState.SAFE