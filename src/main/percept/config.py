"""
Centralized perception configuration.
All tunable parameters in one place — no magic numbers scattered across files.
"""

from dataclasses import dataclass, field
from typing import Dict


@dataclass
class StereoConfig:
    """Stereo depth engine parameters."""
    mono_resolution: str = "THE_720_P"       # THE_400_P | THE_720_P | THE_800_P
    median_filter: str = "KERNEL_7x7"        # MEDIAN_OFF | KERNEL_3x3 | KERNEL_5x5 | KERNEL_7x7
    left_right_check: bool = True
    subpixel: bool = True
    extended_disparity: bool = True           # Better close-range (<70cm) at cost of max range
    lr_check_threshold: int = 5              # 0-10, lower = stricter (fewer holes but more noise rejection)
    confidence_threshold: int = 200           # 0-255, higher = stricter depth confidence
    depth_align_to_rgb: bool = True
    min_depth_mm: int = 100
    max_depth_mm: int = 10_000


@dataclass
class IRConfig:
    """OAK-D Pro IR illuminator settings."""
    flood_mA: int = 0          # 0-1500mA, flood illuminator (helps in low light)
    dot_mA: int = 0            # 0-1200mA, dot projector (helps stereo on textureless surfaces)


@dataclass
class NNConfig:
    """Neural network detection parameters."""
    confidence_threshold: float = 0.4
    blob_name: str = "mobilenet-ssd_openvino_2021.4_6shave.blob"
    input_size: tuple = (300, 300)
    bbox_scale_factor: float = 0.5     # Depth ROI within bounding box (0.2-1.0)
    labels_to_track: list = None       # None = track all, or list of label indices


@dataclass
class ZoneConfig:
    """Proximity zone thresholds in meters."""
    stop: float = 0.30
    close: float = 1.0
    caution: float = 2.0
    max_range: float = 5.0


@dataclass
class StreamConfig:
    """Telemetry stream parameters."""
    width: int = 640
    height: int = 360
    jpeg_quality: int = 60
    interval: float = 0.05      # seconds between MJPEG frames
    port: int = 5000


@dataclass
class PerceptionConfig:
    stereo: StereoConfig = field(default_factory=StereoConfig)
    ir: IRConfig = field(default_factory=IRConfig)
    nn: NNConfig = field(default_factory=NNConfig)
    zones: ZoneConfig = field(default_factory=ZoneConfig)
    stream: StreamConfig = field(default_factory=StreamConfig)
    cam_fps: int = 30


# ── Environment presets ─────────────────────────────────────

def indoor_bright() -> PerceptionConfig:
    """Well-lit indoor: no IR needed, standard stereo."""
    cfg = PerceptionConfig()
    cfg.ir = IRConfig(flood_mA=0, dot_mA=300)
    cfg.stereo.confidence_threshold = 200
    cfg.stereo.extended_disparity = True
    return cfg


def indoor_dark() -> PerceptionConfig:
    """Low-light indoor: max IR flood + dot projector."""
    cfg = PerceptionConfig()
    cfg.ir = IRConfig(flood_mA=1000, dot_mA=800)
    cfg.stereo.confidence_threshold = 180
    cfg.stereo.extended_disparity = True
    cfg.nn.confidence_threshold = 0.35
    return cfg


def outdoor() -> PerceptionConfig:
    """Outdoor daylight: IR off (sunlight washes it out), tighter stereo."""
    cfg = PerceptionConfig()
    cfg.ir = IRConfig(flood_mA=0, dot_mA=0)
    cfg.stereo.confidence_threshold = 220
    cfg.stereo.extended_disparity = False   # Prefer max range outdoors
    cfg.zones.max_range = 8.0
    return cfg


def mixed() -> PerceptionConfig:
    """Mixed indoor/outdoor: moderate IR, balanced parameters."""
    cfg = PerceptionConfig()
    cfg.ir = IRConfig(flood_mA=150, dot_mA=400)
    cfg.stereo.mono_resolution = "THE_720_P"
    cfg.stereo.confidence_threshold = 200
    cfg.stereo.lr_check_threshold = 5
    cfg.stereo.extended_disparity = True
    cfg.stereo.subpixel = True
    cfg.nn.confidence_threshold = 0.4
    cfg.nn.bbox_scale_factor = 0.4
    cfg.zones.max_range = 6.0
    return cfg


PRESETS: Dict[str, callable] = {
    "indoor_bright": indoor_bright,
    "indoor_dark": indoor_dark,
    "outdoor": outdoor,
    "mixed": mixed,
}