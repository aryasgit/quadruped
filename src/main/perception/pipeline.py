"""
OAK-D Pro DepthAI pipeline builder.
Configures: RGB camera, stereo depth, spatial MobileNet-SSD (Myriad X),
on-chip object tracker. All NN inference runs on-device.
"""

import depthai as dai
from pathlib import Path

MODEL_PATH = Path(__file__).parent / "models" / "mobilenet-ssd_openvino_2021.4_6shave.blob"

LABELS = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
    "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

NAV_LABELS = {"person", "bicycle", "car", "bus", "motorbike", "dog", "cat",
              "chair", "bottle", "sofa", "pottedplant"}


def build_pipeline(
    nn_confidence: float = 0.4,
    cam_fps: int = 30,
    ir_flood_mA: int = 0,
    ir_dot_mA: int = 0,
) -> dai.Pipeline:

    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"Blob not found at {MODEL_PATH}\n"
            "Run: wget -O perception/models/mobilenet-ssd_openvino_2021.4_6shave.blob "
            "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/"
            "network/mobilenet-ssd_openvino_2021.4_6shave.blob"
        )

    pipeline = dai.Pipeline()

    # ── RGB camera ──────────────────────────────────────────
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setPreviewSize(300, 300)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(cam_fps)

    # ── Mono cameras ────────────────────────────────────────
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_left.setFps(cam_fps)
    mono_right.setFps(cam_fps)

    # ── Stereo depth ────────────────────────────────────────
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.initialConfig.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_7x7)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    stereo.setExtendedDisparity(False)

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    # ── Spatial detection network (Myriad X) ────────────────
    spatial_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
    spatial_nn.setBlobPath(str(MODEL_PATH))
    spatial_nn.setConfidenceThreshold(nn_confidence)
    spatial_nn.input.setBlocking(False)
    spatial_nn.input.setQueueSize(1)
    spatial_nn.setBoundingBoxScaleFactor(0.5)
    spatial_nn.setDepthLowerThreshold(100)
    spatial_nn.setDepthUpperThreshold(10_000)

    cam_rgb.preview.link(spatial_nn.input)
    stereo.depth.link(spatial_nn.inputDepth)

    # ── On-chip object tracker ──────────────────────────────
    tracker = pipeline.create(dai.node.ObjectTracker)
    tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
    tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

    spatial_nn.passthrough.link(tracker.inputTrackerFrame)
    spatial_nn.passthrough.link(tracker.inputDetectionFrame)
    spatial_nn.out.link(tracker.inputDetections)

    # ── XLink outputs ───────────────────────────────────────
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    xout_rgb.input.setBlocking(False)
    xout_rgb.input.setQueueSize(1)
    cam_rgb.video.link(xout_rgb.input)

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    xout_depth.input.setBlocking(False)
    xout_depth.input.setQueueSize(1)
    stereo.disparity.link(xout_depth.input)

    xout_tracklets = pipeline.create(dai.node.XLinkOut)
    xout_tracklets.setStreamName("tracklets")
    xout_tracklets.input.setBlocking(False)
    xout_tracklets.input.setQueueSize(1)
    tracker.out.link(xout_tracklets.input)

    # Raw detections (fallback if tracker gives nothing)
    xout_det = pipeline.create(dai.node.XLinkOut)
    xout_det.setStreamName("detections")
    xout_det.input.setBlocking(False)
    xout_det.input.setQueueSize(1)
    spatial_nn.out.link(xout_det.input)

    return pipeline


def configure_device(device: dai.Device, ir_flood_mA: int = 0, ir_dot_mA: int = 0):
    """Apply device-level IR settings after pipeline start."""
    if ir_flood_mA > 0:
        device.setIrFloodLightBrightness(ir_flood_mA)
    if ir_dot_mA > 0:
        device.setIrLaserDotProjectorBrightness(ir_dot_mA)