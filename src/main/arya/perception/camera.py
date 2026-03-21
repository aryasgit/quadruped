# arya/perception/camera.py
"""
Layer 1 — OAK-D CAMERA PIPELINE
=================================
Wraps DepthAI pipeline:
  - Stereo depth (30Hz, 400p)
  - MobileNet-SSD detection (20-class COCO)
  - Pedestrian-and-vehicle detector (ADAS)
  - RGB frame at 640×480 for telemetry

Outputs per frame:
  - List[Detection]  (from tracker.py contract)
  - numpy RGB frame  (annotated)

Usage:
    cam = OakDCamera()
    cam.start()
    while True:
        frame, detections = cam.get()
    cam.stop()
"""

import time
import threading
import numpy as np
from pathlib import Path
from typing import List, Optional, Tuple

import depthai as dai
import blobconverter

from arya.perception.tracker import Detection


# ── Model config ─────────────────────────────────────────────────

MODEL_MOBILENET = "mobilenet-ssd"
MODEL_ADAS      = "pedestrian-and-vehicle-detector-adas-0001"
SHAVES          = 6
OPENVINO_VER    = "2022.1"

# Depth zone thresholds (meters)
DEPTH_MIN_MM = 100
DEPTH_MAX_MM = 6000
DEPTH_PERCENTILE = 10   # use 10th percentile for robustness

# Detection thresholds
CONF_THRESHOLD = 0.45

# MobileNet COCO labels (index → string)
MOBILENET_LABELS = [
    "background","aeroplane","bicycle","bird","boat","bottle","bus","car",
    "cat","chair","cow","diningtable","dog","horse","motorbike","person",
    "pottedplant","sheep","sofa","train","tvmonitor"
]

ADAS_LABELS = ["background", "person", "vehicle"]


def _get_blob(model_name: str) -> str:
    return blobconverter.from_zoo(
        name       = model_name,
        shaves     = SHAVES,
        version    = OPENVINO_VER,
        zoo_type   = "depthai",
    )


# ── Camera pipeline ───────────────────────────────────────────────

class OakDCamera:
    """
    Thread-safe OAK-D perception source.
    Runs DepthAI in a background thread.
    get() returns latest (frame, detections) — non-blocking.
    """

    def __init__(self, verbose: bool = False):
        self.verbose = verbose

        self._frame:      Optional[np.ndarray] = None
        self._detections: List[Detection]      = []
        self._lock   = threading.Lock()
        self._stop   = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._pipeline = self._build_pipeline()

    # ── DepthAI pipeline definition ───────────────────────────────

    def _build_pipeline(self) -> dai.Pipeline:
        p = dai.Pipeline()

        # ---- Mono cams for stereo ----
        left  = p.create(dai.node.MonoCamera)
        right = p.create(dai.node.MonoCamera)
        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        left.setFps(30)
        right.setFps(30)

        # ---- Stereo depth ----
        stereo = p.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        stereo.setExtendedDisparity(False)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        left.out.link(stereo.left)
        right.out.link(stereo.right)

        depth_out = p.create(dai.node.XLinkOut)
        depth_out.setStreamName("depth")
        stereo.depth.link(depth_out.input)

        # ---- RGB camera ----
        cam_rgb = p.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setFps(30)

        # ---- MobileNet-SSD — needs 300x300 ----
        manip_mob = p.create(dai.node.ImageManip)
        manip_mob.initialConfig.setResize(300, 300)
        manip_mob.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        cam_rgb.preview.link(manip_mob.inputImage)

        nn_mob = p.create(dai.node.MobileNetDetectionNetwork)
        nn_mob.setBlobPath(_get_blob(MODEL_MOBILENET))
        nn_mob.setConfidenceThreshold(CONF_THRESHOLD)
        nn_mob.setNumInferenceThreads(2)
        manip_mob.out.link(nn_mob.input)

        mob_out = p.create(dai.node.XLinkOut)
        mob_out.setStreamName("mob_det")
        nn_mob.out.link(mob_out.input)

        # ---- ADAS detector — needs 672x384 ----
        manip_adas = p.create(dai.node.ImageManip)
        manip_adas.initialConfig.setResize(672, 384)
        manip_adas.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        cam_rgb.preview.link(manip_adas.inputImage)

        nn_adas = p.create(dai.node.MobileNetDetectionNetwork)
        nn_adas.setBlobPath(_get_blob(MODEL_ADAS))
        nn_adas.setConfidenceThreshold(CONF_THRESHOLD)
        nn_adas.setNumInferenceThreads(2)
        manip_adas.out.link(nn_adas.input)

        adas_out = p.create(dai.node.XLinkOut)
        adas_out.setStreamName("adas_det")
        nn_adas.out.link(adas_out.input)

        # ---- RGB passthrough for telemetry (original 640x480) ----
        rgb_out = p.create(dai.node.XLinkOut)
        rgb_out.setStreamName("rgb")
        cam_rgb.preview.link(rgb_out.input)

        return p

    # ── Background thread ─────────────────────────────────────────

    def _run(self):
        with dai.Device(self._pipeline) as device:
            q_depth = device.getOutputQueue("depth",  maxSize=1, blocking=False)
            q_mob   = device.getOutputQueue("mob_det",  maxSize=1, blocking=False)
            q_adas  = device.getOutputQueue("adas_det", maxSize=1, blocking=False)
            q_rgb   = device.getOutputQueue("rgb",    maxSize=1, blocking=False)

            while not self._stop.is_set():
                try:
                    depth_msg = q_depth.get()
                    mob_msg   = q_mob.tryGet()
                    adas_msg  = q_adas.tryGet()
                    rgb_msg   = q_rgb.tryGet()

                    depth_arr = depth_msg.getFrame()   # uint16, mm
                    frame     = None

                    if rgb_msg is not None:
                        frame = rgb_msg.getCvFrame()   # BGR uint8

                    dets = self._fuse_detections(
                        depth_arr,
                        mob_msg,
                        adas_msg,
                        frame,
                    )

                    with self._lock:
                        self._frame      = frame
                        self._detections = dets

                except Exception as e:
                    if self.verbose:
                        print(f"[camera] error: {e}")
                    time.sleep(0.01)

    # ── Detection fusion ──────────────────────────────────────────

    def _fuse_detections(
        self,
        depth: np.ndarray,
        mob_msg,
        adas_msg,
        frame: Optional[np.ndarray],
    ) -> List[Detection]:
        """
        Merge both detector outputs; assign depth from stereo.
        Dedup by IoU threshold to avoid double-counting overlapping boxes.
        """
        raw = []

        if mob_msg is not None:
            for d in mob_msg.detections:
                label = MOBILENET_LABELS[d.label] if d.label < len(MOBILENET_LABELS) else "unknown"
                raw.append((d, label))

        # NEW — only take person from ADAS, skip vehicle (too noisy at close range)
        if adas_msg is not None:
            for d in adas_msg.detections:
                label = ADAS_LABELS[d.label] if d.label < len(ADAS_LABELS) else "unknown"
                if label == "person":          # vehicle from ADAS is unreliable indoors
                    raw.append((d, label))

        H, W = depth.shape[:2]
        FW, FH = 640, 480   # preview frame size

        out: List[Detection] = []
        for d, label in raw:
            cx_norm = (d.xmin + d.xmax) / 2
            cy_norm = (d.ymin + d.ymax) / 2
            cx_px   = int(cx_norm * FW)
            cy_px   = int(cy_norm * FH)

            w_px = int((d.xmax - d.xmin) * FW)
            h_px = int((d.ymax - d.ymin) * FH)

            # Map pixel to depth-frame coordinate
            # NEW — correct mapping with bounds
            dx = int(cx_norm * W)
            dy = int(cy_norm * H)
            dx = max(0, min(dx, W-1))
            dy = max(0, min(dy, H-1))

            # ROI for more robust depth estimate
            pad = 8
            roi = depth[max(0,dy-pad):dy+pad, max(0,dx-pad):dx+pad]
            valid = roi[(roi > DEPTH_MIN_MM) & (roi < DEPTH_MAX_MM)]
            if valid.size == 0:
                continue
            z_mm = float(np.percentile(valid, DEPTH_PERCENTILE))
            z_m  = z_mm / 1000.0

            out.append(Detection(
                x     = float(cx_px),
                y     = float(cy_px),
                z_m   = z_m,
                w     = float(w_px),
                h     = float(h_px),
                label = label,
                conf  = float(d.confidence),
            ))

            # Annotate frame in-place
            if frame is not None:
                self._draw_box(frame, d, label, z_m, FW, FH)

        return self._dedup(out)

    # ── IoU dedup ─────────────────────────────────────────────────

    @staticmethod
    def _dedup(dets: List[Detection], iou_thresh: float = 0.50) -> List[Detection]:
        """Remove duplicate detections from overlapping models."""
        kept = []
        for d in sorted(dets, key=lambda x: -x.conf):
            overlap = False
            for k in kept:
                # Simple AABB IoU
                ax1,ay1 = d.x-d.w/2, d.y-d.h/2
                ax2,ay2 = d.x+d.w/2, d.y+d.h/2
                bx1,by1 = k.x-k.w/2, k.y-k.h/2
                bx2,by2 = k.x+k.w/2, k.y+k.h/2
                ix = max(0, min(ax2,bx2)-max(ax1,bx1))
                iy = max(0, min(ay2,by2)-max(ay1,by1))
                inter = ix*iy
                union = d.w*d.h + k.w*k.h - inter
                if union > 0 and inter/union > iou_thresh:
                    overlap = True
                    break
            if not overlap:
                kept.append(d)
        return kept

    # ── Annotation ────────────────────────────────────────────────

    @staticmethod
    def _draw_box(frame, d, label, z_m, fw, fh):
        import cv2
        x1 = int(d.xmin * fw)
        y1 = int(d.ymin * fh)
        x2 = int(d.xmax * fw)
        y2 = int(d.ymax * fh)
        color = (0,0,200) if label == "person" else (0,180,0)
        cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
        cv2.putText(frame, f"{label} {z_m:.1f}m",
                    (x1, max(y1-6,12)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, color, 1, cv2.LINE_AA)

    # ── Public API ────────────────────────────────────────────────

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def get(self) -> Tuple[Optional[np.ndarray], List[Detection]]:
        """Non-blocking read of latest frame + detections."""
        with self._lock:
            return self._frame, list(self._detections)

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()