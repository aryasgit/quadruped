"""
perception.py — OAK-D PRO PERCEPTION PIPELINE
===============================================
Wraps the existing DepthAI pipeline (pipeline.py, detector.py) and
produces PerceptionFrame snapshots on a thread-safe queue.

Runs entirely in its own daemon thread. The main loop and dashboard
never touch DepthAI directly — they read from the output queue.

Architecture:
  1. Build DepthAI pipeline (stereo + MobileNet-SSD + tracker)
  2. Connect to OAK-D Pro device
  3. Main loop: pull frames, run detection post-processing,
     classify proximity zone, encode JPEG, push to queue
  4. Auto-reconnect with exponential backoff on USB disconnect

Dependencies:
  - depthai 2.24.0 (pinned — v3.x has breaking changes)
  - perception/pipeline.py (build_pipeline, configure_device)
  - perception/detector.py (process_tracklets, process_detections, filter_threats)

Queue contract:
  Output queue receives PerceptionFrame instances.
  Queue maxsize=2 — if consumer is slow, oldest frame is discarded.
"""

import time
import math
import threading
import queue
import traceback
import numpy as np

from state import (
    PerceptionFrame, DetectionSnapshot, ProximityState,
    classify_distance, BootMode,
    ZONE_STOP_M, ZONE_SLOW_M, ZONE_CAUTION_M,
)


# ── JPEG encoding ────────────────────────────────────────────────

def _encode_jpeg(frame, quality=65):
    """Encode a BGR numpy frame to JPEG bytes. Returns None on failure."""
    if frame is None:
        return None
    try:
        import cv2
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes() if ok else None
    except Exception:
        return None


def _colorize_depth(depth_frame):
    """Convert raw depth frame to a colorized BGR image for display."""
    if depth_frame is None:
        return None
    import cv2
    # Normalize to 0-255 range
    depth_vis = depth_frame.copy()
    depth_vis = np.clip(depth_vis, 0, 10000)
    depth_vis = (depth_vis / 10000.0 * 255).astype(np.uint8)
    return cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)


def _annotate_frame(frame, detections, zone, fps):
    """Draw bounding boxes, labels, and zone overlay on RGB frame."""
    if frame is None:
        return None
    import cv2
    h, w = frame.shape[:2]
    out = frame.copy()

    for det in detections:
        x1 = int(det.bbox[0] * w)
        y1 = int(det.bbox[1] * h)
        x2 = int(det.bbox[2] * w)
        y2 = int(det.bbox[3] * h)

        # Color by distance
        if det.distance_m < 0.40:
            color = (0, 0, 255)       # red
        elif det.distance_m < 0.60:
            color = (0, 80, 255)      # orange
        elif det.distance_m < 0.80:
            color = (0, 200, 255)     # yellow
        else:
            color = (0, 255, 120)     # green

        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)

        tid_str = f" T{det.track_id}" if det.track_id is not None else ""
        label_text = f"{det.label}{tid_str} {det.distance_m:.2f}m"
        cv2.putText(out, label_text, (x1, max(y1 - 6, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    # Zone banner
    if zone == ProximityState.STOP:
        cv2.rectangle(out, (0, 0), (w, 28), (0, 0, 180), -1)
        cv2.putText(out, "!! STOP !!", (w // 2 - 60, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    elif zone == ProximityState.SLOW:
        cv2.rectangle(out, (0, 0), (w, 28), (0, 80, 200), -1)
        cv2.putText(out, "SLOW", (w // 2 - 30, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

    # FPS
    cv2.putText(out, f"FPS: {fps:.1f}", (w - 100, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1, cv2.LINE_AA)

    return out


# ── Detection conversion ────────────────────────────────────────

def _to_snapshot(det) -> DetectionSnapshot:
    """Convert a detector.Detection to a DetectionSnapshot."""
    bearing = 0.0
    if det.spatial_z > 0:
        bearing = math.degrees(math.atan2(det.spatial_x, det.spatial_z))

    return DetectionSnapshot(
        label=det.label,
        confidence=det.confidence,
        distance_m=round(det.distance, 3),
        bearing_deg=round(bearing, 1),
        spatial_x=det.spatial_x,
        spatial_y=det.spatial_y,
        spatial_z=det.spatial_z,
        bbox=det.bbox,
        track_id=getattr(det, 'track_id', None),
        status=getattr(det, 'status', None),
    )


# ── NAV-RELEVANT LABELS ─────────────────────────────────────────
# Objects that affect obstacle avoidance decisions

NAV_LABELS = {
    "person", "bicycle", "car", "motorbike", "bus", "dog", "cat",
    "chair", "sofa", "diningtable", "cow", "horse", "vehicle",
    "bottle", "pottedplant", "tvmonitor",
}

THREAT_RANGE_M = 3.0


# ── Perception thread ───────────────────────────────────────────

class PerceptionThread:
    """
    Wraps the OAK-D pipeline in a daemon thread.
    Outputs PerceptionFrame objects on self.output_queue.
    """

    def __init__(self, output_queue: queue.Queue, nn_confidence: float = 0.4,
                 ir_flood: int = 200, ir_dot: int = 400):
        self.output_queue = output_queue
        self.nn_confidence = nn_confidence
        self.ir_flood = ir_flood
        self.ir_dot = ir_dot
        self._thread = None
        self._stop_event = threading.Event()

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True, name="perception")
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=5.0)

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _push(self, frame: PerceptionFrame):
        """Non-blocking put — discard oldest if queue is full."""
        try:
            self.output_queue.put_nowait(frame)
        except queue.Full:
            try:
                self.output_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.output_queue.put_nowait(frame)
            except queue.Full:
                pass

    def _run(self):
        """Main perception loop with auto-reconnect."""
        import depthai as dai
        from percept.pipeline import build_pipeline, configure_device
        from percept.detector import process_tracklets, process_detections, filter_threats

        backoff = 1.0
        max_backoff = 30.0

        while not self._stop_event.is_set():
            try:
                print("[PERCEPTION] Building pipeline...")
                pipeline = build_pipeline(
                    nn_confidence=self.nn_confidence,
                    ir_flood_mA=self.ir_flood,
                    ir_dot_mA=self.ir_dot,
                )

                print("[PERCEPTION] Connecting to OAK-D Pro...")
                with dai.Device(pipeline) as device:
                    configure_device(device,
                                     ir_flood_mA=self.ir_flood,
                                     ir_dot_mA=self.ir_dot)
                    print(f"[PERCEPTION] Device: {device.getMxId()}")
                    print(f"[PERCEPTION] USB: {device.getUsbSpeed().name}")

                    q_rgb   = device.getOutputQueue("rgb",        maxSize=1, blocking=False)
                    q_depth = device.getOutputQueue("depth",      maxSize=1, blocking=False)
                    q_track = device.getOutputQueue("tracklets",  maxSize=1, blocking=False)
                    q_det   = device.getOutputQueue("detections", maxSize=1, blocking=False)

                    fps_counter = 0
                    fps_timer = time.monotonic()
                    current_fps = 0.0
                    backoff = 1.0  # Reset on successful connect

                    print("[PERCEPTION] Running.")

                    while not self._stop_event.is_set():
                        t0 = time.monotonic()

                        # Blocking on RGB — paces loop at camera FPS
                        in_rgb = q_rgb.get()
                        rgb_frame = in_rgb.getCvFrame()

                        in_depth = q_depth.tryGet()
                        in_track = q_track.tryGet()
                        in_det   = q_det.tryGet()

                        depth_raw = in_depth.getFrame() if in_depth else None

                        # Detections — prefer tracker, fallback to raw NN
                        raw_detections = []
                        if in_track and in_track.tracklets:
                            raw_detections = process_tracklets(
                                in_track.tracklets, min_confidence=self.nn_confidence)
                        elif in_det and in_det.detections:
                            raw_detections = process_detections(
                                in_det.detections, min_confidence=self.nn_confidence)

                        threats = filter_threats(raw_detections, max_range=THREAT_RANGE_M)

                        # Convert to snapshots
                        det_snapshots = [_to_snapshot(d) for d in raw_detections]
                        threat_snapshots = [_to_snapshot(d) for d in threats]

                        # Nearest threat
                        nearest_dist = None
                        nearest_bearing = None
                        if threat_snapshots:
                            nearest = threat_snapshots[0]
                            nearest_dist = nearest.distance_m
                            nearest_bearing = nearest.bearing_deg

                        # Classify zone
                        zone = classify_distance(nearest_dist)

                        # FPS
                        fps_counter += 1
                        elapsed = time.monotonic() - fps_timer
                        if elapsed >= 1.0:
                            current_fps = fps_counter / elapsed
                            fps_counter = 0
                            fps_timer = time.monotonic()

                        loop_ms = (time.monotonic() - t0) * 1000.0

                        # Encode frames
                        annotated = _annotate_frame(rgb_frame, det_snapshots, zone, current_fps)
                        rgb_jpeg = _encode_jpeg(annotated, quality=65)
                        depth_colored = _colorize_depth(depth_raw)
                        depth_jpeg = _encode_jpeg(depth_colored, quality=50)

                        # Build and push frame
                        pf = PerceptionFrame(
                            timestamp=time.time(),
                            rgb_jpeg=rgb_jpeg,
                            depth_jpeg=depth_jpeg,
                            detections=det_snapshots,
                            nearest_distance_m=nearest_dist,
                            nearest_bearing_deg=nearest_bearing,
                            proximity_state=zone,
                            fps=current_fps,
                            loop_ms=loop_ms,
                        )
                        self._push(pf)

            except Exception as e:
                if self._stop_event.is_set():
                    break
                print(f"[PERCEPTION] Error: {e}")
                traceback.print_exc()
                print(f"[PERCEPTION] Reconnecting in {backoff:.0f}s...")
                self._stop_event.wait(timeout=backoff)
                backoff = min(backoff * 2, max_backoff)

        print("[PERCEPTION] Thread stopped.")


# ── Convenience: check OAK-D availability ────────────────────────

class PeripheralError(Exception):
    """Raised when a required peripheral is not detected."""
    pass


def check_oakd_available():
    """
    Probe for OAK-D Pro on USB. Raises PeripheralError if not found.
    Must be called BEFORE spawning any threads.
    """
    try:
        import depthai as dai
        devices = dai.Device.getAllAvailableDevices()
        if not devices:
            raise PeripheralError(
                "OAK-D Pro not detected. Check USB connection. "
                "Must use the USB3 (blue) port."
            )
        dev = devices[0]
        print(f"[HW CHECK] OAK-D found: {dev.getMxId()} ({dev.state.name})")
        return True
    except ImportError:
        raise PeripheralError(
            "depthai library not installed. Run: pip install depthai==2.24.0"
        )