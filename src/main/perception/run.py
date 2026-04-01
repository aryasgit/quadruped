#!/usr/bin/env python3
"""ARYA Perception — main loop."""

import time
import signal
import depthai as dai

from perception.pipeline import build_pipeline, configure_device
from perception.detector import process_tracklets, process_detections, filter_threats
from perception.telemetry import TelemetryState, start_server

ZONE_STOP    = 0.30
ZONE_CLOSE   = 1.0
ZONE_CAUTION = 2.0
THREAT_RANGE = 3.0


def main():
    NN_CONFIDENCE = 0.4
    TELEMETRY_PORT = 5000
    IR_FLOOD = 200
    IR_DOT = 0

    print("[PERCEPTION] Building pipeline...")
    pipeline = build_pipeline(nn_confidence=NN_CONFIDENCE, ir_flood_mA=IR_FLOOD, ir_dot_mA=IR_DOT)

    state = TelemetryState()
    start_server(state, port=TELEMETRY_PORT)

    running = True
    def _shutdown(sig, frame):
        nonlocal running
        print("\n[PERCEPTION] Shutting down...")
        running = False
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print("[PERCEPTION] Connecting to OAK-D Pro...")
    with dai.Device(pipeline) as device:
        configure_device(device, ir_flood_mA=IR_FLOOD, ir_dot_mA=IR_DOT)
        print(f"[PERCEPTION] Device: {device.getMxId()}")
        print(f"[PERCEPTION] USB speed: {device.getUsbSpeed().name}")

        q_rgb   = device.getOutputQueue("rgb",        maxSize=1, blocking=False)
        q_depth = device.getOutputQueue("depth",      maxSize=1, blocking=False)
        q_track = device.getOutputQueue("tracklets",  maxSize=1, blocking=False)
        q_det   = device.getOutputQueue("detections", maxSize=1, blocking=False)

        fps_counter = 0
        fps_timer = time.monotonic()
        current_fps = 0.0
        frame_count = 0
        stop_flag = False

        print(f"[PERCEPTION] Dashboard at http://10.79.106.152:{TELEMETRY_PORT}")

        while running:
            t0 = time.monotonic()

            # Blocking on RGB — paces loop at camera FPS
            in_rgb = q_rgb.get()
            rgb_frame = in_rgb.getCvFrame()

            in_depth = q_depth.tryGet()
            in_track = q_track.tryGet()
            in_det   = q_det.tryGet()

            depth_frame = in_depth.getFrame() if in_depth else None

            # Detections
            detections = []
            source = ""
            if in_track and in_track.tracklets:
                detections = process_tracklets(in_track.tracklets, min_confidence=NN_CONFIDENCE)
                source = "tracker"
            elif in_det and in_det.detections:
                detections = process_detections(in_det.detections, min_confidence=NN_CONFIDENCE)
                source = "raw_nn"

            threats = filter_threats(detections, max_range=THREAT_RANGE)

            # Stop flag
            prev_stop = stop_flag
            stop_flag = any(d.distance <= ZONE_STOP for d in threats)
            if stop_flag and not prev_stop:
                print(f"[!! STOP !!] Object within {ZONE_STOP}m")
            elif not stop_flag and prev_stop:
                print("[CLEAR] Resumed.")

            # Zone
            zone = "CLEAR"
            nearest_dist = None
            if threats:
                nearest_dist = threats[0].distance
                if nearest_dist <= ZONE_STOP:    zone = "STOP"
                elif nearest_dist <= ZONE_CLOSE: zone = "CLOSE"
                elif nearest_dist <= ZONE_CAUTION: zone = "CAUTION"

            # FPS
            fps_counter += 1
            frame_count += 1
            elapsed = time.monotonic() - fps_timer
            if elapsed >= 1.0:
                current_fps = fps_counter / elapsed
                fps_counter = 0
                fps_timer = time.monotonic()

            loop_ms = (time.monotonic() - t0) * 1000.0

            # Diagnostics
            if frame_count <= 120 and frame_count % 20 == 0:
                rn = len(in_det.detections) if in_det else 0
                tn = len(in_track.tracklets) if in_track else 0
                print(f"[DIAG] f={frame_count} depth={depth_frame is not None} "
                      f"raw={rn} track={tn} out={len(detections)} src={source or 'none'}")

            # Push to telemetry — explicit kwargs matching update() signature
            state.update(
                rgb=rgb_frame,
                depth=depth_frame,
                detections=detections,
                fps=current_fps,
                loop_ms=loop_ms,
                zone=zone,
                stop_flag=stop_flag,
                nearest_dist=nearest_dist,
            )

            # Console 1/sec
            if fps_counter == 1:
                if threats:
                    n = threats[0]
                    tid = f"T{n.track_id}" if n.track_id is not None else "raw"
                    print(f"[DET] {len(detections)} obj | {n.label} {tid} "
                          f"@ {n.distance:.2f}m {n.bearing_deg:+.0f}° | {zone}")
                else:
                    print(f"[LOOP] fps={current_fps:.1f} loop={loop_ms:.1f}ms")

    print("[PERCEPTION] Stopped.")


if __name__ == "__main__":
    main()