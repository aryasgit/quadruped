# Camera Technical Reference

> Complete specification reference for both cameras on the robot. Condensed from manufacturer datasheets and lab evaluation reports. Last updated: 2026.

---

## Orbbec Gemini 336 — Depth Camera

**USB VID/PID:** `0x2BC5 / 0x0803`

### Full Specification Matrix

#### Physical

| Parameter | Value |
|---|---|
| Dimensions | 90 × 25 × 30.7 mm |
| Weight | 99 g |
| Stereo Baseline | 50 mm |
| Mount thread (bottom) | ¼-20 UNC, max torque 4.0 N·m |
| Mount threads (side) | 2× M3, max torque 0.4 N·m |
| IP Rating | IP5X |
| Operating temp | -10°C to 45°C (≤15 fps) / -10°C to 40°C (30/60 fps) |
| Humidity | 5%–90% RH, non-condensing |

#### Depth Stream — All Modes

| Resolution | Min-Z | Max fps | Notes |
|---|---|---|---|
| 1280×800 | 0.26 m | 30 | Full res, 16:10, H90°×V65° |
| 1280×720 | — | 30 | 16:9, H90°×V60° |
| 848×480 | 0.18 m | 60 | Navigation default |
| 640×480 | 0.17 m | 60 | 4:3, H81°×V65° |
| 640×360 | — | 60 | |
| 480×270 | 0.11 m | 60 | |
| 424×240 | 0.10 m | 60 | |
| **848×100 (strip)** | — | **100** | H90°×V14°, obstacle reflex |
| 640×400 (Y16) | — | 30 | Raw 16-bit depth |

Depth format: Y16 (16-bit unsigned, millimetres).

#### Accuracy (at 1280×800, 2 m, 81% ROI)

| Metric | Value |
|---|---|
| Z-accuracy | ±2% (absolute median error) |
| Spatial precision | ≤1.5% (plane-fit RMS) |
| Temporal noise | ≤0.9% (per-pixel STD) |
| Fill rate | >99% |
| Optimal range | 0.26–3 m |
| Total range | 0.10–20 m+ |

#### IR Stream

| Resolution | Max fps | Notes |
|---|---|---|
| 1280×800 | 30 | H94°×V68° |
| 848×480 | 60 | H91°×V60° |
| 640×480 | 60 | H81°×V65° |
| **848×100** | **100** | H91°×V14° scan line |

Format: Y8 (8-bit mono). **Global shutter** — zero motion blur.

#### RGB Stream

| Resolution | Max fps | Shutter |
|---|---|---|
| 1920×1080 | 30 | Rolling |
| 1280×720 | 60 | Rolling |
| 848×480 | 60 | Rolling |
| 640×480 | 60 | Rolling |
| 320×180 | 60 | Rolling |

Formats: MJPEG, YUYV. D2C alignment on-board.

#### IMU

| Parameter | Value |
|---|---|
| DoF | 6 (3-axis gyro + 3-axis accel) |
| Output rate | 50–1000 Hz (configurable) |
| Timestamp | Hardware-synchronised with depth and colour |

#### Electrical

| Parameter | Value |
|---|---|
| Interface | USB 3.0 Type-C (data + power) |
| USB 2.0 support | Yes (lower res/fps only) |
| Average power | <3.0 W |
| Peak power | 6.5 W |
| Supply | 5 V via USB |
| Laser class | Class 1 (eye-safe); shuts down ≥73°C NTC |

#### SDK & Software

| Item | Detail |
|---|---|
| SDK | OrbbecSDK v2 (C/C++/Python) |
| ROS 2 | `orbbec_camera` (Humble, Iron, Jazzy) |
| Host platforms | Linux Ubuntu 18/20/22/24, Windows 10/11, macOS 12+ |
| Architectures | x86_64, ARM64 (Jetson Orin Nano native) |
| D2C alignment | On-board hardware + SDK software mode |
| HDR depth | Supported (dual exposure blending) |
| OTA firmware | Supported via SDK/OrbbecViewer |
| Multi-camera sync | Master/slave via Sync Hub accessories |
| Frame trigger modes | Free-run, specific-rate, hardware trigger |

---

## Arducam IMX219 Fisheye 220° — RGB Camera

### Full Specification Matrix

| Parameter | Value |
|---|---|
| Sensor | Sony IMX219, 1/4" CMOS |
| Resolution | 8MP (3280×2464 max) |
| FOV | 220° fisheye (horizontal) |
| Max fps | 180fps @ 720p |
| Interface | CSI-2 (MIPI 2-lane) |
| Shutter | Rolling (electronic) |
| Driver | Arducam OOT (out-of-tree kernel module) |
| JetPack | 6.1 (L4T R36.4.7) |
| CSI driver | nvcsi_t194 |

### GStreamer Pipeline (teleoperation)

```bash
gst-launch-1.0 \
  nvarguscamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,framerate=60/1' ! \
  nvvidconv ! \
  'video/x-raw(memory:NVMM),format=NV12' ! \
  h264enc ! \
  rtph264pay ! \
  udpsink host=<controller-ip> port=5000
```

### Lens Calibration

The 220° fisheye causes heavy barrel distortion. Must calibrate before any CV work.

```python
import cv2
import numpy as np

# Collect checkerboard images (minimum 20 poses)
# Then:
ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints,
    (width, height), None, None
)
# Save K (camera matrix) and D (distortion coefficients)
np.save('camera_matrix.npy', K)
np.save('dist_coeffs.npy', D)

# Per-frame undistortion:
undistorted = cv2.undistort(frame, K, D)
```

### Pipeline Modes Summary

| Mode | Input | Key stages | Output | Tools |
|---|---|---|---|---|
| Teleoperation | CSI raw Bayer | nvarguscamerasrc → nvvidconv → h264enc | WebRTC/UDP stream | GStreamer, WebRTC |
| Undistortion | Raw frame | calibrateCamera → K+D → undistort | Rectilinear frame | OpenCV |
| Object detection | Undistorted frame | nvvidconv CUDA → cv2.cuda remap → TensorRT | Bounding boxes | YOLOv8n TensorRT |
| Visual odometry | 30fps undistorted | ORB features → VIO fusion | 6-DoF pose /odom | ORB-SLAM3 |
| Terrain awareness | Lower-third crop | MobileNetV2 TensorRT → threshold → UART | Gait profile cmd | MobileNetV2, ESP32 |

### Software Install Order

```bash
# 1. Install Arducam driver
chmod +x install_camera.sh && ./install_camera.sh

# 2. Verify
nvgstcapture-1.0 --sensor-id=0

# 3. Calibrate (run once per lens, save K+D)
python calibrate_fisheye.py

# 4. Build GStreamer pipeline
# (see pipeline template above)

# 5. Export TensorRT models
python export_yolov8n_trt.py
python export_mobilenetv2_trt.py
```