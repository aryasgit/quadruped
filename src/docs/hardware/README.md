# Hardware Selection — Camera & Battery

> Research and selection rationale for the perception and power systems. All decisions documented here were made before purchasing. Update this file if hardware changes.

---


For camera and battery selection rationale and full specifications, see
[`docs/hardware/`](../../../../docs/hardware/).

## Contents

- [Camera System](#camera-system)
  - [Selected: Orbbec Gemini 336](#selected-orbbec-gemini-336)
  - [Secondary: Arducam IMX219 Fisheye](#secondary-arducam-imx219-fisheye-220)
  - [Camera Comparison](#camera-comparison)
  - [Mounting](#mounting)
- [Battery System](#battery-system)
  - [Selected: GenX Premium 5200 (Option A)](#selected-genx-premium-5200-option-a)
  - [Battery Comparison](#battery-comparison)
  - [Dual Battery Configuration](#dual-battery-configuration)
- [Integration Notes](#integration-notes)

---

## Camera System

### Selected: Orbbec Gemini 336

**Role:** Primary depth perception — terrain mapping, obstacle detection, foothold planning.

| Parameter | Value |
|---|---|
| Model | G40155-180 |
| Series | Gemini 330 |
| Technology | Active + Passive Stereo |
| ASIC | Orbbec MX6800 (on-board depth engine) |
| Depth Filter | IR-Pass (blocks visible + NIR < 800 nm; passes 850 nm) |
| Stereo Baseline | 50 mm |
| Dimensions | 90 × 25 × 30.7 mm |
| Weight | 99 g |
| IP Rating | IP5X (dust protected) |
| Mount | 1× ¼-20 UNC, 2× M3 side threads |
| Interface | USB 3.0 Type-C (data + power) |
| Power | <3 W avg / 6.5 W peak |

#### Depth Performance

| Resolution | Min-Z | Max fps | Notes |
|---|---|---|---|
| 1280×800 | 0.26 m | 30 fps | Full terrain map |
| 848×480 | 0.18 m | 60 fps | Navigation mode |
| 640×480 | 0.17 m | 60 fps | Standard |
| 848×100 (strip) | — | **100 fps** | Obstacle reflex loop |
| 424×240 | 0.10 m | 60 fps | Close-range |

Z-accuracy: ±2% at 2 m (1280×800, 81% ROI). Fill rate: >99%. Temporal noise: ≤0.9%.

#### Depth Modes in Use

**Navigation (default):** `848×480 @ 30fps` — balances density and USB bandwidth.

**Full terrain mapping:** `1280×800 @ 30fps` — when foothold planning needs maximum density.

**Obstacle reflex:** `848×100 @ 100fps` — 10 ms update cycle, forward ground strip. Feeds the reactive halt/step-over layer directly.

#### Why the 336 Over the 335

The only hardware difference between the two is the IR-Pass optical filter on the 336. This filter blocks visible and near-IR light below ~800 nm, passing only the 850 nm laser dot pattern used for stereo matching. The effect:

- Tiled floors and epoxy-coated corridors (common on campus) no longer cause depth dropout from specular glare
- HDR scenes (bright window + shadowed interior) are handled without visible-to-IR bleed
- Outdoor use under mixed lighting is more deterministic

All other specs — baseline, accuracy, FOV, power, SDK — are identical to the 335. The 336 costs approximately 10–15% more. Given the robot's primary operating environment, this is worth it.

#### Features Used

**MX6800 on-board ASIC:** All stereo matching runs on the camera chip. The Jetson Orin Nano receives ready-processed depth frames with no CPU stereo load.

**Active + Passive Stereo Fusion:** Active mode (850 nm projector on) for dark environments. Passive mode (ambient IR only) for close range and outdoor. The ASIC blends both automatically per frame.

**Global Shutter IR Sensors:** Both stereo sensors use global shutter — all pixels exposed simultaneously. Eliminates rolling-shutter jelly artefact during fast leg motion and servo vibration. Essential for a walking robot.

**6-DoF IMU at up to 1 kHz:** Hardware-timestamped with depth frames. Enables camera-IMU visual-inertial odometry (VIO). The IMU reading at the exact moment of each depth frame is available.

**D2C On-Board Alignment:** Depth-to-colour spatial alignment done on the MX6800. Registered RGBD output with no post-processing alignment required on the Jetson.

**HDR Depth Mode:** Dual-exposure blending for high-contrast scenes. Reduces black holes on near-reflective objects while retaining distant surface depth.

**OTA Firmware:** Updatable over USB via OrbbecSDK or OrbbecViewer. No hardware access required.

#### Known Limitations

- **50 mm baseline** degrades accuracy beyond ~5 m. Adequate for 0–5 m navigation. For long-range, the 336L (95 mm baseline) or LiDAR would complement.
- **Rolling shutter RGB** — colour sensor only. IR sensors are global shutter. Use IR-based VIO rather than RGB-based to avoid skew at walking speed.
- **90 mm width** is 20 mm over the 70 mm chassis bay. Requires external bracket mount (see [Mounting](#mounting)).
- **No on-device AI** — unlike OAK-D Lite, no neural accelerator. All inference on Jetson. Not a bottleneck given Jetson's 40 TOPS NVDLA.
- **Passive stereo degraded** in environments with no 850 nm ambient IR. Active projector is almost always on in practice, making this a rare edge case.
- **IP5X only** — dust protected but not waterproof. Not suitable for rain. The 336L carries IP65 for wet environments.

---

### Secondary: Arducam IMX219 Fisheye 220°

**Role:** Wide-angle RGB camera for teleoperation, object detection, visual odometry, terrain classification.

| Parameter | Value |
|---|---|
| Sensor | Sony IMX219 (1/4") |
| Resolution | 8MP (3280×2464 max) |
| FOV | 220° fisheye horizontal |
| Max FPS | 180fps @ 720p |
| Interface | CSI-2 (MIPI 2-lane) |
| Shutter | Rolling (electronic) |
| Driver | Arducam OOT |
| Platform | JetPack 6.1 · L4T R36.4.7 · nvcsi_t194 |

#### Pipeline Modes

**Teleoperation:** `nvarguscamerasrc → nvvidconv → jpegenc/h264enc → WebRTC/UDP` at 60fps @720p. 220° FOV gives full situational awareness in a single frame.

**Fisheye Undistortion:** `cv2.calibrateCamera()` with checkerboard → store K + D matrices → `cv2.undistort()` per frame → feed to CV pipeline. Must be done before any downstream CV work.

**Object Detection:** `nvarguscamerasrc → nvvidconv (CUDA, zero-copy) → Undistort (GPU) → TensorRT YOLOv8n inference` at ~30fps. Results published to ROS2 / ESP32.

**Visual Odometry:** Undistorted frames at 30fps → ORB feature extraction → VIO fusion with MPU6050 accel/gyro → 6-DOF pose estimation → `/odom` ROS2 topic. Uses ORB-SLAM3 + GY-87 fusion.

**Terrain Awareness:** Undistorted frame lower-third crop (ground zone) → MobileNetV2 classifier (4-class: flat/gravel/stairs/uneven, TensorRT) → confidence threshold >0.85 → UART to ESP32 → adaptive gait profile.

#### Setup Order (Arducam IMX219)

1. Driver install: `Arducam install_camera.sh`
2. Verify: `nvgstcapture-1.0 --sensor-id=0`
3. Calibrate lens: `cv2.calibrateCamera()` checkerboard
4. GStreamer pipeline: `nvarguscamerasrc → nvvidconv`
5. OpenCV undistort: `cv2.undistort()` per frame
6. TensorRT models: export YOLOv8n / MobileNetV2

---

### Camera Comparison

| Parameter | Gemini 336 (primary) | IMX219 Fisheye (secondary) |
|---|---|---|
| Role | Depth + RGBD + IMU | Wide RGB + VO + classification |
| Interface | USB 3.0 | CSI-2 |
| Depth | Yes (active stereo) | No |
| FOV | 90° × 65° depth | 220° fisheye |
| Shutter | Global (IR) / Rolling (RGB) | Rolling |
| On-device compute | MX6800 depth ASIC | None |
| IMU | 6-DoF, up to 1 kHz | None |
| Power | <3 W avg | ~0.5 W |
| Driver | OrbbecSDK v2 / ROS2 | Arducam OOT |
| Best for | Terrain depth, obstacle detection | Teleoperation, detection, VO |

---

### Mounting

The Gemini 336 is 90 mm wide — 20 mm over the 70 mm chassis bay. It cannot be mounted internally.

**Recommended approach:** External face bracket using the ¼-20 UNC thread (bottom centre of camera body) or the two M3 side threads. The camera body is only 30 mm deep, so forward protrusion is minimal.

**Tilt angle:** 10–15° downward from horizontal.

At 25 cm above ground and 10° downward tilt, the 65° vertical FOV covers from ~5 cm below the mounting plane to ~3 m forward on flat ground — sufficient for foothold planning and forward obstacle detection without wasting FOV on the sky.

---

## Battery System

### Selected: GenX Premium 5200 (Option A)

**Role:** Primary power for all robot systems — servos, Jetson Orin Nano, PCA9685, IMU, camera.

| Parameter | Value |
|---|---|
| Chemistry | LiPo |
| Cell config | 4S1P |
| Nominal voltage | 14.8 V |
| Capacity | 5200 mAh |
| Energy | 76.9 Wh |
| Continuous C-rate | **40C** |
| Max continuous current | **208 A** |
| Burst C-rate | 80C |
| Max burst current | **416 A** (10 s) |
| Discharge connector | XT-60 |
| Standard charge rate | 1–3C |
| Max charge rate | — |
| Cycle life | Higher than standard |
| Dimensions | 138 × 43 × 33 mm |
| Weight | 512 g |
| Chassis fit | ✓ Fits easily |
| Chassis volume used | 33% |
| Free space alongside | 54 mm wide strip |

#### Why Option A

The Gemini 336 report noted the robot operates on a 4S LiPo + 5V rail. Option A was selected over the three alternatives for the following reasons:

**Discharge current** is the most critical parameter for a quadruped. When the robot stumbles, changes gait abruptly, or all four legs load simultaneously, instantaneous current can spike. Option A's 208 A continuous / 416 A burst is the highest of all candidates and provides significant headroom. Option C's 92 A continuous is the weakest and risks brownout under simultaneous multi-leg load.

**Chassis fit** is excellent. At 138 × 43 × 33 mm it fits easily within the 150 × 97 × 48 mm chassis cavity using only 33% of the volume. This leaves a 54 mm wide × 138 mm long strip alongside the battery — wide enough for the ESP32, PCA9685, and buck converter in a single row.

**Weight** is the lightest of the fitting options at 512 g. Option B at 972 g and the dual-battery config at 1024 g add substantial payload.

**No BMS complexity.** Option B (Li-ion) includes a BMS but runs at 14.4 V nominal vs 14.8 V for the others. Most 4S electronics accept this, but it adds a variable to verify across every component. Option A stays at standard 4S LiPo voltage.

#### Runtime Estimates (single battery, 80% DoD)

| Load | Current | Runtime |
|---|---|---|
| Idle / standing | ~20 A | ~15.6 min |
| Slow walk | ~40 A | ~7.8 min |
| Normal gait | ~80 A | ~3.9 min |
| Aggressive / fast | ~160 A | ~1.95 min |

Real-world runtime will vary with gait, terrain, and motor load. These figures assume constant draw at 80% depth of discharge (industry standard for LiPo health).

---

### Battery Comparison

Full spec comparison across all evaluated candidates:

| Parameter | Old battery (ref) | **Option A ★** | Option B | Option C |
|---|---|---|---|---|
| Model | Pro-Range 6200 | GenX Premium 5200 | GenX Molicel 15000 | Unnamed 8400 |
| Chemistry | LiPo | LiPo | Li-ion | LiPo |
| Cell config | 4S1P | 4S1P | 4S3P | 4S |
| Nominal voltage | 14.8 V | 14.8 V | 14.4 V | 14.8 V |
| Capacity | 6200 mAh | 5200 mAh | 15000 mAh | 8400 mAh |
| Energy | 91.8 Wh | 76.9 Wh | 216 Wh | 124.3 Wh |
| Continuous C-rate | 35C | **40C** | 12C | 11C |
| Max continuous current | 110 A | **208 A** | 180 A | 92 A |
| Burst C-rate | 70C | **80C** | 15C | — |
| Max burst current | 220 A | **416 A** | 225 A | not rated |
| Dimensions (mm) | 157×50×30 | **138×43×33** | 142×80×39 | 145×45×45 |
| Weight | 630 g | **512 g** | 972 g | ~650 g |
| Chassis fit | ✗ No fit | ✓ Fits easily | ✓ Fits (rotated) | ⚠ Tight (3mm) |
| Chassis vol. used | — | **33%** | 63% | 42% |
| Free space alongside | — | **54 mm** | 17 mm | 52 mm |
| Height clearance | — | 15 mm | 9 mm | 3 mm ⚠ |
| BMS included | — | No | Yes | No |
| Recommendation | ✗ ref only | ★ Top pick | ✓ If runtime priority | ⚠ Verify dims first |

**Old battery:** 157 mm long — 7 mm over chassis. Does not fit. Reference only.

**Option B:** Fits rotated (80 mm width along the 97 mm axis). Maximum runtime at 216 Wh, but only 17 mm free alongside, 9 mm height clearance. Best if runtime is the top priority.

**Option C:** 3 mm height margin is a critical tolerance. Must physically verify before ordering. 11C continuous is the weakest of the fitting options.

---

### Dual Battery Configuration

Two Option A batteries can be run in parallel (same 14.8 V, XT-60 parallel harness):

| Parameter | Single | Dual |
|---|---|---|
| Capacity | 5200 mAh | 10400 mAh |
| Energy | 76.9 Wh | 153.8 Wh |
| Max continuous current | 208 A | 416 A |
| Max burst current | 416 A | 832 A |
| Internal resistance (est.) | ~6 mΩ | ~3 mΩ |
| Weight | 512 g | 1024 g |
| Chassis vol. used | 33% | 66% |
| Free space alongside | 54 mm | 31 mm |
| Runtime at 80 A (normal gait) | 3.9 min | 7.8 min |

**Dual configuration layout (rotated pair):** Rotating each battery (swap W and H) before placing side by side yields combined footprint 138 × 66 × 43 mm — fits within the 150 × 97 × 48 mm chassis. This leaves a 31 mm × 138 mm strip alongside, sufficient for the electronics stack in two rows. Height clearance drops to 5 mm — flat ribbon cables only, no tall capacitors.

**Key benefit of dual:** Internal resistance halves (~6 mΩ → ~3 mΩ), reducing voltage sag under heavy load. Single battery voltage sag at 160 A approaches the 11.1 V cutoff; dual battery holds ~0.7 V higher at the same current draw — more stable power delivery to motors.

**Trade-off:** Total weight doubles to 1024 g, chassis volume used doubles to 66%.

**Recommendation:** Start with single battery during development. Upgrade to dual if runtime or voltage sag under aggressive gait is observed.

---

## Integration Notes

### Power Budget

| Component | Avg draw |
|---|---|
| 12× MG996R servos (at normal gait) | ~40–80 A |
| Jetson Orin Nano | ~10–15 W via 5V rail |
| PCA9685 + logic | ~0.5 W |
| Gemini 336 | <3 W avg / 6.5 W peak |
| MPU6050 | ~3.6 mW |
| OLED display | ~30 mW |

The Gemini 336's 6.5 W peak during laser burst should be decoupled from the main power rail using a bulk capacitor on the USB 5V line to prevent transient voltage dips reaching the servos.

### USB Bandwidth

Simultaneous Gemini 336 depth (1280×800 Y16) + RGB (1080p MJPEG) uses approximately 300–500 Mbps. The Jetson Orin Nano's USB 3.2 Gen 1 ports run at 5 Gbps — sufficient headroom. The 848×100 @ 100fps strip mode can run in parallel on a second USB connection for the obstacle reflex loop.

### ROS 2 Integration

```bash
# Gemini 336
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git src/orbbec_camera
colcon build --packages-select orbbec_camera
ros2 launch orbbec_camera gemini_336.launch.py

# Key topics:
# /camera/depth/image_raw       — Y16 depth
# /camera/color/image_raw       — RGB
# /camera/depth/points          — PointCloud2 (D2C aligned)
# /camera/imu                   — sensor_msgs/Imu
# /camera/ir/image_raw          — IR mono
```

Verify detection first:
```bash
lsusb | grep 2BC5   # should show VID 0x2BC5, PID 0x0803
```