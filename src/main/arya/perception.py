# perception.py  —  depthai 2.24.0
# Dual-model detection + depth + Flask telemetry + MJPEG feed
import depthai as dai
import numpy as np
import threading
import time
import json
import cv2
from flask import Flask, Response, render_template_string
import blobconverter

# -------------------------------------------------------
# Tuning
# -------------------------------------------------------
ZONE_STOP             = 0.35
ZONE_CAUTION          = 0.80
CENTER_STRIP_FRACTION = 0.50
MIN_VALID_PIXELS      = 20
DEPTH_MIN_MM          = 100
DEPTH_MAX_MM          = 6000
DEPTH_PERCENTILE      = 10
PUBLISH_INTERVAL      = 0.5
VERBOSE               = True
TELEMETRY_PORT        = 5000

# MobileNet-SSD labels
MOBILENET_LABELS = [
    "background","aeroplane","bicycle","bird","boat","bottle","bus","car",
    "cat","chair","cow","diningtable","dog","horse","motorbike","person",
    "pottedplant","sheep","sofa","train","tvmonitor"
]

# Pedestrian+vehicle detector labels
PEDVEH_LABELS = {1: "person", 2: "vehicle"}

STOP_OBJECTS    = {"person"}
CAUTION_OBJECTS = {"vehicle","bicycle","motorbike","dog","cat","chair",
                   "sofa","diningtable","cow","horse"}

MOBILENET_CONF  = 0.55
PEDVEH_CONF     = 0.50
PERSON_CONF     = 0.55

# -------------------------------------------------------
# Shared state
# -------------------------------------------------------
_STATE = {
    "zone":       "free",
    "distance_m": 99.0,
    "detections": [],
    "robot_cmd":  "idle",
    "updated_at": 0.0,
}
_STATE_LOCK   = threading.Lock()
_LATEST_FRAME = None
_FRAME_LOCK   = threading.Lock()


def get_obstacle_state() -> dict:
    with _STATE_LOCK:
        return _STATE.copy()

def _write_depth(zone, distance_m):
    with _STATE_LOCK:
        _STATE["zone"]       = zone
        _STATE["distance_m"] = distance_m
        _STATE["updated_at"] = time.monotonic()

def _write_detections(detections):
    with _STATE_LOCK:
        _STATE["detections"] = detections

def set_robot_cmd(cmd: str):
    with _STATE_LOCK:
        _STATE["robot_cmd"] = cmd

def _write_frame(frame):
    global _LATEST_FRAME
    with _FRAME_LOCK:
        _LATEST_FRAME = frame.copy()

def _get_frame():
    with _FRAME_LOCK:
        return _LATEST_FRAME


# -------------------------------------------------------
# Pipeline
# -------------------------------------------------------
def _build_pipeline() -> dai.Pipeline:
    pipeline = dai.Pipeline()

    # --- Mono cameras ---
    cam_left = pipeline.create(dai.node.MonoCamera)
    cam_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    cam_left.setResolution(
        dai.MonoCameraProperties.SensorResolution.THE_400_P)

    cam_right = pipeline.create(dai.node.MonoCamera)
    cam_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    cam_right.setResolution(
        dai.MonoCameraProperties.SensorResolution.THE_400_P)

    # --- Stereo depth ---
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(
        dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    stereo.setExtendedDisparity(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    cam_left.out.link(stereo.left)
    cam_right.out.link(stereo.right)

    # --- RGB camera ---
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setPreviewSize(300, 300)   # detection input
    cam_rgb.setVideoSize(640, 480)     # display stream
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(12)

    # --- MobileNet-SSD (general 20-class) ---
    mobilenet = pipeline.create(dai.node.MobileNetDetectionNetwork)
    mobilenet.setConfidenceThreshold(MOBILENET_CONF)
    mobilenet.setBlobPath(
        blobconverter.from_zoo("mobilenet-ssd", shaves=3))
    mobilenet.setNumInferenceThreads(1)
    mobilenet.input.setBlocking(False)
    cam_rgb.preview.link(mobilenet.input)

    # --- Pedestrian + vehicle detector ---
    pedveh = pipeline.create(dai.node.MobileNetDetectionNetwork)
    pedveh.setConfidenceThreshold(PEDVEH_CONF)
    pedveh.setBlobPath(blobconverter.from_zoo(
        "pedestrian-and-vehicle-detector-adas-0001", shaves=3))
    pedveh.setNumInferenceThreads(1)
    pedveh.input.setBlocking(False)
    cam_rgb.preview.link(pedveh.input)

    # --- Outputs ---
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    xout_mob = pipeline.create(dai.node.XLinkOut)
    xout_mob.setStreamName("mobilenet")
    mobilenet.out.link(xout_mob.input)

    xout_pv = pipeline.create(dai.node.XLinkOut)
    xout_pv.setStreamName("pedveh")
    pedveh.out.link(xout_pv.input)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam_rgb.video.link(xout_rgb.input)

    return pipeline


# -------------------------------------------------------
# Depth classification
# -------------------------------------------------------
def _classify_depth(depth_frame: np.ndarray) -> tuple:
    h, w  = depth_frame.shape
    cx    = w // 2
    hw    = int(w * CENTER_STRIP_FRACTION / 2)
    strip = depth_frame[:, cx - hw : cx + hw]
    valid = strip[(strip >= DEPTH_MIN_MM) & (strip <= DEPTH_MAX_MM)]

    if len(valid) < MIN_VALID_PIXELS:
        return _STATE["zone"], _STATE["distance_m"]

    closest_mm = float(np.percentile(valid, DEPTH_PERCENTILE))
    closest_m  = closest_mm / 1000.0
    current    = _STATE["zone"]

    if current == "free":
        if closest_m < ZONE_CAUTION - 0.05:
            return "caution", closest_m
    elif current == "caution":
        if closest_m < ZONE_STOP - 0.05:
            return "stop", closest_m
        elif closest_m > ZONE_CAUTION + 0.05:
            return "free", closest_m
    elif current == "stop":
        if closest_m > ZONE_STOP + 0.05:
            return "caution", closest_m

    return current, closest_m


# -------------------------------------------------------
# Get depth for a bounding box
# -------------------------------------------------------
def _box_depth(depth_frame, xmin, ymin, xmax, ymax) -> float:
    h, w  = depth_frame.shape
    x1    = max(0, int(xmin * w))
    y1    = max(0, int(ymin * h))
    x2    = min(w-1, int(xmax * w))
    y2    = min(h-1, int(ymax * h))
    roi   = depth_frame[y1:y2, x1:x2]
    valid = roi[(roi >= DEPTH_MIN_MM) & (roi <= DEPTH_MAX_MM)]
    return round(float(np.median(valid)) / 1000.0, 2) if len(valid) > 5 else 99.0


# -------------------------------------------------------
# Merge detections from both models
# -------------------------------------------------------
def _merge_detections(mob_dets, pv_dets, depth_frame, rgb_frame):
    h_r, w_r = rgb_frame.shape[:2]
    results      = []
    zone_override = None
    annotated     = rgb_frame.copy()

    # Color per category
    def _color(label):
        if label == "person":   return (0, 0, 255)
        if label == "vehicle":  return (0, 100, 255)
        if label in CAUTION_OBJECTS: return (0, 165, 255)
        return (0, 200, 0)

    def _draw(det, label, dist_m, source):
        rx1 = int(det.xmin * w_r)
        ry1 = int(det.ymin * h_r)
        rx2 = int(det.xmax * w_r)
        ry2 = int(det.ymax * h_r)
        col = _color(label)
        cv2.rectangle(annotated, (rx1, ry1), (rx2, ry2), col, 2)
        conf_pct = int(det.confidence * 100)
        text = f"{label} {dist_m}m {conf_pct}%"
        # Background for text
        (tw, th), _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(annotated,
                      (rx1, ry1 - th - 6), (rx1 + tw + 4, ry1),
                      col, -1)
        cv2.putText(annotated, text,
                    (rx1 + 2, ry1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1)

    def _process(det, label):
        nonlocal zone_override
        dist_m = _box_depth(
            depth_frame, det.xmin, det.ymin, det.xmax, det.ymax)
        results.append({
            "label":      label,
            "confidence": round(float(det.confidence), 2),
            "dist_m":     dist_m,
            "source":     "pedveh" if label in ("person","vehicle")
                          else "mobilenet",
        })
        if label in STOP_OBJECTS and dist_m < ZONE_CAUTION:
            zone_override = "stop"
        elif label in CAUTION_OBJECTS and dist_m < ZONE_CAUTION:
            if zone_override != "stop":
                zone_override = "caution"
        return dist_m

    # Process pedestrian+vehicle detections (higher priority)
    seen_boxes = []
    for det in pv_dets:
        label  = PEDVEH_LABELS.get(det.label, "unknown")
        if label == "unknown":
            continue
        dist_m = _process(det, label)
        _draw(det, label, dist_m, "pedveh")
        seen_boxes.append((det.xmin, det.ymin, det.xmax, det.ymax))

    # Process MobileNet detections — skip if overlaps with pedveh box
    for det in mob_dets:
        label = MOBILENET_LABELS[det.label] \
                if det.label < len(MOBILENET_LABELS) else "unknown"
        if label in ("background", "unknown"):
            continue
        # Simple overlap check — skip if already detected by pedveh
        overlap = False
        for (bx1, by1, bx2, by2) in seen_boxes:
            iou_x = max(0, min(det.xmax, bx2) - max(det.xmin, bx1))
            iou_y = max(0, min(det.ymax, by2) - max(det.ymin, by1))
            if iou_x * iou_y > 0.3:
                overlap = True
                break
        if overlap:
            continue
        dist_m = _process(det, label)
        _draw(det, label, dist_m, "mobilenet")

    # Sort by distance
    results.sort(key=lambda x: x["dist_m"])
    return zone_override, results, annotated


# -------------------------------------------------------
# Flask dashboard
# -------------------------------------------------------
_flask_app = Flask(__name__)

_HTML = """<!DOCTYPE html>
<html>
<head>
  <title>Quadruped</title>
  <meta charset="utf-8">
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:monospace;background:#0d0d0d;color:#e0e0e0;padding:20px}
    h1{color:#444;font-size:12px;letter-spacing:3px;margin-bottom:16px;
       text-transform:uppercase}
    .grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;
          margin-bottom:16px}
    .card{background:#141414;border:1px solid #222;border-radius:6px;
          padding:14px}
    .label{font-size:10px;color:#444;letter-spacing:1px;
           text-transform:uppercase;margin-bottom:6px}
    .value{font-size:22px;font-weight:bold;line-height:1}
    .free{color:#00e676}.caution{color:#ff9100}.stop{color:#ff1744}
    .idle{color:#555}
    .main{display:grid;grid-template-columns:640px 1fr;gap:14px}
    .feed{background:#0a0a0a;border:1px solid #1a1a1a;border-radius:6px;
          overflow:hidden}
    .feed img{width:640px;height:480px;display:block;object-fit:cover}
    .dets{background:#141414;border:1px solid #222;border-radius:6px;
          padding:14px;overflow-y:auto;max-height:480px}
    .det{padding:8px 10px;margin:5px 0;border-radius:4px;
         border-left:3px solid #333;background:#0f0f0f;
         display:flex;justify-content:space-between;align-items:center}
    .det-l{font-size:13px;font-weight:bold}
    .det-r{font-size:11px;color:#555;text-align:right;line-height:1.6}
    .stop-det{border-left-color:#ff1744}
    .caution-det{border-left-color:#ff9100}
    .free-det{border-left-color:#00e676}
    .badge{font-size:9px;padding:2px 6px;border-radius:3px;
           background:#1a1a1a;color:#555;margin-left:6px}
    .dot{width:7px;height:7px;border-radius:50%;display:inline-block;
         margin-right:6px}
  </style>
</head>
<body>
<h1>Quadruped Telemetry &nbsp;// &nbsp;10.79.106.152</h1>
<div class="grid">
  <div class="card">
    <div class="label">Zone</div>
    <div class="value" id="zone">—</div>
  </div>
  <div class="card">
    <div class="label">Closest obstacle</div>
    <div class="value" id="dist">—</div>
  </div>
  <div class="card">
    <div class="label">Command</div>
    <div class="value idle" id="cmd">—</div>
  </div>
  <div class="card">
    <div class="label">Updated</div>
    <div class="value" id="age" style="font-size:15px">—</div>
  </div>
</div>
<div class="main">
  <div class="feed">
    <img id="feed" alt="feed">
  </div>
  <div class="dets">
    <div class="label" style="margin-bottom:10px">
      Detections &nbsp;<span id="det-count"
        style="color:#333;font-size:10px"></span>
    </div>
    <div id="det-list"><span style="color:#333">None</span></div>
  </div>
</div>
<script>
const STOP_OBJ    = new Set(['person']);
const CAUTION_OBJ = new Set(['vehicle','bicycle','motorbike','dog',
                              'cat','chair','sofa','diningtable']);

function detClass(label) {
  if (STOP_OBJ.has(label))    return 'stop-det';
  if (CAUTION_OBJ.has(label)) return 'caution-det';
  return 'free-det';
}
function dotColor(label) {
  if (STOP_OBJ.has(label))    return '#ff1744';
  if (CAUTION_OBJ.has(label)) return '#ff9100';
  return '#00e676';
}

async function update() {
  try {
    const d = await (await fetch('/api/state')).json();

    // Zone
    const z = d.zone;
    const zEl = document.getElementById('zone');
    zEl.textContent = z.toUpperCase();
    zEl.className = 'value ' + z;

    // Distance
    document.getElementById('dist').textContent =
      d.distance_m > 90 ? '—' : d.distance_m.toFixed(2) + ' m';

    // Command
    document.getElementById('cmd').textContent =
      d.robot_cmd.toUpperCase();

    // Age
    const age = ((Date.now()/1000) - d.updated_at_unix);
    document.getElementById('age').textContent =
      age < 2 ? 'live' : age.toFixed(1) + 's ago';

    // Detections
    const dets = d.detections;
    document.getElementById('det-count').textContent =
      dets.length ? `(${dets.length})` : '';
    document.getElementById('det-list').innerHTML =
      dets.length === 0
        ? '<span style="color:#333">None</span>'
        : dets.map(det => `
          <div class="det ${detClass(det.label)}">
            <div class="det-l">
              <span class="dot"
                style="background:${dotColor(det.label)}"></span>
              ${det.label}
              <span class="badge">${det.source}</span>
            </div>
            <div class="det-r">
              ${det.dist_m > 90 ? '—' : det.dist_m + 'm'}<br>
              ${(det.confidence*100).toFixed(0)}% conf
            </div>
          </div>`).join('');
  } catch(e) {}
}

function refreshFeed() {
  document.getElementById('feed').src = '/video?t=' + Date.now();
}

setInterval(update, 300);
setInterval(refreshFeed, 100);
update();
refreshFeed();
</script>
</body>
</html>"""


@_flask_app.route("/")
def dashboard():
    return render_template_string(_HTML)


@_flask_app.route("/api/state")
def api_state():
    with _STATE_LOCK:
        data = dict(_STATE)
    data["updated_at_unix"] = (
        time.time() - (time.monotonic() - data["updated_at"]))
    return Response(json.dumps(data), mimetype="application/json")


@_flask_app.route("/video")
def video():
    frame = _get_frame()
    if frame is None:
        return Response(status=204)
    _, buf = cv2.imencode(
        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
    return Response(buf.tobytes(), mimetype="image/jpeg")


def _run_flask():
    import logging
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    _flask_app.run(host="0.0.0.0", port=TELEMETRY_PORT,
                   debug=False, threaded=True)


# -------------------------------------------------------
# Main loop
# -------------------------------------------------------
def run_perception(debug_display: bool = False):
    print("[Perception] Loading models...")
    # Models already cached — no download needed
    pipeline = _build_pipeline()

    flask_thread = threading.Thread(target=_run_flask, daemon=True)
    flask_thread.start()
    print(f"[Perception] Dashboard → http://10.79.106.152:{TELEMETRY_PORT}")

    with dai.Device(pipeline) as device:
        print(f"[Perception] USB: {device.getUsbSpeed().name}")
        q_depth = device.getOutputQueue(
            "depth", maxSize=1, blocking=False)
        q_mob   = device.getOutputQueue(
            "mobilenet", maxSize=1, blocking=False)
        q_pv    = device.getOutputQueue(
            "pedveh", maxSize=1, blocking=False)
        q_rgb   = device.getOutputQueue(
            "rgb", maxSize=1, blocking=False)
        print("[Perception] Running.")

        prev_zone    = "free"
        last_publish = 0.0
        last_depth   = None
        last_frame   = None
        last_mob     = []
        last_pv      = []

        while True:
            # Depth
            msg = q_depth.tryGet()
            if msg:
                last_depth = msg.getFrame()

            # RGB
            msg = q_rgb.tryGet()
            if msg:
                last_frame = msg.getCvFrame()

            # MobileNet detections
            msg = q_mob.tryGet()
            if msg:
                last_mob = msg.detections

            # Pedestrian+vehicle detections
            msg = q_pv.tryGet()
            if msg:
                last_pv = msg.detections

            # Process when we have everything
            if last_depth is not None and last_frame is not None:
                depth_zone, dist = _classify_depth(last_depth)

                det_override, results, annotated = _merge_detections(
                    last_mob, last_pv, last_depth, last_frame)

                _write_frame(annotated)
                _write_detections(results)

                # Merge zones
                sev = {"free": 0, "caution": 1, "stop": 2}
                final_zone = depth_zone
                if det_override and sev[det_override] > sev[depth_zone]:
                    final_zone = det_override

                now          = time.monotonic()
                zone_changed = final_zone != prev_zone
                due          = (now - last_publish) >= PUBLISH_INTERVAL

                if zone_changed or due:
                    _write_depth(final_zone, dist)
                    last_publish = now
                    if zone_changed and VERBOSE:
                        top = results[0] if results else None
                        det_str = (f"  [{top['label']} "
                                   f"{top['dist_m']}m]") if top else ""
                        print(f"[Perception] {prev_zone} -> "
                              f"{final_zone}  {dist:.2f}m{det_str}")
                    prev_zone = final_zone

            time.sleep(0.005)