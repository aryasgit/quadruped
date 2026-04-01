"""
dashboard.py — TELEMETRY DASHBOARD (Flask)
============================================
Real-time telemetry dashboard served via Flask.

Architecture decision: Flask with threading (not multiprocessing).
Justification:
  - The Jetson Orin Nano has 8 cores — we're not GIL-bound because
    the heavy work (DepthAI inference, JPEG encoding) happens in C
    extensions that release the GIL.
  - JPEG frames arrive pre-encoded from perception.py via queue —
    Flask just relays bytes, no OpenCV in the Flask process.
  - multiprocessing would require pickling numpy arrays across process
    boundaries, adding 3-5ms latency per frame for zero benefit.
  - Werkzeug's threaded mode handles 2-3 browser clients trivially.
  - The existing telemetry.py on this robot already proves Flask
    stability over multi-hour runs on this exact hardware.

Threading safety:
  - All state comes from a single DashboardData object behind a Lock.
  - MJPEG generator reads JPEG bytes (immutable) under the lock.
  - No shared mutable state outside the lock.
  - Flask runs in a daemon thread — never blocks the control loop.

Color scheme: Matches barq.html exactly.
  --black:   #0a0a0a
  --white:   #f0ede6
  --grey:    #6b6b6b
  --accent:  #c8ff00
  --red:     #ff3b30
  --blue:    #0a84ff
  --border:  rgba(240,237,230,0.15)
  --font:    'JetBrains Mono', 'Fira Code', 'SF Mono', monospace
"""

import time
import json
import threading
from typing import Optional

from flask import Flask, Response, jsonify

from state import (
    DashboardData, BootMode, ProximityState, ControlMode,
    DetectionSnapshot,
)


# ── Shared state ─────────────────────────────────────────────────

_data: DashboardData = DashboardData()
_lock = threading.Lock()


def update_dashboard(data: DashboardData):
    """Called from main thread to push latest aggregate data."""
    global _data
    with _lock:
        _data = data


def _get_data() -> DashboardData:
    with _lock:
        return _data


# ── Flask app ────────────────────────────────────────────────────

app = Flask(__name__)

# Suppress Werkzeug request logging
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)


@app.route("/")
def index():
    return Response(_DASHBOARD_HTML, mimetype="text/html")


@app.route("/api/state")
def api_state():
    d = _get_data()
    payload = {
        "ts": d.timestamp,
        "proximity": d.proximity_state.name,
        "control_mode": d.control_mode.name,
        "nearest_m": d.nearest_distance_m,
        "nearest_bearing": d.nearest_bearing_deg,
        "fps": round(d.perception_fps, 1),
        "height_mode": d.height_mode,
        "last_command": d.last_command,
        "boot_mode": d.boot_mode.name,
        "uptime_s": round(d.uptime_s, 1),
        "imu": {
            "roll": round(d.imu.roll_deg, 2),
            "pitch": round(d.imu.pitch_deg, 2),
            "roll_rate": round(d.imu.roll_rate, 1),
            "pitch_rate": round(d.imu.pitch_rate, 1),
        },
        "step": {
            "is_stepping": d.step.is_stepping,
            "active_leg": d.step.active_leg,
            "phase": round(d.step.phase, 3),
            "direction": d.step.direction,
            "cycle_progress": round(d.step.cycle_progress, 3),
        },
        "detections": [
            {
                "label": det.label,
                "confidence": round(det.confidence, 2),
                "distance_m": round(det.distance_m, 2),
                "bearing_deg": round(det.bearing_deg, 1),
                "track_id": det.track_id,
            }
            for det in d.detections
        ],
    }
    return Response(json.dumps(payload), mimetype="application/json")


@app.route("/stream/rgb")
def stream_rgb():
    """MJPEG stream of annotated RGB frames."""
    def gen():
        while True:
            d = _get_data()
            if d.rgb_jpeg:
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n"
                       + d.rgb_jpeg + b"\r\n")
            time.sleep(0.05)  # ~20 fps max to browser
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stream/depth")
def stream_depth():
    """MJPEG stream of colorized depth map."""
    def gen():
        while True:
            d = _get_data()
            if d.depth_jpeg:
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n"
                       + d.depth_jpeg + b"\r\n")
            time.sleep(0.05)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


# ── Server launcher ──────────────────────────────────────────────

def start_dashboard(port: int = 5000):
    """Start Flask in a daemon thread. Non-blocking."""
    thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=port,
                               debug=False, threaded=True),
        daemon=True,
        name="dashboard",
    )
    thread.start()
    print(f"[DASHBOARD] Running at http://0.0.0.0:{port}")
    return thread


# ── HTML Dashboard ───────────────────────────────────────────────
# Full inline HTML — no external files needed on the Jetson.
# Aesthetic: barq.html military/aerospace HUD.

_DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>BARQ // TELEMETRY</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;500;700&display=swap" rel="stylesheet">
<style>
/* ═══════════════════════════════════════════════
   BARQ TELEMETRY — CSS VARIABLES
   Matches barq.html color palette exactly.
   ═══════════════════════════════════════════════ */
:root {
  --black: #0a0a0a;
  --black-card: #0f0f0f;
  --white: #f0ede6;
  --grey: #6b6b6b;
  --grey-dim: #3a3a3a;
  --accent: #c8ff00;
  --accent-dim: #5a7200;
  --accent-glow: rgba(200,255,0,0.15);
  --red: #ff3b30;
  --red-glow: rgba(255,59,48,0.25);
  --orange: #ff9500;
  --orange-glow: rgba(255,149,0,0.2);
  --amber: #ffcc00;
  --blue: #0a84ff;
  --green: #30d158;
  --border: rgba(240,237,230,0.12);
  --border-strong: rgba(240,237,230,0.22);
  --font: 'JetBrains Mono', 'Fira Code', 'SF Mono', 'Cascadia Code', monospace;
}

*, *::before, *::after { margin:0; padding:0; box-sizing:border-box; }

html { font-size: 13px; }

body {
  font-family: var(--font);
  background: var(--black);
  color: var(--white);
  line-height: 1.4;
  overflow-x: hidden;
  -webkit-font-smoothing: antialiased;
}

::selection { background: var(--accent); color: var(--black); }

/* ═══════════════════════════════════════════════
   HEADER BAR
   ═══════════════════════════════════════════════ */
.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px 16px;
  border-bottom: 1px solid var(--border);
  background: linear-gradient(180deg, #111 0%, var(--black) 100%);
}

.header-left {
  display: flex;
  align-items: center;
  gap: 16px;
}

.logo {
  font-size: 0.7rem;
  font-weight: 500;
  text-transform: uppercase;
  letter-spacing: 0.3em;
  color: var(--accent);
}

.logo span {
  color: var(--grey);
  font-weight: 300;
  letter-spacing: 0.15em;
  margin-left: 8px;
}

.header-badges {
  display: flex;
  gap: 8px;
}

.hbadge {
  font-size: 0.6rem;
  text-transform: uppercase;
  letter-spacing: 0.15em;
  padding: 2px 8px;
  border: 1px solid var(--border);
  border-radius: 2px;
  color: var(--grey);
}

.hbadge.active { border-color: var(--accent-dim); color: var(--accent); }

.header-right {
  display: flex;
  align-items: center;
  gap: 16px;
  font-size: 0.65rem;
  color: var(--grey);
  text-transform: uppercase;
  letter-spacing: 0.1em;
}

#uptime { color: var(--white); }
#clock { color: var(--white); font-weight: 500; }

/* ═══════════════════════════════════════════════
   STATE BADGE (large, colored, flashing for STOP)
   ═══════════════════════════════════════════════ */
.state-bar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 8px 16px;
  border-bottom: 1px solid var(--border);
}

.state-badge {
  font-size: 1.4rem;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.15em;
  padding: 4px 20px;
  border: 2px solid var(--border);
  border-radius: 2px;
  transition: all 0.3s ease;
}

.state-badge.SAFE    { color: var(--green);  border-color: var(--green);  text-shadow: 0 0 12px rgba(48,209,88,0.4); }
.state-badge.CAUTION { color: var(--amber);  border-color: var(--amber);  text-shadow: 0 0 12px rgba(255,204,0,0.4); }
.state-badge.SLOW    { color: var(--orange); border-color: var(--orange); text-shadow: 0 0 12px var(--orange-glow); }
.state-badge.STOP    { color: var(--red);    border-color: var(--red);    text-shadow: 0 0 16px var(--red-glow);
                       animation: flash-stop 0.5s ease-in-out infinite alternate; }

@keyframes flash-stop {
  0%   { opacity: 1; background: rgba(255,59,48,0.15); }
  100% { opacity: 0.7; background: rgba(255,59,48,0.05); }
}

.state-meta {
  display: flex;
  gap: 24px;
  font-size: 0.7rem;
  color: var(--grey);
  text-transform: uppercase;
  letter-spacing: 0.1em;
}

.state-meta .val { color: var(--white); font-weight: 500; margin-left: 6px; }

.override-badge {
  font-size: 0.6rem;
  text-transform: uppercase;
  letter-spacing: 0.2em;
  padding: 3px 10px;
  border: 1px solid var(--border);
  border-radius: 2px;
}

.override-badge.MANUAL     { color: var(--blue); border-color: var(--blue); }
.override-badge.AUTO_AVOID { color: var(--red);  border-color: var(--red);
                             animation: flash-stop 0.8s ease-in-out infinite alternate; }

/* ═══════════════════════════════════════════════
   MAIN GRID
   ═══════════════════════════════════════════════ */
.main-grid {
  display: grid;
  grid-template-columns: 1fr 1fr 320px;
  grid-template-rows: auto auto;
  gap: 1px;
  background: var(--border);
  min-height: calc(100vh - 100px);
}

.panel {
  background: var(--black);
  padding: 12px;
  position: relative;
  overflow: hidden;
}

.panel-label {
  display: inline-block;
  font-size: 0.5rem;
  text-transform: uppercase;
  letter-spacing: 0.3em;
  color: var(--accent);
  background: var(--black);
  padding: 2px 6px;
  border: 1px solid var(--accent-dim);
  border-radius: 2px;
  margin-bottom: 10px;
  position: relative;
  z-index: 1;
}

/* Stream panels */
.stream-container {
  position: relative;
  width: 100%;
  background: #050505;
  border: 1px solid var(--border);
  border-radius: 2px;
  overflow: hidden;
}

.stream-container img {
  width: 100%;
  display: block;
}

.no-feed {
  display: flex;
  align-items: center;
  justify-content: center;
  height: 240px;
  color: var(--grey-dim);
  font-size: 0.7rem;
  text-transform: uppercase;
  letter-spacing: 0.2em;
  border: 1px dashed var(--border);
}

/* ═══════════════════════════════════════════════
   SIDEBAR PANELS
   ═══════════════════════════════════════════════ */

/* Radar */
.radar-wrap {
  position: relative;
  width: 100%;
  aspect-ratio: 1;
  max-height: 220px;
}

.radar-wrap canvas {
  width: 100%;
  height: 100%;
  background: #050505;
  border: 1px solid var(--border);
  border-radius: 2px;
}

/* Detection list */
.det-list {
  max-height: 180px;
  overflow-y: auto;
  scrollbar-width: thin;
  scrollbar-color: var(--accent-dim) var(--black);
}

.det-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 5px 8px;
  border-bottom: 1px solid var(--border);
  font-size: 0.7rem;
}

.det-row:last-child { border-bottom: none; }

.det-label {
  display: flex;
  align-items: center;
  gap: 6px;
  font-weight: 500;
}

.det-dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  flex-shrink: 0;
}

.det-dist {
  color: var(--grey);
  font-weight: 300;
}

.det-empty {
  color: var(--grey-dim);
  font-size: 0.65rem;
  text-transform: uppercase;
  letter-spacing: 0.15em;
  text-align: center;
  padding: 20px 0;
}

/* IMU panel */
.imu-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
}

.imu-gauge {
  text-align: center;
  padding: 8px;
  border: 1px solid var(--border);
  border-radius: 2px;
  background: #050505;
}

.imu-value {
  font-size: 1.4rem;
  font-weight: 300;
  color: var(--accent);
  line-height: 1;
}

.imu-label {
  font-size: 0.5rem;
  text-transform: uppercase;
  letter-spacing: 0.2em;
  color: var(--grey);
  margin-top: 4px;
}

/* Step progress */
.step-bar-container {
  padding: 8px 0;
}

.step-legs {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 4px;
  margin-bottom: 8px;
}

.step-leg {
  text-align: center;
  padding: 4px;
  border: 1px solid var(--border);
  border-radius: 2px;
  font-size: 0.6rem;
  text-transform: uppercase;
  letter-spacing: 0.1em;
  color: var(--grey-dim);
  transition: all 0.15s ease;
}

.step-leg.active {
  border-color: var(--accent);
  color: var(--accent);
  background: rgba(200,255,0,0.08);
  box-shadow: 0 0 8px var(--accent-glow);
}

.step-progress {
  height: 3px;
  background: var(--border);
  border-radius: 1px;
  overflow: hidden;
}

.step-progress-fill {
  height: 100%;
  background: var(--accent);
  border-radius: 1px;
  transition: width 0.1s linear;
  box-shadow: 0 0 6px var(--accent-glow);
}

.step-direction {
  font-size: 0.6rem;
  text-transform: uppercase;
  letter-spacing: 0.15em;
  color: var(--grey);
  margin-top: 6px;
  text-align: center;
}

/* ═══════════════════════════════════════════════
   SCANLINE OVERLAY (aesthetic detail)
   ═══════════════════════════════════════════════ */
body::after {
  content: '';
  position: fixed;
  inset: 0;
  background: repeating-linear-gradient(
    0deg,
    transparent,
    transparent 2px,
    rgba(0,0,0,0.03) 2px,
    rgba(0,0,0,0.03) 4px
  );
  pointer-events: none;
  z-index: 9999;
}

/* ═══════════════════════════════════════════════
   RESPONSIVE
   ═══════════════════════════════════════════════ */
@media (max-width: 1000px) {
  .main-grid {
    grid-template-columns: 1fr 1fr;
  }
  .main-grid > .panel:nth-child(5),
  .main-grid > .panel:nth-child(6) {
    grid-column: span 2;
  }
}

@media (max-width: 700px) {
  .main-grid {
    grid-template-columns: 1fr;
  }
}
</style>
</head>
<body>

<!-- ═══════ HEADER ═══════ -->
<div class="header">
  <div class="header-left">
    <div class="logo">BARQ <span>// TELEMETRY</span></div>
    <div class="header-badges">
      <span class="hbadge active" id="hw-cam">CAM</span>
      <span class="hbadge active" id="hw-robot">ROBOT</span>
      <span class="hbadge active" id="hw-imu">IMU</span>
    </div>
  </div>
  <div class="header-right">
    <span>UPTIME <span id="uptime">0s</span></span>
    <span id="clock">--:--:--</span>
  </div>
</div>

<!-- ═══════ STATE BAR ═══════ -->
<div class="state-bar">
  <div class="state-badge SAFE" id="state-badge">SAFE</div>
  <div class="state-meta">
    <span>NEAREST<span class="val" id="nearest-dist">—</span></span>
    <span>FPS<span class="val" id="fps-val">—</span></span>
    <span>CMD<span class="val" id="last-cmd">idle</span></span>
  </div>
  <div class="override-badge MANUAL" id="override-badge">MANUAL</div>
</div>

<!-- ═══════ MAIN GRID ═══════ -->
<div class="main-grid">

  <!-- Panel 1: RGB Stream -->
  <div class="panel">
    <div class="panel-label">Live Camera Feed</div>
    <div class="stream-container" id="rgb-container">
      <img id="rgb-stream" src="/stream/rgb" alt="RGB" onerror="this.style.display='none'" />
      <div class="no-feed" id="rgb-nofeed" style="display:none">NO CAMERA FEED</div>
    </div>
  </div>

  <!-- Panel 2: Depth Map -->
  <div class="panel">
    <div class="panel-label">Depth Map</div>
    <div class="stream-container" id="depth-container">
      <img id="depth-stream" src="/stream/depth" alt="Depth" onerror="this.style.display='none'" />
      <div class="no-feed" id="depth-nofeed" style="display:none">NO DEPTH DATA</div>
    </div>
  </div>

  <!-- Panel 3: Sidebar — Radar + Detections (spans 2 rows) -->
  <div class="panel" style="grid-row: span 2;">
    <!-- Radar -->
    <div class="panel-label">Proximity Radar</div>
    <div class="radar-wrap">
      <canvas id="radar"></canvas>
    </div>

    <!-- Detection List -->
    <div style="margin-top:12px;">
      <div class="panel-label">Tracked Objects</div>
      <div class="det-list" id="det-list">
        <div class="det-empty">Scanning...</div>
      </div>
    </div>

    <!-- Override Indicator (moved here for sidebar) -->
    <div style="margin-top:12px;">
      <div class="panel-label">Step Progress</div>
      <div class="step-bar-container">
        <div class="step-legs">
          <div class="step-leg" id="leg-FL">FL</div>
          <div class="step-leg" id="leg-FR">FR</div>
          <div class="step-leg" id="leg-RL">RL</div>
          <div class="step-leg" id="leg-RR">RR</div>
        </div>
        <div class="step-progress"><div class="step-progress-fill" id="step-fill" style="width:0%"></div></div>
        <div class="step-direction" id="step-dir">idle</div>
      </div>
    </div>
  </div>

  <!-- Panel 4: IMU -->
  <div class="panel">
    <div class="panel-label">IMU — Orientation</div>
    <div class="imu-grid">
      <div class="imu-gauge">
        <div class="imu-value" id="imu-roll">0.00°</div>
        <div class="imu-label">Roll</div>
      </div>
      <div class="imu-gauge">
        <div class="imu-value" id="imu-pitch">0.00°</div>
        <div class="imu-label">Pitch</div>
      </div>
      <div class="imu-gauge">
        <div class="imu-value" id="imu-rr" style="font-size:1rem;">0.0</div>
        <div class="imu-label">Roll Rate °/s</div>
      </div>
      <div class="imu-gauge">
        <div class="imu-value" id="imu-pr" style="font-size:1rem;">0.0</div>
        <div class="imu-label">Pitch Rate °/s</div>
      </div>
    </div>
    <!-- IMU horizon indicator -->
    <div style="margin-top:10px;position:relative;height:60px;border:1px solid var(--border);border-radius:2px;background:#050505;overflow:hidden;">
      <canvas id="horizon" style="width:100%;height:100%;"></canvas>
    </div>
  </div>

  <!-- Panel 5: System Info -->
  <div class="panel">
    <div class="panel-label">System</div>
    <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:8px;">
      <div class="imu-gauge">
        <div class="imu-value" id="sys-fps" style="font-size:1.2rem;">—</div>
        <div class="imu-label">Perc. FPS</div>
      </div>
      <div class="imu-gauge">
        <div class="imu-value" id="sys-height" style="font-size:1rem;">NORMAL</div>
        <div class="imu-label">Height</div>
      </div>
      <div class="imu-gauge">
        <div class="imu-value" id="sys-boot" style="font-size:0.8rem;">FULL</div>
        <div class="imu-label">Boot Mode</div>
      </div>
    </div>
  </div>

</div>

<script>
/* ═══════════════════════════════════════════════
   TELEMETRY UPDATE LOOP
   ═══════════════════════════════════════════════ */

// Clock
function updateClock() {
  const now = new Date();
  document.getElementById('clock').textContent =
    now.toLocaleTimeString('en-GB', { hour12: false });
}
setInterval(updateClock, 1000);
updateClock();

// Radar canvas
const radarEl = document.getElementById('radar');
const rctx = radarEl.getContext('2d');

function resizeRadar() {
  const rect = radarEl.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  radarEl.width = rect.width * dpr;
  radarEl.height = rect.height * dpr;
  rctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
resizeRadar();
window.addEventListener('resize', resizeRadar);

function drawRadar(detections, nearestBearing) {
  const w = radarEl.width / (window.devicePixelRatio || 1);
  const h = radarEl.height / (window.devicePixelRatio || 1);
  const cx = w / 2;
  const cy = h - 12;
  const scale = (h - 24) / 3.0;

  rctx.clearRect(0, 0, w, h);

  // Zone arcs: STOP (0.4m), SLOW (0.6m), CAUTION (0.8m), FAR (2m)
  const zones = [
    { r: 0.4, color: 'rgba(255,59,48,0.25)',  lw: 1.5 },
    { r: 0.6, color: 'rgba(255,149,0,0.20)',  lw: 1.0 },
    { r: 0.8, color: 'rgba(255,204,0,0.15)',  lw: 0.8 },
    { r: 2.0, color: 'rgba(200,255,0,0.08)',  lw: 0.5 },
  ];

  zones.forEach(z => {
    const px = z.r * scale;
    rctx.beginPath();
    rctx.arc(cx, cy, px, Math.PI, 2 * Math.PI);
    rctx.strokeStyle = z.color;
    rctx.lineWidth = z.lw;
    rctx.stroke();
    // Range label
    rctx.fillStyle = 'rgba(107,107,107,0.5)';
    rctx.font = '9px JetBrains Mono';
    rctx.fillText(z.r + 'm', cx + px - 18, cy - 4);
  });

  // FOV lines
  rctx.beginPath();
  rctx.moveTo(cx, cy);
  rctx.lineTo(cx - (h - 24) * 0.85, 8);
  rctx.moveTo(cx, cy);
  rctx.lineTo(cx + (h - 24) * 0.85, 8);
  rctx.strokeStyle = 'rgba(200,255,0,0.06)';
  rctx.lineWidth = 0.5;
  rctx.stroke();

  // Robot icon
  rctx.beginPath();
  rctx.moveTo(cx, cy - 6);
  rctx.lineTo(cx - 4, cy + 2);
  rctx.lineTo(cx + 4, cy + 2);
  rctx.closePath();
  rctx.fillStyle = 'rgba(200,255,0,0.6)';
  rctx.fill();

  // Detections
  detections.forEach(d => {
    const angle = d.bearing_deg * Math.PI / 180;
    const dist = d.distance_m;
    const px = cx + Math.sin(angle) * dist * scale;
    const py = cy - Math.cos(angle) * dist * scale;

    const color = dist < 0.4 ? '#ff3b30' :
                  dist < 0.6 ? '#ff9500' :
                  dist < 0.8 ? '#ffcc00' : '#30d158';

    // Glow
    const grad = rctx.createRadialGradient(px, py, 0, px, py, 10);
    grad.addColorStop(0, color + '44');
    grad.addColorStop(1, color + '00');
    rctx.beginPath();
    rctx.arc(px, py, 10, 0, Math.PI * 2);
    rctx.fillStyle = grad;
    rctx.fill();

    // Dot
    rctx.beginPath();
    rctx.arc(px, py, 3, 0, Math.PI * 2);
    rctx.fillStyle = color;
    rctx.fill();

    // Label
    rctx.fillStyle = color;
    rctx.font = '9px JetBrains Mono';
    const tid = d.track_id !== null ? ' T' + d.track_id : '';
    rctx.fillText(d.label + tid, px + 6, py - 4);
  });
}

// Horizon indicator
const horizonEl = document.getElementById('horizon');
const hctx = horizonEl.getContext('2d');

function resizeHorizon() {
  const rect = horizonEl.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  horizonEl.width = rect.width * dpr;
  horizonEl.height = 60 * dpr;
  hctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
resizeHorizon();

function drawHorizon(roll, pitch) {
  const w = horizonEl.width / (window.devicePixelRatio || 1);
  const h = 60;
  hctx.clearRect(0, 0, w, h);

  const cx = w / 2;
  const cy = h / 2;
  const rad = roll * Math.PI / 180;
  const pitchPx = pitch * 1.5;  // scale pitch to pixels

  hctx.save();
  hctx.translate(cx, cy);
  hctx.rotate(-rad);

  // Sky/ground split
  hctx.fillStyle = 'rgba(10,132,255,0.15)';
  hctx.fillRect(-w, -h + pitchPx, w * 2, h);
  hctx.fillStyle = 'rgba(255,149,0,0.1)';
  hctx.fillRect(-w, pitchPx, w * 2, h);

  // Horizon line
  hctx.beginPath();
  hctx.moveTo(-w, pitchPx);
  hctx.lineTo(w, pitchPx);
  hctx.strokeStyle = 'rgba(240,237,230,0.3)';
  hctx.lineWidth = 1;
  hctx.stroke();

  hctx.restore();

  // Center crosshair (fixed)
  hctx.beginPath();
  hctx.moveTo(cx - 15, cy);
  hctx.lineTo(cx - 5, cy);
  hctx.moveTo(cx + 5, cy);
  hctx.lineTo(cx + 15, cy);
  hctx.moveTo(cx, cy - 5);
  hctx.lineTo(cx, cy + 5);
  hctx.strokeStyle = 'var(--accent)';
  hctx.lineWidth = 1;
  hctx.stroke();
}

// State update
async function updateState() {
  try {
    const resp = await fetch('/api/state');
    const d = await resp.json();

    // State badge
    const badge = document.getElementById('state-badge');
    badge.textContent = d.proximity;
    badge.className = 'state-badge ' + d.proximity;

    // Override badge
    const ob = document.getElementById('override-badge');
    ob.textContent = d.control_mode;
    ob.className = 'override-badge ' + d.control_mode;

    // Meta
    document.getElementById('nearest-dist').textContent =
      d.nearest_m !== null ? d.nearest_m.toFixed(2) + 'm' : '—';
    document.getElementById('fps-val').textContent = d.fps;
    document.getElementById('last-cmd').textContent = d.last_command.toUpperCase();
    document.getElementById('uptime').textContent = d.uptime_s + 's';

    // IMU
    document.getElementById('imu-roll').textContent = d.imu.roll.toFixed(2) + '°';
    document.getElementById('imu-pitch').textContent = d.imu.pitch.toFixed(2) + '°';
    document.getElementById('imu-rr').textContent = d.imu.roll_rate.toFixed(1);
    document.getElementById('imu-pr').textContent = d.imu.pitch_rate.toFixed(1);
    drawHorizon(d.imu.roll, d.imu.pitch);

    // Step progress
    ['FL','FR','RL','RR'].forEach(leg => {
      const el = document.getElementById('leg-' + leg);
      if (d.step.active_leg === leg && d.step.is_stepping) {
        el.classList.add('active');
      } else {
        el.classList.remove('active');
      }
    });
    document.getElementById('step-fill').style.width =
      (d.step.cycle_progress * 100).toFixed(0) + '%';
    document.getElementById('step-dir').textContent =
      d.step.is_stepping ? d.step.direction.toUpperCase() : 'IDLE';

    // System
    document.getElementById('sys-fps').textContent = d.fps;
    document.getElementById('sys-height').textContent = d.height_mode;
    document.getElementById('sys-boot').textContent = d.boot_mode;

    // Hardware badges
    const isCamera = d.boot_mode === 'FULL' || d.boot_mode === 'CAMERA_ONLY';
    const isRobot = d.boot_mode === 'FULL' || d.boot_mode === 'ROBOT_ONLY';
    document.getElementById('hw-cam').className = 'hbadge' + (isCamera ? ' active' : '');
    document.getElementById('hw-robot').className = 'hbadge' + (isRobot ? ' active' : '');

    // Camera feed visibility
    if (!isCamera) {
      document.getElementById('rgb-stream').style.display = 'none';
      document.getElementById('rgb-nofeed').style.display = 'flex';
      document.getElementById('depth-stream').style.display = 'none';
      document.getElementById('depth-nofeed').style.display = 'flex';
    }

    // Detection list
    const dlEl = document.getElementById('det-list');
    if (d.detections.length === 0) {
      dlEl.innerHTML = '<div class="det-empty">No objects detected</div>';
    } else {
      dlEl.innerHTML = d.detections.map(det => {
        const color = det.distance_m < 0.4 ? '#ff3b30' :
                      det.distance_m < 0.6 ? '#ff9500' :
                      det.distance_m < 0.8 ? '#ffcc00' : '#30d158';
        const tid = det.track_id !== null ? ' T' + det.track_id : '';
        return '<div class="det-row">' +
          '<div class="det-label">' +
            '<div class="det-dot" style="background:' + color + '"></div>' +
            det.label + tid +
          '</div>' +
          '<div class="det-dist">' + det.distance_m.toFixed(2) + 'm / ' +
            det.bearing_deg.toFixed(0) + '°</div>' +
        '</div>';
      }).join('');
    }

    // Radar
    drawRadar(d.detections, d.nearest_bearing);

  } catch(e) {
    // Silent — server may be briefly unreachable
  }
}

// Poll at ~10Hz for responsive dashboard
setInterval(updateState, 100);
updateState();

</script>
</body>
</html>"""