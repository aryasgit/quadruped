# arya/perception/telemetry.py
"""
Layer 4 — TELEMETRY DASHBOARD
==============================
Flask server showing live perception state.
Runs in a daemon thread — zero impact on gait loop.

Endpoints:
  /            — HTML dashboard (auto-refreshes)
  /video       — MJPEG stream (annotated RGB)
  /state       — JSON snapshot of full pipeline state
  /grid        — JSON occupancy grid

Usage:
    tel = Telemetry(port=5000)
    tel.start()

    # In perception loop:
    tel.push(frame=frame, detections=dets, tracks=tracks,
             world=world_state, fsm=fsm_output, cmd=gait_cmd)
"""

import json
import time
import threading
import io
import numpy as np
from typing import Optional, List

from flask import Flask, Response, jsonify


app = Flask(__name__)
_state: dict = {}
_frame_bytes: Optional[bytes] = None
_lock = threading.Lock()


# ── State push (called from perception loop) ──────────────────────

def push(
    frame=None,
    detections=None,
    tracks=None,
    world=None,
    fsm=None,
    cmd=None,
):
    """Push latest perception data to telemetry. Thread-safe."""
    global _frame_bytes, _state

    snap = {
        "ts":         time.time(),
        "fsm_state":  fsm.state.name     if fsm  else "N/A",
        "fsm_trigger": fsm.trigger       if fsm  else "",
        "nearest_m":  world.nearest_m    if world else None,
        "person":     world.person_present if world else False,
        "threat":     world.closing_threat if world else False,
        "left_clear": world.left_clear   if world else True,
        "right_clear": world.right_clear if world else True,
        "gait_cmd":   cmd.direction_override.name if cmd else "N/A",
        "step_scale": round(cmd.step_length_scale, 2) if cmd else 1.0,
        "detections": [
            {"label": d.label, "z_m": round(d.z_m, 2), "conf": round(d.conf, 2)}
            for d in (detections or [])
        ],
        "tracks": [
            {
                "id": t.id, "label": t.label,
                "z_m": round(t.z_m, 2),
                "vx": round(t.vx, 1), "vy": round(t.vy, 1),
                "age": t.age, "hits": t.hits,
            }
            for t in (tracks or [])
        ],
        "grid": world.grid.grid.tolist() if (world and hasattr(world, 'grid')) else [],
    }

    frame_data = None
    if frame is not None:
        try:
            import cv2
            _annotate_telemetry(frame, snap)
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                frame_data = buf.tobytes()
        except Exception:
            pass

    with _lock:
        _state = snap
        if frame_data is not None:
            _frame_bytes = frame_data


def _annotate_telemetry(frame: np.ndarray, snap: dict):
    """Overlay FSM state and nearest obstacle on the video frame."""
    import cv2
    state_str = snap.get("fsm_state", "")
    nearest   = snap.get("nearest_m")
    color_map = {
        "FREE":     (0,200,80),
        "CAUTIOUS": (0,160,220),
        "STOPPING": (0,100,255),
        "STOPPED":  (0,0,220),
        "EVADING":  (180,0,220),
    }
    color = color_map.get(state_str, (180,180,180))

    cv2.putText(frame, f"FSM: {state_str}", (8,22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    if nearest is not None:
        cv2.putText(frame, f"nearest: {nearest:.2f}m", (8,44),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220,220,60), 1, cv2.LINE_AA)


# ── Flask routes ──────────────────────────────────────────────────

@app.route("/")
def index():
    return Response(_HTML, mimetype="text/html")

@app.route("/state")
def state():
    with _lock:
        return jsonify(_state)

@app.route("/video")
def video():
    return Response(_gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

def _gen_frames():
    while True:
        with _lock:
            fb = _frame_bytes
        if fb is not None:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + fb + b"\r\n")
        time.sleep(0.033)


# ── HTML dashboard ────────────────────────────────────────────────

_HTML = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Quadruped Telemetry</title>
<style>
body{background:#111;color:#ddd;font-family:monospace;margin:0;padding:16px}
h2{color:#7df;margin:0 0 12px}
.grid{display:flex;gap:16px;flex-wrap:wrap}
.card{background:#1e1e1e;border:1px solid #333;border-radius:8px;padding:12px;min-width:200px}
.card h3{margin:0 0 8px;font-size:13px;color:#aaa}
#fsm{font-size:28px;font-weight:bold}
.FREE{color:#0c6} .CAUTIOUS{color:#08f} .STOPPING{color:#f80}
.STOPPED{color:#f44} .EVADING{color:#c4f}
table{border-collapse:collapse;font-size:12px;width:100%}
td,th{padding:3px 8px;border-bottom:1px solid #2a2a2a;text-align:left}
th{color:#888}
canvas{background:#0a0a0a;border:1px solid #333;border-radius:4px}
</style>
</head>
<body>
<h2>Quadruped Perception Telemetry</h2>
<div class="grid">
  <div class="card"><h3>FSM State</h3><div id="fsm">—</div><div id="trigger" style="font-size:11px;color:#666;margin-top:4px"></div></div>
  <div class="card"><h3>Environment</h3>
    <div id="nearest">—</div>
    <div id="person" style="margin-top:6px"></div>
    <div id="threat"></div>
    <div id="lanes" style="margin-top:6px;font-size:12px"></div>
  </div>
  <div class="card"><h3>Gait Command</h3><div id="gait">—</div><div id="step" style="font-size:12px;color:#888"></div></div>
  <div class="card"><h3>Video</h3><img src="/video" width="320" height="240" style="border-radius:4px"></div>
</div>
<div class="grid" style="margin-top:16px">
  <div class="card"><h3>Occupancy Grid (top-down, robot at bottom)</h3><canvas id="grid" width="200" height="300"></canvas></div>
  <div class="card" style="flex:1"><h3>Tracks</h3><table><thead><tr><th>ID</th><th>Label</th><th>Z(m)</th><th>Speed</th><th>Age</th></tr></thead><tbody id="tracks"></tbody></table></div>
  <div class="card" style="flex:1"><h3>Detections</h3><table><thead><tr><th>Label</th><th>Z(m)</th><th>Conf</th></tr></thead><tbody id="dets"></tbody></table></div>
</div>
<script>
function poll(){
  fetch('/state').then(r=>r.json()).then(s=>{
    const fsmEl=document.getElementById('fsm');
    fsmEl.textContent=s.fsm_state||'—';
    fsmEl.className=s.fsm_state||'';
    document.getElementById('trigger').textContent=s.fsm_trigger||'';
    document.getElementById('nearest').textContent=s.nearest_m!=null?'Nearest: '+s.nearest_m.toFixed(2)+'m':'Nearest: clear';
    document.getElementById('person').textContent=s.person?'⚠ Person detected':'';
    document.getElementById('person').style.color=s.person?'#f80':'';
    document.getElementById('threat').textContent=s.threat?'⚠ Closing threat!':'';
    document.getElementById('threat').style.color=s.threat?'#f44':'';
    document.getElementById('lanes').textContent='L:'+( s.left_clear?'clear':'blocked')+' R:'+(s.right_clear?'clear':'blocked');
    document.getElementById('gait').textContent=s.gait_cmd||'—';
    document.getElementById('step').textContent='step_scale: '+(s.step_scale||1);
    // tracks
    const tb=document.getElementById('tracks');
    tb.innerHTML=(s.tracks||[]).map(t=>`<tr><td>${t.id}</td><td>${t.label}</td><td>${t.z_m}</td><td>${Math.sqrt(t.vx*t.vx+t.vy*t.vy).toFixed(1)}px/f</td><td>${t.age}</td></tr>`).join('');
    // dets
    const db=document.getElementById('dets');
    db.innerHTML=(s.detections||[]).map(d=>`<tr><td>${d.label}</td><td>${d.z_m}</td><td>${d.conf}</td></tr>`).join('');
    // grid
    const grid=s.grid;
    if(grid&&grid.length){
      const cv=document.getElementById('grid');
      const ctx=cv.getContext('2d');
      const rows=grid.length, cols=grid[0].length;
      const cw=cv.width/cols, ch=cv.height/rows;
      ctx.clearRect(0,0,cv.width,cv.height);
      for(let r=0;r<rows;r++){
        for(let c=0;c<cols;c++){
          const v=grid[r][c];
          const alpha=Math.min(1,v*2);
          ctx.fillStyle=`rgba(255,80,40,${alpha})`;
          ctx.fillRect(c*cw,(rows-1-r)*ch,cw,ch);
        }
      }
      // robot marker
      ctx.fillStyle='#4af';
      ctx.fillRect(cv.width/2-5,cv.height-10,10,8);
    }
  }).catch(()=>{});
}
setInterval(poll,200);
poll();
</script>
</body>
</html>"""


# ── Telemetry server class ────────────────────────────────────────

class Telemetry:
    def __init__(self, host: str = "0.0.0.0", port: int = 5000):
        self.host = host
        self.port = port
        self._thread: Optional[threading.Thread] = None

    def start(self):
        import logging
        log = logging.getLogger("werkzeug")
        log.setLevel(logging.ERROR)
        self._thread = threading.Thread(
            target=app.run,
            kwargs=dict(host=self.host, port=self.port, debug=False, use_reloader=False),
            daemon=True,
        )
        self._thread.start()
        print(f"[telemetry] http://{self.host}:{self.port}")