"""
Telemetry dashboard — cyberpunk edition.
MJPEG streams + real-time stats + proximity radar.
"""

import cv2
import time
import threading
import json
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional
from flask import Flask, Response, render_template_string

from .detector import Detection

STREAM_W, STREAM_H = 640, 360
JPEG_QUALITY = 60


# ── Shared state — explicit kwargs only (no **kwargs) ───────
@dataclass
class TelemetryState:
    rgb_frame: np.ndarray = None
    depth_frame: np.ndarray = None
    detections: List[Detection] = field(default_factory=list)
    fps: float = 0.0
    loop_ms: float = 0.0
    zone: str = "CLEAR"
    stop_flag: bool = False
    nearest_dist: Optional[float] = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update(self, rgb=None, depth=None, detections=None,
               fps=None, loop_ms=None, zone=None,
               stop_flag=None, nearest_dist=None):
        with self.lock:
            if rgb is not None:
                self.rgb_frame = rgb
            if depth is not None:
                self.depth_frame = depth
            if detections is not None:
                self.detections = list(detections)
            if fps is not None:
                self.fps = fps
            if loop_ms is not None:
                self.loop_ms = loop_ms
            if zone is not None:
                self.zone = zone
            if stop_flag is not None:
                self.stop_flag = stop_flag
            if nearest_dist is not None:
                self.nearest_dist = nearest_dist

    def snapshot(self):
        with self.lock:
            return (
                self.rgb_frame.copy() if self.rgb_frame is not None else None,
                self.depth_frame.copy() if self.depth_frame is not None else None,
                list(self.detections),
                self.fps,
                self.loop_ms,
                self.zone,
                self.stop_flag,
                self.nearest_dist,
            )


# ── Frame annotation ────────────────────────────────────────
def annotate_rgb(frame, detections, fps, zone):
    if frame is None:
        return np.zeros((STREAM_H, STREAM_W, 3), dtype=np.uint8)
    h, w = frame.shape[:2]
    out = frame.copy()
    for det in detections:
        x1, y1 = int(det.bbox[0] * w), int(det.bbox[1] * h)
        x2, y2 = int(det.bbox[2] * w), int(det.bbox[3] * h)
        color = ((0,0,255) if det.distance < 0.30 else
                 (0,80,255) if det.distance < 1.0 else
                 (0,200,255) if det.distance < 2.0 else (0,255,180))
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        tid = f" T{det.track_id}" if det.track_id is not None else ""
        cv2.putText(out, f"{det.label}{tid} {det.distance:.2f}m",
                    (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    if zone == "STOP":
        cv2.rectangle(out, (0, 0), (w, 30), (0, 0, 180), -1)
        cv2.putText(out, "!! EMERGENCY STOP !!", (w//2-120, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
    else:
        zc = {"CLOSE":(0,80,255),"CAUTION":(0,200,255),"CLEAR":(0,255,180)}.get(zone,(200,200,200))
        cv2.putText(out, f"{fps:.0f} FPS | {zone}", (8,20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, zc, 1, cv2.LINE_AA)
    return out

def colorize_depth(depth_frame):
    if depth_frame is None:
        return np.zeros((STREAM_H, STREAM_W, 3), dtype=np.uint8)
    normed = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return cv2.applyColorMap(normed, cv2.COLORMAP_MAGMA)


DASHBOARD_HTML = r"""
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>BARQ Dashboard</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;600;800&display=swap');
  :root{--bg:#07080a;--surface:#0d1117;--border:#1a1f2b;--text:#b0bec5;--text-dim:#4a5568;--cyan:#00f0ff;--magenta:#ff00aa;--orange:#ff6a00;--red:#ff2244;--green:#00ff88;--yellow:#ffcc00}
  *{margin:0;padding:0;box-sizing:border-box}
  body{background:var(--bg);color:var(--text);font-family:'JetBrains Mono',monospace;overflow:hidden;height:100vh}
  body::after{content:'';position:fixed;inset:0;background:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,240,255,0.015) 2px,rgba(0,240,255,0.015) 4px);pointer-events:none;z-index:9999}
  .header{height:48px;background:var(--surface);border-bottom:1px solid var(--border);display:flex;align-items:center;padding:0 20px;gap:14px}
  .logo{font-size:15px;font-weight:800;letter-spacing:3px;color:var(--cyan);text-shadow:0 0 20px rgba(0,240,255,0.4)}
  .sep{color:var(--text-dim)}
  .subtitle{font-size:11px;font-weight:300;color:var(--text-dim);letter-spacing:2px}
  .badges{margin-left:auto;display:flex;gap:8px}
  .badge{font-size:10px;font-weight:600;padding:3px 10px;border-radius:3px;letter-spacing:1px}
  .badge-d{background:rgba(0,240,255,0.1);color:var(--cyan);border:1px solid rgba(0,240,255,0.2)}
  .badge-m{background:rgba(255,0,170,0.1);color:var(--magenta);border:1px solid rgba(255,0,170,0.2)}
  .zone-bar{height:4px;transition:background .3s}
  .zone-CLEAR .zone-bar{background:var(--green);box-shadow:0 0 12px var(--green)}
  .zone-CAUTION .zone-bar{background:var(--yellow);box-shadow:0 0 12px var(--yellow)}
  .zone-CLOSE .zone-bar{background:var(--orange);box-shadow:0 0 12px var(--orange)}
  .zone-STOP .zone-bar{background:var(--red);box-shadow:0 0 20px var(--red);animation:ps .5s infinite alternate}
  @keyframes ps{to{opacity:.4}}
  .main{display:grid;grid-template-columns:1fr 300px;height:calc(100vh - 52px)}
  .streams{display:flex;flex-direction:column;padding:10px;gap:8px;min-width:0}
  .sw{flex:1;background:var(--surface);border:1px solid var(--border);border-radius:6px;overflow:hidden;display:flex;flex-direction:column;position:relative}
  .sl{position:absolute;top:8px;left:10px;font-size:10px;font-weight:600;letter-spacing:2px;color:var(--text-dim);background:rgba(7,8,10,0.7);padding:2px 8px;border-radius:3px;z-index:2}
  .sw img{width:100%;flex:1;object-fit:contain;background:#000}
  .sidebar{background:var(--surface);border-left:1px solid var(--border);padding:12px;display:flex;flex-direction:column;gap:10px;overflow-y:auto}
  .card{background:rgba(13,17,23,0.6);border:1px solid var(--border);border-radius:6px;padding:12px}
  .ct{font-size:10px;font-weight:600;letter-spacing:2px;color:var(--text-dim);margin-bottom:10px}
  .zd{text-align:center;padding:14px 12px;border-radius:6px;border:1px solid var(--border);transition:all .3s}
  .zd .zl{font-size:22px;font-weight:800;letter-spacing:4px}
  .zd .zm{font-size:12px;font-weight:300;margin-top:4px;color:var(--text-dim)}
  .zone-CLEAR .zd{border-color:var(--green);color:var(--green);box-shadow:0 0 20px rgba(0,255,136,0.1)}
  .zone-CAUTION .zd{border-color:var(--yellow);color:var(--yellow);box-shadow:0 0 20px rgba(255,204,0,0.1)}
  .zone-CLOSE .zd{border-color:var(--orange);color:var(--orange);box-shadow:0 0 20px rgba(255,106,0,0.15)}
  .zone-STOP .zd{border-color:var(--red);color:var(--red);box-shadow:0 0 30px rgba(255,34,68,0.25);animation:gs .5s infinite alternate}
  @keyframes gs{to{box-shadow:0 0 50px rgba(255,34,68,0.5)}}
  .sr{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid rgba(26,31,43,0.5);font-size:12px}
  .sr:last-child{border-bottom:none}
  .sk{color:var(--text-dim)}.sv{color:var(--cyan);font-weight:600}
  .rw{position:relative;width:100%;padding-top:100%;border-radius:6px;overflow:hidden;background:radial-gradient(circle at 50% 100%,rgba(0,240,255,0.04) 0%,transparent 70%);border:1px solid var(--border)}
  .rc{position:absolute;inset:0;width:100%;height:100%}
  .di{display:flex;justify-content:space-between;align-items:center;padding:5px 0;border-bottom:1px solid rgba(26,31,43,0.4);font-size:11px}
  .di:last-child{border-bottom:none}
  .dn{color:var(--orange);font-weight:600}
  .dd{font-weight:600}
  .dc1{color:var(--red)}.dc2{color:var(--orange)}.dc3{color:var(--yellow)}.dc4{color:var(--green)}
  .db{color:var(--text-dim);font-size:10px}
  .es{color:var(--text-dim);font-size:11px;text-align:center;padding:10px 0}
</style>
</head>
<body class="zone-CLEAR">
<div class="header"><span class="logo">ARYA</span><span class="sep">//</span><span class="subtitle">PERCEPTION SYSTEM</span><div class="badges"><span class="badge badge-d">OAK-D PRO</span><span class="badge badge-m">SPATIAL NN</span></div></div>
<div class="zone-bar"></div>
<div class="main">
  <div class="streams">
    <div class="sw"><span class="sl">RGB + DETECTIONS</span><img src="/stream/rgb" /></div>
    <div class="sw"><span class="sl">DEPTH MAP</span><img src="/stream/depth" /></div>
  </div>
  <div class="sidebar">
    <div class="zd"><div class="zl" id="zl">CLEAR</div><div class="zm" id="zm">No threats</div></div>
    <div class="card"><div class="ct">SYSTEM</div>
      <div class="sr"><span class="sk">FPS</span><span class="sv" id="fps">--</span></div>
      <div class="sr"><span class="sk">Loop</span><span class="sv" id="lms">--</span></div>
      <div class="sr"><span class="sk">Objects</span><span class="sv" id="oc">0</span></div>
      <div class="sr"><span class="sk">Threats</span><span class="sv" id="tc">0</span></div>
    </div>
    <div class="card"><div class="ct">PROXIMITY RADAR</div><div class="rw"><canvas class="rc" id="radar"></canvas></div></div>
    <div class="card" style="flex:1;overflow-y:auto"><div class="ct">TRACKED OBJECTS</div><div id="dl"><div class="es">Scanning...</div></div></div>
  </div>
</div>
<script>
const radar=document.getElementById('radar'),rctx=radar.getContext('2d');
function resizeR(){const r=radar.parentElement.getBoundingClientRect();radar.width=r.width*devicePixelRatio;radar.height=r.height*devicePixelRatio;rctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0)}
resizeR();window.addEventListener('resize',resizeR);
function drawR(dets){const w=radar.width/devicePixelRatio,h=radar.height/devicePixelRatio,cx=w/2,cy=h-10,s=(h-20)/3;rctx.clearRect(0,0,w,h);
[0.3,1,2,3].forEach((r,i)=>{const p=r*s;rctx.beginPath();rctx.arc(cx,cy,p,Math.PI,2*Math.PI);rctx.strokeStyle=i===0?'rgba(255,34,68,0.3)':i===1?'rgba(255,106,0,0.2)':'rgba(0,240,255,0.1)';rctx.lineWidth=i===0?1.5:0.8;rctx.stroke();rctx.fillStyle='rgba(74,85,104,0.6)';rctx.font='9px JetBrains Mono';rctx.fillText(r+'m',cx+p-16,cy-4)});
rctx.beginPath();rctx.moveTo(cx,cy);rctx.lineTo(cx-(h-20)*0.84,10);rctx.moveTo(cx,cy);rctx.lineTo(cx+(h-20)*0.84,10);rctx.strokeStyle='rgba(0,240,255,0.08)';rctx.lineWidth=0.5;rctx.stroke();
rctx.beginPath();rctx.moveTo(cx,cy-6);rctx.lineTo(cx-4,cy+2);rctx.lineTo(cx+4,cy+2);rctx.closePath();rctx.fillStyle='rgba(0,240,255,0.7)';rctx.fill();
dets.forEach(d=>{const px=cx+d.spatial[0]*s,py=cy-d.spatial[2]*s,c=d.distance<0.3?'#ff2244':d.distance<1?'#ff6a00':d.distance<2?'#ffcc00':'#00ff88';const g=rctx.createRadialGradient(px,py,0,px,py,8);g.addColorStop(0,c+'44');g.addColorStop(1,c+'00');rctx.beginPath();rctx.arc(px,py,8,0,Math.PI*2);rctx.fillStyle=g;rctx.fill();rctx.beginPath();rctx.arc(px,py,3,0,Math.PI*2);rctx.fillStyle=c;rctx.fill();rctx.fillStyle=c;rctx.font='9px JetBrains Mono';rctx.fillText(`${d.label}${d.track_id!==null?'T'+d.track_id:''}`,px+6,py-4)})}
function dc(d){return d<0.3?'dc1':d<1?'dc2':d<2?'dc3':'dc4'}
async function poll(){try{const r=await fetch('/api/state'),d=await r.json();
document.getElementById('fps').textContent=d.fps.toFixed(1);document.getElementById('lms').textContent=d.loop_ms.toFixed(1)+' ms';document.getElementById('oc').textContent=d.total;document.getElementById('tc').textContent=d.threats;
document.body.className='zone-'+d.zone;document.getElementById('zl').textContent=d.zone;document.getElementById('zm').textContent=d.nearest_dist!==null?d.nearest_dist.toFixed(2)+'m nearest':'No threats';
const l=document.getElementById('dl');if(!d.detections.length){l.innerHTML='<div class="es">No objects detected</div>'}else{l.innerHTML=d.detections.map(e=>{const t=e.track_id!==null?' T'+e.track_id:'',s=e.status?' · '+e.status:'';return`<div class="di"><span class="dn">${e.label}${t}</span><div><span class="dd ${dc(e.distance)}">${e.distance.toFixed(2)}m</span> <span class="db">${e.bearing.toFixed(0)}°${s}</span></div></div>`}).join('')}
drawR(d.detections)}catch(e){}setTimeout(poll,150)}poll();
</script>
</body></html>
"""


# ── Flask ───────────────────────────────────────────────────
_state: TelemetryState = None

def _create_app():
    app = Flask(__name__)

    @app.route('/')
    def index():
        return render_template_string(DASHBOARD_HTML)

    @app.route('/stream/rgb')
    def stream_rgb():
        def gen():
            while True:
                try:
                    s = _state.snapshot()
                    rgb, _, dets, fps, _, zone = s[0], s[1], s[2], s[3], s[4], s[5]
                    frame = annotate_rgb(rgb, dets, fps, zone)
                    frame = cv2.resize(frame, (STREAM_W, STREAM_H))
                    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    if ok:
                        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                               + buf.tobytes() + b'\r\n')
                except Exception as e:
                    print(f"[STREAM RGB ERR] {e}")
                time.sleep(0.05)
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/stream/depth')
    def stream_depth():
        def gen():
            while True:
                try:
                    s = _state.snapshot()
                    depth = s[1]
                    frame = colorize_depth(depth)
                    frame = cv2.resize(frame, (STREAM_W, STREAM_H))
                    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    if ok:
                        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                               + buf.tobytes() + b'\r\n')
                except Exception as e:
                    print(f"[STREAM DEPTH ERR] {e}")
                time.sleep(0.05)
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/api/state')
    def api_state():
        s = _state.snapshot()
        dets, fps, loop_ms, zone, stop, nearest = s[2], s[3], s[4], s[5], s[6], s[7]
        threats = [d for d in dets if d.is_nav_relevant and d.distance <= 3.0]
        return json.dumps({
            "fps": fps, "loop_ms": loop_ms, "zone": zone,
            "stop_flag": stop, "nearest_dist": nearest,
            "total": len(dets), "threats": len(threats),
            "detections": [{
                "label":d.label, "track_id":d.track_id,
                "distance":round(d.distance,3), "bearing":round(d.bearing_deg,1),
                "confidence":round(d.confidence,2), "status":d.status,
                "spatial":[round(d.spatial_x,3),round(d.spatial_y,3),round(d.spatial_z,3)],
            } for d in dets],
        }), 200, {'Content-Type':'application/json'}

    return app


def start_server(state: TelemetryState, host="0.0.0.0", port=5000):
    global _state
    _state = state
    app = _create_app()
    t = threading.Thread(target=lambda: app.run(host=host, port=port, threaded=True, use_reloader=False), daemon=True)
    t.start()
    print(f"[TELEMETRY] Dashboard at http://{host}:{port}")
    return t