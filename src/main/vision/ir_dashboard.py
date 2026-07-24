"""
IR Camera — Live Telemetry Console
==================================

Generic USB (UVC) infrared camera telemetry, served on localhost:8081.

    /            dashboard (black/green/red console)
    /feed        MJPEG  — raw IR
    /thermal     MJPEG  — colormapped (INFERNO) + hotspot crosshair
    /telemetry   JSON   — fps, resolution, mean/max intensity, hotspot, histogram

Run:  python -m vision.ir_dashboard     (from src/main)
"""

import threading
import time
from pathlib import Path

import cv2
import numpy as np

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
import uvicorn

DEVICE = 0
WIDTH, HEIGHT = 640, 480
JPEG_Q = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
HIST_BINS = 32


class IRHub:
    def __init__(self):
        self._lock = threading.Lock()
        self._raw = None
        self._thermal = None
        self._tel = {
            "res": "-", "fps": 0.0,
            "mean": 0, "max": 0,
            "hot": {"x": 0, "y": 0, "v": 0},
            "hist": [0] * HIST_BINS,
            "ok": False, "error": "",
        }
        self.running = False

    def raw_jpeg(self):
        with self._lock:
            return self._raw

    def thermal_jpeg(self):
        with self._lock:
            return self._thermal

    def telemetry(self):
        with self._lock:
            return dict(self._tel)

    def start(self):
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        if not cap.isOpened():
            with self._lock:
                self._tel["error"] = f"cannot open /dev/video{DEVICE}"
            print(f"[IR] cannot open /dev/video{DEVICE}")
            return

        t0, n = time.time(), 0
        while self.running:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            h, w = gray.shape

            # hotspot = brightest region (IR intensity peak)
            blur = cv2.GaussianBlur(gray, (0, 0), 3)
            _, maxv, _, maxloc = cv2.minMaxLoc(blur)

            # thermal colormap + green crosshair on the hotspot
            thermal = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
            cv2.drawMarker(thermal, maxloc, (94, 255, 51), cv2.MARKER_CROSS, 26, 2)
            cv2.circle(thermal, maxloc, 14, (94, 255, 51), 1)

            hist = cv2.calcHist([gray], [0], None, [HIST_BINS], [0, 256]).flatten()
            hist = (hist / max(hist.max(), 1) * 100).astype(int).tolist()

            ok1, b1 = cv2.imencode(".jpg", frame, JPEG_Q)
            ok2, b2 = cv2.imencode(".jpg", thermal, JPEG_Q)

            n += 1
            fps = None
            if time.time() - t0 >= 1.0:
                fps = round(n / (time.time() - t0), 1)
                t0, n = time.time(), 0

            with self._lock:
                if ok1:
                    self._raw = b1.tobytes()
                if ok2:
                    self._thermal = b2.tobytes()
                self._tel["ok"] = True
                self._tel["res"] = f"{w}x{h}"
                self._tel["mean"] = int(gray.mean())
                self._tel["max"] = int(maxv)
                self._tel["hot"] = {"x": int(maxloc[0]), "y": int(maxloc[1]), "v": int(maxv)}
                self._tel["hist"] = hist
                if fps is not None:
                    self._tel["fps"] = fps

            time.sleep(0.002)


HUB = IRHub()
app = FastAPI()
_HTML = Path(__file__).with_name("ir_dashboard.html")


def _mjpeg(getter):
    while True:
        data = getter()
        if data is None:
            time.sleep(0.05)
            continue
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
        time.sleep(0.03)


@app.get("/")
async def index():
    if _HTML.exists():
        return HTMLResponse(_HTML.read_text(encoding="utf-8"))
    return HTMLResponse("<h1>ir_dashboard.html missing</h1>")


@app.get("/feed")
async def feed():
    return StreamingResponse(_mjpeg(HUB.raw_jpeg),
                             media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/thermal")
async def thermal():
    return StreamingResponse(_mjpeg(HUB.thermal_jpeg),
                             media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/telemetry")
async def telemetry():
    return JSONResponse(HUB.telemetry())


def main():
    HUB.start()
    print("[IR] dashboard starting on http://0.0.0.0:8081")
    uvicorn.run(app, host="0.0.0.0", port=8081, log_level="warning", access_log=False)


if __name__ == "__main__":
    main()
