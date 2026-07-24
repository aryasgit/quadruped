"""
OAK-D Pro — Live Telemetry Dashboard
====================================

Runs the OAK-D Pro pipeline (RGB + stereo depth + BNO086 IMU + system stats) in a
background thread and serves a live dashboard on localhost:

    http://<jetson-ip>:8080/

Streams:
    /rgb        MJPEG  — color camera
    /depth      MJPEG  — stereo depth (JET colormap)
    /telemetry  JSON   — IMU orientation/accel/gyro, chip temp, CPU/mem, FPS

Run:  python -m vision.oak_dashboard     (from src/main)

Each capture source is optional/guarded — if depth or IMU fails to start, RGB and
the rest keep working.
"""

import math
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import depthai as dai

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
import uvicorn

# MAX_PERF: one-off "everything on" mode (OAK_MAX=1). Default stays the cool v1 config.
MAX_PERF = os.environ.get("OAK_MAX", "0") == "1"
RGB_SIZE  = (1280, 720) if MAX_PERF else (640, 400)   # display/output size
MONO_SIZE = (1280, 800) if MAX_PERF else (640, 400)   # stereo input (depth detail)
DEPTH_OUT = (640, 400)                                 # depth stream size (keep encode sane)
TARGET_FPS = 30 if MAX_PERF else 15
DEPTH_RANGE_MM = 6000.0
JPEG_Q = [int(cv2.IMWRITE_JPEG_QUALITY), 70]


def _quat_to_euler(i, j, k, r):
    sinr = 2 * (r * i + j * k); cosr = 1 - 2 * (i * i + j * j)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = max(-1.0, min(1.0, 2 * (r * j - k * i)))
    pitch = math.degrees(math.asin(sinp))
    siny = 2 * (r * k + i * j); cosy = 1 - 2 * (j * j + k * k)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


class OakHub:
    def __init__(self):
        self._lock = threading.Lock()
        self._rgb_jpeg = None
        self._depth_jpeg = None
        self._tel = {
            "device": "-", "usb": "-",
            "rgb_fps": 0.0, "depth_fps": 0.0,
            "imu": {"roll": 0, "pitch": 0, "yaw": 0,
                    "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0},
            "sys": {"temp": 0, "cpu": 0, "mem_used": 0, "mem_total": 0},
            "depth": {"center_mm": 0, "min_mm": 0},
            "ok": False, "error": "",
        }
        self.running = False

    # ---- accessors ----
    def rgb_jpeg(self):
        with self._lock:
            return self._rgb_jpeg

    def depth_jpeg(self):
        with self._lock:
            return self._depth_jpeg

    def telemetry(self):
        with self._lock:
            return dict(self._tel)

    # ---- lifecycle ----
    def start(self):
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            self._pipeline_loop()
        except Exception as e:
            with self._lock:
                self._tel["ok"] = False
                self._tel["error"] = f"{type(e).__name__}: {e}"
            print(f"[OAK] pipeline error: {e}")

    def _pipeline_loop(self):
        with dai.Pipeline() as pipeline:
            device = pipeline.getDefaultDevice()
            with self._lock:
                self._tel["device"] = str(device.getProductName())
                try:
                    self._tel["usb"] = str(device.getUsbSpeed()).split(".")[-1]
                except Exception:
                    pass

            # --- RGB ---
            cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            rgb_out = cam.requestOutput(RGB_SIZE, dai.ImgFrame.Type.NV12, fps=TARGET_FPS)
            rgb_q = rgb_out.createOutputQueue()

            # --- Stereo depth (guarded) ---
            depth_q = None
            try:
                mono_l = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
                mono_r = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
                preset = (dai.node.StereoDepth.PresetMode.HIGH_DETAIL if MAX_PERF
                          else dai.node.StereoDepth.PresetMode.DEFAULT)
                stereo = pipeline.create(dai.node.StereoDepth).build(
                    left=mono_l.requestOutput(MONO_SIZE, fps=TARGET_FPS),
                    right=mono_r.requestOutput(MONO_SIZE, fps=TARGET_FPS),
                    presetMode=preset,
                )
                stereo.setOutputSize(*DEPTH_OUT)
                if MAX_PERF:
                    # everything on: high-res mono in, subpixel + LR check, and a
                    # density-oriented post-processing chain.
                    try: stereo.setLeftRightCheck(True)
                    except Exception: pass
                    try: stereo.setSubpixel(True)
                    except Exception: pass
                    try:
                        pp = stereo.initialConfig.postProcessing
                        pp.speckleFilter.enable = True
                        pp.temporalFilter.enable = True
                        pp.spatialFilter.enable = True
                        pp.spatialFilter.holeFillingRadius = 2
                        pp.spatialFilter.numIterations = 1
                        try: pp.holeFilling.enable = True
                        except Exception: pass
                    except Exception as e:
                        print(f"[OAK] max post-processing skipped: {e}")
                depth_q = stereo.depth.createOutputQueue()
            except Exception as e:
                print(f"[OAK] depth disabled: {e}")

            # --- IMU (guarded) ---
            imu_q = None
            try:
                imu = pipeline.create(dai.node.IMU)
                imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, 100)
                imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, 100)
                imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
                imu.setBatchReportThreshold(1)
                imu.setMaxBatchReports(10)
                imu_q = imu.out.createOutputQueue()
            except Exception as e:
                print(f"[OAK] imu disabled: {e}")

            # --- System logger (guarded) ---
            sys_q = None
            try:
                slog = pipeline.create(dai.node.SystemLogger)
                slog.setRate(1)
                sys_q = slog.out.createOutputQueue()
            except Exception as e:
                print(f"[OAK] system logger disabled: {e}")

            pipeline.start()
            try:
                if MAX_PERF:
                    # full IR dot projector — dense depth on textureless surfaces
                    device.setIrLaserDotProjectorIntensity(1.0)
                    device.setIrFloodLightIntensity(0.3)
                else:
                    device.setIrLaserDotProjectorIntensity(0.0)
                    device.setIrFloodLightIntensity(0.0)
            except Exception:
                pass
            with self._lock:
                self._tel["ok"] = True

            rgb_t, rgb_n = time.time(), 0
            dep_t, dep_n = time.time(), 0

            while self.running and pipeline.isRunning():
                # RGB
                f = rgb_q.tryGet()
                if f is not None:
                    frame = f.getCvFrame()
                    ok, buf = cv2.imencode(".jpg", frame, JPEG_Q)
                    if ok:
                        with self._lock:
                            self._rgb_jpeg = buf.tobytes()
                    rgb_n += 1
                    if time.time() - rgb_t >= 1.0:
                        with self._lock:
                            self._tel["rgb_fps"] = round(rgb_n / (time.time() - rgb_t), 1)
                        rgb_t, rgb_n = time.time(), 0

                # Depth
                if depth_q is not None:
                    d = depth_q.tryGet()
                    if d is not None:
                        raw = d.getFrame().astype(np.float32)
                        h, w = raw.shape
                        center = float(raw[h // 2, w // 2])
                        nz = raw[raw > 0]
                        norm = np.clip(raw, 0, DEPTH_RANGE_MM) / DEPTH_RANGE_MM * 255.0
                        cmap = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_JET)
                        cmap[raw == 0] = 0
                        ok, buf = cv2.imencode(".jpg", cmap, JPEG_Q)
                        if ok:
                            with self._lock:
                                self._depth_jpeg = buf.tobytes()
                                self._tel["depth"]["center_mm"] = int(center)
                                self._tel["depth"]["min_mm"] = int(nz.min()) if nz.size else 0
                        dep_n += 1
                        if time.time() - dep_t >= 1.0:
                            with self._lock:
                                self._tel["depth_fps"] = round(dep_n / (time.time() - dep_t), 1)
                            dep_t, dep_n = time.time(), 0

                # IMU
                if imu_q is not None:
                    pk = imu_q.tryGet()
                    if pk is not None:
                        for p in pk.packets:
                            a = p.acceleroMeter
                            g = p.gyroscope
                            rv = p.rotationVector
                            with self._lock:
                                im = self._tel["imu"]
                                if a is not None:
                                    im["ax"], im["ay"], im["az"] = round(a.x, 2), round(a.y, 2), round(a.z, 2)
                                if g is not None:
                                    im["gx"], im["gy"], im["gz"] = round(g.x, 2), round(g.y, 2), round(g.z, 2)
                                if rv is not None:
                                    r, p2, y = _quat_to_euler(rv.i, rv.j, rv.k, rv.real)
                                    im["roll"], im["pitch"], im["yaw"] = round(r, 1), round(p2, 1), round(y, 1)

                # System
                if sys_q is not None:
                    s = sys_q.tryGet()
                    if s is not None:
                        with self._lock:
                            sy = self._tel["sys"]
                            try:
                                sy["temp"] = round(s.chipTemperature.average, 1)
                            except Exception:
                                pass
                            try:
                                sy["cpu"] = round(s.leonCssCpuUsage.average * 100, 0)
                            except Exception:
                                pass
                            try:
                                sy["mem_used"] = int(s.ddrMemoryUsage.used / (1024 * 1024))
                                sy["mem_total"] = int(s.ddrMemoryUsage.total / (1024 * 1024))
                            except Exception:
                                pass

                time.sleep(0.002)


HUB = OakHub()
app = FastAPI()
_HTML = Path(__file__).with_name("dashboard.html")


def _mjpeg(getter):
    boundary = b"--frame"
    while True:
        data = getter()
        if data is None:
            time.sleep(0.05)
            continue
        yield boundary + b"\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
        time.sleep(0.03)


@app.get("/")
async def index():
    if _HTML.exists():
        return HTMLResponse(_HTML.read_text(encoding="utf-8"))
    return HTMLResponse("<h1>dashboard.html missing</h1>")


@app.get("/rgb")
async def rgb():
    return StreamingResponse(_mjpeg(HUB.rgb_jpeg),
                             media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/depth")
async def depth():
    return StreamingResponse(_mjpeg(HUB.depth_jpeg),
                             media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/telemetry")
async def telemetry():
    return JSONResponse(HUB.telemetry())


def main():
    HUB.start()
    print("[OAK] dashboard starting on http://0.0.0.0:8080")
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="warning", access_log=False)


if __name__ == "__main__":
    main()
