"""
Telemetry auto-launcher
=======================

Detects which cameras are plugged in and starts the matching telemetry dashboard
as a child process. Called by web_control.main() so the right console comes up
automatically alongside the robot controller.

  OAK-D (Luxonis / Movidius, USB vendor 03e7)  ->  vision.oak_dashboard  :8080
  Generic UVC camera (/dev/video*)             ->  vision.ir_dashboard   :8081

Lightweight: this module only uses the standard library (no depthai/cv2), so it's
always safe to import. The heavy deps live in the dashboard modules, which run in
their own processes — a camera dep problem can't take down the controller.
"""

import atexit
import glob
import subprocess
import sys

_procs = []


def oak_present() -> bool:
    """True if a Luxonis/Movidius device (vendor 03e7) is on USB."""
    for f in glob.glob("/sys/bus/usb/devices/*/idVendor"):
        try:
            if open(f).read().strip().lower() == "03e7":
                return True
        except OSError:
            pass
    return False


def uvc_present() -> bool:
    """True if a generic V4L2 (UVC) camera node exists (the OAK does not create one)."""
    return len(glob.glob("/dev/video*")) > 0


def _spawn(module, name, port, out):
    try:
        p = subprocess.Popen(
            [sys.executable, "-m", module],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        _procs.append(p)
        out.append((name, port))
        print(f"[TELEMETRY] {name} dashboard launched -> port {port} (pid {p.pid})")
    except Exception as e:
        print(f"[TELEMETRY] failed to launch {name}: {e}")


def _terminate():
    for p in _procs:
        try:
            p.terminate()
        except Exception:
            pass


def launch_dashboards():
    """
    Detect cameras and launch the matching dashboards. Returns a list of
    (name, port) that were started. Registers cleanup so they stop with us.
    """
    launched = []
    if oak_present():
        _spawn("vision.oak_dashboard", "OAK-D", 8080, launched)
    if uvc_present():
        _spawn("vision.ir_dashboard", "IR-CAM", 8081, launched)
    if not launched:
        print("[TELEMETRY] no camera detected — no telemetry dashboard launched")
    else:
        atexit.register(_terminate)
    return launched
