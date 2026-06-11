#!/usr/bin/env python3
"""
BARQ v1 — Servo calibration GUI (web)
=====================================

Browser-based replacement for the legacy Tkinter servo_gui.py, built for
the headless Jetson: run it here, open it from the Mac.

    # on the Jetson
    ~/barq_v1/venv/bin/python stack/tools/calibration_gui.py
    # on the Mac
    open http://barq.local:8035

Calibration method follows the spotMicro research
(reference/spotMicro/docs/servo_calibration.md): per servo, command pulse
values at two known link angles (0 deg = TRUE ZERO, plus one reference
angle, e.g. +/-90 for links 2-3 and +/-45 for link 1) and fit the linear
map ticks = zero_ticks + slope * angle_deg.

SAFETY
- On start, ALL outputs are off — nothing moves until you enable a servo.
- Enabling a servo snaps it to the slider value. Keep the robot on a
  stand with legs hanging free.
- ESC in the browser, or the red ALL OFF button, kills every output.
- Commands are clamped to each servo's measured mechanical window
  (truths) unless you explicitly unlock the full electrical range.

If the PCA9685 is not reachable (board unpowered), the GUI starts in
SIMULATION mode and says so loudly.
"""

import argparse
import datetime
import sys
import threading
import time
from pathlib import Path

STACK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STACK_ROOT))  # zero-install import of barq1

import yaml
from flask import Flask, jsonify, request, send_from_directory

from barq1 import truths
from barq1.pca9685 import PCA9685, ticks_to_legacy_deg, ticks_to_us
from barq1.servos import ORDER, SERVOS

CALIB_PATH = STACK_ROOT / "config" / "servo_calibration.yaml"

app = Flask(__name__, static_folder=str(Path(__file__).parent / "static"))

LOCK = threading.Lock()
PCA: PCA9685 = None  # set in main()

# Runtime state per servo (guarded by LOCK)
RT = {
    name: {
        "ticks": SERVOS[name].stand,  # slider position; NOT commanded until enabled
        "enabled": False,
        "unlocked": False,            # True -> allow full electrical range
        "points": [],                 # [{"angle_deg": float, "ticks": int}, ...]
    }
    for name in ORDER
}


# ---------------------------------------------------------------------------
# Calibration math (spotMicro 2-point / least-squares linear fit)
# ---------------------------------------------------------------------------

def _fit(points):
    """Least-squares ticks = zero_ticks + slope*angle. None if < 2 points."""
    if len(points) < 2:
        return None
    xs = [p["angle_deg"] for p in points]
    ys = [p["ticks"] for p in points]
    n = len(points)
    mx, my = sum(xs) / n, sum(ys) / n
    den = sum((x - mx) ** 2 for x in xs)
    if den == 0:
        return None
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den
    zero = my - slope * mx
    return {
        "zero_ticks": round(zero, 1),
        "slope_ticks_per_deg": round(slope, 4),
        "slope_us_per_deg": round(ticks_to_us(slope), 3),
        "direction": 1 if slope >= 0 else -1,
    }


def _bounds(name):
    spec, rt = SERVOS[name], RT[name]
    if rt["unlocked"]:
        return truths.PULSE_MIN, truths.PULSE_MAX
    return spec.mech_lo, spec.mech_hi


def _servo_state(name):
    spec, rt = SERVOS[name], RT[name]
    lo, hi = _bounds(name)
    t = rt["ticks"]
    return {
        **spec.as_dict(),
        "lo": lo,
        "hi": hi,
        "ticks": t,
        "us": round(ticks_to_us(t), 1),
        "legacy_deg": round(ticks_to_legacy_deg(t), 1),
        "enabled": rt["enabled"],
        "unlocked": rt["unlocked"],
        "points": rt["points"],
        "fit": _fit(rt["points"]),
    }


def _full_state():
    return {
        "mode": "simulation" if PCA.sim else "hardware",
        "bus": truths.I2C_BUS,
        "addr": f"0x{truths.PCA_ADDR:02x}",
        "freq_hz": truths.PWM_FREQ_HZ,
        "pulse_min": truths.PULSE_MIN,
        "pulse_max": truths.PULSE_MAX,
        "calib_file": str(CALIB_PATH),
        "calib_file_exists": CALIB_PATH.exists(),
        "servos": [_servo_state(n) for n in ORDER],
    }


def _command(name, ticks):
    """Clamp to current bounds, write to hardware, update runtime state."""
    spec, rt = SERVOS[name], RT[name]
    lo, hi = _bounds(name)
    ticks = max(lo, min(hi, int(round(ticks))))
    written = PCA.set_ticks(spec.channel, ticks)
    rt["ticks"] = written
    rt["enabled"] = True
    return written


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.get("/api/state")
def api_state():
    with LOCK:
        return jsonify(_full_state())


@app.post("/api/servo/<name>/command")
def api_command(name):
    ticks = request.get_json(force=True)["ticks"]
    with LOCK:
        if name not in SERVOS:
            return jsonify({"error": f"unknown servo {name}"}), 404
        _command(name, ticks)
        return jsonify(_servo_state(name))


@app.post("/api/servo/<name>/off")
def api_off(name):
    with LOCK:
        PCA.off(SERVOS[name].channel)
        RT[name]["enabled"] = False
        return jsonify(_servo_state(name))


@app.post("/api/servo/<name>/unlock")
def api_unlock(name):
    unlocked = bool(request.get_json(force=True)["unlocked"])
    with LOCK:
        rt = RT[name]
        rt["unlocked"] = unlocked
        lo, hi = _bounds(name)
        rt["ticks"] = max(lo, min(hi, rt["ticks"]))
        if rt["enabled"]:  # re-clamp the live output too
            _command(name, rt["ticks"])
        return jsonify(_servo_state(name))


@app.post("/api/servo/<name>/goto")
def api_goto(name):
    target = request.get_json(force=True)["target"]
    with LOCK:
        spec, rt = SERVOS[name], RT[name]
        if target == "zero":
            fit = _fit(rt["points"])
            if not fit:
                return jsonify({"error": "no zero recorded yet"}), 400
            ticks = fit["zero_ticks"]
        elif target in ("perp", "stand"):
            ticks = getattr(spec, target)
        else:
            return jsonify({"error": f"unknown target {target}"}), 400
        _command(name, ticks)
        return jsonify(_servo_state(name))


@app.post("/api/servo/<name>/record")
def api_record(name):
    angle = float(request.get_json(force=True)["angle_deg"])
    with LOCK:
        rt = RT[name]
        if not rt["enabled"]:
            return jsonify({"error": "servo is off — enable it at the pose first"}), 400
        rt["points"] = [p for p in rt["points"] if p["angle_deg"] != angle]
        rt["points"].append({"angle_deg": angle, "ticks": rt["ticks"]})
        rt["points"].sort(key=lambda p: p["angle_deg"])
        return jsonify(_servo_state(name))


@app.post("/api/servo/<name>/clear_points")
def api_clear_points(name):
    with LOCK:
        RT[name]["points"] = []
        return jsonify(_servo_state(name))


@app.post("/api/all_off")
def api_all_off():
    with LOCK:
        PCA.all_off()
        for rt in RT.values():
            rt["enabled"] = False
        return jsonify(_full_state())


@app.post("/api/pose")
def api_pose():
    pose = request.get_json(force=True)["pose"]
    if pose not in ("perp", "stand"):
        return jsonify({"error": f"unknown pose {pose}"}), 400
    with LOCK:
        for name in ORDER:  # staggered to soften the current inrush
            rt = RT[name]
            rt["ticks"] = getattr(SERVOS[name], pose)
            _command(name, rt["ticks"])
            time.sleep(0.12)
        return jsonify(_full_state())


@app.post("/api/save")
def api_save():
    with LOCK:
        doc = {
            "meta": {
                "generated": datetime.datetime.now().isoformat(timespec="seconds"),
                "tool": "stack/tools/calibration_gui.py",
                "units": "PCA9685 ticks @ 50 Hz (1 tick ~= 4.883 us)",
                "pulse_min": truths.PULSE_MIN,
                "pulse_max": truths.PULSE_MAX,
                "method": "spotMicro 2-point linear fit "
                          "(reference/spotMicro/docs/servo_calibration.md)",
            },
            "servos": {},
        }
        for name in ORDER:
            spec, rt = SERVOS[name], RT[name]
            entry = {"channel": spec.channel, "sm_id": spec.sm_id,
                     "points": rt["points"]}
            fit = _fit(rt["points"])
            if fit:
                entry.update(fit)
            doc["servos"][name] = entry
        CALIB_PATH.parent.mkdir(parents=True, exist_ok=True)
        CALIB_PATH.write_text(yaml.safe_dump(doc, sort_keys=False))
        return jsonify({"saved": str(CALIB_PATH), "state": _full_state()})


def _load_calibration():
    """Restore recorded points from a previous session, if any."""
    if not CALIB_PATH.exists():
        return
    doc = yaml.safe_load(CALIB_PATH.read_text()) or {}
    for name, entry in (doc.get("servos") or {}).items():
        if name in RT and entry.get("points"):
            RT[name]["points"] = [
                {"angle_deg": float(p["angle_deg"]), "ticks": int(p["ticks"])}
                for p in entry["points"]
            ]
    print(f"[calib] restored points from {CALIB_PATH}")


# ---------------------------------------------------------------------------

def main():
    global PCA
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--port", type=int, default=8035)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--bus", type=int, default=truths.I2C_BUS)
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=truths.PCA_ADDR)
    ap.add_argument("--sim", action="store_true",
                    help="force simulation (no I2C writes)")
    args = ap.parse_args()

    if args.sim:
        PCA = PCA9685(sim=True)
        print("[hw] SIMULATION mode (forced with --sim)")
    else:
        try:
            PCA = PCA9685(bus_num=args.bus, addr=args.addr)
            print(f"[hw] PCA9685 ready on bus {args.bus} addr 0x{args.addr:02x}, "
                  f"MODE1=0x{PCA.mode1():02x}, all outputs OFF")
        except Exception as e:  # board unpowered / unwired
            print(f"[hw] PCA9685 not reachable ({e}) -> SIMULATION mode")
            PCA = PCA9685(sim=True)

    _load_calibration()
    print(f"[gui] serving on http://0.0.0.0:{args.port}  "
          f"(from the Mac: http://barq.local:{args.port})")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
