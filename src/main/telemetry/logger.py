"""
L4.5 — TELEMETRY LOGGER
=======================

Structured per-tick logging to JSONL for offline analysis. This is the recorder
behind "IMU as the debugging eyes": every run captures command, phase, IMU
state, and loop timing so we can analyze tilt / cadence / yaw-drift / onset and
correlate with Krish's ground-truth observations.

Cheap and non-blocking-ish (buffered append). Never affects control logic.
"""

import json
import os
import time


class TelemetryLogger:
    def __init__(self, run_name="run", log_dir=None):
        base = log_dir or os.path.join(os.path.dirname(__file__), "..", "logs")
        os.makedirs(base, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.abspath(os.path.join(base, f"{run_name}_{stamp}.jsonl"))
        self._f = open(self.path, "w", buffering=1)
        self._t0 = time.monotonic()

    def log(self, record: dict):
        record["t"] = time.monotonic() - self._t0
        self._f.write(json.dumps(record) + "\n")

    def log_tick(self, phase, command, state, loop_dt, note=None):
        rec = {
            "phase": round(phase, 5),
            "cmd": {"fwd": command.fwd, "strafe": command.strafe,
                    "turn": command.turn, "height": command.height},
            "loop_dt": round(loop_dt, 5),
        }
        if state is not None:
            rec["imu"] = {
                "roll": round(state.roll, 3), "pitch": round(state.pitch, 3),
                "yaw_rate": round(state.yaw_rate, 3),
                "ax": round(state.ax, 4), "ay": round(state.ay, 4),
                "az": round(state.az, 4), "imu_dt": round(state.dt, 5),
            }
        if note:
            rec["note"] = note
        self.log(rec)

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass
