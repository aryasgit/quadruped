"""
tools/stability_logger.py — BARQ Stability Mode Data Logger
============================================================

Records per-frame IMU roll/pitch and per-leg dZ corrections while
stability mode is running. Produces two output files:

  <session_dir>/roll_pitch.json   — compact frame array for the website widget
  <session_dir>/roll_pitch.csv    — human-readable, useful for debugging

Usage
-----
Instantiate before calling run_stability_mode, pass as `logger=`:

    from tools.stability_logger import StabilityLogger
    import controllers.stability_controller as _sc

    logger = StabilityLogger(output_dir="logs")
    run_stability_mode(imu, use_gamepad, controller, current_z, logger=logger)
    logger.save()   # call after run_stability_mode returns

JSON schema (roll_pitch.json)
------------------------------
{
  "meta": {
    "session":    "2024-11-15T14:32:01",
    "dt_nominal": 0.02,
    "count":      1847,
    "stance_z":   -0.18,
    "duration_s": 36.94
  },
  "frames": [
    {"t": 0.000, "roll": 0.12, "pitch": -0.04,
     "dz_fl": 0.0, "dz_fr": 0.0, "dz_rl": 0.0, "dz_rr": 0.0},
    ...
  ]
}

The website widget only needs "frames". "meta" is for diagnostics.

Notes
-----
- StabilityLogger is deliberately import-free from robot hardware.
  It only reads plain floats passed to record(), so it is safe to import
  on any machine (dev laptop, CI, etc.) without triggering hardware init.
- imu.roll / imu.pitch are read *after* posture_step() has ticked the
  filter, so the values are always current-frame corrected.
- _sc._dz is read directly via the module reference passed at construction
  time, so the snapshot is always the value that was actually sent to
  the servos for that frame.
"""

import json
import csv
import os
import math
import time
from datetime import datetime


class StabilityLogger:
    """
    Frame-by-frame recorder for stability mode IMU + correction data.

    Parameters
    ----------
    output_dir : str
        Directory to write output files into. Created if it does not exist.
    session_tag : str | None
        Optional suffix for the session directory name. Defaults to ISO
        timestamp. Use a short string like "trot_test_1" if you want
        human-friendly names.
    stance_z : float
        The stance Z value active when stability mode was entered.
        Stored in meta for reference only — not used in physics.
    sc_module : module
        The controllers.stability_controller module reference so the logger
        can snapshot _dz without an extra import chain. Pass as:
            import controllers.stability_controller as _sc
            logger = StabilityLogger(..., sc_module=_sc)
    """

    def __init__(
        self,
        output_dir: str = "logs",
        session_tag: str | None = None,
        stance_z: float = -0.18,
        sc_module=None,
    ):
        ts = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
        tag = f"_{session_tag}" if session_tag else ""
        self._session_id = f"{ts}{tag}"
        self._session_dir = os.path.join(output_dir, f"stability_{self._session_id}")
        os.makedirs(self._session_dir, exist_ok=True)

        self._stance_z = stance_z
        self._sc = sc_module          # controllers.stability_controller module ref

        self._frames: list[dict] = []
        self._t0: float | None = None

        self._json_path = os.path.join(self._session_dir, "roll_pitch.json")
        self._csv_path  = os.path.join(self._session_dir, "roll_pitch.csv")

        print(f"[LOGGER] Session: {self._session_id}")
        print(f"[LOGGER] Output:  {self._session_dir}/")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Call once at the very top of stability mode, before the loop."""
        self._t0 = time.time()
        self._frames.clear()
        print("[LOGGER] Recording started.")

    def record(self, imu) -> None:
        """
        Call once per stability tick, AFTER posture_step() returns.
        imu   : live IMUFilter instance — reads .roll and .pitch
        """
        if self._t0 is None:
            # start() was never called — auto-start silently
            self.start()

        t = time.time() - self._t0

        # --- Read IMU angles (degrees) ---
        # posture_step() ticks the filter before returning, so these are
        # always current-frame values.
        try:
            roll  = float(imu.roll)
            pitch = float(imu.pitch)
        except AttributeError:
            # Fallback: some IMUFilter implementations expose get_angles()
            try:
                roll, pitch = imu.get_angles()
                roll  = float(roll)
                pitch = float(pitch)
            except Exception:
                roll = pitch = float("nan")

        # --- Read per-leg dZ corrections (metres) ---
        dz = {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0}
        if self._sc is not None:
            try:
                raw = self._sc._dz   # dict keyed "FL","FR","RL","RR"
                dz["fl"] = float(raw.get("FL", 0.0))
                dz["fr"] = float(raw.get("FR", 0.0))
                dz["rl"] = float(raw.get("RL", 0.0))
                dz["rr"] = float(raw.get("RR", 0.0))
            except Exception:
                pass  # leave zeros — don't break the servo loop

        self._frames.append({
            "t":      round(t, 4),
            "roll":   round(roll,  3),
            "pitch":  round(pitch, 3),
            "dz_fl":  round(dz["fl"], 5),
            "dz_fr":  round(dz["fr"], 5),
            "dz_rl":  round(dz["rl"], 5),
            "dz_rr":  round(dz["rr"], 5),
        })

    def save(self) -> tuple[str, str]:
        """
        Write JSON and CSV files. Returns (json_path, csv_path).
        Safe to call even if no frames were recorded.
        """
        if not self._frames:
            print("[LOGGER] No frames to save.")
            return self._json_path, self._csv_path

        duration = self._frames[-1]["t"] if self._frames else 0.0

        # ---- JSON ----
        payload = {
            "meta": {
                "session":    self._session_id,
                "dt_nominal": 0.02,
                "count":      len(self._frames),
                "stance_z":   self._stance_z,
                "duration_s": round(duration, 3),
            },
            "frames": self._frames,
        }
        with open(self._json_path, "w") as f:
            json.dump(payload, f, separators=(",", ":"))

        # ---- CSV ----
        fieldnames = ["t", "roll", "pitch", "dz_fl", "dz_fr", "dz_rl", "dz_rr"]
        with open(self._csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(self._frames)

        print(
            f"[LOGGER] Saved {len(self._frames)} frames "
            f"({duration:.1f}s) → {self._session_dir}/"
        )
        return self._json_path, self._csv_path

    # ------------------------------------------------------------------
    # Convenience properties
    # ------------------------------------------------------------------

    @property
    def frame_count(self) -> int:
        return len(self._frames)

    @property
    def session_dir(self) -> str:
        return self._session_dir

    @property
    def json_path(self) -> str:
        return self._json_path