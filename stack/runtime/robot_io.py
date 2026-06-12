"""
Hardware runtime I/O: calibrated, slew-limited, kill-switchable.
================================================================

The one place that turns joint-angle dicts into PCA9685 writes. Safety
rails (all open-loop appropriate, D-009):

- engage(): first power-on of a pose — servos enabled one at a time,
  staggered, so 12 servos never snap simultaneously. Hand-place the robot
  near the target pose first (no feedback = no soft-start possible).
- apply(): every subsequent command is slew-rate limited per joint, so a
  bad target becomes a slow drift you can catch, not a jump.
- all_off(): drops every output — servos go limp. Wired to SIGINT and any
  exception in the runner.
"""

import json
import math
import sys
import time
from pathlib import Path

STACK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STACK_ROOT))

from barq1.pca9685 import PCA9685
from barq1.servo_map import ServoMap
from barq1.servos import SERVOS
from barq1.trajectories import DT

LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("coxa", "thigh", "wrist")


def named(leg_angles):
    """{leg:(q1,q2,q3)} -> {servo_name: rad}"""
    return {f"{leg}_{j}": q for leg, qs in leg_angles.items()
            for j, q in zip(JOINTS, qs)}


class RobotIO:
    def __init__(self, pca=None, smap=None, max_speed_dps=240.0):
        self.pca = pca or PCA9685()
        self.smap = smap or ServoMap.from_yaml()
        self.max_step_rad = math.radians(max_speed_dps) * DT
        self.current = None          # {servo_name: rad} last commanded
        if self.smap.warnings:
            print("[robot_io] calibration warnings:")
            for w in self.smap.warnings:
                print(f"  ! {w}")

    def engage(self, leg_angles, stagger_s=0.10):
        """Power the servos into a pose, one at a time. Robot ON THE STAND,
        hand-placed near this pose."""
        targets = named(leg_angles)
        for name in SERVOS:                      # deterministic order
            self.pca.set_ticks(SERVOS[name].channel,
                               self.smap.ticks_for(name, targets[name]))
            time.sleep(stagger_s)
        self.current = dict(targets)

    def apply(self, leg_angles):
        """Slew-limited command of all 12 joints. Returns the angles actually
        commanded this frame."""
        if self.current is None:
            raise RuntimeError("call engage() before apply() — servos are off "
                               "and their physical position is unknown")
        targets = named(leg_angles)
        for name, tgt in targets.items():
            cur = self.current[name]
            step = max(-self.max_step_rad, min(self.max_step_rad, tgt - cur))
            self.current[name] = cur + step
            self.pca.set_ticks(SERVOS[name].channel,
                               self.smap.ticks_for(name, self.current[name]))
        return dict(self.current)

    def all_off(self):
        self.pca.all_off()
        self.current = None


class Telemetry:
    """JSONL log: one record per command frame (poor-man's rosbag)."""

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = open(self.path, "w")
        self.t0 = time.monotonic()

    def record(self, frame_idx, angles, imu_rp=None, gyro=None, overruns=0):
        rec = {
            "t": round(time.monotonic() - self.t0, 4),
            "i": frame_idx,
            "q": {k: round(v, 4) for k, v in angles.items()},
            "overruns": overruns,
        }
        if imu_rp is not None:
            rec["imu_rp_deg"] = [round(math.degrees(v), 2) for v in imu_rp]
            rec["gyro_dps"] = [round(v, 1) for v in gyro]
        self._f.write(json.dumps(rec) + "\n")

    def close(self):
        self._f.close()
        print(f"[telemetry] {self.path}")
