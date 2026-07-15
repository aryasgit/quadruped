"""
L3.5 — IMU STATE ESTIMATION
===========================

Turns raw MPU6050 samples into usable state:
  * roll, pitch   (deg) via a complementary filter with MEASURED monotonic dt
    (the old stack hardcoded dt=0.02, biasing gyro integration).
  * yaw_rate      (deg/s) — gyro Z; integrate only over short windows (no mag,
    absolute heading drifts).
  * accel         (g) — for cadence / impact / lean analysis.

This is the sensing side of "IMU as the debugging eyes". It never commands the
robot; it only observes. Reads are guarded/locked at the bus layer.

Honest limits: tilt is reliable; yaw is relative/short-term; absolute forward
velocity/distance is NOT recoverable here (needs Krish's ground-truth).
"""

import math
import time
from dataclasses import dataclass

from hardware import imu

_ALPHA = 0.96          # complementary filter weight on gyro
_G = 9.80665


@dataclass
class RobotState:
    roll: float = 0.0        # deg
    pitch: float = 0.0       # deg
    yaw_rate: float = 0.0    # deg/s (gyro z)
    ax: float = 0.0          # g
    ay: float = 0.0
    az: float = 1.0
    dt: float = 0.0          # s, measured


@dataclass
class _Bias:
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0


class IMUEstimator:
    def __init__(self):
        self._bias = _Bias()
        self.roll = 0.0
        self.pitch = 0.0
        self._t_last = None

    def calibrate(self, samples=200):
        """
        Average gyro (should read ~0) and accel (retain gravity on Z).
        NOTE: assumes the robot is FLAT and STILL — a tilted calibration bakes
        the tilt in as 'level'. We surface that assumption to the operator.
        """
        imu.init()
        ax = ay = az = gx = gy = gz = 0.0
        n = 0
        for _ in range(samples):
            try:
                (a0, a1, a2), (g0, g1, g2) = imu.read()
            except OSError:
                continue
            ax += a0; ay += a1; az += a2
            gx += g0; gy += g1; gz += g2
            n += 1
            time.sleep(0.002)
        n = max(n, 1)
        self._bias = _Bias(
            ax=ax / n, ay=ay / n, az=(az / n) - 1.0,   # keep 1 g on Z
            gx=gx / n, gy=gy / n, gz=gz / n,
        )
        # seed attitude from gravity so the filter starts level-correct
        a = self._bias
        self.roll = 0.0
        self.pitch = 0.0
        self._t_last = None
        return self._bias

    def update(self):
        """Read once, fuse, return RobotState. dt measured from monotonic clock."""
        (ax, ay, az), (gx, gy, gz) = imu.read()
        b = self._bias
        ax -= b.ax; ay -= b.ay; az -= b.az
        gx -= b.gx; gy -= b.gy; gz -= b.gz

        now = time.monotonic()
        if self._t_last is None:
            dt = 0.0
        else:
            dt = now - self._t_last
        self._t_last = now

        # accelerometer attitude (deg)
        acc_roll = math.degrees(math.atan2(ay, az))
        acc_pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))

        if dt <= 0.0 or dt > 0.25:
            # first sample or a stall spike -> trust accel, don't integrate garbage
            self.roll = acc_roll
            self.pitch = acc_pitch
        else:
            self.roll = _ALPHA * (self.roll + gx * dt) + (1 - _ALPHA) * acc_roll
            self.pitch = _ALPHA * (self.pitch + gy * dt) + (1 - _ALPHA) * acc_pitch

        return RobotState(
            roll=self.roll, pitch=self.pitch, yaw_rate=gz,
            ax=ax, ay=ay, az=az, dt=dt,
        )
