"""
Layer 1.3 — IMU DRIVER (MPU6050)
===============================

Authoritative IMU interface for the robot.

Responsibilities:
- Initialize MPU6050
- Perform blocking calibration
- Provide roll, pitch, roll_rate, pitch_rate

NON-RESPONSIBILITIES:
- No balance control
- No posture correction
- No servo output
- No gait logic

This module matches the semantic expectations of the SpotMicro repo:
- roll, pitch in degrees
- suitable to be injected directly into theta_spot[3], theta_spot[4]
"""

import time
import math
import errno
from statistics import mean

from hardware.i2c_bus import get_i2c_bus
from hardware.absolute_truths import (
    MPU_ADDR,
    PWR_MGMT_1,
    ACCEL_XOUT_H,
    GYRO_XOUT_H,
    CONFIG,
)

# -----------------------------
# Low-level safe read
# -----------------------------

def _safe_read_word(bus, addr, reg):
    try:
        h = bus.read_byte_data(addr, reg)
        l = bus.read_byte_data(addr, reg + 1)
        v = (h << 8) | l
        return v - 65536 if v > 32767 else v
    except OSError as e:
        if e.errno == errno.EREMOTEIO:
            raise IOError
        raise


# -----------------------------
# MPU6050 initialization
# -----------------------------

def init_mpu():
    bus = get_i2c_bus()
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    # ~20 Hz low-pass filter (matches repo behavior)
    bus.write_byte_data(MPU_ADDR, CONFIG, 0x04)
    time.sleep(0.05)

    print("[IMU] MPU6050 initialized")


# -----------------------------
# Calibration container
# -----------------------------

class IMUCalibration:
    def __init__(self, ax, ay, az, gx, gy):
        self.ax = ax
        self.ay = ay
        self.az = az
        self.gx = gx
        self.gy = gy


# -----------------------------
# Blocking calibration
# -----------------------------

def calibrate(samples=200):
    """
    Blocking calibration. Robot must be still.
    Returns IMUCalibration object.
    """
    bus = get_i2c_bus()
    print("[IMU] Calibrating — keep robot still")

    ax = ay = az = gx = gy = 0.0
    collected = 0

    while collected < samples:
        try:
            ax += _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H)
            ay += _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 2)
            az += _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 4) - 16384.0

            gx += _safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H)
            gy += _safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H + 2)

            collected += 1
            time.sleep(0.01)
        except IOError:
            time.sleep(0.05)

    calib = IMUCalibration(
        ax / samples,
        ay / samples,
        az / samples,
        gx / samples,
        gy / samples,
    )

    print(f"[IMU] Calibrated | ax={calib.ax:.1f} ay={calib.ay:.1f} az={calib.az:.1f} gx={calib.gx:.2f} gy={calib.gy:.2f}")
    return calib


# -----------------------------
# Complementary filter
# -----------------------------

class IMUFilter:
    def __init__(self, calib: IMUCalibration, alpha=0.96, dt=0.02):
        self.calib = calib
        self.alpha = alpha
        self.dt = dt

        self.roll = 0.0
        self.pitch = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0

    def update(self):
        bus = get_i2c_bus()

        ax = _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H)     - self.calib.ax
        ay = _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 2) - self.calib.ay
        az = _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 4) - self.calib.az

        gx = _safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H)     - self.calib.gx
        gy = _safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H + 2) - self.calib.gy

        accel_roll = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        self.roll_rate = gx / 131.0
        self.pitch_rate = gy / 131.0

        self.roll = self.alpha * (self.roll + self.roll_rate * self.dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + self.pitch_rate * self.dt) + (1 - self.alpha) * accel_pitch

        return self.roll, self.pitch, self.roll_rate, self.pitch_rate


# -----------------------------
# Smoke test
# -----------------------------

if __name__ == "__main__":
    init_mpu()
    calib = calibrate()
    imu = IMUFilter(calib)

    print("[IMU] Live output (Ctrl+C to exit)")
    while True:
        r, p, rr, pr = imu.update()
        print(f"roll={r:+6.2f}° pitch={p:+6.2f}° rr={rr:+6.2f} pr={pr:+6.2f}")
        time.sleep(0.05)
