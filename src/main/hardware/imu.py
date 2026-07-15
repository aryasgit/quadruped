"""
L1 — MPU6050 IMU DRIVER (raw)
=============================

Wakes the MPU6050, explicitly sets full-scale ranges (the old stack relied on
power-on defaults), and returns raw accel/gyro in physical units. All I2C goes
through the locked, guarded hardware.bus, so IMU reads get the SAME retry/lock
policy as servo writes (the old stack left IMU reads unguarded).

No filtering here — that's estimation/imu_state.py.
"""

from config.robot_spec import (
    MPU_ADDR, MPU_PWR_MGMT_1, MPU_CONFIG, MPU_GYRO_CONFIG, MPU_ACCEL_CONFIG,
    MPU_ACCEL_XOUT_H, MPU_GYRO_XOUT_H, GYRO_LSB_PER_DPS, ACCEL_LSB_PER_G,
)
from hardware import bus

_initialized = False


def init():
    """Wake and configure the MPU6050. Idempotent."""
    global _initialized
    if _initialized:
        return
    bus.write_byte(MPU_ADDR, MPU_PWR_MGMT_1, 0x00)     # wake
    bus.write_byte(MPU_ADDR, MPU_CONFIG, 0x04)         # DLPF ~20 Hz
    bus.write_byte(MPU_ADDR, MPU_GYRO_CONFIG, 0x00)    # +/-250 dps (explicit)
    bus.write_byte(MPU_ADDR, MPU_ACCEL_CONFIG, 0x00)   # +/-2 g   (explicit)
    _initialized = True


def read():
    """
    Returns (ax, ay, az) in g and (gx, gy, gz) in deg/s.
    Raises OSError only on a persistent (non-transient) bus fault.
    """
    if not _initialized:
        init()
    ax = bus.read_word(MPU_ADDR, MPU_ACCEL_XOUT_H)
    ay = bus.read_word(MPU_ADDR, MPU_ACCEL_XOUT_H + 2)
    az = bus.read_word(MPU_ADDR, MPU_ACCEL_XOUT_H + 4)
    gx = bus.read_word(MPU_ADDR, MPU_GYRO_XOUT_H)
    gy = bus.read_word(MPU_ADDR, MPU_GYRO_XOUT_H + 2)
    gz = bus.read_word(MPU_ADDR, MPU_GYRO_XOUT_H + 4)
    return (
        (ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G),
        (gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS),
    )
