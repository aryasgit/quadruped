"""
Layer 1.5 — IMU (MPU6050 on the HW-290 10-DOF board)
====================================================

Roll/pitch (complementary filter) + body rates — the ONLY runtime feedback
this robot has (D-009). Magnetometer/barometer on the HW-290 are ignored
for now.

Boot-time bias calibration (robot still, level), per the legacy practice;
nothing persists to disk. Register map = chip datasheet facts.
"""

import math
import time

from barq1.truths import I2C_BUS, MPU_ADDR

# MPU6050 registers
_PWR_MGMT_1 = 0x6B
_SMPLRT_DIV = 0x19
_CONFIG = 0x1A          # DLPF
_GYRO_CONFIG = 0x1B
_ACCEL_CONFIG = 0x1C
_ACCEL_XOUT_H = 0x3B    # 14-byte burst: accel(6) temp(2) gyro(6)

_ACCEL_LSB_PER_G = 16384.0   # +/-2 g
_GYRO_LSB_PER_DPS = 131.0    # +/-250 deg/s


class SimIMU:
    """Stand-in when no hardware: flat, still."""
    def read_raw(self):
        return (0.0, 0.0, 1.0), (0.0, 0.0, 0.0)


class IMU:
    def __init__(self, bus=None, addr=MPU_ADDR, sim=False):
        self.sim = sim
        self.addr = addr
        if sim:
            self.dev = SimIMU()
            self.bus = None
        else:
            from smbus2 import SMBus
            self.bus = bus or SMBus(I2C_BUS)
            self._init_chip()
        self.bias_gyro = (0.0, 0.0, 0.0)
        self.bias_accel = (0.0, 0.0, 0.0)   # x/y only ever applied; z keeps gravity
        self.roll = 0.0
        self.pitch = 0.0
        self._t_last = None

    def _init_chip(self):
        w = lambda r, v: self.bus.write_byte_data(self.addr, r, v)
        w(_PWR_MGMT_1, 0x00)        # wake, internal clock
        time.sleep(0.05)
        w(_PWR_MGMT_1, 0x01)        # PLL w/ X gyro ref (stabler clock)
        w(_SMPLRT_DIV, 0x04)        # 200 Hz sample rate
        w(_CONFIG, 0x03)            # DLPF ~44 Hz accel / 42 Hz gyro
        w(_GYRO_CONFIG, 0x00)       # +/-250 dps
        w(_ACCEL_CONFIG, 0x00)      # +/-2 g
        time.sleep(0.05)

    def read_raw(self):
        """((ax,ay,az) g, (gx,gy,gz) deg/s) — uncalibrated."""
        if self.sim:
            return self.dev.read_raw()
        d = self.bus.read_i2c_block_data(self.addr, _ACCEL_XOUT_H, 14)

        def s16(hi, lo):
            v = (hi << 8) | lo
            return v - 65536 if v & 0x8000 else v

        ax, ay, az = (s16(d[0], d[1]) / _ACCEL_LSB_PER_G,
                      s16(d[2], d[3]) / _ACCEL_LSB_PER_G,
                      s16(d[4], d[5]) / _ACCEL_LSB_PER_G)
        gx, gy, gz = (s16(d[8], d[9]) / _GYRO_LSB_PER_DPS,
                      s16(d[10], d[11]) / _GYRO_LSB_PER_DPS,
                      s16(d[12], d[13]) / _GYRO_LSB_PER_DPS)
        return (ax, ay, az), (gx, gy, gz)

    def calibrate(self, samples=200, dt=0.005):
        """Robot still & level. Averages out gyro bias and accel x/y offset."""
        acc = [0.0] * 6
        for _ in range(samples):
            (ax, ay, az), (gx, gy, gz) = self.read_raw()
            for i, v in enumerate((ax, ay, az, gx, gy, gz)):
                acc[i] += v
            time.sleep(dt)
        n = float(samples)
        self.bias_accel = (acc[0] / n, acc[1] / n, 0.0)
        self.bias_gyro = (acc[3] / n, acc[4] / n, acc[5] / n)
        # seed the filter from gravity
        ax, ay, az = -self.bias_accel[0], -self.bias_accel[1], acc[2] / n
        self.roll = math.atan2(ay, az)
        self.pitch = math.atan2(-ax, math.hypot(ay, az))
        self._t_last = time.monotonic()

    def update(self, alpha=0.98):
        """Complementary filter step. Returns (roll, pitch) rad and
        (gx, gy, gz) deg/s, bias-corrected."""
        (ax, ay, az), (gx, gy, gz) = self.read_raw()
        ax -= self.bias_accel[0]
        ay -= self.bias_accel[1]
        gx -= self.bias_gyro[0]
        gy -= self.bias_gyro[1]
        gz -= self.bias_gyro[2]

        now = time.monotonic()
        dt = 0.0 if self._t_last is None else now - self._t_last
        self._t_last = now

        roll_acc = math.atan2(ay, az if az != 0 else 1e-9)
        pitch_acc = math.atan2(-ax, math.hypot(ay, az) or 1e-9)
        self.roll = alpha * (self.roll + math.radians(gx) * dt) + (1 - alpha) * roll_acc
        self.pitch = alpha * (self.pitch + math.radians(gy) * dt) + (1 - alpha) * pitch_acc
        return (self.roll, self.pitch), (gx, gy, gz)
