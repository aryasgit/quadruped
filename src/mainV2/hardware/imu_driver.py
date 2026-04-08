# hardware/imu_driver.py
"""
Layer 1.1 — IMU DRIVER (HW-290 10DOF)
======================================

Complete sensor fusion driver for the HW-290 module:
    - MPU6050   (accelerometer + gyroscope)
    - QMC5883L  (magnetometer, via MPU6050 I2C bypass)
    - BMP180    (barometer)

Provides:
    - roll, pitch          (degrees, complementary filter)
    - yaw / heading        (degrees, tilt-compensated magnetometer + gyro)
    - roll_rate, pitch_rate, yaw_rate  (°/s)
    - altitude             (meters, relative to calibration point)

Graceful degradation:
    - Magnetometer absent → yaw from gyro only (drifts)
    - Barometer absent    → altitude unavailable (returns 0.0)
    - MPU6050 I2C glitch  → returns last-known values, tracks health

Magnetometer calibration persistence:
    - calibrate_magnetometer() saves offsets to mag_calibration.json
    - calibrate() auto-loads saved offsets on subsequent boots
    - No need to recalibrate unless physical build changes

NON-RESPONSIBILITIES:
    - No balance control
    - No servo output
    - No gait logic
    - No threading (caller controls update rate)

All hardware constants come from truths.py — nothing is hardcoded here.
"""

import struct
import time
import math
import json
import os
from collections import namedtuple

from hardware.truths import (
    get_bus,
    ADDR,
    MPU6050,
    QMC5883L,
    BMP180,
)


# ╔══════════════════════════════════════════════════════════════╗
# ║  PERSISTENT CALIBRATION PATH                                 ║
# ╚══════════════════════════════════════════════════════════════╝

_MAG_CAL_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "mag_calibration.json",
)


# ╔══════════════════════════════════════════════════════════════╗
# ║  DATA STRUCTURES                                             ║
# ╚══════════════════════════════════════════════════════════════╝

IMUReading = namedtuple("IMUReading", [
    "roll",         # degrees
    "pitch",        # degrees
    "yaw",          # degrees (0-360 magnetic north, or gyro-only)
    "roll_rate",    # °/s
    "pitch_rate",   # °/s
    "yaw_rate",     # °/s
    "heading",      # degrees (0-360, tilt-compensated magnetic)
    "altitude",     # meters (relative to calibration point)
])

_RawAccelGyro = namedtuple("_RawAccelGyro", [
    "ax", "ay", "az", "gx", "gy", "gz",
])

_RawMag = namedtuple("_RawMag", ["mx", "my", "mz"])


# ╔══════════════════════════════════════════════════════════════╗
# ║  LOW-LEVEL HELPERS                                           ║
# ╚══════════════════════════════════════════════════════════════╝

def _burst_read(bus, addr, reg, length):
    """Atomic burst read. Returns raw bytes."""
    return bus.read_i2c_block_data(addr, reg, length)


def _read_byte(bus, addr, reg):
    """Single byte read."""
    return bus.read_byte_data(addr, reg)


def _write_byte(bus, addr, reg, value):
    """Single byte write."""
    bus.write_byte_data(addr, reg, value)


def _to_s32(val):
    """
    Simulate C int32_t overflow behavior.

    Python integers have unlimited precision, but the BMP180
    datasheet algorithm assumes 32-bit signed overflow.
    """
    val = val & 0xFFFFFFFF
    if val >= 0x80000000:
        val -= 0x100000000
    return val


def _to_u32(val):
    """
    Simulate C uint32_t overflow behavior.
    """
    return val & 0xFFFFFFFF


# ╔══════════════════════════════════════════════════════════════╗
# ║  BMP180 CALIBRATION DATA                                     ║
# ╚══════════════════════════════════════════════════════════════╝

class _BMP180Cal:
    """
    Holds the 11 factory calibration coefficients read from
    the BMP180 EEPROM. These are unique to each physical chip
    and required for pressure/temperature computation.
    """

    __slots__ = (
        "AC1", "AC2", "AC3", "AC4", "AC5", "AC6",
        "B1", "B2", "MB", "MC", "MD",
    )

    def __init__(self, data_bytes):
        """
        Parse 22 bytes (11 x int16) from BMP180 EEPROM.
        AC1-AC3, B1, B2, MB, MC, MD are signed.
        AC4, AC5, AC6 are unsigned.
        """
        vals = struct.unpack(">hhhHHHhhhhh", bytes(data_bytes))
        (
            self.AC1, self.AC2, self.AC3,
            self.AC4, self.AC5, self.AC6,
            self.B1, self.B2,
            self.MB, self.MC, self.MD,
        ) = vals

    def valid(self):
        """Check that no coefficient is 0x0000 or 0xFFFF (dead EEPROM)."""
        for name in self.__slots__:
            v = getattr(self, name)
            if v == 0 or v == -1 or v == 0xFFFF:
                return False
        return True


# ╔══════════════════════════════════════════════════════════════╗
# ║  IMU CALIBRATION OFFSETS                                     ║
# ╚══════════════════════════════════════════════════════════════╝

class _IMUCalibration:
    """
    Stores zero-offset calibration values determined by averaging
    readings while the robot is stationary on a flat surface.
    """

    __slots__ = (
        "ax", "ay", "az",          # accelerometer offsets (raw units)
        "gx", "gy", "gz",          # gyroscope offsets (raw units)
        "mx", "my", "mz",          # magnetometer hard-iron offsets (raw units)
        "mag_calibrated",           # True only after calibrate_magnetometer()
        "ground_pressure",          # ground-level pressure (Pa)
    )

    def __init__(self):
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0
        self.mx = 0.0
        self.my = 0.0
        self.mz = 0.0
        self.mag_calibrated = False
        self.ground_pressure = BMP180.SEA_LEVEL_PA


# ╔══════════════════════════════════════════════════════════════╗
# ║  IMU DRIVER — MAIN CLASS                                     ║
# ╚══════════════════════════════════════════════════════════════╝

class IMUDriver:
    """
    Complete lifecycle manager for the HW-290 10DOF IMU.

    Usage:
        imu = IMUDriver()
        imu.calibrate()
        while running:
            reading = imu.update()
            print(reading.roll, reading.pitch, reading.yaw)

    Or use the factory:
        imu = create_imu()
    """

    def __init__(self, alpha_ag=0.96, alpha_yaw=0.98):
        """
        Construct the driver and initialize all detected hardware.

        Args:
            alpha_ag:   Complementary filter weight for roll/pitch
                        (higher = trust gyro more, 0.90-0.99 typical)
            alpha_yaw:  Complementary filter weight for yaw
                        (higher = trust gyro more over magnetometer)
        """
        # ── Configuration ─────────────────────────────────────
        self._alpha_ag = alpha_ag
        self._alpha_yaw = alpha_yaw
        self._bus = get_bus()

        # ── Sensor presence flags ─────────────────────────────
        self._has_mag = False
        self._has_baro = False

        # ── Calibration ───────────────────────────────────────
        self._cal = _IMUCalibration()
        self._bmp_cal = None
        self._calibrated = False

        # ── Filter state ──────────────────────────────────────
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._roll_rate = 0.0
        self._pitch_rate = 0.0
        self._yaw_rate = 0.0
        self._heading = 0.0
        self._altitude = 0.0
        self._last_time = None

        # ── Heading smoothing ─────────────────────────────────
        self._raw_heading_prev = 0.0
        self._heading_smooth_alpha = 0.3

        # ── Barometer scheduling ──────────────────────────────
        self._baro_oss = 3
        self._baro_cycle = 0
        self._baro_interval = 10
        self._baro_raw_temp = 0
        self._baro_raw_pres = 0
        self._baro_phase = "idle"
        self._baro_cmd_time = 0.0

        # ── Health tracking ───────────────────────────────────
        self._consecutive_errors = 0
        self._max_errors = 10
        self._total_errors = 0
        self._total_reads = 0

        # ── Initialize hardware ───────────────────────────────
        self._init_mpu6050()
        self._init_magnetometer()
        self._init_barometer()

    # ──────────────────────────────────────────────────────────
    # Hardware initialization
    # ──────────────────────────────────────────────────────────

    def _init_mpu6050(self):
        """Wake MPU6050, verify identity, configure DLPF and bypass."""

        who = _read_byte(self._bus, ADDR.MPU6050, MPU6050.WHO_AM_I)
        if who != MPU6050.WHO_AM_I_EXPECTED:
            raise RuntimeError(
                f"[IMU] MPU6050 WHO_AM_I: got 0x{who:02X}, "
                f"expected 0x{MPU6050.WHO_AM_I_EXPECTED:02X}"
            )

        _write_byte(self._bus, ADDR.MPU6050, MPU6050.PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        _write_byte(
            self._bus, ADDR.MPU6050,
            MPU6050.CONFIG, MPU6050.DLPF_21HZ,
        )
        time.sleep(0.01)

        _write_byte(
            self._bus, ADDR.MPU6050,
            MPU6050.INT_PIN_CFG, MPU6050.BYPASS_ENABLE,
        )
        time.sleep(0.01)

        print("[IMU] MPU6050 initialized (accel ±2g, gyro ±250°/s, DLPF 21Hz)")

    def _init_magnetometer(self):
        """Detect and initialize QMC5883L."""

        try:
            chip_id = _read_byte(
                self._bus, ADDR.QMC5883L, QMC5883L.CHIP_ID_REG,
            )

            _write_byte(
                self._bus, ADDR.QMC5883L,
                QMC5883L.SET_RESET, QMC5883L.SET_RESET_DEFAULT,
            )
            _write_byte(
                self._bus, ADDR.QMC5883L,
                QMC5883L.CONTROL_1, QMC5883L.CTRL1_DEFAULT,
            )
            _write_byte(
                self._bus, ADDR.QMC5883L,
                QMC5883L.CONTROL_2, QMC5883L.CTRL2_DEFAULT,
            )
            time.sleep(0.01)

            self._has_mag = True
            print(f"[IMU] QMC5883L magnetometer initialized (chip_id=0x{chip_id:02X})")

        except OSError:
            self._has_mag = False
            print("[IMU] Magnetometer not found — yaw will use gyro only (drifts)")

    def _init_barometer(self):
        """Detect BMP180, verify identity, read factory calibration."""

        try:
            chip_id = _read_byte(
                self._bus, ADDR.BMP180, BMP180.CHIP_ID_REG,
            )

            if chip_id != BMP180.CHIP_ID_EXPECTED:
                print(
                    f"[IMU] BMP180 chip ID: 0x{chip_id:02X} "
                    f"(expected 0x{BMP180.CHIP_ID_EXPECTED:02X}) — skipping"
                )
                self._has_baro = False
                return

            cal_data = _burst_read(
                self._bus, ADDR.BMP180,
                BMP180.CAL_START, BMP180.CAL_LEN,
            )
            self._bmp_cal = _BMP180Cal(cal_data)

            if not self._bmp_cal.valid():
                print("[IMU] BMP180 calibration EEPROM contains invalid data — skipping")
                self._has_baro = False
                return

            self._has_baro = True
            print(
                f"[IMU] BMP180 barometer initialized "
                f"(AC1={self._bmp_cal.AC1} AC2={self._bmp_cal.AC2} "
                f"AC3={self._bmp_cal.AC3} AC4={self._bmp_cal.AC4} "
                f"AC5={self._bmp_cal.AC5} AC6={self._bmp_cal.AC6})"
            )

        except OSError:
            self._has_baro = False
            print("[IMU] BMP180 not responding — altitude unavailable")

    # ──────────────────────────────────────────────────────────
    # Raw sensor reads
    # ──────────────────────────────────────────────────────────

    def _read_accel_gyro(self):
        """
        Atomic burst read of MPU6050: 14 bytes starting at 0x3B.
        Returns _RawAccelGyro (6 x int16, raw units).
        """
        data = _burst_read(
            self._bus, ADDR.MPU6050,
            MPU6050.BURST_START, MPU6050.BURST_LEN,
        )
        ax, ay, az, _, gx, gy, gz = struct.unpack(">hhhhhhh", bytes(data))
        return _RawAccelGyro(ax, ay, az, gx, gy, gz)

    def _read_magnetometer(self):
        """
        Read QMC5883L: 6 bytes little-endian (xl,xh,yl,yh,zl,zh).
        Returns _RawMag or None on failure.
        """
        if not self._has_mag:
            return None

        try:
            data = _burst_read(
                self._bus, ADDR.QMC5883L,
                QMC5883L.DATA_OUT, QMC5883L.BURST_LEN,
            )
            mx, my, mz = struct.unpack("<hhh", bytes(data))
            return _RawMag(mx, my, mz)
        except OSError:
            return None

    # ──────────────────────────────────────────────────────────
    # BMP180 pressure computation (datasheet with C overflow)
    # ──────────────────────────────────────────────────────────

    def _bmp180_start_temp(self):
        """Command BMP180 to start temperature conversion."""
        try:
            _write_byte(
                self._bus, ADDR.BMP180,
                BMP180.CONTROL, BMP180.CMD_TEMP,
            )
            self._baro_cmd_time = time.monotonic()
            self._baro_phase = "wait_temp"
        except OSError:
            self._baro_phase = "idle"

    def _bmp180_read_temp(self):
        """Read raw temperature result (16-bit)."""
        try:
            data = _burst_read(
                self._bus, ADDR.BMP180,
                BMP180.DATA_MSB, 2,
            )
            self._baro_raw_temp = struct.unpack(">H", bytes(data))[0]
            return True
        except OSError:
            return False

    def _bmp180_start_pressure(self):
        """Command BMP180 to start pressure conversion."""
        try:
            oss = self._baro_oss
            cmd = BMP180.CMD_PRES_OSS0 + (oss << 6)
            _write_byte(
                self._bus, ADDR.BMP180,
                BMP180.CONTROL, cmd,
            )
            self._baro_cmd_time = time.monotonic()
            self._baro_phase = "wait_pres"
        except OSError:
            self._baro_phase = "idle"

    def _bmp180_read_pressure(self):
        """Read raw pressure result (up to 19-bit)."""
        try:
            data = _burst_read(
                self._bus, ADDR.BMP180,
                BMP180.DATA_MSB, 3,
            )
            oss = self._baro_oss
            msb = data[0]
            lsb = data[1]
            xlsb = data[2]
            self._baro_raw_pres = ((msb << 16) | (lsb << 8) | xlsb) >> (8 - oss)
            return True
        except OSError:
            return False

    def _bmp180_compute_pressure(self):
        """
        BMP180 datasheet compensation algorithm.

        All intermediate values are clamped to C int32_t / uint32_t
        range to match the datasheet reference implementation which
        assumes 32-bit overflow behavior.

        Returns pressure in Pa, or None on error.
        """
        cal = self._bmp_cal
        if cal is None:
            return None

        UT = self._baro_raw_temp
        UP = self._baro_raw_pres
        oss = self._baro_oss

        # ── Temperature compensation ─────────────────────────
        X1 = _to_s32(((UT - cal.AC6) * cal.AC5) >> 15)
        denom = X1 + cal.MD
        if denom == 0:
            return None
        X2 = _to_s32((cal.MC << 11) // denom)
        B5 = _to_s32(X1 + X2)

        # ── Pressure compensation ────────────────────────────
        B6 = _to_s32(B5 - 4000)
        X1 = _to_s32((cal.B2 * (_to_s32(B6 * B6) >> 12)) >> 11)
        X2 = _to_s32((cal.AC2 * B6) >> 11)
        X3 = _to_s32(X1 + X2)
        B3 = _to_s32((((_to_s32(cal.AC1 * 4 + X3)) << oss) + 2) >> 2)

        X1 = _to_s32((cal.AC3 * B6) >> 13)
        X2 = _to_s32((cal.B1 * (_to_s32(B6 * B6) >> 12)) >> 16)
        X3 = _to_s32((X1 + X2 + 2) >> 2)
        B4 = _to_u32((cal.AC4 * _to_u32(X3 + 32768)) >> 15)

        if B4 == 0:
            return None

        B7 = _to_u32((_to_u32(UP - B3)) * (50000 >> oss))

        if B7 < 0x80000000:
            p = _to_s32((B7 * 2) // B4)
        else:
            p = _to_s32((B7 // B4) * 2)

        X1 = _to_s32((p >> 8) * (p >> 8))
        X1 = _to_s32((X1 * 3038) >> 16)
        X2 = _to_s32((-7357 * p) >> 16)
        p = _to_s32(p + ((X1 + X2 + 3791) >> 4))

        return p

    def _bmp180_compute(self):
        """Convert raw readings to altitude."""
        pressure_pa = self._bmp180_compute_pressure()
        if pressure_pa is not None and pressure_pa > 0:
            ratio = pressure_pa / self._cal.ground_pressure
            self._altitude = 44330.0 * (1.0 - math.pow(ratio, 0.1903))

    def _bmp180_update_cycle(self):
        """
        Non-blocking barometer state machine. Called every update().
        """
        if not self._has_baro or self._bmp_cal is None:
            return

        self._baro_cycle += 1

        if self._baro_phase == "idle":
            if self._baro_cycle >= self._baro_interval:
                self._baro_cycle = 0
                self._bmp180_start_temp()

        elif self._baro_phase == "wait_temp":
            elapsed = time.monotonic() - self._baro_cmd_time
            if elapsed >= BMP180.WAIT_TEMP:
                if self._bmp180_read_temp():
                    self._bmp180_start_pressure()
                else:
                    self._baro_phase = "idle"

        elif self._baro_phase == "wait_pres":
            wait_times = [
                BMP180.WAIT_PRES_OSS0,
                BMP180.WAIT_PRES_OSS1,
                BMP180.WAIT_PRES_OSS2,
                BMP180.WAIT_PRES_OSS3,
            ]
            elapsed = time.monotonic() - self._baro_cmd_time
            if elapsed >= wait_times[self._baro_oss]:
                if self._bmp180_read_pressure():
                    self._bmp180_compute()
                self._baro_phase = "idle"

    # ──────────────────────────────────────────────────────────
    # Magnetometer calibration persistence
    # ──────────────────────────────────────────────────────────

    def _save_mag_calibration(self):
        """
        Save magnetometer hard-iron offsets to disk.
        Called automatically after calibrate_magnetometer().
        File is stored alongside this module in the hardware/ directory.
        """
        data = {
            "mx": self._cal.mx,
            "my": self._cal.my,
            "mz": self._cal.mz,
            "timestamp": time.time(),
            "note": "Auto-saved by IMUDriver.calibrate_magnetometer()",
        }
        try:
            with open(_MAG_CAL_FILE, "w") as f:
                json.dump(data, f, indent=2)
            print(f"[IMU] Mag calibration saved to {_MAG_CAL_FILE}")
        except OSError as e:
            print(f"[IMU] Warning: could not save mag calibration: {e}")

    def _load_mag_calibration(self):
        """
        Load magnetometer hard-iron offsets from disk.
        Called automatically during calibrate().
        Returns True if loaded successfully.
        """
        try:
            with open(_MAG_CAL_FILE, "r") as f:
                data = json.load(f)

            self._cal.mx = float(data["mx"])
            self._cal.my = float(data["my"])
            self._cal.mz = float(data["mz"])
            self._cal.mag_calibrated = True

            age_hours = (time.time() - data.get("timestamp", 0)) / 3600
            print(
                f"[IMU] Mag calibration loaded from disk "
                f"(mx={self._cal.mx:+.1f} my={self._cal.my:+.1f} "
                f"mz={self._cal.mz:+.1f}, {age_hours:.0f}h old)"
            )
            return True

        except FileNotFoundError:
            return False
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            print(f"[IMU] Warning: mag calibration file corrupt — ignoring ({e})")
            return False

    # ──────────────────────────────────────────────────────────
    # Calibration
    # ──────────────────────────────────────────────────────────

    def calibrate(self, samples=200):
        """
        Blocking calibration. Robot MUST be stationary on a flat surface.

        Averages sensor readings to determine zero offsets.
        Subtracts 1g from accelerometer Z axis.
        Records ground-level barometric pressure.
        Seeds the complementary filter with the first accel reading.

        Magnetometer: loads saved hard-iron offsets from disk if available.
        If no saved calibration exists, heading works but may have
        a fixed angular error. Run calibrate_magnetometer() once to fix.

        Args:
            samples: Number of accel/gyro samples to average.
        """
        print(f"[IMU] Calibrating ({samples} samples) — keep robot still...")

        # ── Accel + Gyro offsets ──────────────────────────────
        sum_ax = sum_ay = sum_az = 0.0
        sum_gx = sum_gy = sum_gz = 0.0
        collected = 0

        while collected < samples:
            try:
                raw = self._read_accel_gyro()
                sum_ax += raw.ax
                sum_ay += raw.ay
                sum_az += raw.az
                sum_gx += raw.gx
                sum_gy += raw.gy
                sum_gz += raw.gz
                collected += 1
                time.sleep(0.005)
            except OSError:
                time.sleep(0.05)

        self._cal.ax = sum_ax / samples
        self._cal.ay = sum_ay / samples
        self._cal.az = (sum_az / samples) - MPU6050.ACCEL_SCALE_2G
        self._cal.gx = sum_gx / samples
        self._cal.gy = sum_gy / samples
        self._cal.gz = sum_gz / samples

        print(
            f"[IMU] Accel offsets:  ax={self._cal.ax:+.1f}  "
            f"ay={self._cal.ay:+.1f}  az={self._cal.az:+.1f}"
        )
        print(
            f"[IMU] Gyro offsets:   gx={self._cal.gx:+.2f}  "
            f"gy={self._cal.gy:+.2f}  gz={self._cal.gz:+.2f}"
        )

        # ── Magnetometer — load saved or use raw ─────────────
        if self._has_mag:
            if self._load_mag_calibration():
                pass    # offsets loaded, mag_calibrated = True
            else:
                # No saved calibration — use raw (zero offsets)
                self._cal.mx = 0.0
                self._cal.my = 0.0
                self._cal.mz = 0.0
                self._cal.mag_calibrated = False

            # Verify sensor is alive
            mag_ok = 0
            for _ in range(10):
                mag = self._read_magnetometer()
                if mag is not None:
                    mag_ok += 1
                time.sleep(0.005)

            if mag_ok > 0:
                if self._cal.mag_calibrated:
                    print(
                        f"[IMU] Mag alive:      {mag_ok}/10 reads OK "
                        f"(using saved calibration)"
                    )
                else:
                    print(
                        f"[IMU] Mag alive:      {mag_ok}/10 reads OK  "
                        f"(run calibrate_magnetometer() for heading accuracy)"
                    )
            else:
                print("[IMU] Magnetometer not responding — disabling")
                self._has_mag = False

        # ── Barometer ground level ────────────────────────────
        if self._has_baro:
            pressures = []
            for _ in range(10):
                try:
                    _write_byte(
                        self._bus, ADDR.BMP180,
                        BMP180.CONTROL, BMP180.CMD_TEMP,
                    )
                    time.sleep(BMP180.WAIT_TEMP)
                    self._bmp180_read_temp()

                    oss = self._baro_oss
                    cmd = BMP180.CMD_PRES_OSS0 + (oss << 6)
                    _write_byte(
                        self._bus, ADDR.BMP180,
                        BMP180.CONTROL, cmd,
                    )
                    time.sleep(BMP180.WAIT_PRES_OSS3)
                    self._bmp180_read_pressure()

                    p = self._bmp180_compute_pressure()
                    if p is not None and 30000 < p < 120000:
                        pressures.append(p)

                except OSError:
                    continue

            if pressures:
                self._cal.ground_pressure = sum(pressures) / len(pressures)
                alt_check = 44330.0 * (
                    1.0 - math.pow(
                        self._cal.ground_pressure / BMP180.SEA_LEVEL_PA,
                        0.1903,
                    )
                )
                print(
                    f"[IMU] Ground pressure: {self._cal.ground_pressure:.0f} Pa "
                    f"(~{alt_check:.0f}m ASL)"
                )
            else:
                print("[IMU] Barometer calibration failed — readings out of range or absent")
                self._has_baro = False

        # ── Seed complementary filter ─────────────────────────
        try:
            raw = self._read_accel_gyro()
            ax = raw.ax - self._cal.ax
            ay = raw.ay - self._cal.ay
            az = raw.az - self._cal.az

            self._roll = math.degrees(math.atan2(ay, az))
            self._pitch = math.degrees(
                math.atan2(-ax, math.sqrt(ay * ay + az * az))
            )

            # Seed yaw from magnetometer if available
            if self._has_mag:
                mag = self._read_magnetometer()
                if mag is not None:
                    self._heading = self._compute_heading(
                        mag.mx - self._cal.mx,
                        mag.my - self._cal.my,
                        mag.mz - self._cal.mz,
                        self._roll,
                        self._pitch,
                    )
                    self._yaw = self._heading
                    self._raw_heading_prev = self._heading

            print(
                f"[IMU] Filter seeded: roll={self._roll:+.1f}° "
                f"pitch={self._pitch:+.1f}° yaw={self._yaw:+.1f}°"
            )

        except OSError:
            print("[IMU] Warning: could not seed filter — starting at 0°")

        self._last_time = time.monotonic()
        self._calibrated = True
        print("[IMU] Calibration complete")

    # ──────────────────────────────────────────────────────────
    # Magnetometer dedicated calibration
    # ──────────────────────────────────────────────────────────

    def calibrate_magnetometer(self, duration=15.0):
        """
        Interactive magnetometer calibration.

        Slowly rotate the robot through all orientations (roll it,
        tilt it, spin it) for the specified duration. This collects
        min/max values on each axis to compute hard-iron offsets.

        Results are automatically saved to disk for future boots.

        Args:
            duration: Seconds to collect data (default 15).

        Call this AFTER the main calibrate() while physically
        rotating the robot through as many orientations as possible.
        """
        if not self._has_mag:
            print("[IMU] No magnetometer available")
            return

        print(f"[IMU] Magnetometer calibration — rotate robot slowly for {duration:.0f}s...")
        print("  (tilt, roll, and yaw through all orientations)")

        min_x = min_y = min_z = float("inf")
        max_x = max_y = max_z = float("-inf")
        collected = 0
        start = time.monotonic()

        while (time.monotonic() - start) < duration:
            mag = self._read_magnetometer()
            if mag is not None:
                min_x = min(min_x, mag.mx)
                max_x = max(max_x, mag.mx)
                min_y = min(min_y, mag.my)
                max_y = max(max_y, mag.my)
                min_z = min(min_z, mag.mz)
                max_z = max(max_z, mag.mz)
                collected += 1

                elapsed = time.monotonic() - start
                remaining = duration - elapsed
                if collected % 100 == 0:
                    print(f"  {remaining:.0f}s remaining — {collected} samples")

            time.sleep(0.01)

        if collected < 50:
            print(f"[IMU] Only {collected} mag samples — calibration may be poor")
            if collected == 0:
                return

        # Hard-iron offsets = center of the min/max sphere
        self._cal.mx = (min_x + max_x) / 2.0
        self._cal.my = (min_y + max_y) / 2.0
        self._cal.mz = (min_z + max_z) / 2.0
        self._cal.mag_calibrated = True

        span_x = max_x - min_x
        span_y = max_y - min_y
        span_z = max_z - min_z

        print(f"[IMU] Mag calibration complete ({collected} samples)")
        print(
            f"  Offsets:  mx={self._cal.mx:+.1f}  "
            f"my={self._cal.my:+.1f}  mz={self._cal.mz:+.1f}"
        )
        print(
            f"  Spans:    X={span_x:.0f}  Y={span_y:.0f}  Z={span_z:.0f}  "
            f"(should be roughly equal)"
        )

        # Save to disk for future boots
        self._save_mag_calibration()

        # Re-seed yaw from calibrated mag
        try:
            raw = self._read_accel_gyro()
            ax = raw.ax - self._cal.ax
            ay = raw.ay - self._cal.ay
            az = raw.az - self._cal.az
            roll = math.degrees(math.atan2(ay, az))
            pitch = math.degrees(
                math.atan2(-ax, math.sqrt(ay * ay + az * az))
            )

            mag = self._read_magnetometer()
            if mag is not None:
                self._heading = self._compute_heading(
                    mag.mx - self._cal.mx,
                    mag.my - self._cal.my,
                    mag.mz - self._cal.mz,
                    roll, pitch,
                )
                self._yaw = self._heading
                self._raw_heading_prev = self._heading
                print(f"  Heading re-seeded: {self._heading:.1f}°")
        except OSError:
            print("  Warning: could not re-seed heading")

    # ──────────────────────────────────────────────────────────
    # Heading computation
    # ──────────────────────────────────────────────────────────

    def _compute_heading(self, mx, my, mz, roll_deg, pitch_deg):
        """
        Tilt-compensated magnetic heading.

        Rotates the magnetometer vector by the known roll and pitch
        to project it onto the horizontal plane, then computes
        heading via atan2.

        Returns heading in degrees [0, 360), 0 = magnetic north.
        """
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)

        cos_roll = math.cos(roll_rad)
        sin_roll = math.sin(roll_rad)
        cos_pitch = math.cos(pitch_rad)
        sin_pitch = math.sin(pitch_rad)

        mx_h = (mx * cos_pitch
                + my * sin_roll * sin_pitch
                + mz * cos_roll * sin_pitch)

        my_h = (my * cos_roll
                - mz * sin_roll)

        heading = math.degrees(math.atan2(-my_h, mx_h))

        if heading < 0:
            heading += 360.0

        return heading

    # ──────────────────────────────────────────────────────────
    # Main update loop
    # ──────────────────────────────────────────────────────────

    def update(self):
        """
        Read all sensors, apply sensor fusion, return filtered state.

        Must be called at a regular rate (20-100 Hz recommended).
        Returns an IMUReading namedtuple.

        On I2C failure, returns last-known values and increments
        the error counter. Check .healthy to monitor.
        """
        self._total_reads += 1

        # ── Timing ────────────────────────────────────────────
        now = time.monotonic()
        if self._last_time is None:
            dt = 0.02
        else:
            dt = now - self._last_time
        self._last_time = now

        dt = max(0.001, min(0.1, dt))

        # ── Read MPU6050 (accel + gyro) ───────────────────────
        try:
            raw = self._read_accel_gyro()

            ax = raw.ax - self._cal.ax
            ay = raw.ay - self._cal.ay
            az = raw.az - self._cal.az
            gx = raw.gx - self._cal.gx
            gy = raw.gy - self._cal.gy
            gz = raw.gz - self._cal.gz

            accel_roll = math.degrees(math.atan2(ay, az))
            accel_pitch = math.degrees(
                math.atan2(-ax, math.sqrt(ay * ay + az * az))
            )

            self._roll_rate = gx / MPU6050.GYRO_SCALE_250DPS
            self._pitch_rate = gy / MPU6050.GYRO_SCALE_250DPS
            self._yaw_rate = gz / MPU6050.GYRO_SCALE_250DPS

            self._roll = (
                self._alpha_ag * (self._roll + self._roll_rate * dt)
                + (1.0 - self._alpha_ag) * accel_roll
            )
            self._pitch = (
                self._alpha_ag * (self._pitch + self._pitch_rate * dt)
                + (1.0 - self._alpha_ag) * accel_pitch
            )

            self._consecutive_errors = 0

        except OSError:
            self._consecutive_errors += 1
            self._total_errors += 1

        # ── Read magnetometer ─────────────────────────────────
        if self._has_mag:
            mag = self._read_magnetometer()
            if mag is not None:
                mx = mag.mx - self._cal.mx
                my = mag.my - self._cal.my
                mz = mag.mz - self._cal.mz

                raw_heading = self._compute_heading(
                    mx, my, mz, self._roll, self._pitch,
                )

                # Low-pass filter on raw heading (handle wraparound)
                hdg_error = raw_heading - self._raw_heading_prev
                if hdg_error > 180.0:
                    hdg_error -= 360.0
                elif hdg_error < -180.0:
                    hdg_error += 360.0

                smoothed = self._raw_heading_prev + self._heading_smooth_alpha * hdg_error
                smoothed = smoothed % 360.0
                self._raw_heading_prev = smoothed
                self._heading = smoothed

                # Complementary filter — yaw
                yaw_error = self._heading - self._yaw
                if yaw_error > 180.0:
                    yaw_error -= 360.0
                elif yaw_error < -180.0:
                    yaw_error += 360.0

                self._yaw = (
                    self._yaw
                    + self._yaw_rate * dt
                    + (1.0 - self._alpha_yaw) * yaw_error
                )
                self._yaw = self._yaw % 360.0
            else:
                self._yaw = (self._yaw + self._yaw_rate * dt) % 360.0
        else:
            self._yaw = (self._yaw + self._yaw_rate * dt) % 360.0

        # ── Update barometer ──────────────────────────────────
        self._bmp180_update_cycle()

        # ── Build and return reading ──────────────────────────
        return IMUReading(
            roll=self._roll,
            pitch=self._pitch,
            yaw=self._yaw,
            roll_rate=self._roll_rate,
            pitch_rate=self._pitch_rate,
            yaw_rate=self._yaw_rate,
            heading=self._heading,
            altitude=self._altitude,
        )

    # ──────────────────────────────────────────────────────────
    # Recalibration
    # ──────────────────────────────────────────────────────────

    def recalibrate(self, samples=200):
        """
        Re-run calibration. Robot must be stationary.

        If magnetometer was previously calibrated (either via
        calibrate_magnetometer() or loaded from disk), those
        offsets are preserved. Only accel/gyro offsets and
        barometer ground level are re-measured.
        """
        print("[IMU] Recalibrating...")

        saved_mx = self._cal.mx
        saved_my = self._cal.my
        saved_mz = self._cal.mz
        saved_mag_cal = self._cal.mag_calibrated

        self._calibrated = False
        self.calibrate(samples)

        if saved_mag_cal:
            self._cal.mx = saved_mx
            self._cal.my = saved_my
            self._cal.mz = saved_mz
            self._cal.mag_calibrated = True
            print("[IMU] Preserved magnetometer calibration from previous session")

    # ──────────────────────────────────────────────────────────
    # Properties
    # ──────────────────────────────────────────────────────────

    @property
    def roll(self):
        """Current roll angle in degrees."""
        return self._roll

    @property
    def pitch(self):
        """Current pitch angle in degrees."""
        return self._pitch

    @property
    def yaw(self):
        """Current yaw angle in degrees [0, 360)."""
        return self._yaw

    @property
    def roll_rate(self):
        """Current roll rate in °/s."""
        return self._roll_rate

    @property
    def pitch_rate(self):
        """Current pitch rate in °/s."""
        return self._pitch_rate

    @property
    def yaw_rate(self):
        """Current yaw rate in °/s."""
        return self._yaw_rate

    @property
    def heading(self):
        """Tilt-compensated magnetic heading in degrees [0, 360)."""
        return self._heading

    @property
    def altitude(self):
        """Altitude relative to calibration point in meters."""
        return self._altitude

    @property
    def healthy(self):
        """True if IMU is responding normally."""
        return self._consecutive_errors < self._max_errors

    @property
    def has_magnetometer(self):
        """True if magnetometer was detected and initialized."""
        return self._has_mag

    @property
    def has_barometer(self):
        """True if barometer was detected and initialized."""
        return self._has_baro

    @property
    def mag_calibrated(self):
        """True if magnetometer has proper hard-iron calibration."""
        return self._cal.mag_calibrated

    @property
    def calibrated(self):
        """True if calibration has been completed."""
        return self._calibrated

    @property
    def error_count(self):
        """Total number of I2C errors since construction."""
        return self._total_errors

    @property
    def read_count(self):
        """Total number of update() calls since construction."""
        return self._total_reads

    @property
    def error_rate(self):
        """Fraction of update() calls that encountered an I2C error."""
        if self._total_reads == 0:
            return 0.0
        return self._total_errors / self._total_reads


# ╔══════════════════════════════════════════════════════════════╗
# ║  FACTORY FUNCTION                                            ║
# ╚══════════════════════════════════════════════════════════════╝

def create_imu(alpha_ag=0.96, alpha_yaw=0.98, samples=200):
    """
    One-liner to create a fully initialized and calibrated IMUDriver.

    Args:
        alpha_ag:   Complementary filter weight for roll/pitch
        alpha_yaw:  Complementary filter weight for yaw
        samples:    Number of calibration samples

    Returns:
        IMUDriver instance, ready to call .update()
    """
    driver = IMUDriver(alpha_ag=alpha_ag, alpha_yaw=alpha_yaw)
    driver.calibrate(samples=samples)
    return driver


# ╔══════════════════════════════════════════════════════════════╗
# ║  SMOKE TEST                                                  ║
# ╚══════════════════════════════════════════════════════════════╝

if __name__ == "__main__":
    import sys

    print("=" * 62)
    print("  IMU DRIVER — SMOKE TEST")
    print("=" * 62)

    imu = create_imu(samples=200)

    # ── Optional: run mag calibration if requested ────────────
    if "--mag-cal" in sys.argv:
        duration = 15.0
        for arg in sys.argv:
            if arg.startswith("--mag-duration="):
                try:
                    duration = float(arg.split("=")[1])
                except ValueError:
                    pass
        imu.calibrate_magnetometer(duration=duration)

    print()
    print(f"  Magnetometer:      {'YES' if imu.has_magnetometer else 'NO'}")
    print(f"  Mag calibrated:    {'YES' if imu.mag_calibrated else 'NO (run with --mag-cal)'}")
    print(f"  Barometer:         {'YES' if imu.has_barometer else 'NO'}")
    print()

    # ── Header ────────────────────────────────────────────────
    print(
        f"{'roll':>8s} {'pitch':>8s} {'yaw':>8s}  "
        f"{'rr':>7s} {'pr':>7s} {'yr':>7s}  "
        f"{'hdg':>6s} {'alt':>6s}  "
        f"{'hp':>3s} {'err':>4s}"
    )
    print("-" * 80)

    # ── Live loop ─────────────────────────────────────────────
    try:
        while True:
            r = imu.update()

            health = " OK" if imu.healthy else "BAD"
            err = imu.error_count

            print(
                f"{r.roll:+8.2f} {r.pitch:+8.2f} {r.yaw:8.2f}  "
                f"{r.roll_rate:+7.2f} {r.pitch_rate:+7.2f} {r.yaw_rate:+7.2f}  "
                f"{r.heading:6.1f} {r.altitude:+6.2f}  "
                f"{health:>3s} {err:4d}"
            )

            time.sleep(0.02)

    except KeyboardInterrupt:
        print()
        print("-" * 80)
        print(f"  Total reads:  {imu.read_count}")
        print(f"  Total errors: {imu.error_count}")
        print(f"  Error rate:   {imu.error_rate:.4%}")
        print("  Exiting.")