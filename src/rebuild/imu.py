# imu.py
# ==================================================
# MPU6050 IMU — Calibration + Orientation
# Single source of truth for body orientation
# ==================================================

import time
import math
import errno
from statistics import mean, variance

# ==================================================
# MPU6050 REGISTERS (HARDWARE TRUTH)
# ==================================================
MPU_ADDR = 0x68

PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C


# ==================================================
# LOW-LEVEL SAFE READ
# ==================================================
def safe_read_word(bus, addr, reg):
    """
    Reads signed 16-bit value from I2C.
    Raises IOError on transient bus failure.
    """
    try:
        h = bus.read_byte_data(addr, reg)
        l = bus.read_byte_data(addr, reg + 1)
        v = (h << 8) | l
        return v - 65536 if v > 32767 else v
    except OSError as e:
        if e.errno == errno.EREMOTEIO:
            raise IOError
        raise


# ==================================================
# MPU INITIALIZATION (NO CALIBRATION)
# ==================================================
def init_mpu(bus):
    """
    Wake MPU6050 and set digital low-pass filter.
    """
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    # ~20 Hz LPF (matches legacy)
    bus.write_byte_data(MPU_ADDR, CONFIG, 0x04)
    time.sleep(0.05)

    print("[IMU] MPU6050 initialized (LPF ≈ 20Hz)")


# ==================================================
# CALIBRATION DATA CONTAINER
# ==================================================
class IMUCalibration:
    def __init__(self, ax_o, ay_o, az_o, gx_o, gy_o):
        self.ax_o = ax_o
        self.ay_o = ay_o
        self.az_o = az_o
        self.gx_o = gx_o
        self.gy_o = gy_o


# ==================================================
# IMU CALIBRATION (BLOCKING, AUTHORITATIVE)
# ==================================================
def calibrate_imu(bus, samples=200):
    """
    Legacy-exact IMU calibration.
    Robot must be still, but no motion rejection is performed.
    Noise is averaged out.
    """

    print("[IMU] starting calibration (legacy mode)")
    print("[IMU] keep robot completely still...")

    ax_o = ay_o = az_o = 0.0
    gx_o = gy_o = 0.0

    collected = 0

    while collected < samples:
        try:
            ax_o += safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H)
            ay_o += safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 2)
            az_o += safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 4) - 16384.0

            gx_o += safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H)
            gy_o += safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H + 2)

            collected += 1
            time.sleep(0.01)

        except IOError:
            # transient I2C glitch — ignore sample
            time.sleep(0.05)
            continue

    ax_o /= samples
    ay_o /= samples
    az_o /= samples

    gx_o /= samples
    gy_o /= samples

    print(
        f"[IMU] calibrated | "
        f"ax_o={ax_o:.1f} ay_o={ay_o:.1f} az_o={az_o:.1f} | "
        f"gx_o={gx_o:.2f} gy_o={gy_o:.2f}"
    )

    return IMUCalibration(ax_o, ay_o, az_o, gx_o, gy_o)



# ==================================================
# COMPLEMENTARY FILTER
# ==================================================
class IMUFilter:
    def __init__(self, calib, alpha=0.96, dt=0.05):
        self.calib = calib
        self.alpha = alpha
        self.dt = dt

        self.roll = 0.0
        self.pitch = 0.0
        self.roll_abs = 0.0
        self.pitch_abs = 0.0


        self._last_print = time.time()

    def update(self, bus):
        """
        Returns:
            roll (deg), pitch (deg),
            roll_rate (deg/s), pitch_rate (deg/s)
        """

        # ----- raw reads -----
        ax = safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H)     - self.calib.ax_o
        ay = safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 2) - self.calib.ay_o
        az = safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 4) - self.calib.az_o

        gx = safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H)     - self.calib.gx_o
        gy = safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H + 2) - self.calib.gy_o

        # ----- accel angles -----
        accel_roll  = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(
            math.atan2(-ax, math.sqrt(ay*ay + az*az))
        )

        # ----- gyro rates -----
        roll_rate  = gx / 131.0
        pitch_rate = gy / 131.0

        # ----- complementary filter -----
        # ----- complementary filter (ABSOLUTE angles) -----
        self.roll_abs = (
            self.alpha * (self.roll_abs + roll_rate * self.dt)
            + (1.0 - self.alpha) * accel_roll
        )

        self.pitch_abs = (
            self.alpha * (self.pitch_abs + pitch_rate * self.dt)
            + (1.0 - self.alpha) * accel_pitch
        )

        # ----- referenced angles (for posture control) -----
        self.roll = self.roll_abs - getattr(self, "roll_ref", 0.0)
        self.pitch = self.pitch_abs - getattr(self, "pitch_ref", 0.0)


        # ----- slow diagnostic print -----
        if time.time() - self._last_print > 1.0:
            print(
                f"[IMU] roll={self.roll:+.2f}° "
                f"pitch={self.pitch:+.2f}° "
                f"roll_rate={roll_rate:+.2f}°/s "
                f"pitch_rate={pitch_rate:+.2f}°/s"
            )
            self._last_print = time.time()

        return self.roll, self.pitch, roll_rate, pitch_rate


# ==================================================
# STAND REFERENCE (NEUTRAL ORIENTATION)
# ==================================================
class StandReference:
    def __init__(self, roll_ref, pitch_ref):
        self.roll_ref = roll_ref
        self.pitch_ref = pitch_ref


def lock_stand_reference(imu_filter, bus, duration=1.0):
    """
    Locks neutral roll/pitch when robot is standing still.
    Blocks until stable.
    """

    print("[IMU] waiting for stable stand…")

    rolls = []
    pitches = []

    last_print = time.time()
    start_time = None

    while True:
        roll, pitch, _, _ = imu_filter.update(bus)

        # stability gate
        if abs(roll) < 1.0 and abs(pitch) < 1.0:
            if start_time is None:
                start_time = time.time()

            rolls.append(roll)
            pitches.append(pitch)

            if time.time() - start_time >= duration:
                roll_ref = mean(rolls)
                pitch_ref = mean(pitches)

                print(
                    f"[STAND REF] locked | "
                    f"roll_ref={roll_ref:+.3f}° "
                    f"pitch_ref={pitch_ref:+.3f}°"
                )

                return StandReference(roll_ref, pitch_ref)

        else:
            rolls.clear()
            pitches.clear()
            start_time = None

            if time.time() - last_print > 1.0:
                print(
                    f"[IMU] unstable | roll={roll:+.2f}° pitch={pitch:+.2f}°"
                )
                last_print = time.time()

        time.sleep(0.01)


# ==================================================
# STANDALONE SMOKE TEST
# ==================================================
if __name__ == "__main__":
    from smbus2 import SMBus

    BUS = 7
    bus = SMBus(BUS)

    init_mpu(bus)
    calib = calibrate_imu(bus)
    imu = IMUFilter(calib)

    stand_ref = lock_stand_reference(imu, bus)

    print("[IMU] live monitoring — Ctrl+C to exit")
    while True:
        imu.update(bus)
        time.sleep(0.05)
