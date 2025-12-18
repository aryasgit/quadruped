import time
import math
import errno
from smbus2 import SMBus

# ===============================
# CONFIG
# ===============================
BUS = 7
ADDR = 0x68

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

ALPHA = 0.96
DT = 0.05

ACCEL_DEADBAND = 300
GYRO_DEADBAND = 80

PRINT_DELTA = 0.5  # degrees

bus = SMBus(BUS)

# ===============================
# SHARED STATE
# ===============================
roll_pitch = {"roll": 0.0, "pitch": 0.0}

# ===============================
def safe_read_word(reg):
    try:
        h = bus.read_byte_data(ADDR, reg)
        l = bus.read_byte_data(ADDR, reg + 1)
        v = (h << 8) | l
        return v - 65536 if v > 32767 else v
    except OSError as e:
        if e.errno == errno.EREMOTEIO:
            raise IOError
        raise

def read_accel():
    return (
        safe_read_word(ACCEL_XOUT_H),
        safe_read_word(ACCEL_XOUT_H + 2),
        safe_read_word(ACCEL_XOUT_H + 4),
    )

def read_gyro():
    return (
        safe_read_word(GYRO_XOUT_H),
        safe_read_word(GYRO_XOUT_H + 2),
        safe_read_word(GYRO_XOUT_H + 4),
    )

def init_mpu():
    bus.write_byte_data(ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)

# ===============================
init_mpu()

print("Calibrating IMU (keep robot still)...")

ax_o = ay_o = az_o = gx_o = gy_o = 0
N = 200

for _ in range(N):
    ax, ay, az = read_accel()
    gx, gy, _ = read_gyro()
    ax_o += ax
    ay_o += ay
    az_o += (az - 16384)
    gx_o += gx
    gy_o += gy
    time.sleep(0.01)

ax_o /= N; ay_o /= N; az_o /= N
gx_o /= N; gy_o /= N

print("IMU READY\n")

# ===============================
roll = pitch = 0.0
prev_r = prev_p = 0.0

while True:
    try:
        ax, ay, az = read_accel()
        gx, gy, _ = read_gyro()

        ax -= ax_o; ay -= ay_o; az -= az_o
        gx -= gx_o; gy -= gy_o

        if abs(ax) < ACCEL_DEADBAND: ax = 0
        if abs(ay) < ACCEL_DEADBAND: ay = 0
        if abs(gx) < GYRO_DEADBAND: gx = 0
        if abs(gy) < GYRO_DEADBAND: gy = 0

        accel_roll  = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

        roll  = ALPHA * (roll  + (gx / 131.0) * DT) + (1 - ALPHA) * accel_roll
        pitch = ALPHA * (pitch + (gy / 131.0) * DT) + (1 - ALPHA) * accel_pitch

        roll_pitch["roll"] = roll
        roll_pitch["pitch"] = pitch

        if abs(roll - prev_r) > PRINT_DELTA or abs(pitch - prev_p) > PRINT_DELTA:
            print(f"IMU → Roll: {roll:6.2f}° | Pitch: {pitch:6.2f}°")
            prev_r, prev_p = roll, pitch

        time.sleep(DT)

    except IOError:
        print("IMU I2C glitch — waiting...")
        time.sleep(0.2)
        init_mpu()

