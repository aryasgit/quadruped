import time
import math
import errno
from smbus2 import SMBus

# ==================================================
# I2C CONFIG
# ==================================================
BUS = 7
MPU_ADDR = 0x68
PCA_ADDR = 0x40

# MPU REGISTERS
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

# PCA REGISTERS
MODE1 = 0x00
PRESCALE = 0xFE

# ==================================================
# SERVO CONFIG
# ==================================================
PULSE_MIN = 80
PULSE_MAX = 561

# Shoulder ABD/ADD channels
SHOULDERS = {
    "RR": 0,
    "RL": 1,
    "FR": 6,
    "FL": 7,
}

# Standing angles (verified)
STAND = {
    "RR": 38,
    "RL": 38,
    "FR": 41,
    "FL": 41,
}

# IMPORTANT: Physical servo direction mapping
# +1 means angle↑ → abduct
# -1 means angle↑ → adduct
SHOULDER_SIGN = {
    "FR": +1,
    "RR": +1,
    "FL": -1,
    "RL": -1,
}

# ==================================================
# CONTROL PARAMETERS
# ==================================================
ALPHA = 0.96
DT = 0.05

K_ROLL = 0.20

MAX_OFFSET = 4.0
MAX_STEP = 0.25
DECAY = 0.95   # return to stand when roll → 0

ACCEL_DEADBAND = 300
GYRO_DEADBAND = 80

# ==================================================
bus = SMBus(BUS)

# ==================================================
# LOW LEVEL HELPERS
# ==================================================
def safe_read_word(addr, reg):
    try:
        h = bus.read_byte_data(addr, reg)
        l = bus.read_byte_data(addr, reg + 1)
        v = (h << 8) | l
        return v - 65536 if v > 32767 else v
    except OSError as e:
        if e.errno == errno.EREMOTEIO:
            raise IOError
        raise

def angle_to_pulse(angle):
    return int(PULSE_MIN + (angle / 270.0) * (PULSE_MAX - PULSE_MIN))

def set_servo_angle(ch, angle):
    pulse = angle_to_pulse(angle)
    base = 0x06 + 4 * ch
    bus.write_byte_data(PCA_ADDR, base, 0)
    bus.write_byte_data(PCA_ADDR, base + 1, 0)
    bus.write_byte_data(PCA_ADDR, base + 2, pulse & 0xFF)
    bus.write_byte_data(PCA_ADDR, base + 3, (pulse >> 8) & 0x0F)

# ==================================================
# INIT DEVICES
# ==================================================
print("Initializing MPU...")
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
time.sleep(0.1)

print("Initializing PCA...")
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
time.sleep(0.01)
prescale = int(25000000 / (4096 * 50) - 1)
bus.write_byte_data(PCA_ADDR, MODE1, 0x10)
bus.write_byte_data(PCA_ADDR, PRESCALE, prescale)
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
time.sleep(0.005)
bus.write_byte_data(PCA_ADDR, MODE1, 0x80)

print("Moving to standing pose")
for leg, ch in SHOULDERS.items():
    set_servo_angle(ch, STAND[leg])
    time.sleep(0.05)

# ==================================================
# IMU CALIBRATION
# ==================================================
print("Calibrating IMU (keep still)")
ax_o = ay_o = az_o = gx_o = gy_o = 0
N = 200

for _ in range(N):
    ax = safe_read_word(MPU_ADDR, ACCEL_XOUT_H)
    ay = safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 2)
    az = safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 4)
    gx = safe_read_word(MPU_ADDR, GYRO_XOUT_H)
    gy = safe_read_word(MPU_ADDR, GYRO_XOUT_H + 2)
    ax_o += ax
    ay_o += ay
    az_o += (az - 16384)
    gx_o += gx
    gy_o += gy
    time.sleep(0.01)

ax_o /= N
ay_o /= N
az_o /= N
gx_o /= N
gy_o /= N

print("Calibration complete\n")

# ==================================================
roll = 0.0
offsets = {k: 0.0 for k in STAND}

print("Layer-1 Roll Stabilization ACTIVE\n")

# ==================================================
# MAIN LOOP
# ==================================================
while True:
    try:
        ax = safe_read_word(MPU_ADDR, ACCEL_XOUT_H) - ax_o
        ay = safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 2) - ay_o
        az = safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 4) - az_o
        gx = safe_read_word(MPU_ADDR, GYRO_XOUT_H) - gx_o

        if abs(ay) < ACCEL_DEADBAND: ay = 0
        if abs(gx) < GYRO_DEADBAND: gx = 0

        accel_roll = math.degrees(math.atan2(ay, az))
        roll = ALPHA * (roll + (gx / 131.0) * DT) + (1 - ALPHA) * accel_roll

        print(f"Roll: {roll:6.2f}°")

        # --- Roll target (control space) ---
        target = {
            "FR":  K_ROLL * roll,
            "FL": -K_ROLL * roll,
            "RR":  K_ROLL * roll,
            "RL": -K_ROLL * roll,
        }

        # --- Apply control ---
        for leg in offsets:
            desired = max(-MAX_OFFSET, min(MAX_OFFSET, target[leg]))
            current = offsets[leg]

            step = desired - current
            step = max(-MAX_STEP, min(MAX_STEP, step))
            offsets[leg] = current + step

            # decay toward stand
            offsets[leg] *= DECAY

            angle = STAND[leg] + SHOULDER_SIGN[leg] * offsets[leg]
            set_servo_angle(SHOULDERS[leg], angle)

        time.sleep(DT)

    except IOError:
        print("I2C glitch — holding posture")
        time.sleep(0.2)

