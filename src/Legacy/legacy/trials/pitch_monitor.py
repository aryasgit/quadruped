import time
import math
from smbus2 import SMBus

# ===============================
# I2C CONFIG
# ===============================
BUS = 7
MPU_ADDR = 0x68
PCA_ADDR = 0x40

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

MODE1 = 0x00
PRESCALE = 0xFE

# ===============================
# SERVO CONFIG
# ===============================
PULSE_MIN = 80
PULSE_MAX = 561

# ---- FEET ONLY (KNEES) ----
# Using YOUR verified standing angles
FEET = {
    "FRF": 10,
    "FLF": 11,
    "RRF": 4,
    "RLF": 5,
}

STAND = {
    "FRF": 137,
    "FLF": 61,
    "RRF": 142,
    "RLF": 61,
}

# Mirrored hardware
SIGN = {
    "FRF": +1,
    "FLF": -1,
    "RRF": +1,
    "RLF": -1,
}

# ===============================
# CONTROL PARAMS (OBVIOUS)
# ===============================
DT = 0.05
ALPHA = 0.96

K_PITCH = 0.25      # BIG so we SEE motion
MAX_STEP = 1.0      # BIG so we SEE motion
MAX_OFFSET = 20.0   # BIG, temporary

bus = SMBus(BUS)

# ===============================
# LOW LEVEL
# ===============================
def read_word(addr, reg):
    h = bus.read_byte_data(addr, reg)
    l = bus.read_byte_data(addr, reg + 1)
    v = (h << 8) | l
    return v - 65536 if v > 32767 else v

def angle_to_pulse(angle):
    return int(PULSE_MIN + (angle / 270.0) * (PULSE_MAX - PULSE_MIN))

def set_servo_angle(ch, angle):
    pulse = angle_to_pulse(angle)
    base = 0x06 + 4 * ch
    bus.write_byte_data(PCA_ADDR, base, 0)
    bus.write_byte_data(PCA_ADDR, base + 1, 0)
    bus.write_byte_data(PCA_ADDR, base + 2, pulse & 0xFF)
    bus.write_byte_data(PCA_ADDR, base + 3, (pulse >> 8) & 0x0F)

# ===============================
# INIT PCA
# ===============================
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
time.sleep(0.01)
prescale = int(25000000 / (4096 * 50) - 1)
bus.write_byte_data(PCA_ADDR, MODE1, 0x10)
bus.write_byte_data(PCA_ADDR, PRESCALE, prescale)
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
bus.write_byte_data(PCA_ADDR, MODE1, 0x80)

# ===============================
# MOVE TO STAND
# ===============================
print("Moving to STAND")
for leg, ch in FEET.items():
    set_servo_angle(ch, STAND[leg])
    time.sleep(0.05)

# ===============================
# INIT MPU
# ===============================
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
time.sleep(0.1)

ax_o = ay_o = az_o = gy_o = 0
for _ in range(200):
    ax_o += read_word(MPU_ADDR, ACCEL_XOUT_H)
    ay_o += read_word(MPU_ADDR, ACCEL_XOUT_H + 2)
    az_o += read_word(MPU_ADDR, ACCEL_XOUT_H + 4) - 16384
    gy_o += read_word(MPU_ADDR, GYRO_XOUT_H + 2)
    time.sleep(0.01)

ax_o /= 200
ay_o /= 200
az_o /= 200
gy_o /= 200

# ===============================
# CONTROL STATE
# ===============================
pitch = 0.0
offsets = {k: 0.0 for k in FEET}

print("PITCH TEST ACTIVE — TILT ROBOT")

# ===============================
# MAIN LOOP
# ===============================
while True:
    ax = read_word(MPU_ADDR, ACCEL_XOUT_H) - ax_o
    ay = read_word(MPU_ADDR, ACCEL_XOUT_H + 2) - ay_o
    az = read_word(MPU_ADDR, ACCEL_XOUT_H + 4) - az_o
    gy = read_word(MPU_ADDR, GYRO_XOUT_H + 2) - gy_o

    accel_pitch = math.degrees(
        math.atan2(-ax, math.sqrt(ay*ay + az*az))
    )

    pitch = ALPHA * (pitch + (gy / 131.0) * DT) \
            + (1 - ALPHA) * accel_pitch

    print(f"PITCH: {pitch:6.2f}")

    # ---- FRONT vs REAR ONLY ----
    targets = {
        "FRF": +K_PITCH * pitch,
        "FLF": +K_PITCH * pitch,
        "RRF": -K_PITCH * pitch,
        "RLF": -K_PITCH * pitch,
    }

    for leg, ch in FEET.items():
        # Clamp offset
        target = max(-MAX_OFFSET, min(MAX_OFFSET, targets[leg]))

        # Rate limit offset (THIS IS THE KEY)
        delta = target - offsets[leg]
        delta = max(-MAX_STEP, min(MAX_STEP, delta))
        offsets[leg] += delta

        angle = STAND[leg] + SIGN[leg] * offsets[leg]
        set_servo_angle(ch, angle)

    time.sleep(DT)

