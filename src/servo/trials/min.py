#!/usr/bin/env python3

import time
from smbus2 import SMBus

# ============================================================
# PCA9685 CONFIG
# ============================================================

I2C_BUS = 7          # CONFIRMED working bus
PCA9685_ADDR = 0x40

MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

PWM_FREQ = 50        # Hz (same as old working code)

# ============================================================
# SERVO CALIBRATION  (COPIED FROM sercon.py)
# ============================================================

SERVO_ANGLE_MIN = 0
SERVO_ANGLE_MAX = 270

PULSE_MIN = 80       # PCA9685 counts
PULSE_RANGE = 481    # PCA9685 counts

# ============================================================
# LOW-LEVEL PCA9685 DRIVER
# ============================================================

bus = SMBus(I2C_BUS)

def pca_reset():
    bus.write_byte_data(PCA9685_ADDR, MODE1, 0x00)
    time.sleep(0.01)

def pca_set_freq(freq_hz):
    prescale = int(25000000 / (4096 * freq_hz) - 1)
    oldmode = bus.read_byte_data(PCA9685_ADDR, MODE1)
    bus.write_byte_data(PCA9685_ADDR, MODE1, (oldmode & 0x7F) | 0x10)  # sleep
    bus.write_byte_data(PCA9685_ADDR, PRESCALE, prescale)
    bus.write_byte_data(PCA9685_ADDR, MODE1, oldmode)
    time.sleep(0.005)
    bus.write_byte_data(PCA9685_ADDR, MODE1, oldmode | 0x80)

def angle_to_counts(angle):
    angle = int(round(angle))
    angle = max(SERVO_ANGLE_MIN, min(SERVO_ANGLE_MAX, angle))
    return PULSE_MIN + int((angle / SERVO_ANGLE_MAX) * PULSE_RANGE)

def set_servo(channel, angle):
    counts = angle_to_counts(angle)
    reg = LED0_ON_L + 4 * channel
    bus.write_i2c_block_data(
        PCA9685_ADDR,
        reg,
        [0, 0, counts & 0xFF, counts >> 8]
    )
    print(f"CH {channel}: angle={angle}°, counts={counts}")

# ============================================================
# MAIN TEST
# ============================================================

if __name__ == "__main__":
    print("Initializing PCA9685...")
    pca_reset()
    pca_set_freq(PWM_FREQ)
    print("PCA9685 ready. Testing servo on CH0.")

    while True:
        set_servo(0, 45)
        time.sleep(2)

        set_servo(0, 90)
        time.sleep(2)

        set_servo(0, 135)
        time.sleep(2)

        set_servo(0, 180)
        time.sleep(2)

