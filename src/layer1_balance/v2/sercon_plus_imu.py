# Copyright (c) 2025 Aryaman Gupta
# All Rights Reserved.
#
# This code is proprietary.
# Unauthorized use, modification, or distribution is prohibited.

import time
import math
import tkinter as tk
from smbus2 import SMBus

# ===============================
# I2C CONFIG
# ===============================
BUS = 7
ADDR = 0x40
MPU_ADDR = 0x68

MODE1 = 0x00
PRESCALE = 0xFE

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

PULSE_MIN = 80
PULSE_MAX = 561

bus = SMBus(BUS)

# ===============================
# SERVO MAP
# ===============================
SERVO_MAP = {
    0: 38, 1: 38, 2: 102, 3: 154, 4: 128, 5: 77, 
    6: 37, 7: 46, 8: 104, 9: 165, 10: 130, 11: 61
}

SERVO_NAMES = [
    "RRS","RLS","RRM","RLM","RRF","RLF",
    "FRS","FLS","FRM","FLM","FRF","FLF"
]

# ===============================
# LOW LEVEL
# ===============================
def angle_to_pulse(angle):
    return int(PULSE_MIN + (angle / 270.0) * (PULSE_MAX - PULSE_MIN))

def set_pulse(channel, pulse):
    base = 0x06 + 4 * channel
    bus.write_byte_data(ADDR, base + 0, 0)
    bus.write_byte_data(ADDR, base + 1, 0)
    bus.write_byte_data(ADDR, base + 2, pulse & 0xFF)
    bus.write_byte_data(ADDR, base + 3, (pulse >> 8) & 0x0F)

def read_word(reg):
    h = bus.read_byte_data(MPU_ADDR, reg)
    l = bus.read_byte_data(MPU_ADDR, reg + 1)
    v = (h << 8) | l
    return v - 65536 if v > 32767 else v

# ===============================
# INIT PCA + IMU
# ===============================
def init_pca():
    bus.write_byte_data(ADDR, MODE1, 0x00)
    time.sleep(0.01)
    prescale = int(25000000 / (4096 * 50) - 1)
    bus.write_byte_data(ADDR, MODE1, 0x10)
    bus.write_byte_data(ADDR, PRESCALE, prescale)
    bus.write_byte_data(ADDR, MODE1, 0x00)
    bus.write_byte_data(ADDR, MODE1, 0x80)

def init_imu():
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)

# ===============================
# SAFE START
# ===============================
init_pca()
init_imu()

for ch, angle in SERVO_MAP.items():
    set_pulse(ch, angle_to_pulse(angle))

# ===============================
# GUI
# ===============================
root = tk.Tk()
root.title("Quadruped Servo Control + IMU Monitor")

# --- IMU DISPLAY ---
imu_frame = tk.Frame(root)
imu_frame.grid(row=0, column=0, columnspan=2, pady=10)

roll_label = tk.Label(imu_frame, text="Roll: 0.00°", font=("Helvetica", 14))
roll_label.pack(side=tk.LEFT, padx=20)

pitch_label = tk.Label(imu_frame, text="Pitch: 0.00°", font=("Helvetica", 14))
pitch_label.pack(side=tk.LEFT, padx=20)

sliders = {}

# --- SLIDERS ---
for i in range(12):
    frame = tk.Frame(root)
    frame.grid(row=(i//2)+1, column=i%2, padx=10, pady=5, sticky="w")

    label = tk.Label(frame, text=f"CH {i} ({SERVO_NAMES[i]})")
    label.pack(anchor="w")

    slider = tk.Scale(
        frame, from_=0, to=270,
        orient=tk.HORIZONTAL, length=300,
        command=lambda val, ch=i: set_pulse(ch, angle_to_pulse(float(val)))
    )
    slider.pack()
    sliders[i] = slider

# Sync to standing
for ch, angle in SERVO_MAP.items():
    sliders[ch].set(angle)

# ===============================
# IMU UPDATE LOOP (GUI SAFE)
# ===============================
def update_imu():
    ax = read_word(ACCEL_XOUT_H)
    ay = read_word(ACCEL_XOUT_H + 2)
    az = read_word(ACCEL_XOUT_H + 4)

    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

    roll_label.config(text=f"Roll: {roll:+6.2f}°")
    pitch_label.config(text=f"Pitch: {pitch:+6.2f}°")

    root.after(50, update_imu)   # ~20 Hz

update_imu()
root.mainloop()
