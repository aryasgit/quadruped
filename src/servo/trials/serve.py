import sys
import time
import tkinter as tk
from smbus2 import SMBus

# ============================================================
# PCA9685 LOW-LEVEL DRIVER (NO BLINKA)
# ============================================================

PCA9685_ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

SERVO_FREQ = 50  # Hz
SERVO_MIN_US = 500
SERVO_MAX_US = 2500

class PCA9685:
    def __init__(self, bus=7, address=PCA9685_ADDR):
        self.bus = SMBus(bus)
        self.address = address
        self.reset()
        self.set_pwm_freq(SERVO_FREQ)

    def reset(self):
        self.bus.write_byte_data(self.address, MODE1, 0x00)
        time.sleep(0.01)

    def set_pwm_freq(self, freq_hz):
        prescaleval = 25000000.0
        prescaleval /= 4096.0
        prescaleval /= freq_hz
        prescaleval -= 1.0
        prescale = int(prescaleval + 0.5)

        oldmode = self.bus.read_byte_data(self.address, MODE1)
        newmode = (oldmode & 0x7F) | 0x10  # sleep
        self.bus.write_byte_data(self.address, MODE1, newmode)
        self.bus.write_byte_data(self.address, PRESCALE, prescale)
        self.bus.write_byte_data(self.address, MODE1, oldmode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, MODE1, oldmode | 0x80)

    def set_pwm(self, channel, on, off):
        reg = LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(self.address, reg, [
            on & 0xFF,
            on >> 8,
            off & 0xFF,
            off >> 8
        ])

# ============================================================
# SERVO CONTROLLER
# ============================================================

class ServoController:
    def __init__(self):
        self.pca = PCA9685()

    def angle_to_counts(self, angle):
        angle = max(0, min(180, angle))
        pulse_us = SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US)
        counts = int(pulse_us * 4096 * SERVO_FREQ / 1_000_000)
        return counts

    def set_angle(self, channel, angle):
        off = self.angle_to_counts(angle)
        self.pca.set_pwm(channel, 0, off)

# ============================================================
# GUI
# ============================================================

SERVO_MAP = {
    "Front Left Hip": 0,
    "Front Left Knee": 1,
    "Front Right Hip": 2,
    "Front Right Knee": 3,
}

STANDING = 90

class ServoGUI:
    def __init__(self, root):
        self.ctrl = ServoController()
        root.title("Servo Test – Jetson Orin (RAW I2C)")
        root.geometry("450x300")

        for name, ch in SERVO_MAP.items():
            self.make_slider(root, name, ch)

    def make_slider(self, root, name, ch):
        frame = tk.Frame(root)
        frame.pack(pady=8, fill="x")

        tk.Label(frame, text=f"{name} (CH {ch})", width=18, anchor="w").pack(side="left")

        s = tk.Scale(
            frame,
            from_=0, to=180,
            orient="horizontal",
            command=lambda v, c=ch: self.ctrl.set_angle(c, int(v))
        )
        s.pack(side="right", fill="x", expand=True)
        s.set(STANDING)
        self.ctrl.set_angle(ch, STANDING)

# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    root = tk.Tk()
    ServoGUI(root)
    root.mainloop()

