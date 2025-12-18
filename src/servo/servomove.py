# ============================================================
# Servo GUI Test – Jetson Orin Nano + PCA9685
# Fresh start, single file
# ============================================================

import sys
import time
import tkinter as tk
from tkinter import messagebox

# ----------------------------
# I2C / PCA9685 imports
# ----------------------------
try:
    from smbus2 import SMBus
    from adafruit_pca9685 import PCA9685
except Exception as e:
    print("ERROR: Failed to import PCA9685 libraries")
    print(e)
    sys.exit(1)

# ============================================================
# CONFIGURATION SECTION
# ============================================================

PCA9685_I2C_ADDRESS = 0x40
SERVO_FREQUENCY = 50  # Hz

# Safe pulse limits (in microseconds)
SERVO_MIN_US = 500
SERVO_MAX_US = 2500

# ----------------------------
# Servo channel mapping
# Edit these according to YOUR robot
# ----------------------------
SERVO_MAP = {
    "Front Left Hip": 0,
    "Front Left Knee": 1,
    "Front Right Hip": 2,
    "Front Right Knee": 3,
    "Rear Left Hip": 4,
    "Rear Left Knee": 5,
    "Rear Right Hip": 6,
    "Rear Right Knee": 7,
}

# ----------------------------
# Standing / neutral positions
# (degrees)
# ----------------------------
STANDING_ANGLES = {
    "Front Left Hip": 90,
    "Front Left Knee": 90,
    "Front Right Hip": 90,
    "Front Right Knee": 90,
    "Rear Left Hip": 90,
    "Rear Left Knee": 90,
    "Rear Right Hip": 90,
    "Rear Right Knee": 90,
}

# ============================================================
# LOW-LEVEL SERVO DRIVER
# ============================================================

class ServoController:
    def __init__(self):
        try:
            self.bus = SMBus(1)   # Jetson I2C-1
            self.pca = PCA9685(self.bus, address=0x40)
            self.pca.frequency = 50
        except Exception as e:
            print("I2C INIT FAILED")
            print(e)
            sys.exit(1)

    def angle_to_duty(self, angle):
        # Clamp angle
        angle = max(0, min(180, angle))

        pulse_us = SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US)
        duty = int((pulse_us / 1_000_000) * SERVO_FREQUENCY * 65535)
        return duty

    def set_angle(self, channel, angle):
        try:
            duty = self.angle_to_duty(angle)
            self.pca.channels[channel].duty_cycle = duty
        except Exception as e:
            print(f"ERROR: Failed to move servo on channel {channel}")
            print(e)

    def shutdown(self):
        try:
            self.pca.deinit()
        except:
            pass

# ============================================================
# GUI APPLICATION
# ============================================================

class ServoGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Quadruped Servo Test – PCA9685")
        self.root.geometry("500x600")

        self.controller = ServoController()
        self.sliders = {}

        title = tk.Label(
            root,
            text="Servo Slider Test (One by One)",
            font=("Arial", 14, "bold")
        )
        title.pack(pady=10)

        for servo_name, channel in SERVO_MAP.items():
            self._create_servo_slider(servo_name, channel)

        reset_btn = tk.Button(
            root,
            text="Move All to Standing Position",
            command=self.move_to_standing,
            bg="#dddddd"
        )
        reset_btn.pack(pady=10)

        exit_btn = tk.Button(
            root,
            text="Exit Safely",
            command=self.safe_exit,
            bg="#ffcccc"
        )
        exit_btn.pack(pady=5)

    # ----------------------------
    # Slider creator
    # ----------------------------
    def _create_servo_slider(self, name, channel):
        frame = tk.Frame(self.root)
        frame.pack(fill="x", padx=10, pady=5)

        label = tk.Label(frame, text=f"{name} (CH {channel})", width=22, anchor="w")
        label.pack(side="left")

        slider = tk.Scale(
            frame,
            from_=0,
            to=180,
            orient="horizontal",
            length=200,
            command=lambda val, ch=channel: self.on_slider_move(ch, val)
        )
        slider.pack(side="right")

        # Set initial standing position
        start_angle = STANDING_ANGLES.get(name, 90)
        slider.set(start_angle)
        self.controller.set_angle(channel, start_angle)

        self.sliders[name] = slider

    # ----------------------------
    # Slider callback
    # ----------------------------
    def on_slider_move(self, channel, value):
        try:
            angle = int(float(value))
            self.controller.set_angle(channel, angle)
        except Exception as e:
            print("ERROR during slider movement")
            print(e)

    # ----------------------------
    # Reset pose
    # ----------------------------
    def move_to_standing(self):
        for name, slider in self.sliders.items():
            angle = STANDING_ANGLES.get(name, 90)
            slider.set(angle)

    # ----------------------------
    # Safe exit
    # ----------------------------
    def safe_exit(self):
        self.controller.shutdown()
        self.root.destroy()

# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = ServoGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.safe_exit)
    root.mainloop()

