import time
import tkinter as tk
from smbus2 import SMBus

# ===============================
# PCA9685 CONFIG
# ===============================
BUS = 7
ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE

PULSE_MIN = 106
PULSE_MAX = 535
ANGLE_RANGE = 270.0   # ← REAL MEASURED RANGE

bus = SMBus(BUS)

# ===============================
# SERVO MAPPING (Channel : Standing Angle)
# ===============================
SERVO_MAP = {
    0: 38,
    1: 37,
    2: 102,
    3: 154,
    4: 142,
    5: 61,
    6: 37,
    7: 46,
    8: 85,
    9: 176,
    10: 137,
    11: 61
}

SERVO_NAMES = [
    "RRS", "RLS", "RRM", "RLM", "RRF", "RLF",
    "FRS", "FLS", "FRM", "FLM", "FRF", "FLF"
]

# ===============================
# PCA FUNCTIONS
# ===============================
def init_pca():
    bus.write_byte_data(ADDR, MODE1, 0x00)
    time.sleep(0.01)

    prescale = int(25000000 / (4096 * 50) - 1)
    bus.write_byte_data(ADDR, MODE1, 0x10)
    bus.write_byte_data(ADDR, PRESCALE, prescale)
    bus.write_byte_data(ADDR, MODE1, 0x00)
    time.sleep(0.005)
    bus.write_byte_data(ADDR, MODE1, 0x80)


def set_pulse(channel, pulse):
    base = 0x06 + 4 * channel
    bus.write_byte_data(ADDR, base + 0, 0x00)
    bus.write_byte_data(ADDR, base + 1, 0x00)
    bus.write_byte_data(ADDR, base + 2, pulse & 0xFF)
    bus.write_byte_data(ADDR, base + 3, (pulse >> 8) & 0x0F)


def angle_to_pulse(angle):
    angle = max(0.0, min(ANGLE_RANGE, angle))  # safety clamp
    return int(PULSE_MIN + (angle / ANGLE_RANGE) * (PULSE_MAX - PULSE_MIN))


# ===============================
# SAFE START
# ===============================
def send_standing_pulses():
    for ch, angle in SERVO_MAP.items():
        set_pulse(ch, angle_to_pulse(angle))


# ===============================
# GUI CALLBACKS
# ===============================
def slider_callback(channel, val):
    set_pulse(channel, angle_to_pulse(float(val)))


def move_to_standing():
    for ch, angle in SERVO_MAP.items():
        sliders[ch].set(angle)


# ===============================
# PROGRAM START
# ===============================
init_pca()
send_standing_pulses()

# ===============================
# GUI SETUP
# ===============================
root = tk.Tk()
root.title("Quadruped Servo Control – PCA9685 (302° Calibrated)")

sliders = {}

for i in range(12):
    frame = tk.Frame(root)
    frame.grid(row=i//2, column=i%2, padx=10, pady=5, sticky="w")

    label = tk.Label(frame, text=f"CH {i} ({SERVO_NAMES[i]})")
    label.pack(anchor="w")

    slider = tk.Scale(
        frame,
        from_=0,
        to=int(ANGLE_RANGE),   # ← 302°
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda val, ch=i: slider_callback(ch, val)
    )
    slider.pack()

    sliders[i] = slider

move_to_standing()

btn = tk.Button(
    root,
    text="Move to Standing Position",
    command=move_to_standing
)
btn.grid(row=6, column=0, columnspan=2, pady=15)

root.mainloop()
