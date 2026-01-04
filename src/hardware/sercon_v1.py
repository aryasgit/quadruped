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

PULSE_MIN = 80
PULSE_MAX = 561

bus = SMBus(BUS)

# ===============================
# SERVO MAPPING (Channel : Standing Angle)
# ===============================
SERVO_MAP = {
    0: 38,   # RRS
    1: 37,   # RLS
    2: 102,   # RRM
    3: 154,  # RLM
    4: 142,  # RRF
    5: 61,   # RLF
    6: 37,   # FRS
    7: 46,   # FLS
    8: 85,   # FRM
    9: 176,  # FLM
    10: 137, # FRF
    11: 61   # FLF
}

SERVO_NAMES = [
    "RRS", "RLS", "RRM", "RLM", "RRF", "RLF",
    "FRS", "FLS", "FRM", "FLM", "FRF", "FLF"
]

# ===============================
# PCA FUNCTIONS (MUST COME FIRST)
# ===============================
def init_pca():
    bus.write_byte_data(ADDR, MODE1, 0x00)
    time.sleep(0.01)

    prescale = int(25000000 / (4096 * 50) - 1)
    bus.write_byte_data(ADDR, MODE1, 0x10)  # sleep
    bus.write_byte_data(ADDR, PRESCALE, prescale)
    bus.write_byte_data(ADDR, MODE1, 0x00)
    time.sleep(0.005)
    bus.write_byte_data(ADDR, MODE1, 0x80)  # restart


def set_pulse(channel, pulse):
    base = 0x06 + 4 * channel
    bus.write_byte_data(ADDR, base + 0, 0x00)
    bus.write_byte_data(ADDR, base + 1, 0x00)
    bus.write_byte_data(ADDR, base + 2, pulse & 0xFF)
    bus.write_byte_data(ADDR, base + 3, (pulse >> 8) & 0x0F)


def angle_to_pulse(angle):
    return int(PULSE_MIN + (angle / 270.0) * (PULSE_MAX - PULSE_MIN))


# ===============================
# HARDWARE-ONLY SAFE START
# ===============================
def send_standing_pulses():
    for ch, angle in SERVO_MAP.items():
        pulse = angle_to_pulse(angle)
        set_pulse(ch, pulse)


# ===============================
# GUI CALLBACKS
# ===============================
def slider_callback(channel, val):
    pulse = angle_to_pulse(float(val))
    set_pulse(channel, pulse)


def move_to_standing():
    for ch, angle in SERVO_MAP.items():
        sliders[ch].set(angle)


# ===============================
# PROGRAM START (ORDER IS CRITICAL)
# ===============================
init_pca()
send_standing_pulses()   # ← FIRST pulses servos ever see

# ===============================
# GUI SETUP
# ===============================
root = tk.Tk()
root.title("Quadruped Servo Control – PCA9685")

sliders = {}

for i in range(12):
    frame = tk.Frame(root)
    frame.grid(row=i//2, column=i%2, padx=10, pady=5, sticky="w")

    label = tk.Label(frame, text=f"CH {i} ({SERVO_NAMES[i]})")
    label.pack(anchor="w")

    slider = tk.Scale(
        frame,
        from_=0,
        to=270,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda val, ch=i: slider_callback(ch, val)
    )
    slider.pack()

    sliders[i] = slider

# Sync GUI with already-set standing pose
move_to_standing()

btn = tk.Button(root, text="Move to Standing Position", command=move_to_standing)
btn.grid(row=6, column=0, columnspan=2, pady=15)

root.mainloop()

