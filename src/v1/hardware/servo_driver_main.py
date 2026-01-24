import time
from smbus2 import SMBus

# ===============================
BUS = 7
ADDR = 0x40

MODE1 = 0x00
PRESCALE = 0xFE

PULSE_MIN = 80
PULSE_MAX = 561

bus = SMBus(BUS)

# ===============================
SERVO_MAP = {
    0: 38,  # RRS
    1: 38,  # RLS
    2: 102,
    3: 154,
    4: 142,
    5: 61,
    6: 41,  # FRS
    7: 41,  # FLS
    8: 105,
    9: 156,
    10: 122,
    11: 73
}

SERVO_NAMES = {
    0: "RRS", 1: "RLS", 2: "RRM", 3: "RLM",
    4: "RRF", 5: "RLF", 6: "FRS", 7: "FLS",
    8: "FRM", 9: "FLM", 10: "FRF", 11: "FLF"
}

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

def angle_to_pulse(angle):
    return int(PULSE_MIN + (angle / 270.0) * (PULSE_MAX - PULSE_MIN))

def set_servo_angle(ch, angle):
    pulse = angle_to_pulse(angle)
    base = 0x06 + 4 * ch
    bus.write_byte_data(ADDR, base, 0)
    bus.write_byte_data(ADDR, base + 1, 0)
    bus.write_byte_data(ADDR, base + 2, pulse & 0xFF)
    bus.write_byte_data(ADDR, base + 3, (pulse >> 8) & 0x0F)

    print(f"SERVO → CH{ch} ({SERVO_NAMES[ch]}) → {angle:.2f}°")

def move_to_standing():
    print("Moving to standing pose")
    for ch, ang in SERVO_MAP.items():
        set_servo_angle(ch, ang)
        time.sleep(0.02)

# ===============================
init_pca()
move_to_standing()

print("Servo driver ACTIVE (waiting)")

while True:
    time.sleep(1)

