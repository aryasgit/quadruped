import time, math
from smbus2 import SMBus

BUS = 7
PCA_ADDR = 0x40

MODE1 = 0x00
PRESCALE = 0xFE

PULSE_MIN = 80
PULSE_MAX = 561

# FEET ONLY
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

SIGN = {
    "FRF": +1,
    "FLF": -1,
    "RRF": +1,
    "RLF": -1,
}

bus = SMBus(BUS)

def angle_to_pulse(angle):
    return int(PULSE_MIN + (angle / 270.0) * (PULSE_MAX - PULSE_MIN))

def set_servo_angle(ch, angle):
    p = angle_to_pulse(angle)
    base = 0x06 + 4 * ch
    bus.write_byte_data(PCA_ADDR, base, 0)
    bus.write_byte_data(PCA_ADDR, base+1, 0)
    bus.write_byte_data(PCA_ADDR, base+2, p & 0xFF)
    bus.write_byte_data(PCA_ADDR, base+3, (p >> 8) & 0x0F)

# PCA INIT
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
time.sleep(0.01)
prescale = int(25000000 / (4096 * 50) - 1)
bus.write_byte_data(PCA_ADDR, MODE1, 0x10)
bus.write_byte_data(PCA_ADDR, PRESCALE, prescale)
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
bus.write_byte_data(PCA_ADDR, MODE1, 0x80)

# Move to stand
for leg, ch in FEET.items():
    set_servo_angle(ch, STAND[leg])
    time.sleep(0.05)

print("SOFTWARE PITCH WAVE ACTIVE")

t = 0.0

while True:
    # fake pitch in degrees
    pitch = 10.0 * math.sin(t)
    t += 0.1

    print(f"pitch = {pitch:5.2f}")

    for leg, ch in FEET.items():
        direction = +1 if leg.startswith("FR") or leg.startswith("FL") else -1
        angle = STAND[leg] + SIGN[leg] * direction * pitch
        set_servo_angle(ch, angle)

    time.sleep(0.05)

