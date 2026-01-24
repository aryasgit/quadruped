import time
from smbus2 import SMBus

from absolute_truths import (
    BUS, PCA_ADDR,
    MODE1, PRESCALE,
    PULSE_MIN, PULSE_MAX,

    WRISTS, THIGHS, COXA,
    WRIST_MECH, THIGH_MECH, COXA_MECH,
    WRIST_STAND, THIGH_STAND, COXA_STAND,
)

# ===============================
# BUS
# ===============================
bus = SMBus(BUS)

# ===============================
# SERVO COMMAND (AUTHORITATIVE)
# ===============================
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
# PCA INIT (MATCH CORE)
# ===============================
def init_pca():
    bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
    time.sleep(0.01)
    prescale = int(25000000 / (4096 * 50) - 1)
    bus.write_byte_data(PCA_ADDR, MODE1, 0x10)
    bus.write_byte_data(PCA_ADDR, PRESCALE, prescale)
    bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
    bus.write_byte_data(PCA_ADDR, MODE1, 0x80)
    time.sleep(0.01)

# ===============================
# POSES
# ===============================
def go_perpendicular():
    print("[POSE] PERPENDICULAR")

    for k, ch in WRISTS.items():
        m = WRIST_MECH[k]
        set_servo_angle(ch, m["perp"])

    for k, ch in THIGHS.items():
        m = THIGH_MECH[k]
        set_servo_angle(ch, m["perp"])

    for k, ch in COXA.items():
        m = COXA_MECH[k]
        set_servo_angle(ch, m["perp"])

def go_stand():
    print("[POSE] STAND")

    for k, ch in WRISTS.items():
        set_servo_angle(ch, WRIST_STAND[k])

    for k, ch in THIGHS.items():
        set_servo_angle(ch, THIGH_STAND[k])

    for k, ch in COXA.items():
        set_servo_angle(ch, COXA_STAND[k])

# ===============================
# CLI
# ===============================
def main():
    init_pca()
    print("\n--- CALIBRATION CLI ---")
    print("p → perpendicular")
    print("s → stand")
    print("q → quit\n")

    while True:
        cmd = input("> ").strip().lower()
        if cmd == "p":
            go_perpendicular()
        elif cmd == "s":
            go_stand()
        elif cmd == "q":
            break
        else:
            print("unknown command")

if __name__ == "__main__":
    main()
