import time
from smbus2 import SMBus

# ===============================
# PCA9685 CONFIG
# ===============================
BUS = 7
ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE

CHANNEL = 0        # ONLY channel 0
PULSE = 561        # Raw PWM pulse (0–4095)

bus = SMBus(BUS)

# ===============================
# PCA9685 INIT
# ===============================
def init_pca():
    bus.write_byte_data(ADDR, MODE1, 0x00)
    time.sleep(0.01)

    prescale = int(25000000 / (4096 * 50) - 1)  # 50 Hz
    bus.write_byte_data(ADDR, MODE1, 0x10)      # sleep
    bus.write_byte_data(ADDR, PRESCALE, prescale)
    bus.write_byte_data(ADDR, MODE1, 0x00)
    time.sleep(0.005)
    bus.write_byte_data(ADDR, MODE1, 0x80)      # restart

# ===============================
# SET RAW PWM PULSE
# ===============================
def set_pulse(channel, pulse):
    base = 0x06 + 4 * channel
    bus.write_byte_data(ADDR, base + 0, 0x00)          # ON_L
    bus.write_byte_data(ADDR, base + 1, 0x00)          # ON_H
    bus.write_byte_data(ADDR, base + 2, pulse & 0xFF)  # OFF_L
    bus.write_byte_data(ADDR, base + 3, pulse >> 8)    # OFF_H

# ===============================
# MAIN
# ===============================
init_pca()
set_pulse(CHANNEL, PULSE)

print(f"Sent PWM pulse {PULSE} to channel {CHANNEL}")
