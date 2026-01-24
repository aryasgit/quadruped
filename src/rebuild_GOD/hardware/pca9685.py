"""
Layer 1.2 — PCA9685 SERVO DRIVER
===============================

Authoritative low-level driver for PCA9685.

Responsibilities:
- Initialize PCA9685 at 50 Hz
- Convert mechanical angle (degrees) -> PWM pulse
- Write pulse to PCA channel

NON-RESPONSIBILITIES:
- No leg semantics
- No gait logic
- No IK
- No balance
- No timing loops

This driver trusts Layer 0 for all electrical and mechanical truths.
"""

import time
from hardware.i2c_bus import get_i2c_bus
from hardware.absolute_truths import (
    PCA_ADDR,
    MODE1,
    PRESCALE,
    PULSE_MIN,
    PULSE_MAX,
)

# Internal state
_initialized = False


def init_pca():
    """
    Initialize PCA9685 for 50 Hz servo operation.
    This must be called once at startup.
    """
    global _initialized
    if _initialized:
        return

    bus = get_i2c_bus()

    # Reset
    bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
    time.sleep(0.01)

    # Set prescale for 50 Hz
    prescale = int(25_000_000 / (4096 * 50) - 1)

    bus.write_byte_data(PCA_ADDR, MODE1, 0x10)      # sleep
    bus.write_byte_data(PCA_ADDR, PRESCALE, prescale)
    bus.write_byte_data(PCA_ADDR, MODE1, 0x00)      # wake
    bus.write_byte_data(PCA_ADDR, MODE1, 0x80)      # restart

    time.sleep(0.01)
    _initialized = True


# ----------------------------
# Angle → PWM conversion
# ----------------------------

def angle_to_pulse(angle_deg: float) -> int:
    """
    Convert a mechanical angle in degrees to PCA9685 pulse value.

    Assumes servo travel is 270° (as per Layer 0 truth).
    Clamps to electrical limits.
    """
    if angle_deg < 0:
        angle_deg = 0.0
    elif angle_deg > 270:
        angle_deg = 270.0

    pulse = int(PULSE_MIN + (angle_deg / 270.0) * (PULSE_MAX - PULSE_MIN))

    if pulse < PULSE_MIN:
        pulse = PULSE_MIN
    elif pulse > PULSE_MAX:
        pulse = PULSE_MAX

    return pulse


# ----------------------------
# Output primitive
# ----------------------------

def set_servo_angle(channel: int, angle_deg: float):
    """
    Set a single PCA9685 channel to a mechanical angle (degrees).
    """
    if not _initialized:
        init_pca()

    bus = get_i2c_bus()
    pulse = angle_to_pulse(angle_deg)

    base = 0x06 + 4 * channel
    bus.write_byte_data(PCA_ADDR, base, 0x00)               # ON_L
    bus.write_byte_data(PCA_ADDR, base + 1, 0x00)           # ON_H
    bus.write_byte_data(PCA_ADDR, base + 2, pulse & 0xFF)   # OFF_L
    bus.write_byte_data(PCA_ADDR, base + 3, (pulse >> 8))   # OFF_H


# ---- Smoke test ----
if __name__ == "__main__":
    print("[PCA] Initializing...")
    init_pca()
    print("[PCA] Sweeping channel 0")
    for a in (0, 90, 180, 270, 180, 90):
        set_servo_angle(0, a)
        time.sleep(0.5)
