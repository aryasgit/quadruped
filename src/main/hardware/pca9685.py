"""
L1 — PCA9685 SERVO DRIVER
=========================

Angle (physical servo degrees) -> PWM pulse -> bus write. All I2C goes through
the locked, guarded hardware.bus. No leg/gait/IK knowledge.
"""

import time

from config.robot_spec import (
    PCA_ADDR, PCA_MODE1, PCA_PRESCALE, PWM_FREQ_HZ,
    PULSE_MIN, PULSE_MAX, SERVO_TRAVEL_DEG, CHANNEL,
)
from hardware import bus

_initialized = False


def init():
    """Configure the PCA9685 for servo PWM. Idempotent."""
    global _initialized
    if _initialized:
        return
    prescale = int(round(25_000_000.0 / (4096 * PWM_FREQ_HZ) - 1))
    bus.write_byte(PCA_ADDR, PCA_MODE1, 0x00)
    time.sleep(0.01)
    bus.write_byte(PCA_ADDR, PCA_MODE1, 0x10)          # sleep
    bus.write_byte(PCA_ADDR, PCA_PRESCALE, prescale)   # set frame rate
    bus.write_byte(PCA_ADDR, PCA_MODE1, 0x00)          # wake
    time.sleep(0.005)
    bus.write_byte(PCA_ADDR, PCA_MODE1, 0x80)          # restart
    time.sleep(0.005)
    _initialized = True


def angle_to_pulse(angle_deg):
    a = 0.0 if angle_deg < 0 else SERVO_TRAVEL_DEG if angle_deg > SERVO_TRAVEL_DEG else angle_deg
    pulse = int(round(PULSE_MIN + (a / SERVO_TRAVEL_DEG) * (PULSE_MAX - PULSE_MIN)))
    return max(PULSE_MIN, min(PULSE_MAX, pulse))


def set_channel(channel, angle_deg):
    if not _initialized:
        init()
    pulse = angle_to_pulse(angle_deg)
    base = 0x06 + 4 * channel
    bus.write_byte(PCA_ADDR, base, 0x00)               # ON_L
    bus.write_byte(PCA_ADDR, base + 1, 0x00)           # ON_H
    bus.write_byte(PCA_ADDR, base + 2, pulse & 0xFF)   # OFF_L
    bus.write_byte(PCA_ADDR, base + 3, (pulse >> 8) & 0x0F)  # OFF_H


def apply_pose(servo_degrees):
    """servo_degrees: {joint_name: physical degrees}. Writes all mapped joints."""
    for joint, ch in CHANNEL.items():
        if joint in servo_degrees:
            set_channel(ch, servo_degrees[joint])
