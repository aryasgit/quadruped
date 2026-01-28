"""
Layer 1.1 — I2C BUS OWNER
=========================

Single authoritative SMBus instance for the entire robot.

Responsibilities:
- Open the I2C bus defined in absolute_truths.py
- Own bus lifetime
- Provide a getter so all drivers share the same bus

NON-RESPONSIBILITIES (forbidden here):
- No PCA9685 logic
- No MPU6050 logic
- No retries, filters, or control policy
- No robot semantics

If something breaks here, the failure is electrical, not logical.
"""

from smbus2 import SMBus
from hardware.absolute_truths import BUS

# Private singleton bus instance
__bus = None


def get_i2c_bus():
    """
    Return the shared SMBus instance.

    Creates the bus on first call, reuses it thereafter.
    All hardware drivers MUST use this function.
    """
    global __bus
    if __bus is None:
        __bus = SMBus(BUS)
    return __bus


def close_i2c_bus():
    """
    Explicitly close the I2C bus (rarely needed).
    Provided for completeness and clean shutdowns.
    """
    global __bus
    if __bus is not None:
        __bus.close()
        __bus = None


# ---- Smoke test (manual use only) ----
if __name__ == "__main__":
    bus = get_i2c_bus()
    print(f"[I2C] Opened bus {BUS}: {bus}")
    close_i2c_bus()
    print("[I2C] Closed bus")
