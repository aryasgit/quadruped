# hardware/i2c_bus.py
"""
Layer 1.1 — I2C BUS OWNER
=========================

Single authoritative SMBus instance for the entire robot.

Responsibilities:
- Open the I2C bus defined in absolute_truths.py
- Own bus lifetime
- Provide a getter so all drivers share the same bus

BUG FIXES (vs the original build) — behaviour of callers is unchanged:
- The shared SMBus is now wrapped so EVERY read/write is serialized by a single
  lock. smbus2 is not thread-safe and the robot interleaves servo writes (main
  loop + trick threads) on one handle; the unlocked bus was the root cause of the
  intermittent "Errno 121 (Remote I/O)" storms that crashed the old build.
- All reads AND writes now get a bounded, escalating retry on transient EREMOTEIO
  (the Tegra bus has genuine multi-millisecond NAK bursts). Only a persistent
  fault re-raises, so real wiring/power failures still surface.

Drivers keep calling `bus.write_byte_data(...)` / `bus.read_byte_data(...)`
exactly as before — they transparently get the lock + retry.
"""

import threading
import time

from smbus2 import SMBus
from hardware.absolute_truths import BUS

# Transient-NAK retry policy (shared by reads and writes).
_RETRIES = 8
_RETRY_DELAY_S = 0.0008


class _LockingRetryingBus:
    """Wraps an SMBus: serializes access with a lock and retries transient NAKs."""

    def __init__(self, smbus):
        self._bus = smbus
        self._lock = threading.RLock()

    def _retry(self, fn, *args):
        with self._lock:
            for attempt in range(_RETRIES + 1):
                try:
                    return fn(*args)
                except OSError:
                    if attempt >= _RETRIES:
                        raise
                    time.sleep(_RETRY_DELAY_S * (attempt + 1))  # escalating backoff

    def write_byte_data(self, addr, reg, value):
        return self._retry(self._bus.write_byte_data, addr, reg, value)

    def read_byte_data(self, addr, reg):
        return self._retry(self._bus.read_byte_data, addr, reg)

    def read_i2c_block_data(self, addr, reg, length):
        return self._retry(self._bus.read_i2c_block_data, addr, reg, length)

    def write_i2c_block_data(self, addr, reg, data):
        return self._retry(self._bus.write_i2c_block_data, addr, reg, data)

    def close(self):
        with self._lock:
            self._bus.close()

    def __getattr__(self, name):
        # Any other SMBus method passes through (unlocked); the ones above cover
        # every call the drivers actually make.
        return getattr(self._bus, name)


# Private singleton bus instance
__bus = None


def get_i2c_bus():
    """
    Return the shared, lock+retry-wrapped I2C bus.

    Creates the bus on first call, reuses it thereafter.
    All hardware drivers MUST use this function.
    """
    global __bus
    if __bus is None:
        __bus = _LockingRetryingBus(SMBus(BUS))
    return __bus


def close_i2c_bus():
    """Explicitly close the I2C bus (rarely needed)."""
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
