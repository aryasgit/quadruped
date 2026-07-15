"""
L1 — I2C BUS OWNER (locked, uniformly guarded)
==============================================

The one and only I2C handle for the whole robot. Every device driver goes
through here. Two hard rules that fix the old stack's #1 failure class:

  1. ALL access is serialized by a single lock. smbus2 is not thread-safe and
     the robot interleaves IMU reads with servo writes; the unlocked shared bus
     was the ROOT CAUSE of the intermittent Errno-121 (EREMOTEIO) storms.
  2. ALL reads AND writes get the SAME bounded retry-on-transient policy, then
     re-raise on a persistent fault (so real wiring/power failures still surface).
     The old stack retried servo writes but not IMU reads — asymmetric and fragile.

This is the only hardware-stateful, thread-shared module.
"""

import threading
import time

from smbus2 import SMBus
from config.robot_spec import I2C_BUS

# The Jetson I2C bus has genuine electrical NAK bursts (EREMOTEIO) that can last
# several ms even with no bus contention (the lock already removed the
# software-induced storms). Retry deeper with a short escalating backoff so a
# transient burst doesn't surface as a fault.
_RETRIES = 8
_RETRY_DELAY_S = 0.0008

_bus = None
_lock = threading.RLock()


def _get():
    global _bus
    if _bus is None:
        _bus = SMBus(I2C_BUS)
    return _bus


def _retry(fn):
    """Run an SMBus op under the lock with bounded retry on transient EREMOTEIO."""
    with _lock:
        for attempt in range(_RETRIES + 1):
            try:
                return fn(_get())
            except OSError:
                if attempt >= _RETRIES:
                    raise
                time.sleep(_RETRY_DELAY_S * (attempt + 1))   # escalating backoff


def write_byte(addr, reg, value):
    _retry(lambda b: b.write_byte_data(addr, reg, value))


def read_byte(addr, reg):
    return _retry(lambda b: b.read_byte_data(addr, reg))


def read_word(addr, reg_high):
    """Read a big-endian signed 16-bit word (two consecutive registers)."""
    def op(b):
        hi = b.read_byte_data(addr, reg_high)
        lo = b.read_byte_data(addr, reg_high + 1)
        v = (hi << 8) | lo
        return v - 65536 if v >= 0x8000 else v
    return _retry(op)


def close():
    global _bus
    with _lock:
        if _bus is not None:
            _bus.close()
            _bus = None
