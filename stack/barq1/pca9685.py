"""
Layer 1 — PCA9685 DRIVER (from scratch)
=======================================

Owns the PCA9685: init at 50 Hz, write pulse ticks to channels, turn
outputs off. Nothing else — no leg semantics, no IK, no gait, no loops.

Improvements over the legacy driver (src/main/hardware/pca9685.py):
- Canonical unit is PCA9685 *ticks*, the unit every measured truth is in.
  Microseconds and the legacy 0-270 "servo angle" are derived views.
- Each servo update is ONE auto-increment block write (legacy did four
  single-byte writes per update: 4x the bus traffic, torn-write window).
- Outputs can be switched OFF (full-off bit), so a servo can free-wheel
  for hand positioning. Init forces ALL outputs off — nothing moves on
  startup (the legacy GUI snapped all 12 servos to stand pose at launch).
- Instance-based and thread-safe; optional simulation bus for desk work.
"""

import threading
import time

from barq1.truths import (
    I2C_BUS,
    PCA_ADDR,
    PWM_FREQ_HZ,
    PULSE_MIN,
    PULSE_MAX,
    SERVO_TRAVEL_DEG,
)

# PCA9685 register map (datasheet facts)
_MODE1 = 0x00
_MODE2 = 0x01
_LED0_ON_L = 0x06          # + 4*channel
_ALL_LED_ON_L = 0xFA
_PRESCALE = 0xFE

# MODE1 bits
_BIT_RESTART = 0x80
_BIT_AI = 0x20             # register auto-increment
_BIT_SLEEP = 0x10

# MODE2 bits
_BIT_OUTDRV = 0x04         # totem-pole outputs

# LEDn_OFF_H bit 4: full OFF (output disabled, no pulses)
_BIT_FULL_OFF = 0x10

_OSC_HZ = 25_000_000       # nominal internal oscillator
_NUM_CHANNELS = 16


# ---------------------------------------------------------------------------
# Unit conversions (pure functions)
# ---------------------------------------------------------------------------

def ticks_to_us(ticks: float, freq_hz: int = PWM_FREQ_HZ) -> float:
    """PCA9685 ticks -> pulse width in microseconds (nominal oscillator)."""
    return ticks * 1_000_000.0 / (4096 * freq_hz)


def us_to_ticks(us: float, freq_hz: int = PWM_FREQ_HZ) -> float:
    """Pulse width in microseconds -> PCA9685 ticks (nominal oscillator)."""
    return us * 4096 * freq_hz / 1_000_000.0


def legacy_deg_to_ticks(deg: float) -> int:
    """Legacy 0-270 'servo angle' -> ticks.

    Reproduces the legacy angle_to_pulse() exactly, including its int()
    truncation, so every perp/stand/mech truth maps to the same pulse the
    robot was calibrated with.
    """
    if deg < 0.0:
        deg = 0.0
    elif deg > SERVO_TRAVEL_DEG:
        deg = SERVO_TRAVEL_DEG
    return int(PULSE_MIN + (deg / SERVO_TRAVEL_DEG) * (PULSE_MAX - PULSE_MIN))


def ticks_to_legacy_deg(ticks: float) -> float:
    """Ticks -> legacy 0-270 'servo angle' (continuous, for display)."""
    return (ticks - PULSE_MIN) * SERVO_TRAVEL_DEG / (PULSE_MAX - PULSE_MIN)


# ---------------------------------------------------------------------------
# Buses
# ---------------------------------------------------------------------------

class SimBus:
    """Drop-in SMBus stand-in: records register writes, answers reads."""

    def __init__(self):
        self.regs = {}

    def write_byte_data(self, addr, reg, val):
        self.regs[(addr, reg)] = val & 0xFF

    def write_i2c_block_data(self, addr, reg, data):
        for i, b in enumerate(data):
            self.regs[(addr, reg + i)] = b & 0xFF

    def read_byte_data(self, addr, reg):
        return self.regs.get((addr, reg), 0)

    def close(self):
        pass


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

class PCA9685:
    """One PCA9685 chip. All public methods are thread-safe."""

    def __init__(self, bus_num: int = I2C_BUS, addr: int = PCA_ADDR,
                 freq_hz: int = PWM_FREQ_HZ, sim: bool = False):
        self.addr = addr
        self.bus_num = bus_num
        self.freq_hz = freq_hz
        self.sim = sim
        self._lock = threading.Lock()
        self._last = {ch: None for ch in range(_NUM_CHANNELS)}  # None = output off

        if sim:
            self.bus = SimBus()
        else:
            from smbus2 import SMBus  # imported here so sim mode needs no smbus2
            self.bus = SMBus(bus_num)

        self._init_chip()

    # -- low level ----------------------------------------------------------

    def _w8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _wblock(self, reg, data):
        self.bus.write_i2c_block_data(self.addr, reg, data)

    def _init_chip(self):
        # Same prescale formula as the legacy driver (int truncation -> 121
        # at 50 Hz). The measured truths were taken against this prescale;
        # changing it would invalidate them.
        prescale = int(_OSC_HZ / (4096 * self.freq_hz) - 1)
        with self._lock:
            self._w8(_MODE1, _BIT_AI)                 # wake, auto-increment on
            self._w8(_MODE2, _BIT_OUTDRV)             # totem-pole outputs
            self._w8(_MODE1, _BIT_AI | _BIT_SLEEP)    # sleep to set prescale
            self._w8(_PRESCALE, prescale)
            self._w8(_MODE1, _BIT_AI)                 # wake
            time.sleep(0.005)
            self._w8(_MODE1, _BIT_AI | _BIT_RESTART)  # restart PWM
            self._all_off_locked()                    # SAFE START: no pulses

    def _all_off_locked(self):
        self._wblock(_ALL_LED_ON_L, [0x00, 0x00, 0x00, _BIT_FULL_OFF])
        self._last = {ch: None for ch in range(_NUM_CHANNELS)}

    # -- public API ---------------------------------------------------------

    def set_ticks(self, channel: int, ticks: int, clamp: bool = True) -> int:
        """Command a pulse of `ticks` on `channel`. Returns the value written.

        clamp=True bounds to the measured electrical range of the DS3240MG
        (PULSE_MIN..PULSE_MAX). Pass clamp=False only for bench experiments.
        """
        if not 0 <= channel < _NUM_CHANNELS:
            raise ValueError(f"channel {channel} out of range 0..15")
        ticks = int(round(ticks))
        if clamp:
            ticks = max(PULSE_MIN, min(PULSE_MAX, ticks))
        ticks = max(0, min(4095, ticks))
        base = _LED0_ON_L + 4 * channel
        with self._lock:
            self._wblock(base, [0x00, 0x00, ticks & 0xFF, (ticks >> 8) & 0x0F])
            self._last[channel] = ticks
        return ticks

    def off(self, channel: int):
        """Stop pulses on one channel — servo goes limp (hand-movable)."""
        if not 0 <= channel < _NUM_CHANNELS:
            raise ValueError(f"channel {channel} out of range 0..15")
        base = _LED0_ON_L + 4 * channel
        with self._lock:
            self._wblock(base, [0x00, 0x00, 0x00, _BIT_FULL_OFF])
            self._last[channel] = None

    def all_off(self):
        """Stop pulses on ALL channels at once (panic / safe state)."""
        with self._lock:
            self._all_off_locked()

    @property
    def last_commanded(self) -> dict:
        """{channel: ticks or None} of the last value written per channel."""
        with self._lock:
            return dict(self._last)

    def mode1(self) -> int:
        """Read back MODE1 — cheap liveness check for diagnostics."""
        with self._lock:
            return self.bus.read_byte_data(self.addr, _MODE1)

    def close(self):
        with self._lock:
            self.bus.close()


# ---- Smoke test (sim only — never moves hardware by itself) ----
if __name__ == "__main__":
    pca = PCA9685(sim=True)
    print(f"[PCA] sim init ok, MODE1=0x{pca.mode1():02x}")
    wrote = pca.set_ticks(0, legacy_deg_to_ticks(135))
    print(f"[PCA] ch0 <- {wrote} ticks = {ticks_to_us(wrote):.1f} us "
          f"= {ticks_to_legacy_deg(wrote):.1f} legacy-deg")
    pca.all_off()
    print(f"[PCA] all off: {pca.last_commanded[0]=}")
