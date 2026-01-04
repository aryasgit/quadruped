from smbus2 import SMBus
import time
class PCA9685:
    # Registers
    MODE1      = 0x00
    MODE2      = 0x01
    PRESCALE   = 0xFE
    LED0_ON_L  = 0x06
    def __init__(self, bus=7, addr=0x40, freq=50):
        """
        Jetson-safe PCA9685 driver
        - bus=7 for Orin Nano
        - addr=0x40 default
        - freq=50Hz for servos
        """
        self.bus_id = bus
        self.addr = addr
        self.bus = SMBus(bus)
        # Let hardware settle (VERY IMPORTANT on Jetson)
        time.sleep(0.1)
        self._reset_safe()
        self.set_pwm_freq(freq)
    # --------------------------------------------------
    # Low-level helpers
    # --------------------------------------------------
    def _write_byte_retry(self, reg, value, retries=5, delay=0.05):
        for _ in range(retries):
            try:
                self.bus.write_byte_data(self.addr, reg, value)
                return
            except OSError:
                time.sleep(delay)
        raise RuntimeError(f"I2C write failed at reg 0x{reg:02X}")
    def _read_byte_retry(self, reg, retries=5, delay=0.05):
        for _ in range(retries):
            try:
                return self.bus.read_byte_data(self.addr, reg)
            except OSError:
                time.sleep(delay)
        raise RuntimeError(f"I2C read failed at reg 0x{reg:02X}")
    # --------------------------------------------------
    # Device init
    # --------------------------------------------------
    def _reset_safe(self):
        """
        Reset MODE1 safely with retry.
        Prevents Errno 121 on Jetson.
        """
        self._write_byte_retry(self.MODE1, 0x00)
        time.sleep(0.01)
    def set_pwm_freq(self, freq_hz):
        """
        Set PWM frequency (50Hz for servos)
        """
        prescale = int(round(25000000.0 / (4096 * freq_hz)) - 1)
        old_mode = self._read_byte_retry(self.MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10  # sleep
        self._write_byte_retry(self.MODE1, sleep_mode)
        self._write_byte_retry(self.PRESCALE, prescale)
        self._write_byte_retry(self.MODE1, old_mode)
        time.sleep(0.005)
        # Restart
        self._write_byte_retry(self.MODE1, old_mode | 0x80)
    # --------------------------------------------------
    # PWM control
    # --------------------------------------------------
    def set_pwm(self, channel, on, off):
        """
        Set raw PWM counts.
        on/off must be 0–4095
        """
        if not (0 <= channel <= 15):
            raise ValueError("Channel must be 0–15")
        on  = max(0, min(4095, int(on)))
        off = max(0, min(4095, int(off)))
        base = self.LED0_ON_L + 4 * channel
        for _ in range(5):
            try:
                self.bus.write_i2c_block_data(
                    self.addr,
                    base,
                    [
                        on & 0xFF,
                        (on >> 8) & 0xFF,
                        off & 0xFF,
                        (off >> 8) & 0xFF,
                    ],
                )
                return
            except OSError:
                time.sleep(0.02)
        raise RuntimeError(f"I2C PWM write failed on channel {channel}")
    # --------------------------------------------------
    # Cleanup
    # --------------------------------------------------
    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass
