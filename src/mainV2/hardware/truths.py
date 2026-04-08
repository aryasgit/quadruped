# hardware/truths.py
"""
Layer 1.0 — HARDWARE TRUTHS
============================

Single source of truth for every physical, electrical, and
mechanical constant in the robot.

Contents:
    - I2C bus configuration and thread-safe bus singleton
    - Device addresses (all sensors present and future)
    - Register maps for MPU6050, HMC5883L, QMC5883L, BMP180, PCA9685
    - Servo channel assignments (leg-centric, uniform keys)
    - Mechanical limits, perpendicular angles, stand pose (measured)
    - Scale factors, timing constants, identification values

Rules:
    1. ALL values are physically measured or from datasheets.
    2. ALL dicts are frozen (MappingProxyType) — writes raise TypeError.
    3. ALL scalar groups are frozen namespaces — writes raise AttributeError.
    4. NO logic, NO math, NO imports of other project modules.
    5. DO NOT modify measured servo values. They are tuned on the real robot.

Three files in the hardware layer:
    truths.py        ← this file
    imu_driver.py    ← IMU lifecycle (init, calibrate, filter)
    servo_driver.py  ← servo lifecycle (init, command, limits)
"""

import threading
from types import MappingProxyType


# ╔══════════════════════════════════════════════════════════════╗
# ║  IMMUTABILITY INFRASTRUCTURE                                 ║
# ╚══════════════════════════════════════════════════════════════╝

def _freeze(obj):
    """
    Recursively convert dicts to MappingProxyType.
    Tuples and primitives pass through unchanged.
    """
    if isinstance(obj, dict):
        return MappingProxyType({k: _freeze(v) for k, v in obj.items()})
    if isinstance(obj, list):
        return tuple(_freeze(item) for item in obj)
    return obj


class _FrozenNamespace:
    """
    Immutable attribute container for grouped scalar constants.

    After construction, any attempt to set or delete an attribute
    raises AttributeError. Dict values passed in are deep-frozen
    via MappingProxyType automatically.

    Usage:
        FOO = _FrozenNamespace(BAR=42, BAZ=0x1A)
        FOO.BAR      → 42
        FOO.BAR = 0  → AttributeError: Cannot modify hardware truth 'BAR'
    """

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            if isinstance(value, dict):
                value = _freeze(value)
            elif isinstance(value, list):
                value = tuple(value)
            object.__setattr__(self, key, value)
        object.__setattr__(self, "_frozen", True)

    def __setattr__(self, name, value):
        if object.__getattribute__(self, "_frozen"):
            raise AttributeError(
                f"Cannot modify hardware truth '{name}'"
            )
        object.__setattr__(self, name, value)

    def __delattr__(self, name):
        raise AttributeError(
            f"Cannot delete hardware truth '{name}'"
        )

    def __repr__(self):
        items = {
            k: v for k, v in self.__dict__.items() if k != "_frozen"
        }
        return f"HardwareTruths({items})"

    def __iter__(self):
        """Iterate over (name, value) pairs for introspection."""
        for k, v in self.__dict__.items():
            if k != "_frozen":
                yield k, v


# ╔══════════════════════════════════════════════════════════════╗
# ║  I2C BUS — CONFIGURATION AND SINGLETON                      ║
# ╚══════════════════════════════════════════════════════════════╝

I2C_BUS_NUMBER = 7

_bus = None
_bus_lock = threading.Lock()


def get_bus():
    """
    Return the shared SMBus instance (lazy, thread-safe).

    Every hardware driver in the stack MUST use this function.
    Never construct your own SMBus — this is the single owner.
    """
    global _bus
    if _bus is None:
        with _bus_lock:
            if _bus is None:
                from smbus2 import SMBus
                _bus = SMBus(I2C_BUS_NUMBER)
    return _bus


def close_bus():
    """
    Close the shared SMBus. Call ONLY at full shutdown.
    """
    global _bus
    with _bus_lock:
        if _bus is not None:
            _bus.close()
            _bus = None


# ╔══════════════════════════════════════════════════════════════╗
# ║  DEVICE ADDRESSES                                            ║
# ╚══════════════════════════════════════════════════════════════╝

ADDR = _FrozenNamespace(
    PCA9685   = 0x40,
    MPU6050   = 0x68,
    HMC5883L  = 0x1E,
    QMC5883L  = 0x0D,
    BMP180    = 0x77,
)


# ╔══════════════════════════════════════════════════════════════╗
# ║  MPU6050 — ACCELEROMETER + GYROSCOPE                        ║
# ╚══════════════════════════════════════════════════════════════╝

MPU6050 = _FrozenNamespace(

    WHO_AM_I            = 0x75,
    WHO_AM_I_EXPECTED   = 0x68,

    PWR_MGMT_1          = 0x6B,
    CONFIG              = 0x1A,
    GYRO_CONFIG         = 0x1B,
    ACCEL_CONFIG        = 0x1C,
    INT_PIN_CFG         = 0x37,

    ACCEL_XOUT_H        = 0x3B,
    TEMP_OUT_H          = 0x41,
    GYRO_XOUT_H         = 0x43,

    BURST_START         = 0x3B,
    BURST_LEN           = 14,

    BYPASS_ENABLE       = 0x02,

    ACCEL_SCALE_2G      = 16384.0,
    ACCEL_SCALE_4G      = 8192.0,
    ACCEL_SCALE_8G      = 4096.0,
    ACCEL_SCALE_16G     = 2048.0,

    GYRO_SCALE_250DPS   = 131.0,
    GYRO_SCALE_500DPS   = 65.5,
    GYRO_SCALE_1000DPS  = 32.8,
    GYRO_SCALE_2000DPS  = 16.4,

    DLPF_260HZ          = 0x00,
    DLPF_184HZ          = 0x01,
    DLPF_94HZ           = 0x02,
    DLPF_44HZ           = 0x03,
    DLPF_21HZ           = 0x04,
    DLPF_10HZ           = 0x05,
    DLPF_5HZ            = 0x06,
)


# ╔══════════════════════════════════════════════════════════════╗
# ║  HMC5883L — 3-AXIS MAGNETOMETER (original HW-290)           ║
# ╚══════════════════════════════════════════════════════════════╝

HMC5883L = _FrozenNamespace(

    CRA                 = 0x00,
    CRB                 = 0x01,
    MODE                = 0x02,

    DATA_OUT            = 0x03,
    BURST_LEN           = 6,

    ID_REG_A            = 0x0A,
    ID_REG_B            = 0x0B,
    ID_REG_C            = 0x0C,
    ID_EXPECTED         = (0x48, 0x34, 0x33),

    CRA_DEFAULT         = 0x70,
    CRB_DEFAULT         = 0x20,
    MODE_CONTINUOUS     = 0x00,

    SCALE_DEFAULT       = 1090.0,
)


# ╔══════════════════════════════════════════════════════════════╗
# ║  QMC5883L — 3-AXIS MAGNETOMETER (clone HW-290 boards)       ║
# ╚══════════════════════════════════════════════════════════════╝

QMC5883L = _FrozenNamespace(

    DATA_OUT            = 0x00,
    BURST_LEN           = 6,

    CONTROL_1           = 0x09,
    CONTROL_2           = 0x0A,
    SET_RESET           = 0x0B,

    CHIP_ID_REG         = 0x0D,
    CHIP_ID_EXPECTED    = 0xFF,

    CTRL1_DEFAULT       = 0x0D,
    CTRL2_DEFAULT       = 0x00,
    SET_RESET_DEFAULT   = 0x01,

    SCALE_DEFAULT       = 12000.0,
)


# ╔══════════════════════════════════════════════════════════════╗
# ║  BMP180 — BAROMETRIC PRESSURE SENSOR                        ║
# ╚══════════════════════════════════════════════════════════════╝

BMP180 = _FrozenNamespace(

    CHIP_ID_REG         = 0xD0,
    CHIP_ID_EXPECTED    = 0x55,

    CONTROL             = 0xF4,
    DATA_MSB            = 0xF6,
    DATA_LSB            = 0xF7,
    DATA_XLSB           = 0xF8,

    CAL_START           = 0xAA,
    CAL_LEN             = 22,

    CMD_TEMP            = 0x2E,
    CMD_PRES_OSS0       = 0x34,
    CMD_PRES_OSS1       = 0x74,
    CMD_PRES_OSS2       = 0xB4,
    CMD_PRES_OSS3       = 0xF4,

    WAIT_TEMP           = 0.005,
    WAIT_PRES_OSS0      = 0.005,
    WAIT_PRES_OSS1      = 0.008,
    WAIT_PRES_OSS2      = 0.014,
    WAIT_PRES_OSS3      = 0.026,

    SEA_LEVEL_PA        = 101325.0,
)


# ╔══════════════════════════════════════════════════════════════╗
# ║  PCA9685 — 16-CHANNEL PWM SERVO DRIVER                      ║
# ╚══════════════════════════════════════════════════════════════╝
#
# The PCA9685 generates PWM at a configurable frequency.
# Servo position is encoded as pulse WIDTH (in microseconds),
# which is independent of PWM frequency. The tick values
# that represent a given pulse width change with frequency.
#
# Physical pulse limits come from the servo datasheet.
# Tick values are computed at runtime by the servo driver
# based on the configured frequency.
# ──────────────────────────────────────────────────────────────

PCA9685 = _FrozenNamespace(

    # ── Registers ─────────────────────────────────────────────
    MODE1               = 0x00,
    PRESCALE            = 0xFE,
    LED0_ON_L           = 0x06,     # channel N base = LED0_ON_L + 4*N

    # ── Internal oscillator ───────────────────────────────────
    OSC_CLOCK_HZ        = 25_000_000,
    TICKS_PER_CYCLE     = 4096,

    # ── PWM frequency ─────────────────────────────────────────
    # 100 Hz: 2× faster servo response than 50 Hz
    # Period = 10ms, giving plenty of headroom for 2.5ms max pulse
    # Servo datasheet supports 50-330 Hz
    FREQ_HZ             = 100,

    # ── Servo pulse limits (microseconds) ─────────────────────
    # From servo datasheet:
    #   500 μs  = full one direction
    #   1500 μs = neutral / center
    #   2500 μs = full other direction
    # These are physical constants of the servo, not the PCA9685.
    PULSE_MIN_US        = 500,
    PULSE_MAX_US        = 2500,
    PULSE_CENTER_US     = 1500,

    # ── Servo dead band ───────────────────────────────────────
    # From datasheet: 3μs dead band
    # Pulse changes smaller than this are ignored by the servo
    DEAD_BAND_US        = 3,
)


# ╔══════════════════════════════════════════════════════════════╗
# ║  SERVO CONFIGURATION — PHYSICALLY MEASURED (SACRED DATA)    ║
# ╚══════════════════════════════════════════════════════════════╝
#
# Every value below was measured on the real robot and tuned
# through physical testing. DO NOT change these values unless
# you are physically re-calibrating the robot.
#
# Organisation: LEG → JOINT, using uniform keys:
#   Legs:   "FR" (front-right), "FL" (front-left),
#           "RR" (rear-right),  "RL" (rear-left)
#   Joints: "coxa" (hip yaw), "thigh" (upper leg), "wrist" (lower leg)
# ──────────────────────────────────────────────────────────────

CHANNELS = _freeze({
    "FR": {"coxa": 6,   "thigh": 8,   "wrist": 10},
    "FL": {"coxa": 7,   "thigh": 9,   "wrist": 11},
    "RR": {"coxa": 0,   "thigh": 2,   "wrist": 4},
    "RL": {"coxa": 1,   "thigh": 3,   "wrist": 5},
})

LIMITS = _freeze({
    "FR": {
        "coxa":  {"low": 0,   "high": 90,   "inverted": True},
        "thigh": {"low": 0,   "high": 270,  "inverted": True},
        "wrist": {"low": 0,   "high": 200,  "inverted": True},
    },
    "FL": {
        "coxa":  {"low": 0,   "high": 90,   "inverted": False},
        "thigh": {"low": 0,   "high": 270,  "inverted": False},
        "wrist": {"low": 0,   "high": 200,  "inverted": False},
    },
    "RR": {
        "coxa":  {"low": 0,   "high": 90,   "inverted": True},
        "thigh": {"low": 0,   "high": 270,  "inverted": True},
        "wrist": {"low": 0,   "high": 200,  "inverted": True},
    },
    "RL": {
        "coxa":  {"low": 0,   "high": 90,   "inverted": False},
        "thigh": {"low": 0,   "high": 270,  "inverted": False},
        "wrist": {"low": 0,   "high": 200,  "inverted": False},
    },
})

PERPENDICULAR = _freeze({
    "FR": {"coxa": 47,   "thigh": 138,  "wrist": 47},
    "FL": {"coxa": 39,   "thigh": 128,  "wrist": 153},
    "RR": {"coxa": 44,   "thigh": 140,  "wrist": 47},
    "RL": {"coxa": 50,   "thigh": 135,  "wrist": 153},
})

STAND = _freeze({
    "FR": {"coxa": 47,   "thigh": 98,   "wrist": 122},
    "FL": {"coxa": 39,   "thigh": 168,  "wrist": 78},
    "RR": {"coxa": 44,   "thigh": 100,  "wrist": 122},
    "RL": {"coxa": 50,   "thigh": 175,  "wrist": 78},
})


# ╔══════════════════════════════════════════════════════════════╗
# ║  CONVENIENCE ITERABLES                                       ║
# ╚══════════════════════════════════════════════════════════════╝

LEGS   = ("FR", "FL", "RR", "RL")
JOINTS = ("coxa", "thigh", "wrist")


# ╔══════════════════════════════════════════════════════════════╗
# ║  SMOKE TEST                                                  ║
# ╚══════════════════════════════════════════════════════════════╝

if __name__ == "__main__":

    print("=" * 60)
    print("  HARDWARE TRUTHS — VERIFICATION")
    print("=" * 60)

    # ----------------------------------------------------------
    # 1. Immutability checks
    # ----------------------------------------------------------
    print("\n[1] Immutability")

    passed = 0
    failed = 0

    try:
        ADDR.PCA9685 = 0xFF
        print("  FAIL — _FrozenNamespace allowed attribute write")
        failed += 1
    except AttributeError:
        print("  PASS — _FrozenNamespace blocked attribute write")
        passed += 1

    try:
        del ADDR.PCA9685
        print("  FAIL — _FrozenNamespace allowed attribute delete")
        failed += 1
    except AttributeError:
        print("  PASS — _FrozenNamespace blocked attribute delete")
        passed += 1

    try:
        CHANNELS["XX"] = {"coxa": 0, "thigh": 0, "wrist": 0}
        print("  FAIL — MappingProxyType allowed top-level write")
        failed += 1
    except TypeError:
        print("  PASS — MappingProxyType blocked top-level write")
        passed += 1

    try:
        CHANNELS["FR"]["coxa"] = 99
        print("  FAIL — MappingProxyType allowed nested write")
        failed += 1
    except TypeError:
        print("  PASS — MappingProxyType blocked nested write")
        passed += 1

    print(f"  Result: {passed} passed, {failed} failed")

    # ----------------------------------------------------------
    # 2. Servo data cross-check
    # ----------------------------------------------------------
    print("\n[2] Servo Configuration")

    for leg in LEGS:
        ch = CHANNELS[leg]
        st = STAND[leg]
        pp = PERPENDICULAR[leg]
        lm = LIMITS[leg]

        print(f"\n  ── {leg} ──")
        for joint in JOINTS:
            inv = "INV" if lm[joint]["inverted"] else "   "
            print(
                f"    {joint:6s}  ch={ch[joint]:2d}  "
                f"limits=[{lm[joint]['low']:3d}..{lm[joint]['high']:3d}] {inv}  "
                f"perp={pp[joint]:3d}  stand={st[joint]:3d}"
            )

            s = st[joint]
            lo = lm[joint]["low"]
            hi = lm[joint]["high"]
            if not (lo <= s <= hi):
                print(f"    *** WARNING: stand angle {s} outside limits [{lo}..{hi}]")

            p = pp[joint]
            if not (lo <= p <= hi):
                print(f"    *** WARNING: perp angle {p} outside limits [{lo}..{hi}]")

    # ----------------------------------------------------------
    # 3. Channel uniqueness check
    # ----------------------------------------------------------
    print("\n[3] Channel Uniqueness")

    all_channels = []
    for leg in LEGS:
        for joint in JOINTS:
            all_channels.append((leg, joint, CHANNELS[leg][joint]))

    channel_numbers = [c for _, _, c in all_channels]
    if len(channel_numbers) == len(set(channel_numbers)):
        print(f"  PASS — all {len(channel_numbers)} channels are unique")
    else:
        seen = {}
        for leg, joint, ch in all_channels:
            if ch in seen:
                print(f"  FAIL — channel {ch} used by both "
                      f"{seen[ch]} and {leg}.{joint}")
            seen[ch] = f"{leg}.{joint}"

    # ----------------------------------------------------------
    # 4. Device address listing
    # ----------------------------------------------------------
    print("\n[4] Device Addresses")

    for name, value in ADDR:
        print(f"  {name:12s}  0x{value:02X}")

    # ----------------------------------------------------------
    # 5. PCA9685 configuration
    # ----------------------------------------------------------
    print("\n[5] PCA9685 Configuration")

    freq = PCA9685.FREQ_HZ
    period_ms = 1000.0 / freq
    prescale = round(PCA9685.OSC_CLOCK_HZ / (PCA9685.TICKS_PER_CYCLE * freq)) - 1

    # Compute tick values at configured frequency
    us_per_tick = (period_ms * 1000.0) / PCA9685.TICKS_PER_CYCLE
    pulse_min_ticks = round(PCA9685.PULSE_MIN_US / us_per_tick)
    pulse_max_ticks = round(PCA9685.PULSE_MAX_US / us_per_tick)
    pulse_center_ticks = round(PCA9685.PULSE_CENTER_US / us_per_tick)

    print(f"  Frequency:      {freq} Hz")
    print(f"  Period:         {period_ms:.2f} ms")
    print(f"  Prescaler:      {prescale}")
    print(f"  μs per tick:    {us_per_tick:.3f}")
    print(f"  Pulse min:      {PCA9685.PULSE_MIN_US} μs = {pulse_min_ticks} ticks")
    print(f"  Pulse center:   {PCA9685.PULSE_CENTER_US} μs = {pulse_center_ticks} ticks")
    print(f"  Pulse max:      {PCA9685.PULSE_MAX_US} μs = {pulse_max_ticks} ticks")
    print(f"  Dead band:      {PCA9685.DEAD_BAND_US} μs = {round(PCA9685.DEAD_BAND_US / us_per_tick)} ticks")
    print(f"  Max pulse/period: {PCA9685.PULSE_MAX_US / 1000.0 / period_ms * 100:.1f}%")

    # ----------------------------------------------------------
    # 6. Register map summaries
    # ----------------------------------------------------------
    print("\n[6] MPU6050 Key Registers")
    print(f"  WHO_AM_I         0x{MPU6050.WHO_AM_I:02X}  (expect 0x{MPU6050.WHO_AM_I_EXPECTED:02X})")
    print(f"  PWR_MGMT_1       0x{MPU6050.PWR_MGMT_1:02X}")
    print(f"  CONFIG (DLPF)    0x{MPU6050.CONFIG:02X}")
    print(f"  INT_PIN_CFG      0x{MPU6050.INT_PIN_CFG:02X}  (bypass=0x{MPU6050.BYPASS_ENABLE:02X})")
    print(f"  Burst read       0x{MPU6050.BURST_START:02X} × {MPU6050.BURST_LEN} bytes")

    print("\n[7] Magnetometer")
    print(f"  HMC5883L         0x{ADDR.HMC5883L:02X}")
    print(f"  QMC5883L         0x{ADDR.QMC5883L:02X}")

    print("\n[8] BMP180")
    print(f"  Address          0x{ADDR.BMP180:02X}")
    print(f"  Chip ID reg      0x{BMP180.CHIP_ID_REG:02X}  (expect 0x{BMP180.CHIP_ID_EXPECTED:02X})")

    # ----------------------------------------------------------
    # 7. I2C bus probe
    # ----------------------------------------------------------
    print("\n[9] I2C Bus Probe")

    try:
        bus = get_bus()
        print(f"  Bus {I2C_BUS_NUMBER} opened successfully")

        devices_to_probe = [
            ("PCA9685",  ADDR.PCA9685),
            ("MPU6050",  ADDR.MPU6050),
            ("BMP180",   ADDR.BMP180),
        ]

        for name, addr in devices_to_probe:
            try:
                bus.read_byte(addr)
                print(f"  {name:12s} (0x{addr:02X})  ── FOUND")
            except OSError:
                print(f"  {name:12s} (0x{addr:02X})  ── not responding")

        try:
            bus.write_byte_data(
                ADDR.MPU6050,
                MPU6050.PWR_MGMT_1,
                0x00,
            )
            import time
            time.sleep(0.1)

            bus.write_byte_data(
                ADDR.MPU6050,
                MPU6050.INT_PIN_CFG,
                MPU6050.BYPASS_ENABLE,
            )
            time.sleep(0.01)

            mag_found = False
            for name, addr in [("HMC5883L", ADDR.HMC5883L),
                               ("QMC5883L", ADDR.QMC5883L)]:
                try:
                    bus.read_byte(addr)
                    print(f"  {name:12s} (0x{addr:02X})  ── FOUND (via bypass)")
                    mag_found = True
                except OSError:
                    pass

            if not mag_found:
                print(f"  Magnetometer    ── not found at "
                      f"0x{ADDR.HMC5883L:02X} or 0x{ADDR.QMC5883L:02X}")

        except OSError:
            print("  Could not enable MPU6050 bypass for magnetometer probe")

    except Exception as e:
        print(f"  Bus {I2C_BUS_NUMBER} FAILED: {e}")

    finally:
        close_bus()
        print(f"\n  Bus closed")

    print("\n" + "=" * 60)
    print("  VERIFICATION COMPLETE")
    print("=" * 60)