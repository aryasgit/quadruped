# hardware/ — Layer 0, 1.1, 1.2, 1.3

> All electrical and mechanical truths. All hardware I/O. Nothing above this layer touches the bus.

---

## Files

| File | Layer | Responsibility |
|---|---|---|
| `absolute_truths.py` | 0 | All constants — channels, limits, stand angles, I2C addresses |
| `i2c_bus.py` | 1.1 | Shared SMBus singleton |
| `pca9685.py` | 1.2 | PCA9685 angle→PWM driver |
| `imu.py` | 1.3 | MPU6050 init, calibration, complementary filter |
| `servo_gui.py` | Tool | Tkinter calibration GUI |
| `test_oled.py` | Tool | SH1106 OLED smoke test |

---

## absolute_truths.py — Layer 0

**The only place in the entire codebase where physical facts are written down.** If a channel number, stand angle, or mechanical limit appears anywhere else, that is a bug.

### I2C

```python
BUS      = 7       # Jetson Orin Nano I2C bus
PCA_ADDR = 0x40    # PCA9685
MPU_ADDR = 0x68    # MPU6050
```

### Servo Pulse Range

```python
PULSE_MIN = 106    # ~0° (500 µs)
PULSE_MAX = 535    # ~270° (2500 µs)
```

These are PCA9685 12-bit count values at 50 Hz.

### Channel Maps

```python
COXA  = { "FL": 7, "FR": 6, "RL": 1, "RR": 0 }
THIGHS = { "TFL": 9, "TFR": 8, "TRL": 3, "TRR": 2 }
WRISTS = { "WFL": 11, "WFR": 10, "WRL": 5, "WRR": 4 }
```

### Mechanical Limits

Each joint has `min`, `max`, and `perp` (perpendicular position, measured physically).

`min` and `max` are servo angles in degrees (0–270 range). The **order** of min/max encodes servo mount direction:
- `min < max` → normal mount (increasing angle = increasing extension)
- `min > max` → inverted mount (increasing angle = decreasing extension)

```python
THIGH_MECH = {
    "TFL": {"min": 0,   "max": 270, "perp": 128},
    "TFR": {"min": 270, "max": 0,   "perp": 138},
    ...
}
```

### Stand Angles

Physically measured servo angles for the standing pose. These are what the robot holds when `delta = 0`.

```python
THIGH_STAND = {
    "TFR":  98,
    "TFL": 168,
    "TRR": 100,
    "TRL": 175,
}
```

**To update:** Use `servo_gui.py`, dial in the pose visually, read the slider values.

---

## i2c_bus.py — Layer 1.1

Single shared `SMBus` instance. All drivers call `get_i2c_bus()` — never open their own bus.

```python
from hardware.i2c_bus import get_i2c_bus

bus = get_i2c_bus()   # creates once, reuses thereafter
```

Closing the bus (rare — only on controlled shutdown):

```python
from hardware.i2c_bus import close_i2c_bus
close_i2c_bus()
```

---

## pca9685.py — Layer 1.2

Angle → PWM conversion and channel write.

### Init

Must be called once before any `set_servo_angle` call:

```python
from hardware.pca9685 import init_pca
init_pca()
```

Initialises at 50 Hz. Safe to call multiple times (no-op after first).

### Set Angle

```python
from hardware.pca9685 import set_servo_angle

set_servo_angle(channel=9, angle_deg=168.0)
```

`angle_deg` is a **mechanical servo angle** (0–270°). Clamped internally.

### Angle → Pulse Formula

```
pulse = PULSE_MIN + (angle / 270) × (PULSE_MAX − PULSE_MIN)
```

At 50 Hz with 4096 steps: `PULSE_MIN=106` ≈ 500 µs, `PULSE_MAX=535` ≈ 2500 µs.

---

## imu.py — Layer 1.3

MPU6050 over I2C. Blocking calibration + complementary filter.

### Init + Calibrate

```python
from hardware.imu import init_mpu, calibrate, IMUFilter

init_mpu()                    # configure registers, enable low-pass filter
calib = calibrate(samples=200)  # robot must be FLAT and STILL
imu = IMUFilter(calib)
```

Calibration takes ~4 seconds at 200 samples.

### Read

```python
roll, pitch, roll_rate, pitch_rate = imu.update()
```

All angles in degrees. Rates in deg/s.

- **roll** — positive = tilt left
- **pitch** — positive = nose up
- Rates come from raw gyro (131 LSB/°/s at ±250°/s range)

### Complementary Filter

```
angle = alpha × (angle + rate × dt) + (1 − alpha) × accel_angle
```

Default `alpha = 0.96`, `dt = 0.02` (50 Hz).

- Higher alpha → trusts gyro more, slower drift correction
- Lower alpha → trusts accelerometer more, noisier

### Low-Pass Filter

Set at init with `CONFIG = 0x04` (~21 Hz cutoff). Reduces servo vibration noise coupling into IMU.

---

## servo_gui.py — Calibration Tool

Tkinter GUI with one slider per servo (0–270°). All 12 servos laid out in a grid.

### Run

```bash
python hardware/servo_gui.py
```

Initialises PCA9685 and drives all servos to their stand angles before the GUI opens. The "Move to Standing Position" button resets all sliders to stand.

### Use for Calibration

1. Open the GUI
2. Use sliders to find the physical perpendicular position for each joint
3. Note the slider value → update `perp` in `absolute_truths.py`
4. Dial in the stand pose → update `WRIST_STAND` / `THIGH_STAND` / `COXA_STAND`

---

## test_oled.py — OLED Smoke Test

Draws a border rectangle and "OLED TEST" text on the SH1106 display.

```bash
python hardware/test_oled.py
```

Runs indefinitely at 10 Hz. Ctrl+C to exit.

Requires `luma.oled` and `pillow`:
```bash
pip install luma.oled pillow
```