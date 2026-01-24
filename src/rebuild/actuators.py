# actuators.py
import time
from smbus2 import SMBus
from absolute_truths import (
    BUS, PCA_ADDR, MODE1, PRESCALE,
    PULSE_MIN, PULSE_MAX,
    WRISTS, THIGHS, COXA,
    WRIST_STAND, THIGH_STAND, COXA_STAND,
    WRIST_MECH, THIGH_MECH, COXA_MECH,
)

# single SMBus instance (initialized by init_actuators)
bus = None

def init_actuators(bus_id=None):
    """
    Initialize the PCA9685 on the Jetson I2C bus.
    If bus_id is None, uses BUS from absolute_truths.
    """
    global bus
    if bus is None:
        bus = SMBus(bus_id if bus_id is not None else BUS)
    # reset PCA9685 (follows the repo's init style)
    bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
    time.sleep(0.01)
    prescale = int(25000000 / (4096 * 50) - 1)
    bus.write_byte_data(PCA_ADDR, MODE1, 0x10)
    bus.write_byte_data(PCA_ADDR, PRESCALE, prescale)
    bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
    # restart
    bus.write_byte_data(PCA_ADDR, MODE1, 0x80)
    time.sleep(0.01)

def angle_to_pulse(angle_deg):
    """
    Map angle in degrees (expected domain 0..270) -> PCA9685 12-bit pulse.
    Uses PULSE_MIN / PULSE_MAX present in absolute_truths.
    """
    # clamp to [0, 270] to avoid overruns at electrical layer
    if angle_deg < 0:
        angle_deg = 0.0
    if angle_deg > 270:
        angle_deg = 270.0
    pulse = int(PULSE_MIN + (angle_deg / 270.0) * (PULSE_MAX - PULSE_MIN))
    return pulse

def _write_pwm(channel, pulse):
    """
    Low-level write: channel -> LEDn registers.
    Channel numbering matches PCA9685 0..15
    """
    # base register for channel
    base = 0x06 + 4 * channel
    # LEDn_ON_L, LEDn_ON_H, LEDn_OFF_L, LEDn_OFF_H
    bus.write_byte_data(PCA_ADDR, base, 0)
    bus.write_byte_data(PCA_ADDR, base + 1, 0)
    bus.write_byte_data(PCA_ADDR, base + 2, pulse & 0xFF)
    bus.write_byte_data(PCA_ADDR, base + 3, (pulse >> 8) & 0x0F)

def set_servo_angle(ch, angle_deg):
    """
    Public safe setter for a single servo channel.
    ch: PCA channel (0..15)
    angle_deg: mechanical angle (0..270 domain used throughout repo)
    """
    pulse = angle_to_pulse(angle_deg)
    _write_pwm(ch, pulse)

def go_stand():
    """
    Convenience: move all servos to stand positions (uses stand angles from absolute_truths).
    """
    # wrists
    for k, ch in WRISTS.items():
        set_servo_angle(ch, WRIST_STAND[k])
    # thighs
    for k, ch in THIGHS.items():
        set_servo_angle(ch, THIGH_STAND[k])
    # coxa
    for k, ch in COXA.items():
        set_servo_angle(ch, COXA_STAND[k])

def apply_leg_angles(thetalf, thetarf, thetarr, thetalr, move=True, scale=None, zeros=None, servo_table=None, dir_factors=None):
    """
    Convenience to accept the 4 IK outputs (each is [theta1, theta2, theta3]) and send angles to PCA.
    - servo_table: optional list mapping logical servo order (0..11) to PCA channels; if None, caller must supply.
    - scale, zeros, dir_factors: optional per-servo calibration arrays to match repo->hardware mapping.
    
    The repo computes angle values (radians). This function expects degrees.
    Typical usage: convert repo rad->deg and call this function from the main loop.
    See Spot_Micro_Control_v01.py where thetalf/thetarf/... are produced and where the repo leaves the servo call placeholder. :contentReference[oaicite:3]{index=3}
    """
    if not move:
        return

    # If caller provided servo_table/scale/zeros/dir_factors, we'll use them. Otherwise expect caller to perform per-servo transforms.
    # By default we assume caller gives angles in degrees already and knows their mapping.
    # This wrapper intentionally leaves per-servo zeros / scale to the top-level adaptation code.
    # Example minimal loop (caller must supply servo_table):
    if servo_table is None:
        raise ValueError("servo_table must be supplied (list of 12 PCA channel indices) by caller")

    # flatten the 4 leg lists into 12-angle list in repo order expected by many Spot scripts:
    # left front, right front, right rear, left rear — each [s0, s1, s2]
    angles = []
    angles += thetalf
    angles += thetarf
    angles += thetarr
    angles += thetalr

    # angles expected in radians in repo; convert to degrees if numbers are small (heuristic)
    # If angles look like radians (abs < 2*pi), convert.
    converted = []
    for a in angles:
        if abs(a) <= 2 * 3.14159265:
            converted.append(a * 180.0 / 3.14159265)
        else:
            converted.append(a)  # already degrees

    # apply per-servo scale/dir/zero if supplied
    for i, deg in enumerate(converted):
        if scale is not None:
            deg = deg * scale[i]
        if dir_factors is not None:
            deg = deg * dir_factors[i]
        if zeros is not None:
            deg = deg + zeros[i]
        ch = servo_table[i]
        set_servo_angle(ch, deg)
