# hardware/absolute_truths.py
# ==================================================
# Mechanical & electrical truths (read-only constants)
# - Servo channel mapping, mechanical limits, stand angles
# - I2C addresses and bus number for Jetson Orin Nano setup
#
# If anything here is wrong the physical robot is wrong.
# Keep this file free of logic and math beyond simple constants.
# ==================================================

# --- I2C / hardware (Jetson / SMBus) ---
BUS = 7                # Jetson I2C bus used in your setup (keep as you provided)
PCA_ADDR = 0x40        # PCA9685 I2C address (repo uses this address)
MPU_ADDR = 0x68        # MPU6050 IMU I2C address

# PCA9685 registers (hardware truth)
MODE1    = 0x00
PRESCALE = 0xFE

# MPU registers (hardware truth)
PWR_MGMT_1     = 0x6B
ACCEL_XOUT_H   = 0x3B
GYRO_XOUT_H    = 0x43
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C

# --- Servo electrical limits (do not change lightly) ---
# These are pulse steps used by the PCA9685 driver; defined by you
PULSE_MIN = 106
PULSE_MAX = 535

# --- Servo channel map (logical names → PCA channel) ---
WRISTS = {
    "WFR": 10,
    "WFL": 11,
    "WRR": 4,
    "WRL": 5,
}

THIGHS = {
    "TFR": 8,
    "TFL": 9,
    "TRR": 2,
    "TRL": 3,
}

COXA = {
    "FR": 6,
    "FL": 7,
    "RR": 0,
    "RL": 1,
}

# --- Mechanical angle limits and perpendicular (measured) --- 
# (these dictionaries express the mechanical mapping of servo-angle domain)
WRIST_MECH = {
    "WFL": {"min": 0,   "max": 200, "perp": 155},
    "WRL": {"min": 0,   "max": 200, "perp": 151},
    "WFR": {"min": 200, "max": 0,   "perp": 48},
    "WRR": {"min": 200, "max": 0,   "perp": 50},
}

THIGH_MECH = {
    "TFL": {"min": 0,   "max": 270, "perp": 126},
    "TRL": {"min": 0,   "max": 270, "perp": 134},
    "TFR": {"min": 270, "max": 0,   "perp": 139},
    "TRR": {"min": 270, "max": 0,   "perp": 141},
}

COXA_MECH = {
    "FL": {"min": 0,  "max": 90, "perp": 43},
    "RL": {"min": 0,  "max": 90, "perp": 54},
    "FR": {"min": 90, "max": 0,  "perp": 46},
    "RR": {"min": 90, "max": 0,  "perp": 46},
}

# --- Stand pose (measured absolute angles) ---
WRIST_STAND = {
    "WFR": 123,
    "WFL": 80,
    "WRR": 125,
    "WRL": 76,
}

THIGH_STAND = {
    "TFR": 99,
    "TFL": 166,
    "TRR": 101,
    "TRL": 174,
}

COXA_STAND = {
    "FR": 46,
    "FL": 43,
    "RR": 46,
    "RL": 54,
}

# That's it — pure facts only.

# hardware/i2c_bus.py
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


# hardware/imu.py
"""
Layer 1.3 — IMU DRIVER (MPU6050)
===============================

Authoritative IMU interface for the robot.

Responsibilities:
- Initialize MPU6050
- Perform blocking calibration
- Provide roll, pitch, roll_rate, pitch_rate

NON-RESPONSIBILITIES:
- No balance control
- No posture correction
- No servo output
- No gait logic

This module matches the semantic expectations of the SpotMicro repo:
- roll, pitch in degrees
- suitable to be injected directly into theta_spot[3], theta_spot[4]
"""

import time
import math
import errno
from statistics import mean

from hardware.i2c_bus import get_i2c_bus
from hardware.absolute_truths import (
    MPU_ADDR,
    PWR_MGMT_1,
    ACCEL_XOUT_H,
    GYRO_XOUT_H,
    CONFIG,
)

# -----------------------------
# Low-level safe read
# -----------------------------

def _safe_read_word(bus, addr, reg):
    try:
        h = bus.read_byte_data(addr, reg)
        l = bus.read_byte_data(addr, reg + 1)
        v = (h << 8) | l
        return v - 65536 if v > 32767 else v
    except OSError as e:
        if e.errno == errno.EREMOTEIO:
            raise IOError
        raise


# -----------------------------
# MPU6050 initialization
# -----------------------------

def init_mpu():
    bus = get_i2c_bus()
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    # ~20 Hz low-pass filter (matches repo behavior)
    bus.write_byte_data(MPU_ADDR, CONFIG, 0x04)
    time.sleep(0.05)

    print("[IMU] MPU6050 initialized")


# -----------------------------
# Calibration container
# -----------------------------

class IMUCalibration:
    def __init__(self, ax, ay, az, gx, gy):
        self.ax = ax
        self.ay = ay
        self.az = az
        self.gx = gx
        self.gy = gy


# -----------------------------
# Blocking calibration
# -----------------------------

def calibrate(samples=200):
    """
    Blocking calibration. Robot must be still.
    Returns IMUCalibration object.
    """
    bus = get_i2c_bus()
    print("[IMU] Calibrating — keep robot still")

    ax = ay = az = gx = gy = 0.0
    collected = 0

    while collected < samples:
        try:
            ax += _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H)
            ay += _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 2)
            az += _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 4) - 16384.0

            gx += _safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H)
            gy += _safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H + 2)

            collected += 1
            time.sleep(0.01)
        except IOError:
            time.sleep(0.05)

    calib = IMUCalibration(
        ax / samples,
        ay / samples,
        az / samples,
        gx / samples,
        gy / samples,
    )

    print(f"[IMU] Calibrated | ax={calib.ax:.1f} ay={calib.ay:.1f} az={calib.az:.1f} gx={calib.gx:.2f} gy={calib.gy:.2f}")
    return calib


# -----------------------------
# Complementary filter
# -----------------------------

class IMUFilter:
    def __init__(self, calib: IMUCalibration, alpha=0.96, dt=0.02):
        self.calib = calib
        self.alpha = alpha
        self.dt = dt

        self.roll = 0.0
        self.pitch = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0

    def update(self):
        bus = get_i2c_bus()

        ax = _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H)     - self.calib.ax
        ay = _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 2) - self.calib.ay
        az = _safe_read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 4) - self.calib.az

        gx = _safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H)     - self.calib.gx
        gy = _safe_read_word(bus, MPU_ADDR, GYRO_XOUT_H + 2) - self.calib.gy

        accel_roll = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        self.roll_rate = gx / 131.0
        self.pitch_rate = gy / 131.0

        self.roll = self.alpha * (self.roll + self.roll_rate * self.dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + self.pitch_rate * self.dt) + (1 - self.alpha) * accel_pitch

        return self.roll, self.pitch, self.roll_rate, self.pitch_rate


# -----------------------------
# Smoke test
# -----------------------------

if __name__ == "__main__":
    init_mpu()
    calib = calibrate()
    imu = IMUFilter(calib)

    print("[IMU] Live output (Ctrl+C to exit)")
    while True:
        r, p, rr, pr = imu.update()
        print(f"roll={r:+6.2f}° pitch={p:+6.2f}° rr={rr:+6.2f} pr={pr:+6.2f}")
        time.sleep(0.05)

# hardware/pca9685.py
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

#layer2/joint_conventions.py

"""
Layer 2.5 — JOINT CONVENTION MAPPING
===================================

Adapter between IK math space and real servo space.
This layer is the ONLY place where mirroring is handled.
"""

# =================================================
# JOINT SIGN CONVENTIONS
# =================================================
# +1 → use IK delta as-is
# -1 → invert IK delta

JOINT_SIGN = {
    # Coxa (yaw) — mirrored left/right
    "FL_COXA": -1,
    "FR_COXA": +1,
    "RL_COXA": -1,
    "RR_COXA": +1,

    # Thigh (pitch) — SAME for all legs
    "FL_THIGH": +1,
    "FR_THIGH": +1,
    "RL_THIGH": +1,
    "RR_THIGH": +1,

    # Wrist (knee) — RIGHT SIDE IS MIRRORED ❗
    "FL_WRIST": +1,
    "RL_WRIST": +1,
    "FR_WRIST": -1,   # ← FIX
    "RR_WRIST": -1,   # ← FIX
}


# =================================================
# JOINT ZERO OFFSETS (degrees)
# =================================================
# Leave at zero unless you see static bias at stand

JOINT_OFFSET = {
    "FL_COXA":  0.0,
    "FR_COXA":  0.0,
    "RL_COXA":  0.0,
    "RR_COXA":  0.0,

    "FL_THIGH": 0.0,
    "FR_THIGH": 0.0,
    "RL_THIGH": 0.0,
    "RR_THIGH": 0.0,

    "FL_WRIST": 0.0,
    "FR_WRIST": 0.0,
    "RL_WRIST": 0.0,
    "RR_WRIST": 0.0,
}


# =================================================
# APPLY CONVENTIONS
# =================================================

def apply_joint_conventions(deltas: dict) -> dict:
    """
    Apply sign + offset corrections to IK deltas.
    """
    corrected = {}

    for joint, delta in deltas.items():
        if joint not in JOINT_SIGN:
            raise KeyError(f"Unknown joint '{joint}'")

        corrected[joint] = (
            JOINT_SIGN[joint] * delta
            + JOINT_OFFSET[joint]
        )

    return corrected

#layer2/joint_space.py
"""
Layer 2 — JOINT SPACE NORMALIZATION
=================================

This layer is the ONLY bridge between:
- mathematical joint angles (IK / posture / gait)
- physical servo angles (Layer 0 truths)

Responsibilities:
- Define canonical joint ordering
- Apply direction flips
- Apply mechanical zero offsets
- Clamp to mechanical limits

NON-RESPONSIBILITIES:
- No IK math
- No balance
- No gait timing
- No hardware I/O

Everything above this layer is dimensionless math.
Everything below this layer is physical metal.
"""

from hardware.absolute_truths import (
    WRISTS,
    THIGHS,
    COXA,
    WRIST_MECH,
    THIGH_MECH,
    COXA_MECH,
    WRIST_STAND,
    THIGH_STAND,
    COXA_STAND,
)

# -------------------------------------------------
# Canonical joint order (MATCHES SPOTMICRO REPO)
# -------------------------------------------------
# Order: LF, RF, RR, LR
# Each leg: [coxa, thigh, wrist]

JOINT_ORDER = [
    "FL_COXA", "FL_THIGH", "FL_WRIST",
    "FR_COXA", "FR_THIGH", "FR_WRIST",
    "RR_COXA", "RR_THIGH", "RR_WRIST",
    "RL_COXA", "RL_THIGH", "RL_WRIST",
]

# -------------------------------------------------
# Mechanical truth tables (flattened)
# -------------------------------------------------

MECH_LIMITS = {
    "FL_COXA":  COXA_MECH["FL"],
    "FR_COXA":  COXA_MECH["FR"],
    "RR_COXA":  COXA_MECH["RR"],
    "RL_COXA":  COXA_MECH["RL"],

    "FL_THIGH": THIGH_MECH["TFL"],
    "FR_THIGH": THIGH_MECH["TFR"],
    "RR_THIGH": THIGH_MECH["TRR"],
    "RL_THIGH": THIGH_MECH["TRL"],

    "FL_WRIST": WRIST_MECH["WFL"],
    "FR_WRIST": WRIST_MECH["WFR"],
    "RR_WRIST": WRIST_MECH["WRR"],
    "RL_WRIST": WRIST_MECH["WRL"],
}

STAND_ANGLES = {
    "FL_COXA":  COXA_STAND["FL"],
    "FR_COXA":  COXA_STAND["FR"],
    "RR_COXA":  COXA_STAND["RR"],
    "RL_COXA":  COXA_STAND["RL"],

    "FL_THIGH": THIGH_STAND["TFL"],
    "FR_THIGH": THIGH_STAND["TFR"],
    "RR_THIGH": THIGH_STAND["TRR"],
    "RL_THIGH": THIGH_STAND["TRL"],

    "FL_WRIST": WRIST_STAND["WFL"],
    "FR_WRIST": WRIST_STAND["WFR"],
    "RR_WRIST": WRIST_STAND["WRR"],
    "RL_WRIST": WRIST_STAND["WRL"],
}

# -------------------------------------------------
# Normalization core
# -------------------------------------------------

def normalize_joint(joint_name: str, delta_angle: float) -> float:
    """
    Convert a math-space delta angle into a physical servo angle.

    delta_angle:
        +ve means forward in math space
        0 means stand pose

    Returns:
        physical servo angle (degrees), clamped
    """
    stand = STAND_ANGLES[joint_name]
    mech = MECH_LIMITS[joint_name]

    physical = stand + delta_angle

    lo = min(mech["min"], mech["max"])
    hi = max(mech["min"], mech["max"])

    if physical < lo:
        physical = lo
    elif physical > hi:
        physical = hi

    return physical


def normalize_all(deltas: dict) -> dict:
    """
    Normalize all joints.

    deltas:
        dict mapping joint_name -> delta_angle

    Returns:
        dict mapping joint_name -> physical_angle
    """
    out = {}
    for j in JOINT_ORDER:
        out[j] = normalize_joint(j, deltas.get(j, 0.0))
    return out


# -------------------------------------------------
# Stand sanity check
# -------------------------------------------------

if __name__ == "__main__":
    print("[L2] Stand pose sanity check")
    zeros = {j: 0.0 for j in JOINT_ORDER}
    phys = normalize_all(zeros)
    for j in JOINT_ORDER:
        print(f"{j:10s} -> {phys[j]:6.1f}°")


#layer3/kinematics.py
import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
import matplotlib.pyplot as plt
from layer3.util import RotMatrix3D, point_to_rad


class kinematics():
    
    def __init__(self):
        
        # note: leg IDs
        left_front = 0
        left_back  = 1
        right_front= 2
        right_back = 3
        
        self.right_legs = [right_front, right_back]
        
        self.link_1 = 0.01605
        self.link_2 = 0.10832
        self.link_3 = 0.13476
        self.phi = radians(90)
        
        # body dimensions
        self.length = 0.20830
        self.width = 0.0789
        self.hight = 0.074
        
        # leg origins (left_f, left_b, right_b, right_f), i.e., the coordinate of j1
        self.leg_origins = np.matrix([[self.length/2, self.width/2, 0],
                          [-self.length/2, self.width/2, 0],
                          [-self.length/2, -self.width/2, 0],
                          [self.length/2, -self.width/2, 0],
                          [self.length/2, self.width/2, 0]])
        
    # this method adjust inputs to the IK calculator by adding rotation and 
    # offset of that rotation from the center of the robot
    def leg_IK(self, xyz, rot = [0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
        
        # check is the leg is from the right side 
        is_right = (legID in self.right_legs)
        
        # add offset of each leg from the axis of rotation
        XYZ = asarray((inv(RotMatrix3D(rot,is_radians)) * \
            ((array(xyz) + self.leg_origins[legID,:] - array(center_offset)).transpose())).transpose())
        
        # subtract the offset between the leg and the center of rotation 
        # so that the resultant coordiante is relative to the origin (j1) of the leg
        xyz_ = asarray(XYZ - self.leg_origins[legID,:] + array(center_offset)).flatten()

        # calculate the angles and coordinates of the leg relative to the origin of the leg
        return self.leg_IK_calc(xyz_, is_right)


    # IK calculator
    def leg_IK_calc(self, xyz, is_right=False): 

        x, y, z = xyz[0], xyz[1], xyz[2]    # unpack coordinates
        
        # length of vector projected on the YZ plane. equiv. to len_A = sqrt(y**2 + z**2)
        len_A = norm([0, y, z])
        len_A = max(len_A, self.link_1 + 1e-6)
        
        # a_1 : angle from the positive y-axis to the end-effector (0 <= a_1 < 2pi)
        # a_2 : angle bewtween len_A and leg's projection line on YZ plane
        # a_3 : angle between link1 and length len_A
        a_1 = point_to_rad(y,z)                     
        # --- SAFE asin clamp ---
        ratio = (sin(self.phi) * self.link_1) / max(len_A, 1e-6)
        ratio = max(-1.0, min(1.0, ratio))
        a_2 = asin(ratio)
        a_3 = pi - a_2 - self.phi                   
        
        # angle of link1 about the x-axis 
        if is_right: theta_1 = a_1 - a_3
        else: 
            theta_1 = a_1 + a_3
            if theta_1 >= 2*pi: theta_1 -= 2*pi
        
        j2 = array([0,self.link_1*cos(theta_1),self.link_1*sin(theta_1)])
        j4 = array(xyz)
        j4_2_vec = j4 - j2 # vector from j2 to j4
        
        if is_right: R = theta_1 - self.phi - pi/2
        else: R = theta_1 + self.phi - pi/2
        
        # create rotation matrix to work on a new 2D plane (XZ_)
        rot_mtx = RotMatrix3D([-R,0,0],is_radians=True)
        j4_2_vec_ = rot_mtx * (np.reshape(j4_2_vec,[3,1]))
        
        # xyz in the rotated coordinate system + offset due to link_1 removed
        x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]
        
        len_B = norm([x_, z_]) # norm(j4-j2)
        len_B = max(len_B, 1e-6)

        
        # handling mathematically invalid input, i.e., point too far away to reach
        if len_B >= (self.link_2 + self.link_3): 
            len_B = (self.link_2 + self.link_3) * 0.99999
            # self.node.get_logger().warn('target coordinate: [%f %f %f] too far away' % (x, y, z))
            print('target coordinate: [%f %f %f] too far away' % (x, y, z))
        
        # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
        # b_2 : angle between len_B and link_2
        # b_3 : angle between link_2 and link_3
        b_1 = point_to_rad(x_, z_)  
        b_2 = acos((self.link_2**2 + len_B**2 - self.link_3**2) / (2 * self.link_2 * len_B)) 
        b_3 = acos((self.link_2**2 + self.link_3**2 - len_B**2) / (2 * self.link_2 * self.link_3))  
        
        # assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
        theta_2 = b_1 - b_2    
        theta_3 = pi - b_3
        
        # CALCULATE THE COORDINATES OF THE JOINTS FOR VISUALIZATION
        j1 = np.array([0,0,0])
        
        # calculate joint 3
        j3_ = np.reshape(np.array([self.link_2*cos(theta_2),0, self.link_2*sin(theta_2)]),[3,1])
        j3 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j3_, [1,3])).flatten()
        
        # calculate joint 4
        j4_ = j3_ + np.reshape(np.array([self.link_3*cos(theta_2+theta_3),0, self.link_3*sin(theta_2+theta_3)]), [3,1])
        j4 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j4_, [1,3])).flatten()
        
        # modify angles to match robot's configuration (i.e., adding offsets)
        angles = self.angle_corrector(angles=[theta_1, theta_2, theta_3], is_right=is_right)
        # print(degrees(angles[0]))
        return [angles[0], angles[1], angles[2], j1, j2, j3, j4]
    
    
    def base_pose(self, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        
        # offset due to non-centered axes of rotation
        offset = RotMatrix3D(rot, is_radians) * \
            (matrix(center_offset).transpose()) - matrix(center_offset).transpose()
        
        # rotate the base around the center of rotation (if there is no offset, then the center of 
        # rotation will be at the center of the robot)
        rotated_base = RotMatrix3D(rot, is_radians) * self.leg_origins.transpose() - offset
        return rotated_base.transpose()
       
    # get coordinates of leg joints relative to j1
    def leg_pose(self, xyz, rot, legID, is_radians, center_offset=[0,0,0]):
        
        # get the coordinates of each joints relative to the leg's origin
        pose_relative = self.leg_IK(xyz, rot, legID, is_radians, center_offset)[3:]
        
        # adjust the coordinates according to the robot's orientation (roll, pitch, yaw)
        pose_true = RotMatrix3D(rot,is_radians) * (array(pose_relative).transpose())
        return pose_true.transpose()
    
    # plot rectangular base where each corner represents the origin of leg
    def plot_base(self, ax, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        # get coordinates
        p = (self.base_pose(rot, is_radians, center_offset)).transpose()     
        # plot coordinates
        ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'r')
        return
       
    # plot leg 
    def plot_leg(self, ax, xyz, rot=[0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
        # get coordinates
        p = ((self.leg_pose(xyz, rot, legID, is_radians, center_offset) \
                + self.base_pose(rot,is_radians,center_offset)[legID]).transpose())
        # plot coordinates
        ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'b')
        return

    def plot_robot(self, ax, xyz, rot=[0,0,0], leg_N=4, is_radians=True, center_offset=[0,0,0]):
        ax.clear()
        self.plot_base(ax, rot, is_radians, center_offset)

        for leg in range(leg_N):
            self.plot_leg(ax, xyz[leg], rot, leg, is_radians, center_offset)

        return

        
    # TO-DO : modify this function depending on your robot's configuration
    # adjusting angle for specific configurations of motors, incl. orientation
    # this will vary for each robot (possibly for each leg as well)
    def angle_corrector(self, angles=[0,0,0], is_right=True):
        angles[1] -= 1.5*pi; # add offset 
        
        if is_right:
            theta_1 = angles[0] - pi
            theta_2 = angles[1] + 45*pi/180 # 45 degrees initial offset
        else: 
            if angles[0] > pi:  
                theta_1 = angles[0] - 2*pi
            else: theta_1 = angles[0]
            
            theta_2 = -angles[1] - 45*pi/180
        
        theta_3 = -angles[2] + 45*pi/180
        return [theta_1, theta_2, theta_3]
        
    # set view  
    @staticmethod
    def ax_view(limit):
        ax = plt.axes(projection="3d")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        return ax
    

#layer3/leg_ik.py
"""
Layer 3 — LEG INVERSE KINEMATICS (IK)
===================================

Pure geometry. No hardware. No balance.

Uses exact 3D IK (SpotMicro-style).

Contract:
- Input foot positions are HIP-LOCAL
- Output angles are DELTAS from stand
- Units: degrees
"""

import math
from layer3.kinematics import kinematics   # your new solver


# -------------------------------------------------
# Global IK engine (stateless geometry)
# -------------------------------------------------

_IK = kinematics()


# -------------------------------------------------
# Leg ID mapping (consistent across stack)
# -------------------------------------------------

LEG_ID = {
    'FL': 0,
    'RL': 1,
    'FR': 2,
    'RR': 3,
}


# -------------------------------------------------
# Stand pose reference (computed ONCE)
# -------------------------------------------------

_STAND_Z = -0.18
_STAND_Y = 0.07
_STAND_X = 0.0

_STAND_REF = {}

def _compute_stand_reference():
    for leg, leg_id in LEG_ID.items():
        y = _STAND_Y if leg in ('FL', 'RL') else -_STAND_Y
        t1, t2, t3, *_ = _IK.leg_IK(
            xyz=[_STAND_X, y, _STAND_Z],
            legID=leg_id,
            is_radians=True
        )

        _STAND_REF[leg] = (t1, t2, t3)




_compute_stand_reference()


# -------------------------------------------------
# Core solver
# -------------------------------------------------

def solve_leg_ik(x, y, z, leg):
    leg_id = LEG_ID[leg]

    t1, t2, t3, *_ = _IK.leg_IK(
        xyz=[x, y, z],
        legID=leg_id,
        is_radians=True
    )

    # Convert to degrees
    deg = [
        math.degrees(t1),
        math.degrees(t2),
        math.degrees(t3),
    ]

    # Delta from stand
    ref = _STAND_REF[leg]
    delta = (
        deg[0] - math.degrees(ref[0]),
        deg[1] - math.degrees(ref[1]),
        deg[2] - math.degrees(ref[2]),
    )

    return delta


# -------------------------------------------------
# Solve all legs
# -------------------------------------------------

def solve_all_legs(foot_targets):
    """
    foot_targets:
    {
        'FL': (x, y, z),
        'FR': (x, y, z),
        'RL': (x, y, z),
        'RR': (x, y, z),
    }
    """

    out = {}

    for leg, (x, y, z) in foot_targets.items():
        c, t, w = solve_leg_ik(x, y, z, leg)
        out[f"{leg}_COXA"]  = c
        out[f"{leg}_THIGH"] = t
        out[f"{leg}_WRIST"] = w

    return out


#layer3/util.py
from math import atan, pi, radians, cos, sin
import numpy as np

def point_to_rad(p1, p2):
    if (p1 > 0 and p2 >= 0): return atan(p2/(p1))
    elif (p1 == 0 and p2 >= 0): return pi/2
    elif (p1 < 0 and p2 >= 0): return -abs(atan(p2/p1)) + pi
    elif (p1 < 0 and p2 < 0): return atan(p2/p1) + pi
    elif (p1 > 0 and p2 < 0): return -abs(atan(p2/p1)) + 2*pi
    elif (p1 == 0 and p2 < 0): return pi * 3/2
    elif (p1 == 0 and p2 == 0): return pi * 3/2

def RotMatrix3D(rotation=[0,0,0], is_radians=True, order='xyz'):
    roll, pitch, yaw = rotation

    if not is_radians:
        roll = radians(roll)
        pitch = radians(pitch)
        yaw = radians(yaw)

    rotX = np.matrix([[1, 0, 0],
                      [0, cos(roll), -sin(roll)],
                      [0, sin(roll), cos(roll)]])
    rotY = np.matrix([[cos(pitch), 0, sin(pitch)],
                      [0, 1, 0],
                      [-sin(pitch), 0, cos(pitch)]])
    rotZ = np.matrix([[cos(yaw), -sin(yaw), 0],
                      [sin(yaw), cos(yaw), 0],
                      [0, 0, 1]])

    return rotZ * rotY * rotX


#layer4/stand_controller.py
"""
Layer 4 — STAND GEOMETRY & FIRST MOTION BRIDGE
=============================================

This is the FIRST layer where math is allowed to touch hardware.

Pipeline implemented here:

    Stand foot geometry
        ↓
    Layer 3 (IK) → joint deltas
        ↓
    Layer 2 (joint normalization) → physical angles
        ↓
    Layer 1 (PCA9685) → servo motion

This file exists for ONE purpose:
    Make the robot stand using math, not hardcoded angles.

No gait. No balance. No IMU.
"""
from layer2.joint_conventions import apply_joint_conventions
from layer3.leg_ik import solve_all_legs
from layer2.joint_space import normalize_all
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import WRISTS, THIGHS, COXA
STAND_Z = -0.18


# -------------------------------------------------
# Servo channel lookup (joint_name → PCA channel)
# -------------------------------------------------

SERVO_CHANNELS = {
    # Front Left
    "FL_COXA":  COXA["FL"],
    "FL_THIGH": THIGHS["TFL"],
    "FL_WRIST": WRISTS["WFL"],

    # Front Right
    "FR_COXA":  COXA["FR"],
    "FR_THIGH": THIGHS["TFR"],
    "FR_WRIST": WRISTS["WFR"],

    # Rear Right
    "RR_COXA":  COXA["RR"],
    "RR_THIGH": THIGHS["TRR"],
    "RR_WRIST": WRISTS["WRR"],

    # Rear Left
    "RL_COXA":  COXA["RL"],
    "RL_THIGH": THIGHS["TRL"],
    "RL_WRIST": WRISTS["WRL"],
}

# -------------------------------------------------
# Stand foot geometry (meters, BODY FRAME)
# -------------------------------------------------
# These values define WHAT "standing" means geometrically.
# Adjust Z only for height tuning later.

STAND_FEET = {
    "FL": ( 0.15,  0.05, -0.22),
    "FR": ( 0.15, -0.05, -0.22),
    "RR": (-0.15, -0.05, -0.22),
    "RL": (-0.15,  0.05, -0.22),
}

# -------------------------------------------------
# Stand execution
# -------------------------------------------------

import time

def stand_squat_test():
    """
    Layer 4 integration test:
    Smooth stand → squat → stand
    """

    zs = [z * 0.001 for z in range(-180, -221, -2)]
    zs += [z * 0.001 for z in range(-220, -179, 2)]


    for z in zs:
        foot_targets = {
            "FL": ( 0.15,  0.05, z),
            "FR": ( 0.15, -0.05, z),
            "RR": (-0.15, -0.05, z),
            "RL": (-0.15,  0.05, z),
        }

        deltas = solve_all_legs(foot_targets)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)

        for joint, angle in physical.items():
            ch = SERVO_CHANNELS[joint]
            set_servo_angle(ch, angle)

        time.sleep(0.01)

import time

from hardware.absolute_truths import (
    COXA_STAND,
    THIGH_STAND,
    WRIST_STAND,
)

def coxa_isolation_test():
    ys = [y * 0.001 for y in range(30, 121, 5)]
    ys += [y * 0.001 for y in range(120, 29, -5)]

    for y in ys:
        foot_targets = {
            "FL": ( 0.15,  y, -0.18),
            "FR": ( 0.15, -y, -0.18),
            "RR": (-0.15, -y, -0.18),
            "RL": (-0.15,  y, -0.18),
        }

        deltas = solve_all_legs(foot_targets)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)

        # 🔒 HARD OVERRIDE — ABSOLUTE ANGLES
        physical["FL_THIGH"] = THIGH_STAND["TFL"]
        physical["FR_THIGH"] = THIGH_STAND["TFR"]
        physical["RL_THIGH"] = THIGH_STAND["TRL"]
        physical["RR_THIGH"] = THIGH_STAND["TRR"]

        physical["FL_WRIST"] = WRIST_STAND["WFL"]
        physical["FR_WRIST"] = WRIST_STAND["WFR"]
        physical["RL_WRIST"] = WRIST_STAND["WRL"]
        physical["RR_WRIST"] = WRIST_STAND["WRR"]

        for joint, angle in physical.items():
            ch = SERVO_CHANNELS[joint]
            set_servo_angle(ch, angle)

        time.sleep(0.05)


# -------------------------------------------------
# Smoke test — THIS WILL MOVE THE ROBOT
# -------------------------------------------------

if __name__ == "__main__":
    #print("[L4] Running stand–squat integration test")
    stand_squat_test()
    print("[L4] coxa_isolation_test")
    #coxa_isolation_test()


# layer5/posture_controller_gait_only.py
"""
Layer 5 — GAIT-ONLY KINEMATICS BRIDGE
===================================

STRICT CONTRACT:
- NO IMU
- NO pitch / roll stabilization
- NO posture deltas
- NO gait timing
- NO FSM logic

This layer ONLY:
1. Accepts absolute foot targets from Layer 9
2. Solves IK
3. Applies joint conventions
4. Normalizes to servo angles
5. Locks COXA for stance consistency

This guarantees:
- Deterministic gait geometry
- No step magnitude distortion
- No sequencing overlap
"""

from layer3.leg_ik import solve_all_legs
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all
from hardware.absolute_truths import COXA

# ----------------------------------
# CONSTANTS (MATCH controller.py)
# ----------------------------------
COXA_STAND = 45.0   # same as before, do NOT change silently


def posture_step(foot_targets):
    """
    foot_targets: dict
        {
          "FL": (x, y, z),
          "FR": (x, y, z),
          "RL": (x, y, z),
          "RR": (x, y, z),
        }

    returns:
        physical joint angles dict compatible with controller.py
    """

    # ----------------------------------
    # 1. Inverse kinematics
    # ----------------------------------
    deltas = solve_all_legs(foot_targets)
    # Confirm units: check a couple expected magnitudes



    # ----------------------------------
    # 2. Mechanical sign conventions
    # ----------------------------------
    deltas = apply_joint_conventions(deltas)

    # ================================
    # RIGHT-LEG MIRROR CORRECTION
    # ================================
    for leg in ("FR", "RR"):
        deltas[f"{leg}_THIGH"] *= -1
        deltas[f"{leg}_WRIST"] *= -1

    

    # ----------------------------------
    # 3. Normalize to physical servo space
    # ----------------------------------
    physical = normalize_all(deltas)

    # ----------------------------------
    # 4. COXA LOCK (NO YAW DURING GAIT)
    # ----------------------------------
    #physical["FL_COXA"] = COXA_STAND
    #physical["FR_COXA"] = COXA_STAND
    #physical["RL_COXA"] = COXA_STAND
    #physical["RR_COXA"] = COXA_STAND

    return physical


# layer6/gait_generator.py
import math
from typing import Dict, Tuple

# -------------------------------------------------
# Nominal stance (meters)
# -------------------------------------------------
STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

# -------------------------------------------------
# Gait parameters
# -------------------------------------------------
STEP_LENGTH = 0.10
STEP_HEIGHT = 0.045
DUTY_FACTOR = 0.60

# -------------------------------------------------
# Phase offsets (demo: all legs same arc)
# -------------------------------------------------
PHASE_OFFSET = {
    "FL": 0.0,
    "FR": 0.0,
    "RL": 0.0,
    "RR": 0.0,
}

# -------------------------------------------------
def _wrap_phase(p: float) -> float:
    return p % 1.0

# -------------------------------------------------
def _leg_trajectory(
    phase: float,
    step_length: float,
    step_height: float,
    duty: float,
) -> Tuple[float, float]:
    """
    Returns dx, dz relative to stance
    """

    # ---- STANCE ----
    if phase < duty:
        s = phase / duty
        dx = +step_length / 2 - s * step_length
        dz = 0.0
        return dx, dz

    # ---- SWING ----
    s = (phase - duty) / (1.0 - duty)
    dx = -step_length / 2 + s * step_length
    dz = step_height * math.sin(math.pi * s)

    return dx, dz

# -------------------------------------------------
def generate_foot_targets(
    t: float,
    frequency: float,
    step_length: float = STEP_LENGTH,
    step_height: float = STEP_HEIGHT,
    duty: float = DUTY_FACTOR,
) -> Dict[str, Tuple[float, float, float]]:

    if frequency <= 0.0:
        return {
            "FL": (STANCE_X,  STANCE_Y, STANCE_Z),
            "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
            "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
            "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
        }

    foot_targets = {}
    base_phase = t * frequency

    for leg, offset in PHASE_OFFSET.items():
        phase = _wrap_phase(base_phase + offset)
        dx, dz = _leg_trajectory(phase, step_length, step_height, duty)
        y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y

        foot_targets[leg] = (
            STANCE_X + dx,
            y,
            STANCE_Z + dz,
        )

    return foot_targets


"""
TEST — Layer 6 DIAGONAL ARC WALK (DROP-IN)
=========================================

Purpose:
- Use the SAME perfect arc generator
- Move only diagonal legs at a time
- Other diagonal stays planted
- No FSM
- No posture
- No IMU
- Minimal change from original demo

This is the SIMPLEST real gait.
"""

import time

# ---------------- Layer 6 ----------------
from layer6.gait_generator import _leg_trajectory

# ---------------- Layer 3 ----------------
from layer3.leg_ik import solve_all_legs

# ---------------- Layer 2 ----------------
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

# ---------------- Hardware ----------------
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# -------------------------------------------------
# IMPROVED CONFIG (MORE STABLE)
# -------------------------------------------------

FREQ = 1.5
DT = 0.02

STEP_LENGTH = 0.06    # reduced
STEP_HEIGHT = 0.030
DUTY = 0.87

STANCE_X = 0.0
STANCE_Y = 0.080
STANCE_Z = -0.17

# -------------------------------------------------
# COXA PRELOAD (ANTI-SLIP)
# -------------------------------------------------

COXA_BIAS = {
    "FL": -1.5,
    "FR": +1.5,
    "RL": -1.5,
    "RR": +1.5,
}

# -------------------------------------------------
# Servo map
# -------------------------------------------------

CHANNELS = {
    "FL_COXA":  COXA["FL"],
    "FL_THIGH": THIGHS["TFL"],
    "FL_WRIST": WRISTS["WFL"],

    "FR_COXA":  COXA["FR"],
    "FR_THIGH": THIGHS["TFR"],
    "FR_WRIST": WRISTS["WFR"],

    "RL_COXA":  COXA["RL"],
    "RL_THIGH": THIGHS["TRL"],
    "RL_WRIST": WRISTS["WRL"],

    "RR_COXA":  COXA["RR"],
    "RR_THIGH": THIGHS["TRR"],
    "RR_WRIST": WRISTS["WRR"],
}

# -------------------------------------------------
# Diagonal groups
# -------------------------------------------------

DIAG_A = ("FL", "RR")
DIAG_B = ("FR", "RL")

# -------------------------------------------------
# MAIN
# -------------------------------------------------

print("[TEST] Layer 6 — DIAGONAL ARC WALK")
print("FL+RR → FR+RL → repeat")
print("CTRL+C to stop")

t0 = time.time()

try:
    while True:
        t = time.time() - t0
        phase = (t * FREQ) % 1.0

        feet = {}

        for leg in ("FL", "FR", "RL", "RR"):

            # Diagonal phase shift
            if leg in DIAG_A:
                leg_phase = phase
            else:
                leg_phase = (phase + 0.5) % 1.0

            dx, dz = _leg_trajectory(
                leg_phase,
                STEP_LENGTH,
                STEP_HEIGHT,
                DUTY,
            )

            y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y

            # CRITICAL FIX:
            # stance legs move backward relative to body
            feet[leg] = (
                STANCE_X + dx,
                y,
                STANCE_Z + dz,
            )

        deltas = solve_all_legs(feet)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)

        # Light coxa preload
        for leg, bias in COXA_BIAS.items():
            physical[f"{leg}_COXA"] += bias

        for joint, ch in CHANNELS.items():
            set_servo_angle(ch, physical[joint])

        time.sleep(DT)

except KeyboardInterrupt:
    print("\n[TEST] Stopped cleanly 🫡")



