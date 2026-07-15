"""
L0 — ROBOT SPEC (single source of truth)
========================================

Every physical/electrical constant lives here and NOWHERE else. If a value
describes the machine (geometry, servo wiring, limits, calibration, I2C), it
belongs in this file. Layers above import from here; they never redefine.

Values are transcribed from the verified old stack (hardware/absolute_truths.py
and ik/kinematics.py) so the physical calibration is preserved exactly.

Pure data only — no logic, no I/O.
"""

from math import radians

# =====================================================================
# I2C / hardware
# =====================================================================
I2C_BUS   = 7
PCA_ADDR  = 0x40
MPU_ADDR  = 0x68

# PCA9685 registers
PCA_MODE1    = 0x00
PCA_PRESCALE = 0xFE

# MPU6050 registers
MPU_PWR_MGMT_1   = 0x6B
MPU_ACCEL_XOUT_H = 0x3B
MPU_GYRO_XOUT_H  = 0x43
MPU_CONFIG       = 0x1A
MPU_GYRO_CONFIG  = 0x1B
MPU_ACCEL_CONFIG = 0x1C

# =====================================================================
# Servo electrical / PWM
# =====================================================================
PWM_FREQ_HZ = 50            # DS3240MG servos
PULSE_MIN   = 106           # PCA9685 counts at 0 deg
PULSE_MAX   = 535           # PCA9685 counts at SERVO_TRAVEL_DEG
SERVO_TRAVEL_DEG = 270.0    # DS3240MG are 270-deg units

# =====================================================================
# Leg geometry (meters) — from ik/kinematics.py
# =====================================================================
LINK_COXA  = 0.01605
LINK_THIGH = 0.10832
LINK_WRIST = 0.13476
LEG_REACH  = LINK_THIGH + LINK_WRIST     # 0.24308 max straight-leg reach
PHI = radians(90)                        # coxa plane angle used by IK

BODY_LENGTH = 0.20830
BODY_WIDTH  = 0.07890
BODY_HEIGHT = 0.07400

# Canonical legs and IDs (one ordering for the whole stack)
LEGS = ("FL", "FR", "RL", "RR")
LEG_ID = {"FL": 0, "RL": 1, "FR": 2, "RR": 3}   # right side = {FR, RR}
RIGHT_LEGS = ("FR", "RR")

# Leg hip origins in body frame (x fwd, y left). Correctly paired to LEG_ID
# (fixes the old kinematics.py origin/id swap). Only used for body-rotation
# posture; the flat gait pipeline uses rot=0 where these cancel.
LEG_ORIGIN = {
    "FL": ( BODY_LENGTH / 2,  BODY_WIDTH / 2, 0.0),
    "RL": (-BODY_LENGTH / 2,  BODY_WIDTH / 2, 0.0),
    "FR": ( BODY_LENGTH / 2, -BODY_WIDTH / 2, 0.0),
    "RR": (-BODY_LENGTH / 2, -BODY_WIDTH / 2, 0.0),
}

# =====================================================================
# Nominal stance (hip-local foot target, meters) — THE reference.
# IK stand reference and the gait BOTH read these. Never duplicated.
# =====================================================================
STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

# Center-of-mass compensation (meters), applied to EVERY foot target so the real
# (off-center) CoM sits over the foot-support polygon. Found empirically by the
# IMU leg-lift calibrator (interface/calibrate_com.py). Positive values shift the
# feet; body moves opposite. Updated by calibration.
COM_OFFSET_X = 0.0   # fore-aft (thigh/wrist only, never coxa) — set by calibration
COM_OFFSET_Y = 0.0   # lateral compensation is DISABLED: it would move the coxas
                     # (splay the stance) and mask the problem instead of fixing it.

def stance_foot(leg, z=STANCE_Z):
    """Nominal foot target for a leg at height z (hip-local), pure geometry."""
    y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
    return (STANCE_X, y, z)

def apply_com_offset(feet, ox=None, oy=None):
    """Shift all foot targets by the CoM compensation (or an explicit override)."""
    ox = COM_OFFSET_X if ox is None else ox
    oy = COM_OFFSET_Y if oy is None else oy
    if ox == 0.0 and oy == 0.0:
        return feet
    return {leg: (x + ox, y + oy, z) for leg, (x, y, z) in feet.items()}


def apply_body_tilt(feet, pitch_amt, roll_amt):
    """
    Tilt the body via asymmetric leg heights — THIGH/WRIST ONLY, never coxa.
    A body roll/pitch shifts the CoM horizontally (the lateral shift we otherwise
    can't get without the coxa).
      pitch_amt > 0 : raise FRONT feet  (lowers front of body / shifts CoM back)
      roll_amt  > 0 : raise LEFT feet   (lowers left of body  / shifts CoM right)
    Amounts are foot-height deltas in meters.
    """
    if pitch_amt == 0.0 and roll_amt == 0.0:
        return feet
    out = {}
    for leg, (x, y, z) in feet.items():
        fs = 1.0 if leg in ("FL", "FR") else -1.0   # front / rear
        ls = 1.0 if leg in ("FL", "RL") else -1.0   # left / right
        out[leg] = (x, y, z + pitch_amt * fs + roll_amt * ls)
    return out

# =====================================================================
# Servo channel map (logical joint -> PCA9685 channel)
# =====================================================================
CHANNEL = {
    "FR_COXA": 6,  "FL_COXA": 7,  "RR_COXA": 0,  "RL_COXA": 1,
    "FR_THIGH": 8, "FL_THIGH": 9, "RR_THIGH": 2, "RL_THIGH": 3,
    "FR_WRIST": 10,"FL_WRIST": 11,"RR_WRIST": 4, "RL_WRIST": 5,
}

# Canonical joint order (coxa, thigh, wrist per leg)
JOINT_ORDER = tuple(f"{leg}_{j}" for leg in LEGS for j in ("COXA", "THIGH", "WRIST"))

# =====================================================================
# Mechanical limits (physical servo degrees). Right side is mounted
# mirrored (min > max); consumers sort before clamping.
# =====================================================================
MECH_LIMITS = {
    "FL_COXA":  {"min": 0,  "max": 90},   "FR_COXA":  {"min": 90, "max": 0},
    "RL_COXA":  {"min": 0,  "max": 90},   "RR_COXA":  {"min": 90, "max": 0},
    "FL_THIGH": {"min": 0,  "max": 270},  "FR_THIGH": {"min": 270,"max": 0},
    "RL_THIGH": {"min": 0,  "max": 270},  "RR_THIGH": {"min": 270,"max": 0},
    "FL_WRIST": {"min": 0,  "max": 200},  "FR_WRIST": {"min": 200,"max": 0},
    "RL_WRIST": {"min": 0,  "max": 200},  "RR_WRIST": {"min": 200,"max": 0},
}

# =====================================================================
# Stand pose (measured absolute servo degrees) — calibration anchor
# =====================================================================
STAND_ANGLE = {
    "FR_COXA": 47, "FL_COXA": 39, "RR_COXA": 44, "RL_COXA": 50,
    "FR_THIGH": 98,"FL_THIGH": 168,"RR_THIGH": 100,"RL_THIGH": 175,
    "FR_WRIST": 122,"FL_WRIST": 78,"RR_WRIST": 122,"RL_WRIST": 78,
}

# =====================================================================
# Joint sign conventions + zero offsets (IK-delta space).
# Transcribed verbatim from joints/conventions.py — DO NOT retune without
# re-deriving against angle_corrector AND the mech mount inversion.
# =====================================================================
JOINT_SIGN = {
    "FL_COXA": -1, "FR_COXA": +1, "RL_COXA": -1, "RR_COXA": +1,
    "FL_THIGH": +1,"FR_THIGH": +1,"RL_THIGH": +1,"RR_THIGH": +1,
    "FL_WRIST": +1,"FR_WRIST": -1,"RL_WRIST": +1,"RR_WRIST": -1,
}

JOINT_OFFSET = {j: 0.0 for j in JOINT_ORDER}

# =====================================================================
# IMU scale factors (MPU6050 power-on defaults: +/-250 dps, +/-2 g)
# =====================================================================
GYRO_LSB_PER_DPS = 131.0
ACCEL_LSB_PER_G  = 16384.0
