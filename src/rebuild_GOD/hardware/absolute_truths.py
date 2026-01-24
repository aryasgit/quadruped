# absolute_truths.py
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
    "WFL": {"min": 0,   "max": 200, "perp": 153},
    "WRL": {"min": 0,   "max": 200, "perp": 153},
    "WFR": {"min": 200, "max": 0,   "perp": 47},
    "WRR": {"min": 200, "max": 0,   "perp": 47},
}

THIGH_MECH = {
    "TFL": {"min": 0,   "max": 270, "perp": 130},
    "TRL": {"min": 0,   "max": 270, "perp": 130},
    "TFR": {"min": 270, "max": 0,   "perp": 140},
    "TRR": {"min": 270, "max": 0,   "perp": 140},
}

COXA_MECH = {
    "FL": {"min": 0,  "max": 90, "perp": 45},
    "RL": {"min": 0,  "max": 90, "perp": 45},
    "FR": {"min": 90, "max": 0,  "perp": 45},
    "RR": {"min": 90, "max": 0,  "perp": 45},
}

# --- Stand pose (measured absolute angles) ---
WRIST_STAND = {
    "WFR": 122,
    "WFL": 78,
    "WRR": 122,
    "WRL": 78,
}

THIGH_STAND = {
    "TFR": 100,
    "TFL": 170,
    "TRR": 100,
    "TRL": 170,
}

COXA_STAND = {
    "FR": 45,
    "FL": 45,
    "RR": 45,
    "RL": 45,
}

# That's it — pure facts only.
