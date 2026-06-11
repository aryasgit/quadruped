"""
Layer 0 — ABSOLUTE TRUTHS
=========================

Measured electrical & mechanical facts of the BARQ v1 robot.

Ported VERBATIM from the legacy stack (src/main/hardware/absolute_truths.py),
where every number was measured on the physical robot. Do not edit without
re-measuring on hardware.

If anything here is wrong, the physical robot is wrong.
Constants only: no logic, no math, no imports.

One deliberate change from the legacy file: chip register maps (PCA9685/MPU)
now live in their drivers — they are datasheet facts, not robot facts.
"""

# --- I2C topology (Jetson Orin Nano, 40-pin header pins 3/5) ---
I2C_BUS = 7            # Jetson I2C bus wired to the servo/IMU board
PCA_ADDR = 0x40        # PCA9685 16-channel PWM driver
MPU_ADDR = 0x68        # MPU6050 on the HW-290 10-DOF IMU board

# --- Servo electrical truths ---
# 12x DS3240MG (40 kg.cm, 270 deg travel), measured PCA9685 pulse range.
# Units are PCA9685 *ticks*: 12-bit counter, 4096 ticks per 20 ms frame at
# 50 Hz, so 1 tick ~= 4.883 us (nominal 25 MHz oscillator).
#   PULSE_MIN = 106 ticks ~=  518 us  <->  legacy servo angle   0 deg
#   PULSE_MAX = 535 ticks ~= 2612 us  <->  legacy servo angle 270 deg
# All perp/stand/mech values below were measured against THIS board's
# oscillator with prescale for 50 Hz — keep ticks as the canonical unit.
PWM_FREQ_HZ = 50
PULSE_MIN = 106
PULSE_MAX = 535
SERVO_TRAVEL_DEG = 270.0   # legacy "servo angle" domain spans PULSE_MIN..PULSE_MAX

# --- Servo channel map (logical names -> PCA9685 channel) ---
# Legacy naming: COXA = link 1 (hip), THIGH = link 2 (upper leg),
# WRIST = link 3 (lower leg / knee). F/R = front/rear, L/R = left/right.
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

# --- Mechanical limits and perpendicular reference (measured) ---
# Values are in the legacy 0-270 "servo angle" domain.
# min > max is intentional: it encodes an inverted (mirrored) servo mount.
WRIST_MECH = {
    "WFL": {"min": 0,   "max": 200, "perp": 153},
    "WRL": {"min": 0,   "max": 200, "perp": 153},
    "WFR": {"min": 200, "max": 0,   "perp": 47},
    "WRR": {"min": 200, "max": 0,   "perp": 47},
}

THIGH_MECH = {
    "TFL": {"min": 0,   "max": 270, "perp": 128},
    "TRL": {"min": 0,   "max": 270, "perp": 135},
    "TFR": {"min": 270, "max": 0,   "perp": 138},
    "TRR": {"min": 270, "max": 0,   "perp": 140},
}

COXA_MECH = {
    "FL": {"min": 0,  "max": 90, "perp": 39},
    "RL": {"min": 0,  "max": 90, "perp": 50},
    "FR": {"min": 90, "max": 0,  "perp": 47},
    "RR": {"min": 90, "max": 0,  "perp": 44},
}

# --- Stand pose (measured absolute angles, legacy servo-angle domain) ---
WRIST_STAND = {
    "WFR": 122,
    "WFL": 78,
    "WRR": 122,
    "WRL": 78,
}

THIGH_STAND = {
    "TFR":  98,
    "TFL": 168,
    "TRR": 100,
    "TRL": 175,
}

COXA_STAND = {
    "FR": 47,
    "FL": 39,
    "RR": 44,
    "RL": 50,
}

# That's it — pure facts only.
