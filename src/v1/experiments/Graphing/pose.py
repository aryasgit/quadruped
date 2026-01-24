# Copyright (c) 2025 Aryaman Gupta
# All Rights Reserved.
#
# This code is proprietary.
# Unauthorized use, modification, or distribution is prohibited.

import time, math, errno
from smbus2 import SMBus
import threading 
import tkinter as tk
from collections import deque
import multiprocessing as mp
from plotter_process import plotter_main
mp.set_start_method("fork", force=True)


# ==================================================
# LIVE TELEMETRY IPC (PROCESS-SAFE)
# ==================================================
telemetry_q = mp.Queue(maxsize=50)

# ==================================================
# I2C CONFIG (HARDWARE — DO NOT TWEAK)
# ==================================================
servo_angles = {}
BUS = 7
MPU_ADDR = 0x68
PCA_ADDR = 0x40

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

MODE1 = 0x00
PRESCALE = 0xFE

CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C

# ==================================================
# SERVO CONFIG (MECHANICAL LIMITS)
# ==================================================
# PWM pulse range for your servos
# ❌ Do not change unless servo specs change
PULSE_MIN = 106
PULSE_MAX = 535

# ==================================================
# SHOULDER JOINTS — ROLL CONTROL
# ==================================================
# These joints control LEFT–RIGHT balance
# Real-world effect: widening/narrowing stance

SHOULDERS = {
    "FR": 6,
    "FL": 7,
    "RR": 0,
    "RL": 1,
}

SHOULDER_MAX_OFFSET = {
    "FR": 35.0,
    "FL": 35.0,
    "RR": 30.0,
    "RL": 30.0,
}

# Neutral standing angles (MEASURED, DO NOT GUESS)
SHOULDER_STAND = {
    "FR": 37,
    "FL": 46,
    "RR": 38,
    "RL": 38,
}

# Direction mapping:
# +angle → abduct (move outward)
# -angle → adduct (move inward)
# ❌ DO NOT CHANGE unless servo direction is rewired
SHOULDER_SIGN = {
    "FR": +1,
    "RR": +1,
    "FL": -1,
    "RL": -1,
}

# Roll compensation sign:
# Determines which side pushes during roll
# This mapping is CRITICAL for correct roll response
SHOULDER_ROLL_SIGN = {
    "FR": -1,
    "RR": -1,   # right side resists positive roll
    "FL": +1,
    "RL": +1,   # left side supports positive roll
}

# ==================================================
# FOOT (KNEE) JOINTS — PITCH CONTROL
# ==================================================
# These joints control FRONT–BACK balance
# Real-world effect: shifting vertical load

FEET = {
    "FRF": 10,
    "FLF": 11,
    "RRF": 4,
    "RLF": 5,
}

# Neutral standing angles (MEASURED)
FOOT_STAND = {
    "FRF": 130,
    "FLF": 61,
    "RRF": 128,
    "RLF": 77,
}

# Direction mapping:
# +offset → extension or flexion depending on side
# ❌ Change ONLY if mechanical linkage changes
FOOT_SIGN = {
    "FRF": +1,
    "FLF": -1,
    "RRF": +1,
    "RLF": -1,
}

# Mechanical safety limits — PREVENT DAMAGE
FOOT_LIMITS = {k: (0, 190) for k in FEET}

# ==================================================
# CONTROL PARAMETERS (THIS IS WHERE YOU TUNE)
# ==================================================
DT = 0.05        # Control loop timestep (20 Hz)
ALPHA = 0.96     # Complementary filter weight
                 # ↑ Higher = smoother, slower
                 # ↓ Lower = faster, noisier

# ---------- ROLL CONTROL ----------
K_ROLL = 0.8            # Roll stiffness (SAFE TO TWEAK)
MAX_ROLL = 30.0         # IMU roll clamp (SAFETY)
ROLL_STEP = 1.5         # Shoulder speed (deg per cycle)
ROLL_LIMIT = 20.0       # Max shoulder offset (SAFE TO TWEAK)
K_ROLL_RATE = 0.025     # Roll-rate feedforward (gyro-based)

# ---------- PITCH CONTROL ----------
K_PITCH = 0.8        # Pitch stiffness (SAFE TO TWEAK)
MAX_PITCH = 30.0     # IMU pitch clamp (SAFETY)
PITCH_STEP = 1.5     # Knee speed (deg per cycle)
PITCH_LIMIT = 25.0   # Max knee offset (SAFETY)
K_PITCH_RATE = 0.03  # Pitch-rate feedforward(start conservative)

# IMU noise rejection
ACCEL_DEADBAND = 50
GYRO_DEADBAND = 10
GYRO_RATE_DEADBAND = 1.5 

bus = SMBus(BUS)

# ==================================================
# BALANCE EFFORT MONITOR (for unloading detection)
# ==================================================
last_roll_cmd = 0.0
last_roll_offsets = {k: 0.0 for k in SHOULDERS}
roll_effort = 0.0

# ==================================================
# PRINT INTERVAL
# ==================================================
DEBUG_PRINT = False
PRINT_INTERVAL = 2.0 # seconds
last_print = time.time()
last_contact_print = 0.0
CONTACT_PRINT_DT = 0.3
last_printed_contact = 1.0
CONTACT_PRINT_EPS = 0.08   # print only on meaningful change

# ==================================================
# AIRBORNE DETECTION STATE
# ==================================================
airborne_conf = 0.0          # 0.0 = grounded, 1.0 = airborne
robot_airborne = False

# Gravity unload threshold (raw accel units)
GRAVITY_NOMINAL = 16384
GRAVITY_LOSS_THRESH = 2600   # ~0.15g drop (slow lift)

AIRBORNE_RISE = 0.18         # how fast airborne confidence rises
AIRBORNE_FALL = 0.10         # how fast it decays
# Slow lift detection (quasi-static)
SLOW_LIFT_RISE = 0.06      # slow confidence rise
SLOW_LIFT_FALL = 0.04
SLOW_LIFT_CONTACT_MAX = 0.45
SLOW_LIFT_EFFORT_MAX = 0.08



# ==================================================
# STEP PRIMITIVE (SEALED)
# A STEP consists of:
# UNLOADING → LIFTING → DONE → RECENTERING → CONTACT → IDLE
# Once contact is detected, the step is considered COMPLETE.
# No external code should interfere mid-step.
# ==================================================

step_active = False

# ==================================================
# GAIT SEQUENCER (SAFE, EVENT-DRIVEN)
# ==================================================

GAIT_SEQUENCE = ["FRF", "RLF", "FLF", "RRF"]  # crawl gait (diagonal support)
FRONT_LEGS = ("FRF", "FLF")
REAR_LEGS  = ("RRF", "RLF")


gait_enabled = False
gait_index = 0

STOP_HOLD_TIME = 1.5   # seconds
stop_hold_until = 0.0

show_motion = None   # default


# ==================================================
# MICRO PROPULSION (STANCE PUSH) — OPTION 1
# ==================================================
PROP_PUSH_DEG = 1.8          # degrees (START HERE)
PROP_PUSH_STEP = 0.25        # deg per cycle (smooth)
PROP_CONTACT_MIN = 0.85      # must be firmly grounded
PROP_SLIP_MAX = 0.25         # disable if slipping

FSM_ID = {
    "IDLE": 0,
    "UNLOADING": 1,
    "LIFTING": 2,
    "DONE": 3,
    "RECENTERING": 4,
    "STOP_HOLD": 5,
}

POSTURE_ID = {
    "STAND": 0,
    "LEAN_LEFT": 1,
    "LEAN_RIGHT": 2,
    "LEAN_FWD": 3,
    "LEAN_BACK": 4,
    "SIT": 5,
    "KNEEL": 6,
    "SHOW": 7,
}

# ==================================================
# PUBLIC STEP API (THE ONLY ENTRY POINT)
# ==================================================
def request_step(leg):
    """
    Safely request a single step.
    This is the ONLY allowed way to initiate unload + lift.
    Returns True if accepted, False if rejected.
    """
    global swing_leg, step_active

    # Reject invalid legs
    if leg not in FEET:
        return False

    # Reject if a step is already running
    if step_active or fsm_state != "IDLE":
        return False

    # Arm step
    swing_leg = leg
    step_active = True

    return True

def gait_tick():
    """
    One tick of the gait sequencer.
    Requests the next step only if the system is ready.
    """
    global gait_index

    if not gait_enabled:
        return

    # Do nothing if a step is still running
    if step_active:
        return

    # Request next leg
    leg = GAIT_SEQUENCE[gait_index]
    accepted = request_step(leg)

    if accepted:
        gait_index = (gait_index + 1) % len(GAIT_SEQUENCE)
        
# ==================================================
# DATA LOGGER (NON-BLOCKING)
# ==================================================
LOG_ENABLE = True
LOG_DT = 0.05           # log at control rate (20 Hz)
LOG_FILE = "run_log.csv"
last_log_time = 0.0
log_active = False        # 👈 MASTER GATE
log_start_time = 0.0


SERVO_DELTA_GROUND = 0.15   # deg → constrained
SERVO_DELTA_AIR    = 0.6    # deg → free

if LOG_ENABLE:
    log_f = open(LOG_FILE, "w")
    log_f.write(
        "t,"
        "joint,servo_angle,servo_delta,"
        "motion_cmd,leg_response,foot_contact,"
        "roll,pitch,"
        "posture_roll,posture_pitch,"
        "roll_effort,offset_change,"
        "posture_mode,fsm,stomp_phase\n"
    )
    log_f.flush()
    
# ==================================================
# LOW-LEVEL HARDWARE FUNCTIONS (DO NOT TWEAK)
# ==================================================
def safe_read_word(addr, reg):
    """
    Reads signed 16-bit value from I2C
    Includes protection against transient I2C errors
    """
    try:
        h = bus.read_byte_data(addr, reg)
        l = bus.read_byte_data(addr, reg + 1)
        v = (h << 8) | l
        return v - 65536 if v > 32767 else v
    except OSError as e:
        if e.errno == errno.EREMOTEIO:
            raise IOError
        raise

def angle_to_pulse(angle):
    """
    Converts angle (0–270°) to PCA9685 pulse
    """
    return int(PULSE_MIN + (angle / 270.0) * (PULSE_MAX - PULSE_MIN))

def set_servo_angle(ch, angle):
    """
    Sends final angle command to servo channel
    """
    pulse = angle_to_pulse(angle)
    base = 0x06 + 4 * ch
    bus.write_byte_data(PCA_ADDR, base, 0)
    bus.write_byte_data(PCA_ADDR, base + 1, 0)
    bus.write_byte_data(PCA_ADDR, base + 2, pulse & 0xFF)
    bus.write_byte_data(PCA_ADDR, base + 3, (pulse >> 8) & 0x0F)

# ==================================================
# MID-LIMB (HIP) LOCK — ONE-TIME INITIALIZATION
# ==================================================
# These joints are posture-locked and NOT controlled further

MID_LIMBS = {
    8: 104,   # FRM
    9: 165,   # FLM
    2: 102,   # RRM
    3: 154,   # RLM
}

for ch, angle in MID_LIMBS.items():
    set_servo_angle(ch, angle)
    servo_angles[ch] = angle
    time.sleep(0.02)
    
# ==================================================
# DEVICE INITIALIZATION (DO NOT TWEAK)
# ==================================================
# Wake IMU
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
time.sleep(0.1)

# Set MPU6050 Digital Low Pass Flter (~20Hz)
bus.write_byte_data(MPU_ADDR, CONFIG, 0x04)
time.sleep(0.05)

# Initialize PCA9685 at 50 Hz
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
time.sleep(0.01)
prescale = int(25000000 / (4096 * 50) - 1)
bus.write_byte_data(PCA_ADDR, MODE1, 0x10)
bus.write_byte_data(PCA_ADDR, PRESCALE, prescale)
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
bus.write_byte_data(PCA_ADDR, MODE1, 0x80)

# ==================================================
# MOVE TO STANDING POSE (SAFE STARTUP)
# ==================================================
# Always bring robot to neutral before control starts
for leg, ch in SHOULDERS.items():
    set_servo_angle(ch, SHOULDER_STAND[leg])
    time.sleep(0.02)

for leg, ch in FEET.items():
    set_servo_angle(ch, FOOT_STAND[leg])
    time.sleep(0.02)

# ==================================================
# IMU CALIBRATION (KEEP ROBOT STILL)
# ==================================================
ax_o = ay_o = az_o = gx_o = gy_o = 0
for _ in range(200):
    ax_o += safe_read_word(MPU_ADDR, ACCEL_XOUT_H)
    ay_o += safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 2)
    az_o += safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 4) - 16384
    gx_o += safe_read_word(MPU_ADDR, GYRO_XOUT_H)
    gy_o += safe_read_word(MPU_ADDR, GYRO_XOUT_H + 2)
    time.sleep(0.01)

ax_o /= 200; ay_o /= 200; az_o /= 200
gx_o /= 200; gy_o /= 200

# ==================================================
# CONTROL STATE VARIABLES
# ==================================================
roll = 0.0
pitch = 0.0
posture_locked = False

STAND_ROLL_REF = None
STAND_PITCH_REF = None
stand_ref_locked = False

# ==================================================
# STOMP (SHOW MODE, FSM-BASED)
# ==================================================
stomp_active = False
stomp_phase = "IDLE"   # IDLE / UNLOAD / LIFT / SLAM
stomp_leg = "FRF"      # start with front-right
stomp_t0 = 0.0

# ADD — BALANCE EFFORT MONITOR (one-time initialization)
# Place immediately after roll/pitch / posture variables
last_roll_cmd = 0.0
last_roll_offsets = {k: 0.0 for k in SHOULDERS}  # store last shoulder offsets
roll_effort = 0.0
offset_change = 0.0

# These store smooth joint offsets
roll_offsets = {k: 0.0 for k in SHOULDERS}
pitch_offsets = {k: 0.0 for k in FEET}

print("Layer-1 Roll + Pitch ACTIVE (±30°)\n")
# ==================================================
# START LIVE PLOT PROCESS (SEPARATE PROCESS)
# ==================================================
from plotter_process import plotter_main

plot_proc = mp.Process(
    target=plotter_main,
    args=(telemetry_q,),
    daemon=True
)
plot_proc.start()
print("[PLOT] plotter process started")
# ---- REMOTE TELEMETRY DISABLED ----
# telem_proc = mp.Process(
#     target=telemetry_publisher,
#     args=(telemetry_q, 5555),
#     daemon=True
# )
# telem_proc.start()
# print("[TELEM] remote telemetry started")



# ===============================
# LAYER-2: POSTURE / MOTION STATE
# ===============================
# ==================================================
# SHOW MODE (CROWD DEMO)
# ==================================================
show_active = False
show_start_time = 0.0
SHOW_FREQ_SWAY = 0.4     # Hz (slow = dramatic)
SHOW_FREQ_SQUAT = 0.25
SHOW_ROLL_AMP = 18.0     # degrees
SHOW_PITCH_AMP = 22.0
SHOW_KNEE_AMP = 12.0

# ==================================================
# POSTURE MODES (HIGH-LEVEL INTENT)
# ==================================================
posture_mode = "STAND"   # STAND / LEAN_LEFT / LEAN_RIGHT / LEAN_FWD / LEAN_BACK / SIT / KNEEL
last_posture_change_time = time.time()
POSTURE_SETTLE_TIME = 0.6   # seconds (tuned to ramp speed)

# Posture biases (targets + current)
posture_roll_target  = 0.0
posture_pitch_target = 0.0
posture_roll_bias   = 0.0   # current applied bias (will ramp to target)
posture_pitch_bias  = 0.0
STAND_RECENTER_STEP = 0.8   # slower than RECENTER_STEP
STAND_RECENTER_IMPULSE = 4.0   # degrees (safe: 3–6)
STAND_IMPULSE_TIME = 0.4       # seconds
stand_recenter_until = 0.0
mid_offsets = {ch: 0.0 for ch in MID_LIMBS}
MID_RECENTER_STEP = 0.6


# ==================================================
# POSTURE CALIBRATED TARGETS (MEASURED FROM ROBOT)
# ==================================================
POSTURE_TARGETS = {

    # ✅ TRUE SIT (measured absolute posture)
    "SIT": {
            "posture_pitch": 0.0,   # IMPORTANT: no pitch bias
            "feet": {
                "FRF": +27.0,
                "FLF": -27.0,
                "RRF":  0.0,
                "RLF":  0.0,
            }
        },
    # ✅ KNEEL (reuse old SIT behaviour)
    "KNEEL": {
        "posture_pitch": +2.0,
        "feet": {
            "FRF":  0.0,
            "FLF": +4.0,
            "RRF": +25.0,
            "RLF": +36.0,
        }
    }
}
POSTURE_TARGETS["KNEEL"] = {
    "posture_pitch": 0.0,   # no body pitch bias

    # Shoulder offsets (relative to STAND)
    "shoulders": {
        "FR": -15.0,
        "FL": +16.0,
        "RR": +10.0,
        "RL": -11.0,
    },

    # Feet offsets (relative to STAND)
    "feet": {
        "FRF": 0.0,
        "FLF": 0.0,
        "RRF": 0.0,
        "RLF": 0.0,
    }
}
POSTURE_TARGETS["KNEEL"]["midlimbs"] = {
    8: +19.0,   # FRM
    9: -21.0,   # FLM
    2:  0.0,    # RRM
    3:  0.0,    # RLM
}


POSTURE_MAX_ROLL   = 20.0    # deg - max intentional lean
POSTURE_MAX_PITCH  = 30.0
POSTURE_STEP       = 0.4    # deg per loop - smoothness


# Motion offsets per-joint (added on top of balance offsets)
motion_offsets = {k: 0.0 for k in SHOULDERS}
motion_offsets.update({k: 0.0 for k in FEET})

# ==================================================
# DEMO MODE (PRESENTATION SEQUENCER)
# ==================================================
demo_active = False
demo_index = 0
demo_step_start = 0.0

DEMO_SEQUENCE = [
    ("LEAN_LEFT",  2.0),
    ("LEAN_RIGHT", 2.0),
    ("LEAN_FWD",   2.0),
    ("LEAN_BACK",  2.0),
    ("SIT",        3.0),
    ("STAND",      2.0),
    ("KNEEL",      4.0),
    ("STAND",      2.0),
]

# ==================================================
# RECENTERING PARAMETERS
# ==================================================
RECENTER_STEP = 0.5        # deg per loop (how fast offsets fade out)
RECENTER_THRESH = 0.5      # deg — considered "neutral"

# Simple ramp helper
def ramp(current, target, step):
    d = target - current
    if d > step: d = step
    if d < -step: d = -step
    return current + d
# ==================================================
# POSTURE MODE RESOLVER (SAFE, COMPOSABLE)
# ==================================================
# ==================================================
# SHOW MOTION PRIMITIVES (VISUAL, CROWD MODE)
# ==================================================

def sway(t):
    """
    Left-right body sway with knee pumping.
    Dramatic, safe, visually expressive.
    """
    roll  = SHOW_ROLL_AMP * math.sin(2 * math.pi * SHOW_FREQ_SWAY * t)
    pitch = -SHOW_PITCH_AMP * abs(
        math.sin(2 * math.pi * SHOW_FREQ_SQUAT * t)
    )

    knee = SHOW_KNEE_AMP * math.sin(2 * math.pi * SHOW_FREQ_SWAY * t)

    return roll, pitch, knee

def squat(t):
    """
    Slow squat down → powerful rise.
    """
    phase = math.sin(2 * math.pi * 0.25 * t)

    roll = 0.0
    pitch = -28.0 * max(0.0, phase)   # only squat downward
    knee = 18.0 * max(0.0, phase)

    return roll, pitch, knee

def bow(t):
    """
    Forward bow with pause and return.
    """
    cycle = t % 4.0

    if cycle < 1.2:          # lean forward
        pitch = -18.0 * (cycle / 1.2)
    elif cycle < 2.2:        # hold
        pitch = -18.0
    else:                    # return
        pitch = -18.0 * max(0.0, 1 - (cycle - 2.2) / 1.8)

    return 0.0, pitch, 0.0

def stomp(t):
    """
    EXTREME single-leg stomp.
    Visually violent, demo-only.
    """

    cycle = t % 1.6   # shorter = more aggressive

    roll = 0.0
    pitch = 0.0
    knee = 0.0

    # ---- Phase 1: anticipation (freeze) ----
    if cycle < 0.25:
        pitch = -8.0   # lean forward HARD

    # ---- Phase 2: explosive lift ----
    elif cycle < 0.55:
        knee = 32.0 * ((cycle - 0.25) / 0.30)

    # ---- Phase 3: SLAM (VERY FAST) ----
    elif cycle < 0.65:
        knee = 32.0 - 70.0 * ((cycle - 0.55) / 0.10)

    # ---- Phase 4: recoil ----
    elif cycle < 1.1:
        pitch = +12.0 * (1 - (cycle - 0.65) / 0.45)

    return roll, pitch, knee

def apply_posture_mode():
    global posture_roll_target, posture_pitch_target

    # ❌ Do not allow posture changes mid-step
    if step_active or fsm_state != "IDLE":
        return

    # Defaults
    posture_roll_target  = 0.0
    posture_pitch_target = 0.0

    # ---------- MODE DEFINITIONS ----------
    if posture_mode == "STAND":
        posture_roll_target  = 0.0
        posture_pitch_target = 0.0

        for k in motion_offsets:
            motion_offsets[k] = ramp(motion_offsets[k], 0.0, STAND_RECENTER_STEP)

        for ch in mid_offsets:
            mid_offsets[ch] = ramp(mid_offsets[ch], 0.0, MID_RECENTER_STEP)


    elif posture_mode == "LEAN_LEFT":
        posture_roll_target = +15.0

    elif posture_mode == "LEAN_RIGHT":
        posture_roll_target = -15.0

    elif posture_mode == "LEAN_FWD":
        posture_pitch_target = -20.0

    elif posture_mode == "LEAN_BACK":
        posture_pitch_target = +15.0

    elif posture_mode == "SIT":
        tgt = POSTURE_TARGETS["SIT"]
        posture_pitch_target = 0.0   # DO NOT tilt body

        for leg, val in tgt["feet"].items():
            motion_offsets[leg] = ramp(
                motion_offsets[leg],
                val,      # ⚠️ NO SIGN HERE
                0.6
            )


    elif posture_mode == "KNEEL":
        tgt = POSTURE_TARGETS["KNEEL"]
        posture_pitch_target = tgt["posture_pitch"]

        # Shoulders
        for leg, val in tgt["shoulders"].items():
            motion_offsets[leg] = ramp(motion_offsets[leg], val, 0.8)

        # Feet
        for leg, val in tgt["feet"].items():
            motion_offsets[leg] = ramp(motion_offsets[leg], val, 0.8)

        # 🔥 Mid-limbs (NEW)
        for ch, val in tgt["midlimbs"].items():
            mid_offsets[ch] = ramp(mid_offsets[ch], val, MID_RECENTER_STEP)
    
    elif posture_mode == "SHOW":

        if stomp_active:
            posture_roll_target = posture_roll_target
            posture_pitch_target = posture_pitch_target
            return
        if show_motion is None:
            # Fail-safe: do nothing in SHOW until a motion is selected
            posture_roll_target = 0.0
            posture_pitch_target = 0.0
            return

        t = time.time() - show_start_time
        posture_roll_target, posture_pitch_target, knee = show_motion(t)

        # Clear all knee motion first
        for leg in FEET:
            motion_offsets[leg] = ramp(motion_offsets[leg], 0.0, 0.8)

        # Apply stomp knee to ONE leg
        STOMP_LEG = "FRF"
        motion_offsets[STOMP_LEG] = ramp(
            motion_offsets[STOMP_LEG],
            knee,
            3.5    # faster for impact
        )








    # ---------- HARD SAFETY CLAMPS ----------
    posture_roll_target  = max(-POSTURE_MAX_ROLL,  min(POSTURE_MAX_ROLL,  posture_roll_target))
    posture_pitch_target = max(-POSTURE_MAX_PITCH, min(POSTURE_MAX_PITCH, posture_pitch_target))

def demo_tick():
    global demo_active, demo_index, demo_step_start, posture_mode

    if not demo_active:
        return

    # Never interfere with steps or gait
    if step_active or fsm_state != "IDLE":
        return

    now = time.time()

    # First entry
    if demo_step_start == 0.0:
        demo_step_start = now
        posture_mode = DEMO_SEQUENCE[demo_index][0]
        return

    mode, duration = DEMO_SEQUENCE[demo_index]

    # Advance demo step
    if (now - demo_step_start) >= duration:
        demo_index += 1
        demo_step_start = now

        if demo_index >= len(DEMO_SEQUENCE):
            demo_active = False
            demo_index = 0
            posture_mode = "STAND"
            return

        posture_mode = DEMO_SEQUENCE[demo_index][0]

# Unload posture map (lean away from leg)
UNLOAD_POSTURE = {
    "FRF": {"roll": +POSTURE_MAX_ROLL, "pitch": -POSTURE_MAX_PITCH},
    "FLF": {"roll": -POSTURE_MAX_ROLL, "pitch": -POSTURE_MAX_PITCH},
    "RRF": {"roll": +POSTURE_MAX_ROLL, "pitch": +POSTURE_MAX_PITCH},
    "RLF": {"roll": -POSTURE_MAX_ROLL, "pitch": +POSTURE_MAX_PITCH},
}

# ==================================================
# STANCE PROPULSION STATE (GLOBAL, SAFE)
# ==================================================
prop_target = {k: 0.0 for k in FEET}

# ==================================================
# SHOULDER GEOMETRIC UNLOAD MAP (AUTHORITATIVE)
# Positive = outward, Negative = inward
# ==================================================
UNLOAD_SHOULDER_OFFSETS = {
    # Lifting FRONT RIGHT foot
    "FRF": {
        "RR": +6.0,
        "RL": +6.0,
        "FL": -5.0,
        "FR":  0.0,
    },
    # Lifting FRONT LEFT foot
    "FLF": {
        "RR": +8.0,
        "RL": +8.0,
        "FR": -10.0,
        "FL":  0.0,
    },
    # Lifting REAR RIGHT foot
    "RRF": {
        "FR": +7.0,
        "FL": +7.0,
        "RL": -15.0,
        "RR":  0.0,
    },
    # Lifting REAR LEFT foot
    "RLF": {
        "FR": +5.0,
        "FL": +5.0,
        "RR": -10.0,
        "RL":  0.0,
    },
}

# FSM for simple unload + lift of a single leg (start disabled)
swing_leg = None        # e.g. "FRF" when commanded
fsm_state = "IDLE"      # IDLE / UNLOADING / LIFTING / DONE
disable_roll_pd = False

# ==================================================
# GUI COMMAND STATE (INTENT ONLY)
# ==================================================
gui_selected_leg = None      # "FRF", "FLF", "RRF", "RLF"
gui_enabled = False

# LIFT parameters
LIFT_HEIGHT = 15.0      # deg (knee flex offset as motion)
LIFT_STEP = 0.8         # deg per cycle for knee motion
UNLOAD_SETTLE_THRESH = 1.0  # deg — how close IMU must be to posture bias

# ==================================================
# BRACE STATE (NEW)
# ==================================================
BRACE_ACCEL_THRESHOLD = 3000     # raw accel delta
BRACE_DURATION = 0.25            # seconds
BRACE_GAIN_SCALE = 0.6           # soften controller
BRACE_MAX_BIAS = 12.0            # deg (keep small)
BRACE_GAIN_PITCH = 0.45   # forward/back (less trusted)
BRACE_GAIN_ROLL  = 1.0    # left/right (fully trusted)


brace_active = False
brace_until = 0.0

brace_roll_bias = 0.0
brace_pitch_bias = 0.0

last_ax = last_ay = last_az = 0.0

motion_armed = True


# ==================================================
# MONITOR STATE (READ-ONLY)
# ==================================================
monitor_state = {
    "roll": 0.0,
    "pitch": 0.0,
    "slip": {},
    "stability": 1.0,

    "brace": False,
    "brace_roll_bias": 0.0,
    "brace_pitch_bias": 0.0,

    # Raw acceleration
    "ax": 0.0,
    "ay": 0.0,
    "az": 0.0,
    
    # ADD — debug / unload detection
    "roll_effort": 0.0,
    "offset_change": 0.0,
    "unloaded": False,
    
    #foot contact confidence 0 to 1
    "foot_contact": {},
    "leg_motion_response": {},
    
    # STEP diagnostics
    "step_phase": "IDLE",
    "step_active": False,
    "last_step_ok": True
    
}
# ==================================================
# LIVE TELEMETRY BUFFER (READ-ONLY)
# ==================================================
PLOT_WINDOW_SEC = 12.0
PLOT_HZ = 20
PLOT_LEN = int(PLOT_WINDOW_SEC * PLOT_HZ)

# ==================================================
# FINAL UNIFIED MESSY DARK CONTROL + DIAGNOSTICS GUI
# ==================================================
def start_unified_gui():
    CYBER_BG = "#0b0b0b"
    CYBER_PANEL = "#151515"
    CYBER_BAR_BG = "#222222"
    CYBER_PINK = "#ff2ec4"
    CYBER_PINK_DIM = "#c61aa3"
    CYBER_TEXT = "#e6e6e6"
    CYBER_MUTED = "#9a9a9a"

    def set_stomp():
        global posture_mode, stomp_active, stomp_phase, stomp_t0
        global swing_leg, step_active
        global log_active, log_start_time
        log_active = True
        log_start_time = time.time()
        log_f.write("# EVENT_START %.3f\n" % time.time())
        log_f.flush()


        if step_active:
            return  # never interrupt real steps

        posture_mode = "SHOW"
        stomp_active = True
        stomp_phase = "UNLOAD"
        stomp_t0 = time.time()



    def set_sway():
        global show_motion, posture_mode, show_start_time
        posture_mode = "SHOW"
        show_motion = sway
        show_start_time = time.time()

    def set_squat():
        global show_motion, posture_mode, show_start_time
        posture_mode = "SHOW"
        show_motion = squat
        show_start_time = time.time()

    def set_bow():
        global show_motion, posture_mode, show_start_time
        posture_mode = "SHOW"
        show_motion = bow
        show_start_time = time.time()

    global gui_selected_leg

    root = tk.Tk()
    root.title("QUADRUPED // CONTROL // DIAGNOSTICS")
    root.geometry("1100x680")
    root.configure(bg="#0b0b0b")

    # ---------------- Fonts ----------------
    FONT_BIG  = ("Segoe UI Semibold", 20)
    FONT_HDR  = ("Segoe UI Semibold", 12)
    FONT_MAIN = ("Segoe UI", 10)
    FONT_MONO = ("Consolas", 9)

    # ---------------- Header ----------------
    header = tk.Frame(root, bg="#111111")
    header.pack(fill="x")

    tk.Label(
        header,
        text="QUADRUPED // CONTROL // DIAGNOSTICS",
        font=FONT_BIG,
        fg="#ffffff",
        bg="#111111",
        pady=4
    ).pack(anchor="w", padx=12)

    tk.Label(
        header,
        text="balance • posture FSM • foot contact • reflex layer • diagnostics",
        font=FONT_MONO,
        fg="#9a9a9a",
        bg="#111111"
    ).pack(anchor="w", padx=14, pady=(0, 6))

    # ---------------- Layout ----------------
    main = tk.Frame(root, bg="#0b0b0b")
    main.pack(fill="both", expand=True, padx=8, pady=8)

    left  = tk.Frame(main, bg="#151515", width=300)
    right = tk.Frame(main, bg="#151515")

    left.pack(side="left", fill="y", padx=(0, 8))
    right.pack(side="right", fill="both", expand=True)

    # ==================================================
    # LEFT PANEL — CONTROLS + RAW STATE
    # ==================================================
    tk.Label(left, text="CONTROLS", font=FONT_HDR, fg="#ffffff", bg="#151515").pack(anchor="w", padx=10, pady=(8, 4))
    # ---------------- POSTURE MODES ----------------
    tk.Label(left, text="POSTURE", font=FONT_HDR, fg="#ffffff", bg="#151515").pack(anchor="w", padx=10, pady=(12, 4))

    def set_posture(m):
        global posture_mode, last_posture_change_time
        global posture_roll_bias, posture_pitch_bias
        global stand_recenter_until

        posture_mode = m
        last_posture_change_time = time.time()

        posture_roll_bias  = 0.0
        posture_pitch_bias = 0.0

        if m == "STAND":
            stand_recenter_until = time.time() + STAND_IMPULSE_TIME


    tk.Button(left, text="STAND", command=lambda: set_posture("STAND"),
            bg="#222222", fg="#e6e6e6", relief="flat").pack(fill="x", padx=10, pady=2)

    tk.Button(left, text="LEAN LEFT", command=lambda: set_posture("LEAN_LEFT"),
            bg="#222222", fg="#e6e6e6", relief="flat").pack(fill="x", padx=10, pady=2)

    tk.Button(left, text="LEAN RIGHT", command=lambda: set_posture("LEAN_RIGHT"),
            bg="#222222", fg="#e6e6e6", relief="flat").pack(fill="x", padx=10, pady=2)

    tk.Button(left, text="LEAN FWD", command=lambda: set_posture("LEAN_FWD"),
            bg="#222222", fg="#e6e6e6", relief="flat").pack(fill="x", padx=10, pady=2)

    tk.Button(left, text="LEAN BACK", command=lambda: set_posture("LEAN_BACK"),
            bg="#222222", fg="#e6e6e6", relief="flat").pack(fill="x", padx=10, pady=2)

    tk.Button(left, text="SIT", command=lambda: set_posture("SIT"),
            bg="#333333", fg="#ffffff", relief="flat").pack(fill="x", padx=10, pady=(6,2))

    tk.Button(left, text="KNEEL", command=lambda: set_posture("KNEEL"),
            bg="#222222", fg="#e6e6e6", relief="flat").pack(fill="x", padx=10, pady=2)
    
    def start_show():
        global posture_mode, show_start_time, demo_active, show_motion
        demo_active = False
        posture_mode = "SHOW"
        show_motion = sway          # <-- SET HERE
        show_start_time = time.time()

    tk.Button(
        left,
        text="✨ SHOW MODE",
        command=start_show,
        bg="#ff2ec4",
        fg="#000000",
        relief="flat",
        font=FONT_HDR
    ).pack(fill="x", padx=10, pady=(8,4))

    tk.Label(left, text="SHOW MOTIONS", font=FONT_HDR,
         fg="#ffffff", bg="#151515").pack(anchor="w", padx=10, pady=(6,2))

    tk.Button(left, text="SWAY",
            command=set_sway,
            bg="#222222", fg="#e6e6e6",
            relief="flat", font=FONT_MAIN).pack(fill="x", padx=10, pady=2)

    tk.Button(left, text="SQUAT",
            command=set_squat,
            bg="#222222", fg="#e6e6e6",
            relief="flat", font=FONT_MAIN).pack(fill="x", padx=10, pady=2)

    tk.Button(left, text="BOW",
            command=set_bow,
            bg="#222222", fg="#e6e6e6",
            relief="flat", font=FONT_MAIN).pack(fill="x", padx=10, pady=2)
    
    tk.Button(
        left, text="STOMP",
        command=set_stomp,
        bg="#440000", fg="#ffdddd",
        relief="flat", font=FONT_HDR
    ).pack(fill="x", padx=10, pady=4)

    # ---------------- LOG CONTROL ----------------
    tk.Label(left, text="LOGGING", font=FONT_HDR,
            fg="#ffffff", bg="#151515").pack(anchor="w", padx=10, pady=(10,2))

    def start_log():
        global log_active, log_start_time
        log_active = True
        log_start_time = time.time()
        log_f.write("# EVENT_START %.3f\n" % time.time())
        log_f.flush()                     
        print("[LOG] started")


    def stop_log():
        global log_active
        log_active = False
        log_f.write("# EVENT_END %.3f\n" % time.time())
        log_f.flush()                 
        print("[LOG] stopped")


    tk.Button(left, text="▶ START LOG",
            command=start_log,
            bg="#004400", fg="#ccffcc",
            relief="flat", font=FONT_MAIN).pack(fill="x", padx=10, pady=2)

    tk.Button(left, text="■ STOP LOG",
            command=stop_log,
            bg="#440000", fg="#ffcccc",
            relief="flat", font=FONT_MAIN).pack(fill="x", padx=10, pady=(0,4))




    selected_leg_var = tk.StringVar(value="None")

    def start_demo():
        global demo_active, demo_index, demo_step_start
        demo_active = True
        demo_index = 0
        demo_step_start = 0.0

    tk.Button(
        left,
        text="▶ DEMO MODE",
        command=start_demo,
        bg="#ff2ec4",
        fg="#000000",
        relief="flat",
        font=FONT_HDR
    ).pack(fill="x", padx=10, pady=(10, 2))


    def select_leg(leg):
        global gui_selected_leg
        accepted = request_step(leg)
        if accepted:
            gui_selected_leg = leg
            selected_leg_var.set(leg)
        else:
            selected_leg_var.set("BUSY")

    for leg in ["FRF", "FLF", "RRF", "RLF"]:
        tk.Button(
            left,
            text=f"SELECT {leg}",
            command=lambda l=leg: select_leg(l),
            bg="#222222",
            fg="#e6e6e6",
            relief="flat",
            font=FONT_MAIN
        ).pack(fill="x", padx=10, pady=2)

    tk.Label(left, text="ACTIVE LEG:", font=FONT_MONO, fg="#9a9a9a", bg="#151515").pack(anchor="w", padx=10, pady=(6, 0))
    tk.Label(left, textvariable=selected_leg_var, font=FONT_MONO, fg="#ffffff", bg="#151515").pack(anchor="w", padx=10)

    def lift():
        pass
        
    def lower():
        pass

    tk.Button(left, text="LIFT", command=lift, bg="#333333", fg="#ffffff", relief="flat", font=FONT_HDR).pack(fill="x", padx=10, pady=(12, 4))
    tk.Button(left, text="LOWER", command=lower, bg="#222222", fg="#e6e6e6", relief="flat").pack(fill="x", padx=10, pady=4)
    
    def start_gait():
        global gait_enabled, posture_mode
        posture_mode = "STAND" 
        motion_armed = True
        gait_enabled = True

    def stop_gait():
        global gait_enabled, step_active, swing_leg, fsm_state
        global disable_roll_pd, stop_hold_until, motion_armed

        gait_enabled = False
        step_active = False
        swing_leg = None

        # Clear all intent
        posture_roll_target = 0.0
        posture_pitch_target = 0.0

        posture_roll_bias = 0.0
        posture_pitch_bias = 0.0

        for k in motion_offsets:
            motion_offsets[k] = 0.0

        for k in roll_offsets:
            roll_offsets[k] = 0.0

        # 🔒 HARD FREEZE balance
        disable_roll_pd = True
        motion_armed = False

        stop_hold_until = time.time() + STOP_HOLD_TIME
        fsm_state = "STOP_HOLD"



    tk.Button(
        left,
        text="START CRAWL GAIT",
        command=start_gait,
        bg="#1e1e1e",
        fg="#ffffff",
        relief="flat",
        font=FONT_HDR
    ).pack(fill="x", padx=10, pady=(10, 4))

    tk.Button(
        left,
        text="STOP GAIT",
        command=stop_gait,
        bg="#141414",
        fg="#cccccc",
        relief="flat",
        font=FONT_MAIN
    ).pack(fill="x", padx=10, pady=4)

    # -------- Raw debug dump --------
    raw_state = tk.StringVar()
    tk.Label(
        left,
        textvariable=raw_state,
        font=FONT_MONO,
        fg="#9a9a9a",
        bg="#151515",
        justify="left"
    ).pack(anchor="w", padx=10, pady=6)

    # ==================================================
    # RIGHT PANEL — DIAGNOSTICS
    # ==================================================
    tk.Label(right, text="DIAGNOSTICS", font=FONT_HDR, fg="#ffffff", bg="#151515").pack(anchor="w", padx=10, pady=(8, 4))

    imu_var   = tk.StringVar()
    fsm_var   = tk.StringVar()
    brace_var = tk.StringVar()
    step_phase_var = tk.StringVar()
    step_active_var = tk.StringVar()
    step_ok_var = tk.StringVar()

    tk.Label(right, textvariable=imu_var, font=FONT_MONO, fg="#e6e6e6", bg="#151515", justify="left").pack(anchor="w", padx=10)
    tk.Label(right, textvariable=fsm_var, font=FONT_HDR, fg="#ffffff", bg="#151515").pack(anchor="w", padx=10, pady=4)
    tk.Label(right, textvariable=brace_var, font=FONT_MONO, fg="#9a9a9a", bg="#151515").pack(anchor="w", padx=10)
    tk.Label(
    right,
    textvariable=step_phase_var,
    font=FONT_HDR,
    fg="#ffffff",
    bg="#151515").pack(anchor="w", padx=10, pady=(6, 0))

    tk.Label(
        right,
        textvariable=step_active_var,
        font=FONT_MONO,
        fg="#9a9a9a",
        bg="#151515").pack(anchor="w", padx=10)

    tk.Label(
        right,
        textvariable=step_ok_var,
        font=FONT_MONO,
        fg="#9a9a9a",
        bg="#151515").pack(anchor="w", padx=10, pady=(0, 6))

    # ---------------- Foot Contact ----------------
    tk.Label(right, text="FOOT CONTACT", font=FONT_HDR, fg="#ffffff", bg="#151515").pack(anchor="w", padx=10, pady=(10, 4))

    contact_frame = tk.Frame(right, bg="#151515")
    contact_frame.pack(fill="x", padx=10)

    contact_bars = {}
    last_contact = {k: 1.0 for k in ["FRF","FLF","RRF","RLF"]}

    # ---------------- Slip / Traction ----------------
    slip_bars = {}

    for leg in ["FRF", "FLF", "RRF", "RLF"]:
        row = tk.Frame(contact_frame, bg="#151515")
        row.pack(fill="x", pady=2)

        tk.Label(row, text=leg, width=4, font=FONT_MONO, fg="#ffffff", bg="#151515").pack(side="left")

        bar_bg = tk.Frame(row, bg="#222222", height=12)
        bar_bg.pack(side="left", fill="x", expand=True, padx=6)

        bar_fg = tk.Frame(bar_bg, bg=CYBER_PINK, width=0)
        bar_fg.pack(side="left", fill="y")

        val_lbl = tk.Label(row, text="0.00", width=5, font=FONT_MONO, fg="#9a9a9a", bg="#151515")
        val_lbl.pack(side="right")

        contact_bars[leg] = (bar_fg, val_lbl)

    # ---------------- Slip / Traction ----------------
    tk.Label(
        right,
        text="SLIP / TRACTION",
        font=FONT_HDR,
        fg="#ffffff",
        bg="#151515"
    ).pack(anchor="w", padx=10, pady=(14, 4))

    slip_frame = tk.Frame(right, bg="#151515")
    slip_frame.pack(fill="x", padx=10)

    for leg in ["FRF", "FLF", "RRF", "RLF"]:
        row = tk.Frame(slip_frame, bg="#151515")
        row.pack(fill="x", pady=2)

        tk.Label(
            row,
            text=leg,
            width=4,
            font=FONT_MONO,
            fg="#ffffff",
            bg="#151515"
        ).pack(side="left")

        bar_bg = tk.Frame(row, bg="#222222", height=12)
        bar_bg.pack(side="left", fill="x", expand=True, padx=6)

        bar_fg = tk.Frame(bar_bg, bg=CYBER_PINK, width=0)
        bar_fg.pack(side="left", fill="y")

        val_lbl = tk.Label(
            row,
            text="0.00",
            width=5,
            font=FONT_MONO,
            fg="#9a9a9a",
            bg="#151515"
        )
        val_lbl.pack(side="right")

        slip_bars[leg] = (bar_fg, val_lbl)

    # ---------------- Stability ----------------
    tk.Label(
        right,
        text="STABILITY",
        font=FONT_HDR,
        fg=CYBER_TEXT,
        bg=CYBER_PANEL
    ).pack(anchor="w", padx=10, pady=(14, 4))

    stab_bg = tk.Frame(right, bg=CYBER_BAR_BG, height=14)
    stab_bg.pack(fill="x", padx=10)

    stab_fg = tk.Frame(stab_bg, bg=CYBER_PINK, width=0)
    stab_fg.pack(side="left", fill="y")

    stab_val = tk.Label(
        right,
        text="1.00",
        font=FONT_MONO,
        fg=CYBER_MUTED,
        bg=CYBER_PANEL
    )
    stab_val.pack(anchor="e", padx=10)

    # ---------------- Airborne Confidence ----------------
    tk.Label(
        right,
        text="AIRBORNE CONFIDENCE",
        font=FONT_HDR,
        fg=CYBER_TEXT,
        bg=CYBER_PANEL
    ).pack(anchor="w", padx=10, pady=(14, 4))

    air_bg = tk.Frame(right, bg=CYBER_BAR_BG, height=14)
    air_bg.pack(fill="x", padx=10)

    air_fg = tk.Frame(air_bg, bg="#ff4444", width=0)
    air_fg.pack(side="left", fill="y")

    air_val = tk.Label(
        right,
        text="0.00",
        font=FONT_MONO,
        fg=CYBER_MUTED,
        bg=CYBER_PANEL
    )
    air_val.pack(anchor="e", padx=10)


    # ---------------- Stop Recovery ----------------
    tk.Label(
        right,
        text="STOP RECOVERY",
        font=FONT_HDR,
        fg=CYBER_TEXT,
        bg=CYBER_PANEL
    ).pack(anchor="w", padx=10, pady=(14, 4))

    stop_bg = tk.Frame(right, bg=CYBER_BAR_BG, height=14)
    stop_bg.pack(fill="x", padx=10)

    stop_fg = tk.Frame(stop_bg, bg=CYBER_PINK_DIM, width=0)
    stop_fg.pack(side="left", fill="y")

    stop_val = tk.Label(
        right,
        text="0.00",
        font=FONT_MONO,
        fg=CYBER_MUTED,
        bg=CYBER_PANEL
    )
    stop_val.pack(anchor="e", padx=10)

    # ---------------- Footer / Timeline ----------------
    status_var = tk.StringVar()
    tk.Label(
        root,
        textvariable=status_var,
        font=FONT_MONO,
        fg="#e6e6e6",
        bg="#0b0b0b",
        anchor="w"
    ).pack(fill="x", padx=10, pady=4)

    # ---------------- Update Loop ----------------
    def update():
                # ---- Airborne bar update ----
        air = monitor_state.get("airborne_conf", 0.0)
        air_fg.config(width=int(240 * air))
        air_val.config(text=f"{air:.2f}")

        imu_var.set(
            f"ROLL  {monitor_state['roll']:+6.2f} deg\n"
            f"PITCH {monitor_state['pitch']:+6.2f} deg\n"
            f"STAB  {monitor_state['stability']:.2f}\n"
            f"AX {monitor_state['ax']:6.0f}  AY {monitor_state['ay']:6.0f}  AZ {monitor_state['az']:6.0f}"
        )

        fsm_var.set(f"FSM STATE : {fsm_state}")
        brace_var.set("BRACE : ACTIVE" if monitor_state["brace"] else "BRACE : off")
        step_phase_var.set(f"STEP PHASE : {monitor_state['step_phase']}")
        step_active_var.set(f"STEP ACTIVE : {monitor_state['step_active']}")
        step_ok_var.set(
            "LAST STEP : OK"
            if monitor_state["last_step_ok"]
            else "LAST STEP : WARN"
        )

        raw_state.set(
            f"fsm={fsm_state}\n"
            f"swing_leg={swing_leg}\n"
            f"roll_effort={monitor_state['roll_effort']:.3f}\n"
            f"offset_change={monitor_state['offset_change']:.3f}"
        )

        for leg, (bar, lbl) in contact_bars.items():
            c = monitor_state["foot_contact"].get(leg, 0.0)
            bar.config(width=int(240 * c))
            lbl.config(text=f"{c:.2f}")

            # flash on touchdown
            if c > 0.8 and last_contact[leg] < 0.3:
                bar.config(bg=CYBER_PINK_DIM)
            else:
                bar.config(bg="#e6e6e6")

            last_contact[leg] = c

        # ---- Slip / Traction bars ----
        for leg, (bar, lbl) in slip_bars.items():
            s = monitor_state["slip"].get(leg, 0.0)
            bar.config(width=int(240 * s))
            lbl.config(text=f"{s:.2f}")
        
        status_var.set(
            f"{fsm_state} | swing={swing_leg} | "
            f"FRF={monitor_state['foot_contact'].get('FRF',0):.2f} "
            f"FLF={monitor_state['foot_contact'].get('FLF',0):.2f} "
            f"RRF={monitor_state['foot_contact'].get('RRF',0):.2f} "
            f"RLF={monitor_state['foot_contact'].get('RLF',0):.2f}"
        )

        # ---- Stability bar update ----
        stab = monitor_state.get("stability", 0.0)
        stab_fg.config(width=int(240 * stab))
        stab_val.config(text=f"{stab:.2f}")

        # ---- Stop recovery bar update ----
        hold = monitor_state.get("stop_hold", 0.0)
        stop_fg.config(width=int(240 * hold))
        stop_val.config(text=f"{hold:.2f}")

        root.after(100, update)

    update()
    root.mainloop()


threading.Thread(
    target=start_unified_gui,
    daemon=True
).start()

gui_enabled = True

# store last commanded servo angles
# DEBUG: last servo angles for contact testing
last_servo_angles = {}
for ch in FEET.values():
    last_servo_angles[ch] = 0.0

# ==================================================
# JOINT ↔ SERVO SEMANTIC MAP (FOR LOGGING)
# ==================================================
JOINT_MAP = {
    "SHOULDER_FR": SHOULDERS["FR"],
    "SHOULDER_FL": SHOULDERS["FL"],
    "SHOULDER_RR": SHOULDERS["RR"],
    "SHOULDER_RL": SHOULDERS["RL"],
    "FOOT_FRF": FEET["FRF"],
    "FOOT_FLF": FEET["FLF"],
    "FOOT_RRF": FEET["RRF"],
    "FOOT_RLF": FEET["RLF"],
    "MID_FRM": 8,
    "MID_FLM": 9,
    "MID_RRM": 2,
    "MID_RLM": 3,
}

# ==================================================
# FOOT CONTACT ESTIMATION (READ-ONLY)
# ==================================================
# Per-leg motion response (absolute knee delta per cycle)
leg_motion_response = {k: 0.0 for k in FEET}

# Last knee offsets for response calculation
last_leg_offsets = {k: 0.0 for k in FEET}

# Per-leg response thresholds
LEG_RESPONSE_LOW  = 0.06    # deg → constrained (grounded)
LEG_RESPONSE_HIGH = 0.18    # deg → free (airborne)

# Per-foot contact confidence: 0.0 (air) → 1.0 (ground)
foot_contact = {k: 1.0 for k in FEET}

# Per-foot slip confidence: 0.0 (good traction) → 1.0 (slipping)
slip_score = {k: 0.0 for k in FEET}

# Previous motion offsets (for delta detection)
last_motion_offsets = {k: 0.0 for k in FEET}

# Tunable thresholds (SAFE DEFAULTS)
CONTACT_EFFORT_HIGH = 0.12     # roll_effort above this → resistance
CONTACT_OFFSET_LOW  = 0.06     # offset_change below this → constrained
CONTACT_MOTION_MIN  = 1.0      # deg — ignore noise-level motion

CONTACT_RISE = 0.10          # confidence rise rate
CONTACT_FALL = 0.30            # confidence fall rate

# ==================================================
# SLIP / TRACTION ESTIMATION (READ-ONLY)
# ==================================================
SLIP_RISE = 0.12        # how fast slip confidence rises
SLIP_FALL = 0.04        # how fast it recovers

SLIP_CMD_MIN   = 0.8    # deg — commanded motion must exceed this
SLIP_RESP_MAX  = 0.08   # deg — actual response below this = stuck
SLIP_EFFORT_MIN = 0.10  # roll effort indicating resistance


for leg, ch in SHOULDERS.items():
    servo_angles[ch] = SHOULDER_STAND[leg]

for leg, ch in FEET.items():
    servo_angles[ch] = FOOT_STAND[leg]

# ==================================================
# INITIALIZE SERVO DELTA TRACKING
# ==================================================
for ch, ang in servo_angles.items():
    last_servo_angles[ch] = ang

# ==================================================
# MAIN CONTROL LOOP — LAYER 1 REFLEX
# ==================================================
while True:
    try:
        # Read IMU
        ax = safe_read_word(MPU_ADDR, ACCEL_XOUT_H) - ax_o
        ay = safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 2) - ay_o
        az = safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 4) - az_o
        gx = safe_read_word(MPU_ADDR, GYRO_XOUT_H) - gx_o
        gy = safe_read_word(MPU_ADDR, GYRO_XOUT_H + 2) - gy_o

        # ---- RAW acceleration (DO NOT bias-correct) ----
        ax_raw = safe_read_word(MPU_ADDR, ACCEL_XOUT_H)
        ay_raw = safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 2)
        az_raw = safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 4)
        
        # ---------- BRACE DETECTION ----------
        dax = ax - last_ax
        day = ay - last_ay
        daz = az - last_az
        last_ax, last_ay, last_az = ax, ay, az

        delta_a = math.sqrt(dax*dax + day*day + daz*daz)

        now = time.time()
        # ---------- RELATIVE ORIENTATION (FOR LOGGING & CONTROL) ----------
        rel_roll  = roll - STAND_ROLL_REF if STAND_ROLL_REF is not None else 0.0
        rel_pitch = pitch - STAND_PITCH_REF if STAND_PITCH_REF is not None else 0.0

        # ==================================================
        # LOG DATA (NON-BLOCKING)
        # ==================================================
        if LOG_ENABLE and log_active and (now - last_log_time) >= LOG_DT:
            for joint, ch in JOINT_MAP.items():

                servo_ang = servo_angles.get(ch, 0.0)
                servo_delta = servo_ang - last_servo_angles.get(ch, servo_ang)

                # Infer leg name if applicable
                leg = None
                if "FRF" in joint: leg = "FRF"
                elif "FLF" in joint: leg = "FLF"
                elif "RRF" in joint: leg = "RRF"
                elif "RLF" in joint: leg = "RLF"

                motion_cmd = motion_offsets.get(leg, 0.0) if leg else 0.0
                leg_resp   = leg_motion_response.get(leg, 0.0) if leg else 0.0
                contact    = foot_contact.get(leg, 0.0) if leg else 0.0

                log_f.write(
                    f"{now:.3f},"
                    f"{joint},{servo_ang:.2f},{servo_delta:.2f},"
                    f"{motion_cmd:.2f},{leg_resp:.3f},{contact:.2f},"
                    f"{rel_roll:.2f},{rel_pitch:.2f},"
                    f"{posture_roll_bias:.2f},{posture_pitch_bias:.2f},"
                    f"{roll_effort:.3f},{offset_change:.3f},"
                    f"{posture_mode},{fsm_state},{stomp_phase}\n"
                )

                last_servo_angles[ch] = servo_ang

            last_log_time = now

    
        if (delta_a > BRACE_ACCEL_THRESHOLD) and not brace_active:
            brace_active = True
            brace_until = now + BRACE_DURATION

            mag = max(delta_a, 1.0)
            brace_pitch_bias = -BRACE_GAIN_PITCH * dax / mag * BRACE_MAX_BIAS
            brace_roll_bias  = -BRACE_GAIN_ROLL  * day / mag * BRACE_MAX_BIAS
            if abs(dax) < 0.7 * abs(day):
                brace_pitch_bias = 0.0

            # ---------- BRACE DECAY ----------
        if brace_active:
            if now > brace_until:
                brace_active = False
                brace_roll_bias = 0.0
                brace_pitch_bias = 0.0
            else:
                # gentle decay inside brace window
                brace_roll_bias *= 0.92
                brace_pitch_bias *= 0.92

        # Noise suppression
        if abs(ax) < ACCEL_DEADBAND: ax = 0
        if abs(ay) < ACCEL_DEADBAND: ay = 0
        if abs(gx) < GYRO_DEADBAND: gx = 0
        if abs(gy) < GYRO_DEADBAND: gy = 0

        # IMU tilt estimation
        accel_roll  = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

        # Complementary filter (gyro + accel)
        roll  = ALPHA * (roll  + gx / 131.0 * DT) + (1 - ALPHA) * accel_roll
        pitch = ALPHA * (pitch + gy / 131.0 * DT) + (1 - ALPHA) * accel_pitch

        roll  = max(-MAX_ROLL,  min(MAX_ROLL,  roll))
        pitch = max(-MAX_PITCH, min(MAX_PITCH, pitch))
        
        # ---------- Capture standing reference once ----------
        if not stand_ref_locked:
            if (
                abs(roll) < 1.0 and
                abs(pitch) < 1.0 and
                fsm_state == "IDLE" and
                not step_active
            ):
                STAND_ROLL_REF = roll
                STAND_PITCH_REF = pitch
                stand_ref_locked = True
                print("[STAND REF] locked")


        rel_roll = roll - STAND_ROLL_REF
        rel_pitch = pitch - STAND_PITCH_REF

        monitor_state["roll"] = rel_roll
        monitor_state["pitch"] = rel_pitch



        # ---------- Support stability metric ----------
        stability = 1.0 - (
            (abs(rel_roll) / ROLL_LIMIT + abs(rel_pitch) / PITCH_LIMIT) * 0.5
        )
        stability = max(0.0, min(1.0, stability))
        monitor_state["stability"] = stability

        # ==================================================
        # APPLY POSTURE MODE (HIGH-LEVEL INTENT)
        # ==================================================
        if fsm_state == "IDLE" and not step_active:
            apply_posture_mode()
            demo_tick()

        # ==================================================
        # STOMP FSM (SHOW MODE, SAFE)
        # ==================================================
        if stomp_active and fsm_state == "IDLE" and not step_active:

            # ---- UNLOAD ----
            if stomp_phase == "UNLOAD":
                posture_roll_target  = UNLOAD_POSTURE[stomp_leg]["roll"]
                posture_pitch_target = UNLOAD_POSTURE[stomp_leg]["pitch"]

                if abs(posture_roll_bias - posture_roll_target) < 1.0:
                    stomp_phase = "LIFT"
                    stomp_t0 = time.time()

            # ---- LIFT ----
            elif stomp_phase == "LIFT":
                body_lift = 28.0   # MUCH higher than normal step
                servo_lift = FOOT_SIGN[stomp_leg] * body_lift

                motion_offsets[stomp_leg] = ramp(
                    motion_offsets[stomp_leg],
                    servo_lift,
                    1.8   # faster lift
                )

                if abs(motion_offsets[stomp_leg] - servo_lift) < 0.5:
                    stomp_phase = "SLAM"
                    stomp_t0 = time.time()

            # ---- SLAM ----
            elif stomp_phase == "SLAM":
                # HARD recenter = impact
                motion_offsets[stomp_leg] = ramp(
                    motion_offsets[stomp_leg],
                    0.0,
                    4.5   # VERY FAST downward
                )

                if abs(motion_offsets[stomp_leg]) < 0.5:
                    stomp_phase = "IDLE"
                    stomp_active = False
                    log_f.write("# EVENT_END %.3f\n" % time.time())
                    log_f.flush()

                    log_active = False
                    print("[LOG] stomp capture complete")


                    posture_roll_target = 0.0
                    posture_pitch_target = 0.0


        # ==================================================
        # APPLY POSTURE BIAS (Layer-2)
        # ==================================================


        # Ramp posture bias toward target
        posture_roll_bias  = ramp(posture_roll_bias,  posture_roll_target,  POSTURE_STEP)
        posture_pitch_bias = ramp(posture_pitch_bias, posture_pitch_target, POSTURE_STEP)

        # Effective tilt relative to desired posture
        effective_roll  = rel_roll  - posture_roll_bias
        effective_pitch = rel_pitch - posture_pitch_bias

        # Monitor
        monitor_state["posture_roll_bias"]  = posture_roll_bias
        monitor_state["posture_pitch_bias"] = posture_pitch_bias
        monitor_state["brace"] = brace_active
        monitor_state["brace_roll_bias"] = brace_roll_bias
        monitor_state["brace_pitch_bias"] = brace_pitch_bias
        monitor_state["fsm_state"] = fsm_state
        monitor_state["step_active"] = step_active
        monitor_state["step_phase"] = fsm_state
        monitor_state["ax"] = ax_raw
        monitor_state["ay"] = ay_raw
        monitor_state["az"] = az_raw
        monitor_state["airborne"] = robot_airborne
        monitor_state["airborne_conf"] = airborne_conf


        # ---- derived diagnostics ----
        avg_shoulder = sum(abs(v) for v in roll_offsets.values()) / len(roll_offsets)
        avg_foot     = sum(abs(v) for v in pitch_offsets.values()) / len(pitch_offsets)
        avg_mid      = sum(abs(v) for v in mid_offsets.values()) / max(1, len(mid_offsets))

        avg_contact  = sum(foot_contact.values()) / len(foot_contact)
        avg_slip     = sum(slip_score.values()) / len(slip_score)

        # avg pitch command proxy
        front_pitch = sum(pitch_offsets[l] for l in FEET if l.startswith("F")) / 2
        rear_pitch  = sum(pitch_offsets[l] for l in FEET if l.startswith("R")) / 2

        # ---------- TELEMETRY FEED ----------
        # ---------- TELEMETRY SEND (NON-BLOCKING) ----------
        if not telemetry_q.full():
            telemetry_q.put_nowait({
                "t": time.time(),

                # orientation
                "roll": monitor_state["roll"],
                "pitch": monitor_state["pitch"],
                "effective_roll": effective_roll,
                "effective_pitch": effective_pitch,

                # posture
                "posture_roll": posture_roll_bias,
                "posture_pitch": posture_pitch_bias,

                # control effort
                "roll_effort": roll_effort,
                "offset_change": offset_change,
                "avg_shoulder": avg_shoulder,
                "avg_foot": avg_foot,
                "avg_mid": avg_mid,

                # environment
                "stability": monitor_state["stability"],
                "foot_contact": avg_contact,
                "slip": avg_slip,

                # state
                "brace": 1.0 if brace_active else 0.0,
                "fsm": FSM_ID.get(fsm_state, -1),
                "posture": POSTURE_ID.get(posture_mode, -1),
                "step": 1.0 if step_active else 0.0,
            })

        # ==================================================
        # LAYER-2 FSM: UNLOAD → LIFT
        # ==================================================
        
        # ==================================================
        # APPLY GUI INTENT (SAFE, NON-BLOCKING)
        # ==================================================
           
        if fsm_state == "STOP_HOLD":
            if not posture_locked:
                posture_roll_target = 0.0
                posture_pitch_target = 0.0

            for k in motion_offsets:
                motion_offsets[k] = 0.0

            if time.time() > stop_hold_until:
                disable_roll_pd = False
                motion_armed = True
                fsm_state = "IDLE"

        if fsm_state == "IDLE" and motion_armed:
            if swing_leg is not None:
                step_active = True
                posture_locked = True   # 🔒 LOCK posture
                posture_roll_target  = UNLOAD_POSTURE[swing_leg]["roll"]
                posture_pitch_target = UNLOAD_POSTURE[swing_leg]["pitch"]
                unload_start = time.time()
                disable_roll_pd = True
                fsm_state = "UNLOADING"


        elif fsm_state == "UNLOADING":
            # Authoritative geometric unload
            for shoulder in SHOULDERS:
                body_target = UNLOAD_SHOULDER_OFFSETS[swing_leg].get(shoulder, 0.0)
                servo_target = SHOULDER_SIGN[shoulder] * body_target

                motion_offsets[shoulder] = ramp(
                    motion_offsets[shoulder],
                    servo_target,
                    0.8
                )

            # TIME-GATED EXIT (AUTHORITATIVE)
            if (time.time() - unload_start) >= 0.6:
                fsm_state = "LIFTING"
            
        elif fsm_state == "LIFTING":
            body_lift = LIFT_HEIGHT
            servo_lift = FOOT_SIGN[swing_leg] * body_lift

            motion_offsets[swing_leg] = ramp(
                motion_offsets[swing_leg],
                servo_lift,
                LIFT_STEP
            )

            if abs(motion_offsets[swing_leg] - servo_lift) < 0.3:
                fsm_state = "DONE"
                
        elif fsm_state == "DONE":
            fsm_state = "RECENTERING"

                
        elif fsm_state == "RECENTERING":
            # Gradually remove posture bias
            posture_roll_bias = ramp(posture_roll_bias, 0.0, POSTURE_STEP)
            posture_pitch_bias = ramp(posture_pitch_bias, 0.0, POSTURE_STEP)

            # Gradually remove all motion offsets (shoulders + feet)
            for k in motion_offsets:
                motion_offsets[k] = ramp(
                    motion_offsets[k],
                    0.0,
                    RECENTER_STEP
                )
            # Check if everything is near neutral
            all_centered = (
                abs(posture_roll_bias) < RECENTER_THRESH and
                abs(posture_pitch_bias) < RECENTER_THRESH and
                all(abs(motion_offsets[k]) < RECENTER_THRESH for k in motion_offsets)
            )
            if all_centered:
                # Reset roll offsets so PD resumes from neutral
                for k in roll_offsets:
                    roll_offsets[k] = 0.0

                # Re-enable roll PD cleanly
                disable_roll_pd = False

                swing_leg = None
                gui_selected_leg = None
                fsm_state = "IDLE"    
                    
        # ==================================================
        # STEP COMPLETION — CONTACT-GATED (FINAL)
        # ==================================================
        if (
            step_active
            and swing_leg is not None
            and foot_contact[swing_leg] > 0.75
            and fsm_state in ("DONE", "RECENTERING")
        ):
            completed_leg = swing_leg
            # Clear posture intent
            posture_roll_target  = 0.0
            posture_pitch_target = 0.0

            # Clear motion offsets
            for k in motion_offsets:
                motion_offsets[k] = 0.0

            # Reset roll offsets cleanly
            for k in roll_offsets:
                roll_offsets[k] = 0.0

            # Restore balance controller
            disable_roll_pd = False

            # Clear step state
            swing_leg = None
            gui_selected_leg = None
            step_active = False
            posture_locked = False   # 🔓 RELEASE posture control

            # FSM returns to neutral
            monitor_state["last_step_ok"] = (foot_contact[completed_leg] > 0.75)
            fsm_state = "IDLE"
            
        # ==================================================
        # STEP INVARIANT CHECK (DEBUG / SAFE)
        # ==================================================
        if fsm_state == "IDLE" and not step_active:
            # HARD SAFETY: roll PD must be enabled in IDLE
            if disable_roll_pd:
                disable_roll_pd = False

            assert swing_leg is None
            assert disable_roll_pd is False


            assert abs(posture_roll_bias) <= POSTURE_MAX_ROLL
            assert abs(posture_pitch_bias) <= POSTURE_MAX_PITCH

            assert posture_mode in (
                "STAND",
                "LEAN_LEFT",
                "LEAN_RIGHT",
                "LEAN_FWD",
                "LEAN_BACK",
                "SIT",
                "KNEEL",
                "SHOW",
            )


            # Posture mode must be valid
            assert posture_mode in (
                "STAND",
                "LEAN_LEFT",
                "LEAN_RIGHT",
                "LEAN_FWD",
                "LEAN_BACK",
                "SIT",
                "KNEEL",
                "SHOW",
            )


        # ==================================================
        # AIRBORNE SAFETY OVERRIDE
        # ==================================================
        if robot_airborne and not posture_locked:
            disable_roll_pd = False
            posture_roll_target = 0.0
            posture_pitch_target = 0.0

            # Cancel steps safely
            step_active = False
            swing_leg = None
            fsm_state = "IDLE"

        # ==================================================
        # AIRBORNE RECOVERY (RE-ENABLE BALANCE)
        # ==================================================
        '''if not robot_airborne and disable_roll_pd:
            if fsm_state == "IDLE" and not step_active:
                disable_roll_pd = False'''


        # ---------- ROLL REFLEX ----------
        # Shoulders widen/narrow stance to resist tipping
        gain_scale = BRACE_GAIN_SCALE if brace_active else 1.0
        roll_rate = gx / 131.0      #deg/s
        if abs(roll_rate) < GYRO_RATE_DEADBAND:
            roll_rate = 0.0
        # USE effective_roll instead of raw roll
        roll_cmd = -(gain_scale * (K_ROLL * effective_roll + K_ROLL_RATE * roll_rate)) + brace_roll_bias
        
        roll_cmd = max(-ROLL_LIMIT, min(ROLL_LIMIT, roll_cmd))
        
        # ADD — measure change in roll command (effort)
        # Place right after roll_cmd is computed
        roll_effort = abs(roll_cmd - last_roll_cmd)
        last_roll_cmd = roll_cmd
        
        # ADD — publish to monitor_state for GUI
        monitor_state["roll_effort"] = roll_effort
        
        

        for leg, ch in SHOULDERS.items():
            applied_cmd = 0.0 if disable_roll_pd else roll_cmd
            delta = applied_cmd - roll_offsets[leg]
            delta = max(-ROLL_STEP, min(ROLL_STEP, delta))
            if abs(delta) < 0.05:
                delta = 0.0
            roll_offsets[leg] += delta

            roll_offsets[leg] = max(
                -SHOULDER_MAX_OFFSET[leg],
                min(SHOULDER_MAX_OFFSET[leg], roll_offsets[leg])
            )   

            # Final shoulder angle = stand + balance + motion
            angle = (
                SHOULDER_STAND[leg]
                + roll_offsets[leg]         # Layer-1 balance
                + motion_offsets[leg]       # Layer-2 motion
            )

            set_servo_angle(ch, angle)
            servo_angles[ch] = angle
            
        # ADD — compute average shoulder offset change (how much roll offsets moved this loop)
        # Place right after the shoulder loop ends
        offset_sum = 0.0
        for leg in SHOULDERS:
            offset_sum += abs(roll_offsets[leg] - last_roll_offsets[leg])
            last_roll_offsets[leg] = roll_offsets[leg]
        offset_change = offset_sum / max(1, len(SHOULDERS))    

        # ADD — publish to monitor_state for GUI
        monitor_state["offset_change"] = offset_change
       

        # ---------- PITCH REFLEX ----------
        # Knees flex/extend to shift vertical load
        # ---- STOMP OVERRIDE (HARD) ----
        if posture_mode == "SHOW" and show_motion == stomp:
            stomp_phase = (time.time() - show_start_time) % 1.6
            if leg == "FRF" and stomp_phase < 1.1:
                pitch_offsets[leg] = 0.0
                continue

        for leg, ch in FEET.items():
            pitch_rate = gy / 131.0     # deg/s
            if abs(pitch_rate) < GYRO_RATE_DEADBAND:
                pitch_rate = 0.0

            tgt = (
                gain_scale * (K_PITCH * effective_pitch + K_PITCH_RATE * pitch_rate)
                + brace_pitch_bias
            ) * (1 if leg.startswith("F") else -1)
            tgt = max(-PITCH_LIMIT, min(PITCH_LIMIT, tgt))

            if posture_mode == "STAND" and not step_active:
                tgt = max(-15.0, min(15.0, tgt))   # allow gentle pitch balance
            delta = tgt - pitch_offsets[leg]

            delta = max(-PITCH_STEP, min(PITCH_STEP, delta))
            if abs(delta) < 0.05:
                delta = 0.0
            pitch_offsets[leg] += delta

            motion_offsets[leg] = max(-40.0, min(40.0, motion_offsets[leg]))

            angle = (
                FOOT_STAND[leg]
                + FOOT_SIGN[leg] * pitch_offsets[leg]
                + motion_offsets[leg]
                + prop_target.get(leg, 0.0)
            )

            angle = max(FOOT_LIMITS[leg][0], min(FOOT_LIMITS[leg][1], angle))
            if posture_mode == "SHOW" and show_motion == stomp:
                angle = max(FOOT_LIMITS[leg][0]-10, min(FOOT_LIMITS[leg][1]+10, angle))

            set_servo_angle(ch, angle)
            servo_angles[ch] = angle
        
        # ---------- MID-LIMB POSTURE (kneel only) ----------
        for ch, base in MID_LIMBS.items():
            angle = base + mid_offsets[ch]
            set_servo_angle(ch, angle)
            servo_angles[ch] = angle
        

        # ==================================================
        # PER-LEG MOTION RESPONSE (POST-PITCH, ONCE PER CYCLE)
        # ==================================================
        for leg in FEET:
            leg_motion_response[leg] = abs(
                pitch_offsets[leg] - last_leg_offsets[leg]
            )
            last_leg_offsets[leg] = pitch_offsets[leg]

        # ==================================================
        # FOOT CONTACT CONFIDENCE UPDATE (READ-ONLY)
        # ==================================================
        for leg in FEET:

            # ---------- SLOW LIFT / GRAVITY UNLOAD ----------
            gravity_loss = abs(abs(az_raw) - GRAVITY_NOMINAL)

            if (
                fsm_state == "IDLE"
                and not step_active
                and gravity_loss > GRAVITY_LOSS_THRESH
                and abs(motion_offsets[leg]) < CONTACT_MOTION_MIN
                and leg_motion_response[leg] > LEG_RESPONSE_HIGH
            ):
                foot_contact[leg] = max(0.0, foot_contact[leg] - CONTACT_FALL)
                pass

            # ---------- ACTIVE STEP CASE ----------
            if fsm_state in ("LIFTING", "DONE"):
                if leg == swing_leg and abs(motion_offsets[leg]) > 0.5:
                    foot_contact[leg] = max(0.0, foot_contact[leg] - CONTACT_FALL)

            # ---------- RECOVERY ----------
            elif fsm_state in ("RECENTERING", "IDLE"):
                foot_contact[leg] = min(1.0, foot_contact[leg] + CONTACT_RISE)

        monitor_state["foot_contact"] = dict(foot_contact)
        # ==================================================
        # AIRBORNE DETECTION (FUSED, AUTHORITATIVE)
        # ==================================================

        # 1) Overall ground confidence (worst foot)
        ground_conf = min(foot_contact.values())
        avg_contact = sum(foot_contact.values()) / len(foot_contact)

        # 2) Count how many legs are "free moving"
        free_legs = sum(
            leg_motion_response[l] > LEG_RESPONSE_HIGH
            for l in FEET
        )

        # 3) Acceleration magnitude anomaly (gravity disturbance)
        acc_mag = math.sqrt(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw)
        acc_anomaly = abs(acc_mag - 16384)

        # 4) Discrete airborne evidence score
        airborne_score = 0
        slow_lift = (
            avg_contact < SLOW_LIFT_CONTACT_MAX and
            roll_effort < SLOW_LIFT_EFFORT_MAX
        )



        if ground_conf < 0.25:
            airborne_score += 2

        if free_legs >= 3:
            airborne_score += 2

        if acc_anomaly > 3500:
            airborne_score += 1
        
        # Convert score → confidence ramp
        # PRIMARY airborne evidence: contact collapse
        if avg_contact < 0.35:
            airborne_conf = min(1.0, airborne_conf + AIRBORNE_RISE)

        # SECONDARY: multiple free-moving legs
        elif free_legs >= 3:
            airborne_conf = min(1.0, airborne_conf + AIRBORNE_RISE)

        # TERTIARY: acceleration anomaly (fast lift / drop)
        elif acc_anomaly > 3500:
            airborne_conf = min(1.0, airborne_conf + AIRBORNE_RISE)

        else:
            airborne_conf = max(0.0, airborne_conf - SLOW_LIFT_FALL)

        robot_airborne = airborne_conf > 0.6
        monitor_state["leg_motion_response"] = dict(leg_motion_response)

        # ==================================================
        # SLIP / TRACTION CONFIDENCE UPDATE (READ-ONLY)
        # ==================================================
        for leg in FEET:

            # Only evaluate slip if foot is confidently on ground
            if foot_contact.get(leg, 0.0) < 0.8:
                slip_score[leg] = max(0.0, slip_score[leg] - SLIP_FALL)
                continue

            commanded = abs(motion_offsets.get(leg, 0.0))
            response  = leg_motion_response.get(leg, 0.0)

            slipping = (
                commanded > SLIP_CMD_MIN and
                response  < SLIP_RESP_MAX and
                roll_effort > SLIP_EFFORT_MIN
            )

            if slipping:
                slip_score[leg] = min(1.0, slip_score[leg] + SLIP_RISE)
            else:
                slip_score[leg] = max(0.0, slip_score[leg] - SLIP_FALL)

        monitor_state["slip"] = dict(slip_score)

        # ==================================================
        # PHASE-GATED STANCE PROPULSION (FRONT + REAR)
        # ==================================================
        for k in prop_target:
            prop_target[k] = 0.0

        # 1) FRONT LEGS: push during REAR unload/lift
        if fsm_state in ("UNLOADING", "LIFTING") and swing_leg in REAR_LEGS:
            for leg in FRONT_LEGS:
                if foot_contact.get(leg, 0.0) < PROP_CONTACT_MIN:
                    continue
                if slip_score.get(leg, 0.0) > PROP_SLIP_MAX:
                    continue

                prop_target[leg] = -PROP_PUSH_DEG


        # 2) REAR LEGS: gentle push during IDLE stance (compensate backward loss)
        if fsm_state == "IDLE":
            for leg in REAR_LEGS:
                if foot_contact.get(leg, 0.0) < PROP_CONTACT_MIN:
                    continue
                if slip_score.get(leg, 0.0) > PROP_SLIP_MAX:
                    continue

                prop_target[leg] = +0.6 * PROP_PUSH_DEG
        
        # ==================================================
        # GAIT SEQUENCER TICK
        # ==================================================
        gait_tick()              
        time.sleep(DT)

    except IOError:
        print("I2C glitch — holding posture")
        time.sleep(0.2)
        if LOG_ENABLE:
            log_f.flush()
        break
