# Copyright (c) 2025 Aryaman Gupta
# All Rights Reserved.
#
# This code is proprietary.
# Unauthorized use, modification, or distribution is prohibited.

import time, math, errno
from smbus2 import SMBus
import threading 
import tkinter as tk
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
PULSE_MIN = 80
PULSE_MAX = 561

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

gait_enabled = False
gait_index = 0

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

SERVO_DELTA_GROUND = 0.15   # deg → constrained
SERVO_DELTA_AIR    = 0.6    # deg → free

if LOG_ENABLE:
    log_f = open(LOG_FILE, "w")
    log_f.write(
        "t,roll,pitch,"
        "posture_roll,posture_pitch,"
        "roll_effort,offset_change,"
        "fsm,swing,brace\n"
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

# ===============================
# LAYER-2: POSTURE / MOTION STATE
# ===============================
# Posture biases (targets + current)
posture_roll_target  = 0.0
posture_pitch_target = 0.0
posture_roll_bias   = 0.0   # current applied bias (will ramp to target)
posture_pitch_bias  = 0.0

POSTURE_MAX_ROLL   = 8.0    # deg - max intentional lean
POSTURE_MAX_PITCH  = 6.0
POSTURE_STEP       = 0.3    # deg per loop - smoothness


# Motion offsets per-joint (added on top of balance offsets)
motion_offsets = {k: 0.0 for k in SHOULDERS}
motion_offsets.update({k: 0.0 for k in FEET})

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

# Unload posture map (lean away from leg)
UNLOAD_POSTURE = {
    "FRF": {"roll": +POSTURE_MAX_ROLL, "pitch": -POSTURE_MAX_PITCH},
    "FLF": {"roll": -POSTURE_MAX_ROLL, "pitch": -POSTURE_MAX_PITCH},
    "RRF": {"roll": +POSTURE_MAX_ROLL, "pitch": +POSTURE_MAX_PITCH},
    "RLF": {"roll": -POSTURE_MAX_ROLL, "pitch": +POSTURE_MAX_PITCH},
}

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

brace_active = False
brace_until = 0.0

brace_roll_bias = 0.0
brace_pitch_bias = 0.0

last_ax = last_ay = last_az = 0.0

# ==================================================
# MONITOR STATE (READ-ONLY)
# ==================================================
monitor_state = {
    "roll": 0.0,
    "pitch": 0.0,

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
# FINAL UNIFIED MESSY DARK CONTROL + DIAGNOSTICS GUI
# ==================================================
def start_unified_gui():
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

    selected_leg_var = tk.StringVar(value="None")

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
        global gait_enabled
        gait_enabled = True

    def stop_gait():
        global gait_enabled, step_active, swing_leg, fsm_state, disable_roll_pd

        gait_enabled = False

        # Hard cancel any active step
        step_active = False
        swing_leg = None

        # Clear posture intent
        posture_roll_target = 0.0
        posture_pitch_target = 0.0

        # Clear motion offsets
        for k in motion_offsets:
            motion_offsets[k] = 0.0

        # Restore balance controller
        disable_roll_pd = False

        # Force clean recovery
        fsm_state = "RECENTERING"

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

    for leg in ["FRF", "FLF", "RRF", "RLF"]:
        row = tk.Frame(contact_frame, bg="#151515")
        row.pack(fill="x", pady=2)

        tk.Label(row, text=leg, width=4, font=FONT_MONO, fg="#ffffff", bg="#151515").pack(side="left")

        bar_bg = tk.Frame(row, bg="#222222", height=12)
        bar_bg.pack(side="left", fill="x", expand=True, padx=6)

        bar_fg = tk.Frame(bar_bg, bg="#e6e6e6", width=0)
        bar_fg.pack(side="left", fill="y")

        val_lbl = tk.Label(row, text="0.00", width=5, font=FONT_MONO, fg="#9a9a9a", bg="#151515")
        val_lbl.pack(side="right")

        contact_bars[leg] = (bar_fg, val_lbl)

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
        imu_var.set(
            f"ROLL  {monitor_state['roll']:+6.2f} deg\n"
            f"PITCH {monitor_state['pitch']:+6.2f} deg\n"
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
                bar.config(bg="#ffffff")
            else:
                bar.config(bg="#e6e6e6")

            last_contact[leg] = c

        status_var.set(
            f"{fsm_state} | swing={swing_leg} | "
            f"FRF={monitor_state['foot_contact'].get('FRF',0):.2f} "
            f"FLF={monitor_state['foot_contact'].get('FLF',0):.2f} "
            f"RRF={monitor_state['foot_contact'].get('RRF',0):.2f} "
            f"RLF={monitor_state['foot_contact'].get('RLF',0):.2f}"
        )

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

# Previous motion offsets (for delta detection)
last_motion_offsets = {k: 0.0 for k in FEET}

# Tunable thresholds (SAFE DEFAULTS)
CONTACT_EFFORT_HIGH = 0.12     # roll_effort above this → resistance
CONTACT_OFFSET_LOW  = 0.06     # offset_change below this → constrained
CONTACT_MOTION_MIN  = 1.0      # deg — ignore noise-level motion

CONTACT_RISE = 0.25           # confidence rise rate
CONTACT_FALL = 0.06            # confidence fall rate

for leg, ch in SHOULDERS.items():
    servo_angles[ch] = SHOULDER_STAND[leg]

for leg, ch in FEET.items():
    servo_angles[ch] = FOOT_STAND[leg]


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
        
        # ---------- BRACE DETECTION ----------
        dax = ax - last_ax
        day = ay - last_ay
        daz = az - last_az
        last_ax, last_ay, last_az = ax, ay, az

        delta_a = math.sqrt(dax*dax + day*day + daz*daz)

        now = time.time()
        
        # ==================================================
        # LOG DATA (NON-BLOCKING)
        # ==================================================
        if LOG_ENABLE and (now - last_log_time) >= LOG_DT:
            log_f.write(
                f"{now:.3f},"
                f"{roll:.3f},{pitch:.3f},"
                f"{posture_roll_bias:.3f},{posture_pitch_bias:.3f},"
                f"{roll_effort:.3f},{offset_change:.3f},"
                f"{fsm_state},{swing_leg},{int(brace_active)}\n"
            )
            last_log_time = now
    
        if (delta_a > BRACE_ACCEL_THRESHOLD) and not brace_active:
            brace_active = True
            brace_until = now + BRACE_DURATION

            mag = max(delta_a, 1.0)
            brace_pitch_bias = -dax / mag * BRACE_MAX_BIAS
            brace_roll_bias  = -day / mag * BRACE_MAX_BIAS
            
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
        
        monitor_state["roll"] = roll
        monitor_state["pitch"] = pitch
        
        # ==================================================
        # APPLY POSTURE BIAS (Layer-2)
        # ==================================================

        # Ramp posture bias toward target
        posture_roll_bias  = ramp(posture_roll_bias,  posture_roll_target,  POSTURE_STEP)
        posture_pitch_bias = ramp(posture_pitch_bias, posture_pitch_target, POSTURE_STEP)

        # Effective tilt relative to desired posture
        effective_roll  = roll  - posture_roll_bias
        effective_pitch = pitch - posture_pitch_bias

        # Monitor
        monitor_state["posture_roll_bias"]  = posture_roll_bias
        monitor_state["posture_pitch_bias"] = posture_pitch_bias
        monitor_state["brace"] = brace_active
        monitor_state["brace_roll_bias"] = brace_roll_bias
        monitor_state["brace_pitch_bias"] = brace_pitch_bias
        monitor_state["fsm_state"] = fsm_state
        monitor_state["step_active"] = step_active
        monitor_state["step_phase"] = fsm_state
        monitor_state["ax"] = ax
        monitor_state["ay"] = ay
        monitor_state["az"] = az

        # ==================================================
        # LAYER-2 FSM: UNLOAD → LIFT
        # ==================================================
        
        # ==================================================
        # APPLY GUI INTENT (SAFE, NON-BLOCKING)
        # ==================================================
           
        if fsm_state == "IDLE":
            if swing_leg is not None:
                step_active = True
                posture_roll_target  = UNLOAD_POSTURE[swing_leg]["roll"]
                posture_pitch_target = UNLOAD_POSTURE[swing_leg]["pitch"]
                unload_start = time.time()

                # IMPORTANT: freeze roll PD application
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
            posture_roll_target = 0.0
            posture_pitch_target = 0.0
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
            

            # FSM returns to neutral
            monitor_state["last_step_ok"] = (foot_contact[completed_leg] > 0.75)
            fsm_state = "IDLE"
            
        # ==================================================
        # STEP INVARIANT CHECK (DEBUG / SAFE)
        # ==================================================
        if fsm_state == "IDLE" and not step_active:
            # Ownership must be fully released
            assert swing_leg is None
            assert disable_roll_pd is False

            # Posture bias must be within absolute safety bounds
            assert abs(posture_roll_bias) <= POSTURE_MAX_ROLL
            assert abs(posture_pitch_bias) <= POSTURE_MAX_PITCH

            # Motion offsets must be cleared
            assert all(abs(motion_offsets[k]) < RECENTER_THRESH for k in motion_offsets)
       
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
        for leg, ch in FEET.items():
            
            pitch_rate = gy / 131.0     # deg/s
            if abs(pitch_rate) < GYRO_RATE_DEADBAND:
                pitch_rate = 0.0

            tgt = (
                gain_scale * (K_PITCH * effective_pitch + K_PITCH_RATE * pitch_rate)
                + brace_pitch_bias
            ) * (1 if leg.startswith("F") else -1)
            tgt = max(-PITCH_LIMIT, min(PITCH_LIMIT, tgt))

            delta = tgt - pitch_offsets[leg]
            delta = max(-PITCH_STEP, min(PITCH_STEP, delta))
            if abs(delta) < 0.05:
                delta = 0.0
            pitch_offsets[leg] += delta

            angle = (
                FOOT_STAND[leg]
                + FOOT_SIGN[leg] * pitch_offsets[leg]
                + motion_offsets[leg]
            )

            angle = max(FOOT_LIMITS[leg][0], min(FOOT_LIMITS[leg][1], angle))
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
            if abs(motion_offsets[leg]) < CONTACT_MOTION_MIN:
                continue

            if fsm_state in ("LIFTING", "DONE"):
                if (
                    leg == swing_leg and
                    abs(motion_offsets[leg]) > 0.5
                ):
                    foot_contact[leg] = max(0.0, foot_contact[leg] - CONTACT_FALL)

            elif fsm_state in ("RECENTERING", "IDLE"):
                foot_contact[leg] = min(1.0, foot_contact[leg] + CONTACT_RISE)

        monitor_state["foot_contact"] = dict(foot_contact)
        monitor_state["leg_motion_response"] = dict(leg_motion_response)
        
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
