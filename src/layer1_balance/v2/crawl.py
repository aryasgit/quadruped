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
# MID-LIMB (HIP) LOCK — ONE-TIME INITIALIZATION
# ==================================================
# These joints are posture-locked and NOT controlled further

MID_LIMBS = {
    8: 104,   # FRM
    9: 165,   # FLM
    2: 102,   # RRM
    3: 154,   # RLM
}

# ==================================================
# MID-LIMB SIGN CONVENTION (FORWARD MOTION)
# +1 → angle increase moves body forward
# -1 → angle decrease moves body forward
# ==================================================
MID_SIGN = {
    8: +1,   # FRM
    9: -1,   # FLM
    2: +1,   # RRM
    3: -1,   # RLM
}


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
disable_pitch_pd_leg = None

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
# Mid-limb offsets for propulsion (independent of balance)
mid_offsets = {ch: 0.0 for ch in MID_LIMBS}

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
unload_start = 0.0

# ==================================================
# GUI COMMAND STATE (INTENT ONLY)
# ==================================================
gui_selected_leg = None      # "FRF", "FLF", "RRF", "RLF"
gui_lift_active = False     # True when Lift button pressed
gui_resume_request = False # Return to normal standing
gui_enabled = False

# ==================================================
# CRAWL GAIT STATE
# ==================================================
crawl_enabled = False
crawl_phase = "IDLE"
crawl_phase_start = 0.0
crawl_lift_start = 0.0

CRAWL_SEQUENCE = ["FRF", "RLF", "FLF", "RRF"]
crawl_index = 0

CRAWL_STEP_DELAY = 0.4    # seconds pause between steps
last_crawl_time = 0.0
CRAWL_LIFT_TIME = 0.6
CRAWL_LOWER_TIME = 0.4
crawl_step_complete = False

# ==================================================
# CRAWL PROPULSION PARAMETERS (MID-LIMBS)
# ==================================================
PROPULSION_OFFSET = 10.0     # deg — body push per step (safe, tune later)
PROPULSION_STEP   = 0.6      # deg per control cycle
propulsion_applied = False
# ==================================================
# CRAWL TIMING (AUTO LIFT / LOWER)
# ==================================================
CRAWL_LIFT_TIME = 0.6    # seconds foot stays lifted
CRAWL_LOWER_TIME = 0.3  # seconds after lowering before next leg
crawl_hold_start = 0.0
CRAWL_HOLD_TIME = 0.15
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
}


# store last commanded servo angles


for leg, ch in SHOULDERS.items():
    servo_angles[ch] = SHOULDER_STAND[leg]

for leg, ch in FEET.items():
    servo_angles[ch] = FOOT_STAND[leg]

# ==================================================
# READ-ONLY MONITOR GUI
# ==================================================
def start_monitor_gui():
    root = tk.Tk()
    root.title("Quadruped Layer-1 Monitor")

    # IMU labels
    imu_frame = tk.LabelFrame(root, text="IMU")
    imu_frame.pack(padx=10, pady=5, fill="x")
    # Acceleration labels (raw)
    accel_frame = tk.LabelFrame(root, text="Acceleration (raw)")
    accel_frame.pack(padx=10, pady=5, fill="x")

    ax_var = tk.StringVar()
    ay_var = tk.StringVar()
    az_var = tk.StringVar()
    
    # ADD — new GUI vars for unload debug
    roll_effort_var = tk.StringVar()
    offset_change_var = tk.StringVar()
    unloaded_var = tk.StringVar()
    
    roll_var = tk.StringVar()
    pitch_var = tk.StringVar()
    brace_var = tk.StringVar()
    brace_roll_var = tk.StringVar()
    brace_pitch_var = tk.StringVar()
    tk.Label(imu_frame, textvariable=roll_var, font=("Arial", 12)).pack(anchor="w")
    tk.Label(imu_frame, textvariable=pitch_var, font=("Arial", 12)).pack(anchor="w")
    tk.Label(imu_frame, textvariable=brace_var, font=("Arial", 12)).pack(anchor="w")
    tk.Label(imu_frame, textvariable=brace_roll_var, font=("Arial", 12)).pack(anchor="w")
    tk.Label(imu_frame, textvariable=brace_pitch_var, font=("Arial", 12)).pack(anchor="w")
    tk.Label(accel_frame, textvariable=ax_var, font=("Arial", 10)).pack(anchor="w")
    tk.Label(accel_frame, textvariable=ay_var, font=("Arial", 10)).pack(anchor="w")
    tk.Label(accel_frame, textvariable=az_var, font=("Arial", 10)).pack(anchor="w")
    
    # Place them under the accel_frame (or imu_frame) so they're visible
    tk.Label(imu_frame, textvariable=roll_effort_var, font=("Arial", 10)).pack(anchor="w")
    tk.Label(imu_frame, textvariable=offset_change_var, font=("Arial", 10)).pack(anchor="w")
    tk.Label(imu_frame, textvariable=unloaded_var, font=("Arial", 10)).pack(anchor="w")

    # Servo labels
    servo_frame = tk.LabelFrame(root, text="Servo Angles (deg)")
    servo_frame.pack(padx=10, pady=5)

    servo_vars = {}

    for ch in sorted(servo_angles.keys()):
        var = tk.StringVar()
        servo_vars[ch] = var
        tk.Label(
            servo_frame,
            textvariable=var,
            font=("Arial", 10),
            width=22,
            anchor="w"
        ).pack(anchor="w")

    def update():
        roll_var.set(f"Roll  : {monitor_state['roll']:+6.2f}°")
        pitch_var.set(f"Pitch : {monitor_state['pitch']:+6.2f}°")
        brace_var.set("Brace : ACTIVE" if monitor_state["brace"] else "Brace : off")
        brace_roll_var.set(f"Brace Roll Bias  : {monitor_state['brace_roll_bias']:+6.2f}°")
        brace_pitch_var.set(f"Brace Pitch Bias : {monitor_state['brace_pitch_bias']:+6.2f}°")
        
        # ADD — update debug vars from shared monitor_state
        roll_effort_var.set(f"Roll Effort   : {monitor_state.get('roll_effort', 0.0):.3f}")
        offset_change_var.set(f"Offset Change : {monitor_state.get('offset_change', 0.0):.3f}")
        unloaded_var.set(f"UNLOADED      : {monitor_state.get('unloaded', False)}")
        
        ax_var.set(f"AX : {monitor_state['ax']:6.0f}")
        ay_var.set(f"AY : {monitor_state['ay']:6.0f}")
        az_var.set(f"AZ : {monitor_state['az']:6.0f}")

        for ch, var in servo_vars.items():
            var.set(f"CH {ch:2d} → {servo_angles[ch]:6.2f}°")

        root.after(100, update)  # 10 Hz GUI refresh

    update()
    root.mainloop()
    
threading.Thread(
    target=start_monitor_gui,
    daemon=True
).start()   

# ==================================================
# COMMAND GUI — UNLOAD / LIFT / RESUME
# ==================================================
def start_command_gui():
    global gui_selected_leg, gui_lift_active, gui_resume_request, crawl_enabled

    root = tk.Tk()
    root.title("Quadruped Unload / Lift Control")
    root.geometry("360x420")

    # ---------------------------
    # UNLOAD CONTROLS
    # ---------------------------
    frame_unload = tk.LabelFrame(root, text="Unload Leg")
    frame_unload.pack(fill="x", padx=10, pady=5)

    def unload(leg):
        global gui_selected_leg, crawl_enabled
        crawl_enabled = False
        gui_selected_leg = leg

    for leg in ["FRF", "FLF", "RRF", "RLF"]:
        tk.Button(
            frame_unload,
            text=f"Unload {leg}",
            width=10,
            command=lambda l=leg: unload(l)
        ).pack(side="left", padx=4, pady=4)

    # ---------------------------
    # LIFT CONTROLS
    # ---------------------------
    frame_lift = tk.LabelFrame(root, text="Lift / Lower")
    frame_lift.pack(fill="x", padx=10, pady=5)

    def lift():
        global gui_lift_active
        gui_lift_active = True

    def lower():
        global gui_lift_active
        gui_lift_active = False

    tk.Button(frame_lift, text="Lift (+)", width=15, command=lift).pack(pady=4)
    tk.Button(frame_lift, text="Lower", width=15, command=lower).pack(pady=4)

    # ---------------------------
    # RESUME
    # ---------------------------
    def resume():
        global gui_resume_request, crawl_enabled
        crawl_enabled = False
        gui_resume_request = True

    tk.Button(
        root,
        text="Resume Normal Standing",
        width=28,
        command=resume
    ).pack(pady=8)

    # ---------------------------
    # CRAWL GAIT CONTROLS  ✅ FIXED
    # ---------------------------
    frame_crawl = tk.LabelFrame(root, text="Crawl Gait")
    frame_crawl.pack(fill="x", padx=10, pady=6)

    def start_crawl():
        global crawl_enabled, crawl_step_complete
        crawl_enabled = True
        crawl_step_complete = True

    def stop_crawl():
        global crawl_enabled
        crawl_enabled = False

    tk.Button(
        frame_crawl,
        text="Start Crawl",
        width=24,
        command=start_crawl
    ).pack(pady=3)

    tk.Button(
        frame_crawl,
        text="Stop Crawl",
        width=24,
        command=stop_crawl
    ).pack(pady=3)
    
    root.mainloop()
    
threading.Thread(
    target=start_command_gui,
    daemon=True).start()
    
gui_enabled = True
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
        monitor_state["ax"] = ax
        monitor_state["ay"] = ay
        monitor_state["az"] = az
        
        # ================= DEBUG TRACE =================
        if crawl_enabled:
            print(
                "[TRACE]",
                "fsm:", fsm_state,
                "| swing:", swing_leg,
                "| step_complete:", crawl_step_complete,
                "| crawl_enabled:", crawl_enabled
            )
        # ==============================================
        # ==================================================
        # CRAWL GAIT SCHEDULER (STEP-COMPLETE DRIVEN)
        # ==================================================
        if crawl_enabled and crawl_step_complete and fsm_state == "IDLE":
            swing_leg = CRAWL_SEQUENCE[crawl_index]
            print("[CRAWL] next leg:", swing_leg)
            crawl_index = (crawl_index + 1) % len(CRAWL_SEQUENCE)
            crawl_step_complete = False   # <<< ARM NEXT STEP
            crawl_phase = "UNLOAD"
            crawl_phase_start = now

        if DEBUG_PRINT and (time.time() - last_print) > PRINT_INTERVAL:
            print(f"Roll:{roll:6.2f}°  Pitch:{pitch:6.2f}°")
            last_print = time.time()

        # ==================================================
        # APPLY GUI RESUME REQUEST
        # ==================================================
        if gui_resume_request and not crawl_enabled:
            # cancel unloading & lifting
            swing_leg = None
            gui_selected_leg = None
            gui_lift_active = False

            posture_roll_target = 0.0
            posture_pitch_target = 0.0

            for k in motion_offsets:
                motion_offsets[k] = 0.0

            disable_roll_pd = False
            fsm_state = "RECENTERING"

            gui_resume_request = False
        # ==================================================
        # LAYER-2 FSM: UNLOAD → LIFT
        # ==================================================
            
        # ==================================================
        # GUI LIFT TRIGGER
        # ==================================================
        # If user presses Lift while unloading, enter LIFTING
        if gui_enabled and not crawl_enabled and fsm_state == "UNLOADING" and gui_lift_active:
            fsm_state = "LIFTING"
        # ==================================================
        # APPLY GUI INTENT (SAFE, NON-BLOCKING)
        # ==================================================

        # Resume to normal standing
        if gui_enabled and gui_resume_request:
            swing_leg = None
            gui_selected_leg = None
            gui_lift_active = False

            posture_roll_target = 0.0
            posture_pitch_target = 0.0

            for k in motion_offsets:
                motion_offsets[k] = 0.0

            disable_roll_pd = False
            fsm_state = "IDLE"
            gui_resume_request = False

            
        if fsm_state == "IDLE":
            if swing_leg is not None and unload_start == 0.0:
                posture_roll_target  = UNLOAD_POSTURE[swing_leg]["roll"]
                posture_pitch_target = UNLOAD_POSTURE[swing_leg]["pitch"]
                unload_start = time.time()

                disable_roll_pd = True
                fsm_state = "UNLOADING"

        elif fsm_state == "UNLOADING":
            # ==================================================
            # AUTHORITATIVE SHOULDER UNLOADING (NO PD FIGHT)
            # ==================================================
            for shoulder in SHOULDERS:
                # Body-space unload command (positive = outward, negative = inward)
                body_target = UNLOAD_SHOULDER_OFFSETS[swing_leg].get(shoulder, 0.0)

                # Convert body-space direction → servo-space direction
                servo_target = SHOULDER_SIGN[shoulder] * body_target

                motion_offsets[shoulder] = ramp(
                    motion_offsets[shoulder],
                    servo_target,
                    0.8
                )
            # -------- UNLOADED DETECTION (READ-ONLY) --------
            UNLOADED = (
                abs(roll  - posture_roll_bias)  < UNLOAD_SETTLE_THRESH and
                abs(pitch - posture_pitch_bias) < UNLOAD_SETTLE_THRESH
            )
            monitor_state["unloaded"] = bool(UNLOADED)
            
            # ---------- UNLOAD COMPLETE ----------
            if crawl_enabled:
                # Crawl: time-based unload (IMU-independent)
                if (time.time() - unload_start) > 0.6:
                    gui_lift_active = True
                    crawl_lift_start = time.time()
                    fsm_state = "LIFTING"
            else:
                # Manual unload: IMU-confirmed
                if UNLOADED and (time.time() - unload_start) > 0.5:
                    fsm_state = "LIFTING"

        elif fsm_state == "LIFTING":
            disable_pitch_pd_leg = swing_leg
            # BODY-space lift command
            body_lift = LIFT_HEIGHT if gui_lift_active else 0.0

            # Convert BODY-space → SERVO-space using FOOT_SIGN
            servo_lift = FOOT_SIGN[swing_leg] * body_lift

            motion_offsets[swing_leg] = ramp(
                motion_offsets[swing_leg],
                servo_lift,
                LIFT_STEP
            )

            # If fully lifted, enter DONE (only when lifting)
            if int(time.time() * 10) % 5 == 0:
                print(
                    f"[LIFT DBG] leg={swing_leg} "
                    f"target={servo_lift:.2f} "
                    f"current={motion_offsets[swing_leg]:.2f} "
                    f"gui_lift={gui_lift_active}"
                )
          
            # --------------------------------------------------
            # AUTO LOWER DURING CRAWL
            # --------------------------------------------------
            if crawl_enabled and gui_lift_active:
                if (time.time() - crawl_lift_start) > CRAWL_LIFT_TIME:
                    gui_lift_active = False
                    
            # ---- FORCE EXIT FROM LIFTING (TIME-BASED) ----
            if crawl_enabled and gui_lift_active:
                    fsm_state = "DONE"
                            
        elif fsm_state == "DONE":
            # ==================================================
            # MID-LIMB PROPULSION — STANCE LEGS ONLY (ONE-SHOT)
            # ==================================================
            if crawl_enabled and not propulsion_applied:
                if swing_leg == "FRF":
                    stance = [2, 9]   # RRM, FLM
                elif swing_leg == "FLF":
                    stance = [3, 8]   # RLM, FRM
                elif swing_leg == "RRF":
                    stance = [8, 9]   # FRM, FLM
                elif swing_leg == "RLF":
                    stance = [2, 3]   # RRM, RLM
                for ch in stance:
                    mid_offsets[ch] = ramp(
                        mid_offsets[ch],
                        MID_SIGN[ch] * PROPULSION_OFFSET,
                        PROPULSION_STEP
                    )
                propulsion_applied = True

            if not gui_lift_active:
                unload_start = 0.0
                if crawl_enabled:
                    crawl_hold_start = time.time()
                    fsm_state = "RECENTERING"
                else:
                    posture_roll_target = 0.0
                    posture_pitch_target = 0.0
                    fsm_state = "RECENTERING"
            if crawl_enabled:
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
                
            # ==================================================
            # RECENTER MID-LIMBS (PROPULSION RESET)
            # ==================================================
            for ch in mid_offsets:
                mid_offsets[ch] = ramp(
                    mid_offsets[ch],
                    0.0,
                    PROPULSION_STEP
                )
                
            # Check if everything is near neutral
            all_centered = (
                abs(posture_roll_bias) < RECENTER_THRESH and
                abs(posture_pitch_bias) < RECENTER_THRESH and
                all(abs(motion_offsets[k]) < RECENTER_THRESH for k in motion_offsets)
            )
            
            if all_centered and (not crawl_enabled or (time.time() - crawl_hold_start) > CRAWL_HOLD_TIME):
                print("[FSM] RECENTER COMPLETE — step finished")
                disable_roll_pd = False
                disable_pitch_pd_leg = None
                gui_selected_leg = None
                propulsion_applied = False
                unload_start = 0.0
                if crawl_enabled:
                    crawl_step_complete = True
                    swing_leg = None   # ✅ RELEASE CURRENT LEG (CRITICAL)
                crawl_phase = "IDLE"
                fsm_state = "IDLE"

        
        # ==================================================
        # ROLL REFLEX
        # ==================================================
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
            
        # ==================================================
        # APPLY MID-LIMB SERVO COMMANDS (PROPULSION)
        # ==================================================
        for ch, base in MID_LIMBS.items():
            angle = base + mid_offsets[ch]
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
        
        # ==================================================
        # PITCH REFLEX
        # ==================================================
        # Knees flex/extend to shift vertical load
        for leg, ch in FEET.items():
            if leg == disable_pitch_pd_leg:
                pitch_offsets[leg] = 0.0   # <<< CRITICAL
            else:
                pitch_rate = gy / 131.0
                if abs(pitch_rate) < GYRO_RATE_DEADBAND:
                    pitch_rate = 0.0

                tgt = (gain_scale * (K_PITCH * effective_pitch + K_PITCH_RATE * pitch_rate)
                       + brace_pitch_bias) * (1 if leg.startswith("F") else -1)

                tgt = max(-PITCH_LIMIT, min(PITCH_LIMIT, tgt))

                delta = tgt - pitch_offsets[leg]
                delta = max(-PITCH_STEP, min(PITCH_STEP, delta))
                if abs(delta) < 0.05:
                    delta = 0.0

                pitch_offsets[leg] += delta

            # Final foot angle = stand + balance + motion
            angle = (
                FOOT_STAND[leg]
                + FOOT_SIGN[leg] * pitch_offsets[leg]   # Layer-1 balance
                + motion_offsets[leg]                   # Layer-2 motion
            )

            angle = max(FOOT_LIMITS[leg][0], min(FOOT_LIMITS[leg][1], angle))
            set_servo_angle(ch, angle)
            servo_angles[ch] = angle

        time.sleep(DT)

    except IOError:
        print("I2C glitch — holding posture")
        time.sleep(0.2)
