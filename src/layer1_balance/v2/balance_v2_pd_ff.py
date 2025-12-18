import time, math, errno
from smbus2 import SMBus
import threading 
import tkinter as tk
# ==================================================
# I2C CONFIG (HARDWARE — DO NOT TWEAK)
# ==================================================
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
    "FR": 25.0,
    "FL": 25.0,
    "RR": 25.0,
    "RL": 25.0,
}

# Neutral standing angles (MEASURED, DO NOT GUESS)
SHOULDER_STAND = {
    "FR": 41,
    "FL": 41,
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
    "FRF": 128,
    "FLF": 61,
    "RRF": 142,
    "RLF": 61,
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

# These store smooth joint offsets
roll_offsets = {k: 0.0 for k in SHOULDERS}
pitch_offsets = {k: 0.0 for k in FEET}

print("Layer-1 Roll + Pitch ACTIVE (±30°)\n")

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
}


# store last commanded servo angles
servo_angles = {}

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
        monitor_state["brace"] = brace_active
        monitor_state["brace_roll_bias"] = brace_roll_bias
        monitor_state["brace_pitch_bias"] = brace_pitch_bias
        monitor_state["ax"] = ax
        monitor_state["ay"] = ay
        monitor_state["az"] = az

        if DEBUG_PRINT and (time.time() - last_print) > PRINT_INTERVAL:
            print(f"Roll:{roll:6.2f}°  Pitch:{pitch:6.2f}°")
            last_print = time.time()

        # ---------- ROLL REFLEX ----------
        # Shoulders widen/narrow stance to resist tipping
        gain_scale = BRACE_GAIN_SCALE if brace_active else 1.0
        roll_rate = gx / 131.0      #deg/s
        if abs(roll_rate) < GYRO_RATE_DEADBAND:
            roll_rate = 0.0
        roll_cmd = -(gain_scale * (K_ROLL * roll + K_ROLL_RATE * roll_rate)) + brace_roll_bias
        roll_cmd = max(-ROLL_LIMIT, min(ROLL_LIMIT, roll_cmd))

        for leg, ch in SHOULDERS.items():
            delta = roll_cmd - roll_offsets[leg]
            delta = max(-ROLL_STEP, min(ROLL_STEP, delta))
            if abs(delta) < 0.05:
                delta = 0.0
            roll_offsets[leg] += delta

            roll_offsets[leg] = max(
                -SHOULDER_MAX_OFFSET[leg],
                min(SHOULDER_MAX_OFFSET[leg], roll_offsets[leg])
            )   

            angle = SHOULDER_STAND[leg] + roll_offsets[leg]
            set_servo_angle(ch, angle)
            servo_angles[ch]=angle
            

        # ---------- PITCH REFLEX ----------
        # Knees flex/extend to shift vertical load
        for leg, ch in FEET.items():
            
            pitch_rate = gy / 131.0     # deg/s
            if abs(pitch_rate) < GYRO_RATE_DEADBAND:
                pitch_rate = 0.0
            tgt = (gain_scale * (K_PITCH * pitch + K_PITCH_RATE * pitch_rate)+ brace_pitch_bias) * (1 if leg.startswith("F") else -1)
            tgt = max(-PITCH_LIMIT, min(PITCH_LIMIT, tgt))

            delta = tgt - pitch_offsets[leg]
            delta = max(-PITCH_STEP, min(PITCH_STEP, delta))
            if abs(delta) < 0.05:
                delta = 0.0
            pitch_offsets[leg] += delta

            angle = FOOT_STAND[leg] + FOOT_SIGN[leg] * pitch_offsets[leg]
            angle = max(FOOT_LIMITS[leg][0], min(FOOT_LIMITS[leg][1], angle))
            set_servo_angle(ch, angle)
            servo_angles[ch]=angle

        time.sleep(DT)

    except IOError:
        print("I2C glitch — holding posture")
        time.sleep(0.2)
