import time, math, threading
import tkinter as tk
from smbus2 import SMBus

# ==================================================
# I2C CONFIG
# ==================================================
BUS = 7
MPU_ADDR = 0x68
PCA_ADDR = 0x40

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

MODE1 = 0x00
PRESCALE = 0xFE

# ==================================================
# SERVO CONFIG
# ==================================================
PULSE_MIN = 80
PULSE_MAX = 561

# ---------------- STANDING POSE -------------------
SERVO_MAP = {
    0: 38,  1: 37,  2: 102, 3: 154,
    4: 142, 5: 61,  6: 37,  7: 46,
    8: 85,  9: 176, 10: 137, 11: 61
}

SERVO_NAMES = [
    "RRS","RLS","RRM","RLM","RRF","RLF",
    "FRS","FLS","FRM","FLM","FRF","FLF"
]

# ---------------- FEET ONLY ----------------------
FEET = {
    "RRF": 4,
    "RLF": 5,
    "FRF": 10,
    "FLF": 11,
}

FOOT_STAND = {
    "RRF": 142,
    "RLF": 61,
    "FRF": 137,
    "FLF": 61,
}

FOOT_SIGN = {
    "RRF": -1,
    "FRF": -1,
    "RLF": +1,
    "FLF": +1,
}

FOOT_LIMITS = {k: (0, 190) for k in FEET}

# ==================================================
# CONTROL TUNING (VISIBLE & SAFE)
# ==================================================
DT = 0.05
ALPHA = 0.96

# ---- PITCH AUTHORITY ----
K_PITCH = 0.8          # Large range support (±30°)
MAX_PITCH = 30.0       # Degrees

FOOT_STEP = 1.5        # Fast enough to SEE
MAX_OFFSET = 25.0      # Safety

ACCEL_DEADBAND = 50
GYRO_DEADBAND  = 10

bus = SMBus(BUS)

# ==================================================
# LOW LEVEL
# ==================================================
def safe_read_word(addr, reg):
    h = bus.read_byte_data(addr, reg)
    l = bus.read_byte_data(addr, reg + 1)
    v = (h << 8) | l
    return v - 65536 if v > 32767 else v

def angle_to_pulse(angle):
    return int(PULSE_MIN + (angle / 270.0) * (PULSE_MAX - PULSE_MIN))

def set_servo_angle(ch, angle):
    pulse = angle_to_pulse(angle)
    base = 0x06 + 4 * ch
    bus.write_byte_data(PCA_ADDR, base, 0)
    bus.write_byte_data(PCA_ADDR, base + 1, 0)
    bus.write_byte_data(PCA_ADDR, base + 2, pulse & 0xFF)
    bus.write_byte_data(PCA_ADDR, base + 3, (pulse >> 8) & 0x0F)

# ==================================================
# GUI (READ ONLY)
# ==================================================
def start_gui(current_angles):
    root = tk.Tk()
    root.title("Layer-1 Pitch Monitor (Read-Only)")

    sliders = {}

    for ch in range(12):
        frame = tk.Frame(root)
        frame.grid(row=ch//2, column=ch%2, padx=6, pady=4)

        label = tk.Label(frame, text=f"{SERVO_NAMES[ch]} (CH{ch})")
        label.pack(anchor="w")

        slider = tk.Scale(
            frame, from_=0, to=270,
            orient=tk.HORIZONTAL, length=260
        )
        slider.pack()
        slider.configure(state="disabled")

        sliders[ch] = slider

    def update():
        for ch, ang in current_angles.items():
            sliders[ch].set(ang)
        root.after(40, update)

    update()
    root.mainloop()

# ==================================================
# PCA INIT
# ==================================================
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
time.sleep(0.01)
prescale = int(25000000 / (4096 * 50) - 1)
bus.write_byte_data(PCA_ADDR, MODE1, 0x10)
bus.write_byte_data(PCA_ADDR, PRESCALE, prescale)
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
bus.write_byte_data(PCA_ADDR, MODE1, 0x80)

# ==================================================
# LOCK STAND
# ==================================================
print("Locking STAND pose...")
current_angles = SERVO_MAP.copy()
for _ in range(60):
    done = True
    for ch, tgt in SERVO_MAP.items():
        d = tgt - current_angles[ch]
        if abs(d) > 0.4:
            done = False
            current_angles[ch] += 0.4 if d > 0 else -0.4
            set_servo_angle(ch, current_angles[ch])
    if done:
        break
    time.sleep(0.02)

# ==================================================
# IMU INIT + CALIBRATION
# ==================================================
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
time.sleep(0.1)

ax_o = ay_o = az_o = gy_o = 0
for _ in range(200):
    ax_o += safe_read_word(MPU_ADDR, ACCEL_XOUT_H)
    ay_o += safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 2)
    az_o += safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 4) - 16384
    gy_o += safe_read_word(MPU_ADDR, GYRO_XOUT_H + 2)
    time.sleep(0.01)

ax_o /= 200; ay_o /= 200; az_o /= 200; gy_o /= 200

# ==================================================
# START GUI
# ==================================================


# ==================================================
# CONTROL LOOP
# ==================================================
def control_loop():
    pitch = 0.0
    offsets = {k: 0.0 for k in FEET}

    print("Layer-1 Pitch ACTIVE (±30°)\n")

    while True:
        ax = safe_read_word(MPU_ADDR, ACCEL_XOUT_H) - ax_o
        ay = safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 2) - ay_o
        az = safe_read_word(MPU_ADDR, ACCEL_XOUT_H + 4) - az_o
        gy = safe_read_word(MPU_ADDR, GYRO_XOUT_H + 2) - gy_o

        if abs(ax) < ACCEL_DEADBAND: ax = 0
        if abs(gy) < GYRO_DEADBAND: gy = 0

        accel_pitch = math.degrees(
            math.atan2(-ax, math.sqrt(ay*ay + az*az))
        )

        pitch = ALPHA * (pitch + (gy / 131.0) * DT) \
                + (1 - ALPHA) * accel_pitch

        pitch = max(-MAX_PITCH, min(MAX_PITCH, pitch))
        print(f"Pitch: {pitch:6.2f}°")

        targets = {
            "FRF": +K_PITCH * pitch,
            "FLF": +K_PITCH * pitch,
            "RRF": -K_PITCH * pitch,
            "RLF": -K_PITCH * pitch,
        }

        for leg, ch in FEET.items():
            tgt = max(-MAX_OFFSET, min(MAX_OFFSET, targets[leg]))
            delta = tgt - offsets[leg]
            delta = max(-FOOT_STEP, min(FOOT_STEP, delta))
            offsets[leg] += delta

            angle = FOOT_STAND[leg] + FOOT_SIGN[leg] * offsets[leg]
            angle = max(FOOT_LIMITS[leg][0], min(FOOT_LIMITS[leg][1], angle))

            set_servo_angle(ch, angle)
            current_angles[ch] = angle

        time.sleep(DT)
threading.Thread(target=control_loop,daemon=True).start()        
start_gui(current_angles)
