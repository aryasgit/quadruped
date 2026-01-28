# ==================================================
# GOD BALANCE CONTROLLER
# Single-file bringup for quadruped COM balance
# Absolute authority. Visible motion. No abstractions.
# ==================================================

import time
import math
from smbus2 import SMBus

# ------------------ HARDWARE TRUTHS ------------------
from absolute_truths import (
    BUS, PCA_ADDR,
    MODE1, PRESCALE,
    PULSE_MIN, PULSE_MAX,

    WRISTS, THIGHS, COXA,
    WRIST_STAND, THIGH_STAND, COXA_STAND,

    WRIST_MECH, THIGH_MECH, COXA_MECH
)

# ------------------ IMU ------------------
from imu import (
    init_mpu,
    calibrate_imu,
    IMUFilter,
    lock_stand_reference,
)

# ------------------ OPTIONAL COM PLOTTER ------------------
USE_PLOTTER = True
if USE_PLOTTER:
    import multiprocessing as mp
    from com_plotter import com_plotter_main

# ==================================================
# I2C
# ==================================================
bus = SMBus(BUS)

# ==================================================
# PCA9685 LOW LEVEL
# ==================================================
def angle_to_pulse(angle):
    return int(PULSE_MIN + (angle / 270.0) * (PULSE_MAX - PULSE_MIN))

def set_servo(ch, angle):
    pulse = angle_to_pulse(angle)
    base = 0x06 + 4 * ch
    bus.write_byte_data(PCA_ADDR, base, 0)
    bus.write_byte_data(PCA_ADDR, base + 1, 0)
    bus.write_byte_data(PCA_ADDR, base + 2, pulse & 0xFF)
    bus.write_byte_data(PCA_ADDR, base + 3, (pulse >> 8) & 0x0F)

def init_pca():
    bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
    time.sleep(0.01)
    prescale = int(25000000 / (4096 * 50) - 1)
    bus.write_byte_data(PCA_ADDR, MODE1, 0x10)
    bus.write_byte_data(PCA_ADDR, PRESCALE, prescale)
    bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
    bus.write_byte_data(PCA_ADDR, MODE1, 0x80)
    time.sleep(0.01)
    print("[PCA] initialized @ 50Hz")

# ==================================================
# FOOT GEOMETRY (BODY FRAME, METERS)
# ==================================================
FOOT_POS = {
    "FL": (-0.12, -0.08),
    "FR": (-0.12, +0.08),
    "RL": (+0.11, -0.08),
    "RR": (+0.11, +0.08),
}
BODY_H = 0.24

# ==================================================
# GAINS (CRANK THESE)
# ==================================================
COXA_GAIN  = 80.0     # deg / meter
THIGH_GAIN = 120.0
WRIST_GAIN = 160.0

MAX_COXA  = 25.0
MAX_THIGH = 35.0
MAX_WRIST = 45.0

# ==================================================
# SIGN MAPS (AUTHORITATIVE)
# ==================================================
# ==================================================
# SIGN MAPS (computed from mechanical truths in absolute_truths)
# - We infer sign by checking whether mech.min < mech.max
# - mech key names end with the leg code (e.g. "WFL","TFR","FL")
# ==================================================
def _infer_signs_from_mech(mech_dict):
    """
    mech_dict: e.g. WRIST_MECH, THIGH_MECH, COXA_MECH
    returns: { 'FL': +1, 'FR': -1, ... }
    """
    signs = {}
    for mech_key, meta in mech_dict.items():
        # leg code is last two chars (works for keys like "WFL","TFR","FL")
        leg = mech_key[-2:]
        # if min < max mechanical mapping is increasing => +1
        # if min > max => reversed => -1
        mmin = meta.get("min")
        mmax = meta.get("max")
        if mmin is None or mmax is None:
            # fallback: assume +1
            signs[leg] = +1
            continue
        signs[leg] = +1 if mmin < mmax else -1
    return signs

# compute authoritative sign maps once
COXA_SIGN  = _infer_signs_from_mech(COXA_MECH)   # keys like "FL","FR","RL","RR"
THIGH_SIGN = _infer_signs_from_mech(THIGH_MECH)  # keys like "TFL","TFR","TRL","TRR"
WRIST_SIGN = _infer_signs_from_mech(WRIST_MECH)  # keys like "WFL","WFR","WRL","WRR"

# ==================================================
# INIT
# ==================================================
print("[BOOT] init PCA + IMU")
init_pca()

for k, ch in COXA.items():
    set_servo(ch, COXA_STAND[k])
for k, ch in THIGHS.items():
    set_servo(ch, THIGH_STAND[k])
for k, ch in WRISTS.items():
    set_servo(ch, WRIST_STAND[k])

init_mpu(bus)
calib = calibrate_imu(bus)
imu = IMUFilter(calib)
lock_stand_reference(imu, bus)

# ==================================================
# COM PLOTTER
# ==================================================
if USE_PLOTTER:
    q = mp.Queue(maxsize=50)
    p = mp.Process(target=com_plotter_main, args=(q,), daemon=True)
    p.start()

# ==================================================
# MAIN LOOP
# ==================================================
print("\n[GOD MODE] BALANCE ACTIVE\n")

last_print = time.time()

while True:
    roll, pitch, _, _ = imu.update(bus)

    # -------- COM PROJECTION --------
    com_x = BODY_H * math.tan(math.radians(pitch))
    com_y = -BODY_H * math.tan(math.radians(roll))

    # -------- PER LEG COMMAND --------
    for leg, (fx, fy) in FOOT_POS.items():

        # influence of COM relative to foot
        dx = com_x - fx
        dy = com_y - fy

        # ---------- COXA (Y / roll) ----------
        coxa_off = COXA_SIGN[leg] * COXA_GAIN * dy
        coxa_off = max(-MAX_COXA, min(MAX_COXA, coxa_off))

        # ---------- THIGH (X / pitch) ----------
        thigh_off = THIGH_SIGN[leg] * THIGH_GAIN * dx
        thigh_off = max(-MAX_THIGH, min(MAX_THIGH, thigh_off))

        # ---------- WRIST (LOAD / magnitude) ----------
        load = math.hypot(dx, dy)
        wrist_off = WRIST_SIGN[leg] * WRIST_GAIN * load
        wrist_off = max(-MAX_WRIST, min(MAX_WRIST, wrist_off))

        # ---------- COMMAND ----------
        set_servo(COXA[leg],  COXA_STAND[leg]  + coxa_off)
        set_servo(THIGHS["T"+leg], THIGH_STAND["T"+leg] + thigh_off)
        set_servo(WRISTS["W"+leg], WRIST_STAND["W"+leg] + wrist_off)

    # -------- PLOTTER --------
    if USE_PLOTTER:
        if not q.full():
            q.put_nowait({
                "com_x": com_x,
                "com_y": com_y,
                "stability": 1.0,
            })

    # -------- PRINT (SLOW) --------
    if time.time() - last_print > 1.0:
        print(
            f"roll={roll:+.2f} pitch={pitch:+.2f} | "
            f"COM=({com_x:+.3f},{com_y:+.3f})"
        )
        last_print = time.time()

    time.sleep(0.05)
