import time
import imu_roll_pitch as imu
from servo_driver import set_servo_angle

# ===============================
# SHOULDERS ONLY
# ===============================
SHOULDERS = {
    "RR": 0,
    "RL": 1,
    "FR": 6,
    "FL": 7,
}

STAND = {
    "RR": 38,
    "RL": 38,
    "FR": 41,
    "FL": 41,
}

# ===============================
K_ROLL = 0.20
K_PITCH = 0.20

MAX_OFFSET = 4.0
MAX_STEP = 0.25
DT = 0.05

current = {k: 0.0 for k in STAND}

# ===============================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def compute_offsets(roll, pitch):
    off = {k: 0.0 for k in STAND}

    p = K_PITCH * pitch
    off["FR"] += p; off["FL"] += p
    off["RR"] -= p; off["RL"] -= p

    r = K_ROLL * roll
    off["FL"] -= r; off["RL"] -= r
    off["FR"] += r; off["RR"] += r

    for k in off:
        off[k] = clamp(off[k], -MAX_OFFSET, MAX_OFFSET)
    return off

# ===============================
print("Layer-1 stabilizer running")

while True:
    try:
        roll = imu.roll_pitch["roll"]
        pitch = imu.roll_pitch["pitch"]

        target = compute_offsets(roll, pitch)

        for leg in target:
            delta = clamp(target[leg] - current[leg], -MAX_STEP, MAX_STEP)
            current[leg] += delta
            set_servo_angle(SHOULDERS[leg], STAND[leg] + current[leg])

    except Exception:
        pass

    time.sleep(DT)

