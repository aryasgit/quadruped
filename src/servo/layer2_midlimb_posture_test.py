import time, math
from smbus2 import SMBus

# ==================================================
# I2C CONFIG
# ==================================================
BUS = 7
PCA_ADDR = 0x40
MPU_ADDR = 0x68

MODE1 = 0x00
PRESCALE = 0xFE

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

PULSE_MIN = 80
PULSE_MAX = 561

bus = SMBus(BUS)

# ==================================================
# MIDLIMBS (POSTURE)
# ==================================================
MID = {
    "FRM": 8,
    "FLM": 9,
    "RRM": 2,
    "RLM": 3,
}

MID_STAND = {
    "FRM": 85,
    "FLM": 176,
    "RRM": 102,
    "RLM": 154,
}

# Right: +angle → forward
# Left:  +angle → backward
MID_SIGN = {
    "FRM": +1,
    "RRM": +1,
    "FLM": -1,
    "RLM": -1,
}

# ==================================================
# FEET (LOAD REDISTRIBUTION)
# ==================================================
FEET = {
    "FRF": 10,
    "FLF": 11,
    "RRF": 4,
    "RLF": 5,
}

FOOT_STAND = {
    "FRF": 137,
    "FLF": 61,
    "RRF": 142,
    "RLF": 61,
}

FOOT_SIGN = {
    "FRF": -1,
    "FLF": +1,
    "RRF": -1,
    "RLF": +1,
}

FOOT_LIMITS = {k: (0, 190) for k in FEET}

# ==================================================
# LAYER-2 PARAMETERS
# ==================================================
DT = 0.05

# Posture (midlimb)
POSTURE_KP = 0.9
POSTURE_LIMIT = 20.0
MID_STEP = 0.25

# Load redistribution (feet)
LOAD_K = 0.25
LOAD_LIMIT = 5.0
LOAD_STEP = 0.15

# ==================================================
# STATE
# ==================================================
mid_offsets = {k: 0.0 for k in MID}
load_offsets = {k: 0.0 for k in FEET}

# ==================================================
# LOW LEVEL
# ==================================================
def angle_to_pulse(angle):
    return int(PULSE_MIN + (angle / 270.0) * (PULSE_MAX - PULSE_MIN))

def set_servo_angle(ch, angle):
    pulse = angle_to_pulse(angle)
    base = 0x06 + 4 * ch
    bus.write_byte_data(PCA_ADDR, base, 0)
    bus.write_byte_data(PCA_ADDR, base+1, 0)
    bus.write_byte_data(PCA_ADDR, base+2, pulse & 0xFF)
    bus.write_byte_data(PCA_ADDR, base+3, (pulse >> 8) & 0x0F)

def read_word(reg):
    h = bus.read_byte_data(MPU_ADDR, reg)
    l = bus.read_byte_data(MPU_ADDR, reg+1)
    v = (h << 8) | l
    return v - 65536 if v > 32767 else v

# ==================================================
# INIT PCA + IMU
# ==================================================
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
time.sleep(0.01)
prescale = int(25000000 / (4096 * 50) - 1)
bus.write_byte_data(PCA_ADDR, MODE1, 0x10)
bus.write_byte_data(PCA_ADDR, PRESCALE, prescale)
bus.write_byte_data(PCA_ADDR, MODE1, 0x00)
bus.write_byte_data(PCA_ADDR, MODE1, 0x80)

bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
time.sleep(0.1)

# ==================================================
# MOVE TO STAND
# ==================================================
for leg, ch in MID.items():
    set_servo_angle(ch, MID_STAND[leg])
    time.sleep(0.03)

for leg, ch in FEET.items():
    set_servo_angle(ch, FOOT_STAND[leg])
    time.sleep(0.03)

print("\nLayer-2 Posture + Load Redistribution ACTIVE")
print("Lock robot at ~10–12° pitch and observe correction\n")

# ==================================================
# MAIN LOOP
# ==================================================
last_print = time.time()

while True:


    # ---------- PRINT ----------
    if time.time() - last_print > 0.25:
        print(
            f"PITCH:{pitch:6.2f}° | "
            f"MID:{ {k: round(v,1) for k,v in mid_offsets.items()} } | "
            f"LOAD:{ {k: round(v,1) for k,v in load_offsets.items()} }"
        )
        last_print = time.time()

    time.sleep(DT)
