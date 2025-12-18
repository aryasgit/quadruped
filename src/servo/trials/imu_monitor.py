import time
from smbus2 import SMBus

# ===============================
# CONFIG
# ===============================
BUS_ID = 7
MPU_ADDR = 0x68

# Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

# Sensitivity & deadband tuning
ACCEL_DEADBAND = 300     # ignore tiny accel noise (LSB)
GYRO_DEADBAND  = 80      # ignore tiny gyro noise (LSB)

PRINT_DELTA_ACCEL = 600  # print only if accel changes this much
PRINT_DELTA_GYRO  = 150  # print only if gyro changes this much

CALIBRATION_SAMPLES = 200

bus = SMBus(BUS_ID)

# ===============================
# LOW LEVEL
# ===============================
def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low  = bus.read_byte_data(MPU_ADDR, reg + 1)
    val = (high << 8) | low
    if val > 32767:
        val -= 65536
    return val

def read_accel_raw():
    return (
        read_word(ACCEL_XOUT_H),
        read_word(ACCEL_XOUT_H + 2),
        read_word(ACCEL_XOUT_H + 4),
    )

def read_gyro_raw():
    return (
        read_word(GYRO_XOUT_H),
        read_word(GYRO_XOUT_H + 2),
        read_word(GYRO_XOUT_H + 4),
    )

# ===============================
# INIT MPU
# ===============================
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
time.sleep(0.1)

print("Calibrating IMU offsets... Keep robot still")

# ===============================
# OFFSET CALIBRATION
# ===============================
ax_off = ay_off = az_off = 0
gx_off = gy_off = gz_off = 0

for _ in range(CALIBRATION_SAMPLES):
    ax, ay, az = read_accel_raw()
    gx, gy, gz = read_gyro_raw()

    ax_off += ax
    ay_off += ay
    az_off += (az - 16384)  # remove gravity
    gx_off += gx
    gy_off += gy
    gz_off += gz

    time.sleep(0.01)

ax_off /= CALIBRATION_SAMPLES
ay_off /= CALIBRATION_SAMPLES
az_off /= CALIBRATION_SAMPLES
gx_off /= CALIBRATION_SAMPLES
gy_off /= CALIBRATION_SAMPLES
gz_off /= CALIBRATION_SAMPLES

print("Calibration done")
print(f"Accel offsets: {ax_off:.1f}, {ay_off:.1f}, {az_off:.1f}")
print(f"Gyro  offsets: {gx_off:.1f}, {gy_off:.1f}, {gz_off:.1f}\n")

# ===============================
# STATE MEMORY (for change detect)
# ===============================
prev_ax = prev_ay = prev_az = 0
prev_gx = prev_gy = prev_gz = 0

print("IMU monitor running (filtered)")
print("Printing only on significant motion\n")

# ===============================
# MAIN LOOP
# ===============================
try:
    while True:
        ax, ay, az = read_accel_raw()
        gx, gy, gz = read_gyro_raw()

        # Apply offsets
        ax -= ax_off
        ay -= ay_off
        az -= az_off
        gx -= gx_off
        gy -= gy_off
        gz -= gz_off

        # Deadband filtering
        ax = 0 if abs(ax) < ACCEL_DEADBAND else int(ax)
        ay = 0 if abs(ay) < ACCEL_DEADBAND else int(ay)
        az = 0 if abs(az) < ACCEL_DEADBAND else int(az)

        gx = 0 if abs(gx) < GYRO_DEADBAND else int(gx)
        gy = 0 if abs(gy) < GYRO_DEADBAND else int(gy)
        gz = 0 if abs(gz) < GYRO_DEADBAND else int(gz)

        # Change detection
        accel_change = (
            abs(ax - prev_ax) > PRINT_DELTA_ACCEL or
            abs(ay - prev_ay) > PRINT_DELTA_ACCEL or
            abs(az - prev_az) > PRINT_DELTA_ACCEL
        )

        gyro_change = (
            abs(gx - prev_gx) > PRINT_DELTA_GYRO or
            abs(gy - prev_gy) > PRINT_DELTA_GYRO or
            abs(gz - prev_gz) > PRINT_DELTA_GYRO
        )

        if accel_change or gyro_change:
            print(
                f"ACCEL AX:{ax:6d} AY:{ay:6d} AZ:{az:6d} | "
                f"GYRO GX:{gx:6d} GY:{gy:6d} GZ:{gz:6d}"
            )

            prev_ax, prev_ay, prev_az = ax, ay, az
            prev_gx, prev_gy, prev_gz = gx, gy, gz

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nIMU monitor stopped")

