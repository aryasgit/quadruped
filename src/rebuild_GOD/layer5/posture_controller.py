import time

from hardware.imu import (
    init_mpu,
    calibrate,
    IMUFilter,
)

init_mpu()
calib = calibrate()
imu = IMUFilter(calib)


from layer3.leg_ik import solve_all_legs
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import WRISTS, THIGHS, COXA, BUS


SERVO_CHANNELS = {
    "FL_COXA": COXA["FL"], "FL_THIGH": THIGHS["TFL"], "FL_WRIST": WRISTS["WFL"],
    "FR_COXA": COXA["FR"], "FR_THIGH": THIGHS["TFR"], "FR_WRIST": WRISTS["WFR"],
    "RR_COXA": COXA["RR"], "RR_THIGH": THIGHS["TRR"], "RR_WRIST": WRISTS["WRR"],
    "RL_COXA": COXA["RL"], "RL_THIGH": THIGHS["TRL"], "RL_WRIST": WRISTS["WRL"],
}

BASE_Z = -0.18
ROLL_GAIN  = 0.005   # meters per degree
PITCH_GAIN = 0.005

def posture_step():
    roll, pitch, _, _ = imu.update()

    dz_left  = -ROLL_GAIN * roll
    dz_right =  ROLL_GAIN * roll
    dz_front = -PITCH_GAIN * pitch
    dz_rear  =  PITCH_GAIN * pitch

    foot_targets = {
        "FL": ( 0.15,  0.05, BASE_Z + dz_left  + dz_front),
        "FR": ( 0.15, -0.05, BASE_Z + dz_right + dz_front),
        "RR": (-0.15, -0.05, BASE_Z + dz_right + dz_rear),
        "RL": (-0.15,  0.05, BASE_Z + dz_left  + dz_rear),
    }

    deltas = solve_all_legs(foot_targets)
    deltas = apply_joint_conventions(deltas)
    physical = normalize_all(deltas)

    for joint, angle in physical.items():
        set_servo_angle(SERVO_CHANNELS[joint], angle)

if __name__ == "__main__":
    print("[L5] IMU posture control test — HOLD ROBOT")

    while True:
        posture_step()
        time.sleep(0.02)   # ~50 Hz
