"""
FULL STACK POSTURE TEST (REFERENCE-STABLE)
=========================================

Safe bench test.
"""

import time

from hardware.imu import init_mpu, calibrate, IMUFilter
from layer5.posture_controller import posture_step
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS


DT = 0.03

STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18


CHANNELS = {
    "FL_COXA": COXA["FL"], "FL_THIGH": THIGHS["TFL"], "FL_WRIST": WRISTS["WFL"],
    "FR_COXA": COXA["FR"], "FR_THIGH": THIGHS["TFR"], "FR_WRIST": WRISTS["WFR"],
    "RL_COXA": COXA["RL"], "RL_THIGH": THIGHS["TRL"], "RL_WRIST": WRISTS["WRL"],
    "RR_COXA": COXA["RR"], "RR_THIGH": THIGHS["TRR"], "RR_WRIST": WRISTS["WRR"],
}


def nominal_stance():
    return {
        "FL": (STANCE_X,  STANCE_Y, STANCE_Z),
        "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
        "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
        "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
    }


print("[TEST] Initializing IMU")
init_mpu()
calib = calibrate()
imu = IMUFilter(calib, dt=DT)

print("[TEST] Standing reference locked — tilt robot gently")

try:
    while True:
        deltas = posture_step(nominal_stance(), imu)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)

        for j, ch in CHANNELS.items():
            set_servo_angle(ch, physical[j])

        time.sleep(DT)

except KeyboardInterrupt:
    print("\n[TEST] Stopped cleanly")
