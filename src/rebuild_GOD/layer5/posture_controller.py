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

# =====================
# PITCH PID CONTROLLER
# =====================
PITCH_KP = 0.6
PITCH_KI = 0.8
PITCH_KD = 0.04

_pitch_i = 0.0
_prev_pitch = 0.0

PITCH_I_LIMIT = 12.0    # degrees-equivalent
PITCH_CMD_LIMIT = 20.0 # max virtual pitch command

# =====================
# GROUND PITCH ESTIMATOR
# =====================
pitch_ref = 0.0

PITCH_REF_ADAPT_RATE = 0.02   # deg/sec
PITCH_REF_MAX = 20.0          # safety


SERVO_CHANNELS = {
    "FL_COXA": COXA["FL"], "FL_THIGH": THIGHS["TFL"], "FL_WRIST": WRISTS["WFL"],
    "FR_COXA": COXA["FR"], "FR_THIGH": THIGHS["TFR"], "FR_WRIST": WRISTS["WFR"],
    "RR_COXA": COXA["RR"], "RR_THIGH": THIGHS["TRR"], "RR_WRIST": WRISTS["WRR"],
    "RL_COXA": COXA["RL"], "RL_THIGH": THIGHS["TRL"], "RL_WRIST": WRISTS["WRL"],
}

BASE_Z = -0.18
ROLL_GAIN  = 0.005   # meters per degree
PITCH_GAIN = 0.005
X_FRONT_GAIN = 0.0015
X_REAR_GAIN  = 0.0005   # stronger


def posture_step():
    global pitch_ref, _pitch_i, _prev_pitch
    roll, pitch_meas, _, _ = imu.update()
    # ---------------------------
    # Ground pitch adaptation
    # ---------------------------
    pitch_error_raw = pitch_meas - pitch_ref

    # Adapt reference ONLY when robot is quiet (important)
    pitch_ref += PITCH_REF_ADAPT_RATE * pitch_error_raw * 0.02
    pitch_ref = max(min(pitch_ref, PITCH_REF_MAX), -PITCH_REF_MAX)

    # Final control error
    pitch_err = pitch_meas - pitch_ref

    dt = 0.02

    # Integrator
    _pitch_i += pitch_err * dt
    _pitch_i = max(min(_pitch_i, PITCH_I_LIMIT), -PITCH_I_LIMIT)

    # Derivative
    pitch_d = (pitch_err - _prev_pitch) / dt
    _prev_pitch = pitch_err

    # PID output
    pitch_cmd = (
        PITCH_KP * pitch_err +
        PITCH_KI * _pitch_i +
        PITCH_KD * pitch_d
    )


    pitch_cmd = max(min(pitch_cmd, PITCH_CMD_LIMIT), -PITCH_CMD_LIMIT)
    DX_REAR_POLY = 0.0025 * pitch_cmd   # meters per degree



    dz_left  = -ROLL_GAIN * roll
    dz_right =  ROLL_GAIN * roll

    foot_targets = {
        "FL": ( 0.15,  0.05, BASE_Z),
        "FR": ( 0.15, -0.05, BASE_Z),

        # PUSH REAR FEET BACK WHEN NOSE DOWN
        "RR": (-0.15 - DX_REAR_POLY, -0.05, BASE_Z),
        "RL": (-0.15 - DX_REAR_POLY,  0.05, BASE_Z),
    }


    
    deltas = solve_all_legs(foot_targets)
    deltas = apply_joint_conventions(deltas)
    DELTA_LIMITS = {
        "COXA": 20.0,
        "THIGH": 50.0,
        "WRIST": 60.0,
    }

    for j in deltas:
        if "COXA" in j:
            deltas[j] = max(min(deltas[j], DELTA_LIMITS["COXA"]),
                            -DELTA_LIMITS["COXA"])
        elif "THIGH" in j:
            deltas[j] = max(min(deltas[j], DELTA_LIMITS["THIGH"]),
                            -DELTA_LIMITS["THIGH"])
        elif "WRIST" in j:
            deltas[j] = max(min(deltas[j], DELTA_LIMITS["WRIST"]),
                            -DELTA_LIMITS["WRIST"])

    physical = normalize_all(deltas)

    # ---------------------------------
    # POSTURE PITCH — JOINT SPACE ONLY
    # (DO NOT GO THROUGH IK)
    # ---------------------------------

    k = 1.5 * pitch_cmd   # degrees per degree

    # LEFT SIDE
    physical["FL_THIGH"] +=  0.6 * k
    physical["FL_WRIST"] -= 1.4 * k


    physical["RL_THIGH"] -=  k
    physical["RL_WRIST"] += 1.2 * k

    # RIGHT SIDE (mirrored mechanics)
    physical["FR_THIGH"] -= 0.6 * k
    physical["FR_WRIST"] += 1.4 * k

    physical["RR_THIGH"] +=  k
    physical["RR_WRIST"] -= 1.2 * k

    


    for joint, angle in physical.items():
        set_servo_angle(SERVO_CHANNELS[joint], angle)

if __name__ == "__main__":
    print("[L5] IMU posture control test — HOLD ROBOT")

    while True:
        posture_step()
        time.sleep(0.02)   # ~50 Hz
