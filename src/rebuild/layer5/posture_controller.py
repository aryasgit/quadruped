#layer5/posture_controller.py
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
PITCH_KD = 0.03

_pitch_i = 0.0
_prev_pitch = 0.0
_pitch_filt = 0.0


PITCH_I_LIMIT = 12.0    # degrees-equivalent
PITCH_CMD_LIMIT = 20.0 # max virtual pitch command

# =====================
# ROLL PD CONTROLLER (NO INTEGRAL)
# =====================
ROLL_KP = 0.6
ROLL_KD = 0.03

_prev_roll = 0.0
ROLL_CMD_LIMIT = 20.0



# =====================
# GROUND PITCH ESTIMATOR
# =====================
pitch_ref = 0.0

PITCH_REF_ADAPT_RATE = 0.02   # deg/sec
PITCH_REF_MAX = 20.0          # safety

# =====================
# LAYER 6 INPUTS (AUTHORITATIVE)
# =====================
POSTURE_ENABLED = True
ALLOW_PITCH_REF_ADAPT = True

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


def posture_step(foot_targets):
    global pitch_ref, _pitch_i, _prev_pitch, _prev_roll
    if not POSTURE_ENABLED:
        return
    roll, pitch_meas, _, _ = imu.update()
    
    # ---------------------------
    # ROLL PD (NO INTEGRAL)
    # ---------------------------
    roll_err = -roll        # roll_ref = 0
    dt = 0.02

    roll_d = (roll_err - _prev_roll) / dt
    _prev_roll = roll_err

    roll_cmd = (
        ROLL_KP * roll_err +
        ROLL_KD * roll_d
    )

    roll_cmd = max(min(roll_cmd, ROLL_CMD_LIMIT), -ROLL_CMD_LIMIT)

    global _pitch_filt
    alpha = 0.85   # strong smoothing, still responsive
    _pitch_filt = alpha * _pitch_filt + (1 - alpha) * pitch_meas
    pitch_meas = _pitch_filt

    # ---------------------------
    # Ground pitch adaptation
    # ---------------------------
    pitch_error_raw = pitch_meas - pitch_ref

    if ALLOW_PITCH_REF_ADAPT:
        pitch_ref += PITCH_REF_ADAPT_RATE * pitch_error_raw * 0.02
        pitch_ref = max(min(pitch_ref, PITCH_REF_MAX), -PITCH_REF_MAX)


    # Final control error
    pitch_err = pitch_meas - pitch_ref

    dt = 0.02

    # Integrator
    if abs(pitch_err) > 0.8:   # degrees
        _pitch_i += 0.7*pitch_err * dt
        _pitch_i = max(min(_pitch_i, PITCH_I_LIMIT), -PITCH_I_LIMIT)


    # Derivative
    pitch_d = (pitch_err - _prev_pitch) / dt
    pitch_d = max(min(pitch_d, 40.0), -40.0)
    _prev_pitch = pitch_err

    # PID output
    pitch_cmd = (
        PITCH_KP * pitch_err +
        PITCH_KI * _pitch_i +
        PITCH_KD * pitch_d
    )


    pitch_cmd = max(min(pitch_cmd, PITCH_CMD_LIMIT), -PITCH_CMD_LIMIT)
    kp = 2.0 * pitch_cmd * min(1.0, abs(pitch_cmd) / 5.0)
    kr = 1.5 * roll_cmd  * min(1.0, abs(roll_cmd)  / 5.0)

    DX_REAR_POLY = 0.0025 * pitch_cmd   # meters per degree

    dz_left  = -ROLL_GAIN * roll
    dz_right =  ROLL_GAIN * roll

    
    deltas = solve_all_legs(foot_targets)
    deltas = apply_joint_conventions(deltas)

    # ================================
    # POSTURE — APPLY ONLY TO STANCE LEGS
    # ================================

    STANCE_LEGS = set(foot_targets.keys())

    # Detect swing leg (the one whose Z is changing)
    swing_leg = None
    for leg, (_, _, z) in foot_targets.items():
        if z > BASE_Z + 1e-4:
            swing_leg = leg
            break

    for leg in ("FL", "FR", "RL", "RR"):
        if leg == swing_leg:
            continue  # ❌ DO NOT TOUCH SWING LEG

        t = f"{leg}_THIGH"
        w = f"{leg}_WRIST"

        # Pitch posture
        deltas[t] += 0.6 * kp
        deltas[w] -= 1.4 * kp

        # Roll posture
        deltas[t] += 1.2 * kr
        deltas[w] -= 1.6 * kr

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

    # -------------------------------
    # COXA LOCK (STANCE)
    # -------------------------------
    COXA_STAND = 45.0
    for k in ("FL_COXA", "FR_COXA", "RL_COXA", "RR_COXA"):
        physical[k] = COXA_STAND
        

    return physical