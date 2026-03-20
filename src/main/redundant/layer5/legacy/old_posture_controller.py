# layer5/posture_controller.py

import time

from hardware.imu import init_mpu, calibrate, IMUFilter

init_mpu()
calib = calibrate()
imu = IMUFilter(calib)

from IK.leg_ik import solve_all_legs
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import WRISTS, THIGHS, COXA, COXA_STAND as COXA_STAND_MAP

# =====================
# PITCH PID CONTROLLER
# =====================
PITCH_KP = 0.6
PITCH_KI = 0.8
PITCH_KD = 0.03

_pitch_i    = 0.0
_prev_pitch = 0.0
_pitch_filt = 0.0

PITCH_I_LIMIT   = 12.0
PITCH_CMD_LIMIT = 20.0

# =====================
# ROLL PD CONTROLLER
# =====================
ROLL_KP = 0.6
ROLL_KD = 0.03

_prev_roll     = 0.0
ROLL_CMD_LIMIT = 20.0

# =====================
# GROUND PITCH ESTIMATOR
# =====================
pitch_ref = 0.0

PITCH_REF_ADAPT_RATE = 0.02
PITCH_REF_MAX        = 20.0

# =====================
# LAYER 6 INPUTS
# =====================
POSTURE_ENABLED       = True
ALLOW_PITCH_REF_ADAPT = True

SERVO_CHANNELS = {
    "FL_COXA":  COXA["FL"],    "FL_THIGH": THIGHS["TFL"], "FL_WRIST": WRISTS["WFL"],
    "FR_COXA":  COXA["FR"],    "FR_THIGH": THIGHS["TFR"], "FR_WRIST": WRISTS["WFR"],
    "RR_COXA":  COXA["RR"],    "RR_THIGH": THIGHS["TRR"], "RR_WRIST": WRISTS["WRR"],
    "RL_COXA":  COXA["RL"],    "RL_THIGH": THIGHS["TRL"], "RL_WRIST": WRISTS["WRL"],
}

BASE_Z = -0.18

DELTA_LIMITS = {
    "COXA":  20.0,
    "THIGH": 50.0,
    "WRIST": 60.0,
}

# -------------------------------------------------------
# Which legs are on the right side (roll sign is negated)
# -------------------------------------------------------
RIGHT_LEGS = {"FR", "RR"}


def posture_step(foot_targets: dict) -> dict:
    global pitch_ref, _pitch_i, _prev_pitch, _prev_roll, _pitch_filt

    if not POSTURE_ENABLED:
        return

    roll, pitch_meas, _, _ = imu.update()
    dt = 0.02

    # ------------------------------------------------------------------
    # ROLL PD  — error is just -roll (we want roll == 0)
    # ------------------------------------------------------------------
    roll_err = -roll
    roll_d   = (roll_err - _prev_roll) / dt
    _prev_roll = roll_err

    roll_cmd = ROLL_KP * roll_err + ROLL_KD * roll_d
    roll_cmd = max(min(roll_cmd, ROLL_CMD_LIMIT), -ROLL_CMD_LIMIT)

    # ------------------------------------------------------------------
    # PITCH — low-pass filter
    # ------------------------------------------------------------------
    alpha       = 0.85
    _pitch_filt = alpha * _pitch_filt + (1.0 - alpha) * pitch_meas
    pitch_meas  = _pitch_filt

    # ------------------------------------------------------------------
    # Ground pitch adaptation
    # ------------------------------------------------------------------
    pitch_error_raw = pitch_meas - pitch_ref

    if ALLOW_PITCH_REF_ADAPT:
        pitch_ref += PITCH_REF_ADAPT_RATE * pitch_error_raw * dt
        pitch_ref  = max(min(pitch_ref, PITCH_REF_MAX), -PITCH_REF_MAX)

    # ------------------------------------------------------------------
    # PITCH PID
    # ------------------------------------------------------------------
    pitch_err = pitch_meas - pitch_ref

    if abs(pitch_err) > 0.8:
        _pitch_i += 0.7 * pitch_err * dt
        _pitch_i  = max(min(_pitch_i, PITCH_I_LIMIT), -PITCH_I_LIMIT)

    pitch_d    = (pitch_err - _prev_pitch) / dt
    pitch_d    = max(min(pitch_d, 40.0), -40.0)
    _prev_pitch = pitch_err

    pitch_cmd = (
        PITCH_KP * pitch_err +
        PITCH_KI * _pitch_i  +
        PITCH_KD * pitch_d
    )
    pitch_cmd = max(min(pitch_cmd, PITCH_CMD_LIMIT), -PITCH_CMD_LIMIT)

    # ------------------------------------------------------------------
    # FIX 3: Linear gains — no quadratic deadband
    # kp / kr are now just the clamped PID outputs directly.
    # ------------------------------------------------------------------
    kp = pitch_cmd   # degrees of joint correction per degree of pitch error
    kr = roll_cmd    # degrees of joint correction per degree of roll error

    # ------------------------------------------------------------------
    # IK baseline for current foot targets
    # ------------------------------------------------------------------
    deltas = solve_all_legs(foot_targets)
    deltas = apply_joint_conventions(deltas)

    # ------------------------------------------------------------------
    # Detect swing leg
    # ------------------------------------------------------------------
    swing_leg = None
    for leg, (_, _, z) in foot_targets.items():
        if z > BASE_Z + 1e-4:
            swing_leg = leg
            break

    # ------------------------------------------------------------------
    # FIX 1: Apply roll correction with OPPOSITE signs per side.
    #
    # Convention:
    #   Positive roll_cmd means body is tilting right → left legs must
    #   extend (more thigh/wrist extension pushes body up on that side)
    #   and right legs must retract.
    #
    #   Left  legs: thigh += kr,  wrist -= kr   (extends leg → raises body)
    #   Right legs: thigh -= kr,  wrist += kr   (retracts leg → raises body)
    # ------------------------------------------------------------------
    for leg in ("FL", "FR", "RL", "RR"):
        if leg == swing_leg:
            continue

        t    = f"{leg}_THIGH"
        w    = f"{leg}_WRIST"
        sign = -1.0 if leg in RIGHT_LEGS else 1.0

        # Pitch correction (same direction for all legs)
        deltas[t] += 0.6 * kp
        deltas[w] -= 1.4 * kp

        # FIX 1: Roll correction — opposite sides get opposite signs
        deltas[t] += sign * 1.2 * kr
        deltas[w] -= sign * 1.6 * kr

    # ------------------------------------------------------------------
    # Clamp deltas
    # ------------------------------------------------------------------
    for j in deltas:
        if "COXA" in j:
            lim = DELTA_LIMITS["COXA"]
        elif "THIGH" in j:
            lim = DELTA_LIMITS["THIGH"]
        else:
            lim = DELTA_LIMITS["WRIST"]
        deltas[j] = max(min(deltas[j], lim), -lim)

    physical = normalize_all(deltas)

    # ------------------------------------------------------------------
    # FIX 4: Lock coxas to their per-leg calibrated stand angles
    #         (not a single hardcoded 45°)
    # ------------------------------------------------------------------
    for leg_key, stand_angle in COXA_STAND_MAP.items():
        joint_key = f"{leg_key}_COXA"
        if joint_key in physical:
            physical[joint_key] = stand_angle

    return physical