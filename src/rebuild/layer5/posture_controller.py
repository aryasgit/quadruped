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
from layer7.leg_fsm import LegFSM
from layer8.gait_phase import GaitPhase
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import WRISTS, THIGHS, COXA, BUS

leg_fsm = LegFSM()
gait = GaitPhase(swing_duration=0.6, lift_height=0.035)

foot_contact = {
    "FL": True,
    "FR": True,
    "RL": True,
    "RR": True,
}



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


def posture_step():
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
    # =========================
    # LAYER 8 — INJECT LIFT INTO DELTAS (PRE-NORMALIZATION)
    # =========================
    if leg_fsm.active_leg:
        dz = gait.lift()
        leg = leg_fsm.active_leg

        phase = gait.phase   # 0 → 1

        # PRIMARY vertical lift
        WRIST_LIFT_DEG = 30.0 * phase

        # VERY SMALL thigh bias to avoid singularity
        THIGH_LIFT_DEG = 6.0 * phase

        if leg == "FL":
            deltas["FL_THIGH"] -= THIGH_LIFT_DEG   # NOTE SIGN
            deltas["FL_WRIST"] += WRIST_LIFT_DEG

        elif leg == "FR":
            deltas["FR_THIGH"] += THIGH_LIFT_DEG   # NOTE SIGN
            deltas["FR_WRIST"] -= WRIST_LIFT_DEG

        elif leg == "RL":
            deltas["RL_THIGH"] -= THIGH_LIFT_DEG   # NOTE SIGN
            deltas["RL_WRIST"] += WRIST_LIFT_DEG

        elif leg == "RR":
            deltas["RR_THIGH"] += THIGH_LIFT_DEG   # NOTE SIGN
            deltas["RR_WRIST"] -= WRIST_LIFT_DEG


        if phase > 0.2:
            foot_contact[leg] = False
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
    # -------------------------------
    # HARD ZERO COXA FOR POSTURE MODE
    # -------------------------------
    physical["FL_COXA"] = 45.0
    physical["FR_COXA"] = 45.0
    physical["RL_COXA"] = 45.0
    physical["RR_COXA"] = 45.0

    # ---------------------------------
    # POSTURE PITCH — JOINT SPACE ONLY
    # (DO NOT GO THROUGH IK)
    # ---------------------------------

    k = 2 * pitch_cmd * min(1.0, abs(pitch_cmd) / 5.0)

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

    # ---------------------------------
    # POSTURE ROLL — JOINT SPACE ONLY
    # ---------------------------------
    kr= 1.5 * roll_cmd * min(1.0, abs(roll_cmd) / 5.0)    # conservative, safe

    REAR_SCALE = 1.0       # symmetric support polygon

    EXT_GAIN = 1.2
    CON_GAIN = 2.2    # <-- THIS is the fix
    WRIST_EXT = 1.6
    WRIST_CON = 2.1

    # roll_cmd > 0 → left side DOWN

    # LEFT SIDE (EXTEND)
    physical["FL_THIGH"] += EXT_GAIN * kr
    physical["FL_WRIST"] -= WRIST_EXT * kr

    physical["RL_THIGH"] += EXT_GAIN * REAR_SCALE * kr
    physical["RL_WRIST"] -= WRIST_EXT * REAR_SCALE * kr

    # RIGHT SIDE (CONTRACT HARDER)
    physical["FR_THIGH"] += CON_GAIN * kr
    physical["FR_WRIST"] -= WRIST_CON * kr

    physical["RR_THIGH"] += CON_GAIN * REAR_SCALE * kr
    physical["RR_WRIST"] -= WRIST_CON * REAR_SCALE * kr



    return physical




if __name__ == "__main__":
    print("[L5] IMU posture control test — HOLD ROBOT")

    while True:
        print("FSM active leg:", leg_fsm.active_leg)


        # =========================
        # LAYER 7 — LEG FSM
        # =========================
        leg_fsm.update(
            allow_leg_lift=True,     # forced ON for diagnostics
            foot_contact=foot_contact,
            swing_done=False,
            load_done=True,
        )

        # =========================
        # LAYER 8 — GAIT PHASE
        # =========================
        if leg_fsm.active_leg and gait._t0 is None:
            gait.start()

        phase, swing_done = gait.update()

        # =========================
        # LAYER 5 — POSTURE (AFTER GAIT UPDATE)
        # =========================
        physical = posture_step()
        if physical is None:
            time.sleep(0.02)
            continue

        # =========================
        # FSM COMPLETION
        # =========================
        if swing_done and leg_fsm.active_leg:
            foot_contact[leg_fsm.active_leg] = True
            gait.reset()

            leg_fsm.update(
                allow_leg_lift=True,
                foot_contact=foot_contact,
                swing_done=True,
                load_done=True,
            )
        
        # =========================
        # FINAL ACTUATION (ABSOLUTE)
        # =========================
        for joint, angle in physical.items():
            set_servo_angle(SERVO_CHANNELS[joint], angle)


        time.sleep(0.02)
