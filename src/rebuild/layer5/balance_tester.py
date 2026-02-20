"""
BALANCE TESTER
==============

Simple test script for the posture controller.
Robot will actively balance when tilted.

Press Ctrl+C to exit.
"""

import time

# ---------------- Layer 5 ----------------
from layer5.posture_controller import posture_step

# ---------------- Layer 2 ----------------
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

# ---------------- Hardware ----------------
from hardware.pca9685 import set_servo_angle
from hardware.imu import IMUFilter, init_mpu, calibrate
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# -------------------------------------------------
# CONFIG
# -------------------------------------------------

DT = 0.02  # 50Hz loop

# Stance values
STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

# Coxa preload
COXA_DELTA_BIAS = {
    "FL": +1.5,
    "FR": +1.5,
    "RL": +1.5,
    "RR": +1.5,
}


# -------------------------------------------------
# Servo channel map
# -------------------------------------------------

CHANNELS = {}
for _leg in ("FL", "FR", "RL", "RR"):
    CHANNELS[f"{_leg}_COXA"] = COXA[_leg]
    CHANNELS[f"{_leg}_THIGH"] = THIGHS[f"T{_leg}"]
    CHANNELS[f"{_leg}_WRIST"] = WRISTS[f"W{_leg}"]


# -------------------------------------------------
# Helpers
# -------------------------------------------------

def get_stand_feet():
    return {
        "FL": (STANCE_X,  STANCE_Y, STANCE_Z),
        "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
        "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
        "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
    }


def apply_coxa_bias(deltas: dict) -> dict:
    biased = deltas.copy()
    for leg, bias in COXA_DELTA_BIAS.items():
        key = f"{leg}_COXA"
        if key in biased:
            biased[key] += bias
    return biased


def send_to_servos(physical: dict):
    for joint, ch in CHANNELS.items():
        set_servo_angle(ch, physical[joint])


def execute_step_with_imu(feet: dict, imu: IMUFilter) -> bool:
    """Execute IK pipeline with IMU posture control."""
    try:
        deltas = posture_step(feet, imu)
        deltas = apply_coxa_bias(deltas)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)
        send_to_servos(physical)
        return True
    except Exception as e:
        print(f"\n[WARN] Pipeline error: {e}")
        return False


# -------------------------------------------------
# MAIN
# -------------------------------------------------

def main():
    print("=" * 50)
    print("  BALANCE TESTER")
    print("=" * 50)
    print()
    print("This script tests the IMU-based posture controller.")
    print("The robot will actively balance when tilted.")
    print()
    print("Press Ctrl+C to exit.")
    print("-" * 50)
    
    # Initialize IMU
    print("[INIT] Starting IMU...")
    init_mpu()
    print("[INIT] Calibrating IMU (keep robot still)...")
    calib = calibrate()
    imu = IMUFilter(calib)
    time.sleep(0.3)
    
    # Prime the IMU reference
    print("[INIT] Setting reference pose...")
    feet = get_stand_feet()
    for _ in range(10):
        execute_step_with_imu(feet, imu)
        time.sleep(DT)
    
    print("[INIT] Ready! Balancing active.")
    print("-" * 50)
    print()
    
    loop_count = 0
    
    try:
        while True:
            loop_start = time.time()
            
            # Run balance step
            feet = get_stand_feet()
            execute_step_with_imu(feet, imu)
            
            # Display IMU values (every 25 loops = 0.5 sec)
            loop_count += 1
            if loop_count % 25 == 0:
                r, p, rr, pr = imu.roll, imu.pitch, imu.roll_rate, imu.pitch_rate
                print(f"\r[BALANCING] roll={r:+6.2f}° pitch={p:+6.2f}° | rr={rr:+5.1f} pr={pr:+5.1f}", 
                      end="", flush=True)
            
            # Timing
            elapsed = time.time() - loop_start
            sleep_time = max(0, DT - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\n[EXIT] Caught Ctrl+C")
    
    finally:
        print("[SHUTDOWN] Done")


if __name__ == "__main__":
    main()
