"""
test_posture.py — Physical posture controller test
===================================================

What this does:
  1. Inits hardware (IMU + PCA)
  2. Drives robot to stand pose
  3. Runs posture loop at 50 Hz
  4. Prints roll/pitch/compensation live to terminal
  5. Ctrl+C to exit and return to stand

Run:
    cd <your project root>
    python test_posture.py

Phases to test physically:
  Phase 1 — gains=0: confirm stand is stable, no drift
  Phase 2 — tilt robot by hand, watch terminal values
  Phase 3 — enable gains, confirm legs react correctly
  Phase 4 — push harder, confirm clamping kicks in
"""

import time
import sys

# --- Hardware ---
from hardware.imu import init_mpu, calibrate, IMUFilter
from hardware.pca9685 import init_pca, set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS

# --- Pipeline ---
from ik.solver import solve_all_legs
from joints.conventions import apply_joint_conventions
from joints.space import normalize_all

# --- Posture controller ---
from joints.posture import posture_step, reset_reference


# -------------------------------------------------
# Servo channel map
# -------------------------------------------------

CHANNELS = {}
for _leg in ("FL", "FR", "RL", "RR"):
    CHANNELS[f"{_leg}_COXA"]  = COXA[_leg]
    CHANNELS[f"{_leg}_THIGH"] = THIGHS[f"T{_leg}"]
    CHANNELS[f"{_leg}_WRIST"] = WRISTS[f"W{_leg}"]


# -------------------------------------------------
# Nominal stand feet
# -------------------------------------------------

STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

STAND_FEET = {
    "FL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
    "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
}


# -------------------------------------------------
# Pipeline helpers
# -------------------------------------------------

def send_deltas(deltas: dict):
    conv = apply_joint_conventions(deltas)
    phys = normalize_all(conv)
    for joint, angle in phys.items():
        ch = CHANNELS.get(joint)
        if ch is not None:
            set_servo_angle(ch, angle)


def go_stand():
    deltas = solve_all_legs(STAND_FEET)
    send_deltas(deltas)


# -------------------------------------------------
# Test modes
# -------------------------------------------------

# Set POSTURE_ENABLED = False to test bare stand (zero gains)
# Set PRINT_JOINTS = True to also print per-joint deltas
POSTURE_ENABLED = True
PRINT_JOINTS    = False
DT = 0.02   # 50 Hz


# -------------------------------------------------
# Main
# -------------------------------------------------

def main():
    print("=" * 55)
    print("  POSTURE CONTROLLER PHYSICAL TEST")
    print("=" * 55)

    # --- init hardware ---
    print("[INIT] PCA9685...")
    init_pca()

    print("[INIT] MPU6050...")
    init_mpu()

    print("[INIT] Calibrating IMU — keep robot FLAT and STILL")
    calib = calibrate(samples=200)
    imu   = IMUFilter(calib)

    # --- drive to stand ---
    print("[INIT] Moving to stand pose...")
    go_stand()
    time.sleep(0.5)

    reset_reference()   # will lock on first posture_step call

    print()
    print(f"[RUN]  POSTURE_ENABLED = {POSTURE_ENABLED}")
    print(f"[RUN]  Loop rate = {int(1/DT)} Hz")
    print("[RUN]  Ctrl+C to exit")
    print()
    print(f"{'t(s)':>7}  {'roll':>7}  {'pitch':>7}  {'dz_FL':>7}  {'dz_FR':>7}  {'dz_RL':>7}  {'dz_RR':>7}")
    print("-" * 60)

    t0 = time.time()

    try:
        while True:
            t_start = time.time()
            elapsed = t_start - t0

            if POSTURE_ENABLED:
                deltas = posture_step(STAND_FEET, imu)
            else:
                deltas = solve_all_legs(STAND_FEET)

            send_deltas(deltas)

            # --- terminal printout ---
            roll, pitch, _, _ = imu.update()   # re-read for display (cheap)

            # estimate per-leg Z adjustments for display only
            import math
            rr = math.radians(roll)
            pr = math.radians(pitch)
            dz = {}
            for leg in ("FL", "FR", "RL", "RR"):
                from joints.posture_controller import LEG_X, LEG_Y, PITCH_GAIN, ROLL_GAIN
                dz[leg] = -PITCH_GAIN * LEG_X[leg] * math.sin(pr) \
                          - ROLL_GAIN  * LEG_Y[leg] * math.sin(rr)

            print(
                f"{elapsed:7.2f}  {roll:+7.2f}  {pitch:+7.2f}  "
                f"{dz['FL']:+7.4f}  {dz['FR']:+7.4f}  "
                f"{dz['RL']:+7.4f}  {dz['RR']:+7.4f}",
                end="\r"
            )

            if PRINT_JOINTS:
                print()
                for j, v in sorted(deltas.items()):
                    print(f"    {j:12s}: {v:+7.2f}°")

            # --- timing ---
            loop_t = time.time() - t_start
            sleep  = max(0.0, DT - loop_t)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n\n[EXIT] Returning to stand...")
        go_stand()
        print("[EXIT] Done.")


if __name__ == "__main__":
    main()