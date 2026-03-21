"""
test_stability.py — Stability controller physical test
=======================================================

What this does:
  1. Inits hardware (IMU + PCA)
  2. Drives robot to stand pose
  3. Runs incremental stability loop at 50 Hz
  4. Prints roll / pitch / per-leg dz live to terminal
  5. Ctrl+C to exit and return to stand

Run:
    cd /home/a/quadruped/src/main
    python3 krish/test_stability.py

Phases to test physically:
  Phase 1 — POSTURE_ENABLED = False: confirm stand is stable, no drift
  Phase 2 — tilt robot by hand, watch dz values accumulate in terminal
  Phase 3 — POSTURE_ENABLED = True, confirm legs react correctly
  Phase 4 — snap back to level quickly, confirm no overshoot / jitter
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time

# --- Hardware ---
from hardware.imu import init_mpu, calibrate, IMUFilter
from hardware.pca9685 import init_pca, set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS

# --- Pipeline ---
from ik.solver import solve_all_legs
from joints.conventions import apply_joint_conventions
from joints.space import normalize_all

# --- Stability controller ---
# stability_controller.posture_step returns fully-processed servo angles.
# Send with send_physical() — do NOT pass through send_deltas().
from joints.stability_controller import posture_step, reset_reference, _dz


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
    """For bare IK output (no posture) — applies conventions + normalize."""
    conv = apply_joint_conventions(deltas)
    phys = normalize_all(conv)
    for joint, angle in phys.items():
        ch = CHANNELS.get(joint)
        if ch is not None:
            set_servo_angle(ch, angle)


def send_physical(physical: dict):
    """
    For posture_step output — angles are already fully processed.
    Do NOT call send_deltas on posture output; it would double-apply conventions.
    """
    for joint, angle in physical.items():
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
# Set PRINT_JOINTS = True to also print per-joint physical angles
POSTURE_ENABLED = True
PRINT_JOINTS    = False
DT = 0.02   # 50 Hz


# -------------------------------------------------
# Main
# -------------------------------------------------

def main():
    print("=" * 70)
    print("  STABILITY CONTROLLER PHYSICAL TEST")
    print("=" * 70)

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

    reset_reference()

    print()
    print(f"[RUN]  POSTURE_ENABLED = {POSTURE_ENABLED}")
    print(f"[RUN]  Loop rate       = {int(1/DT)} Hz")
    print("[RUN]  Ctrl+C to exit")
    print()
    print(f"{'t(s)':>7}  {'roll':>7}  {'pitch':>7}  {'dz_FL':>7}  {'dz_FR':>7}  {'dz_RL':>7}  {'dz_RR':>7}")
    print("-" * 70)

    t0 = time.time()

    try:
        while True:
            t_start = time.time()
            elapsed = t_start - t0

            if POSTURE_ENABLED:
                physical = posture_step(STAND_FEET, imu)
                send_physical(physical)
            else:
                deltas = solve_all_legs(STAND_FEET)
                send_deltas(deltas)

            # --- terminal printout ---
            # Re-read IMU for display (posture_step already consumed one reading)
            roll, pitch, _, _ = imu.update()

            print(
                f"{elapsed:7.2f}  {roll:+7.2f}  {pitch:+7.2f}"
                f"  {_dz['FL']:+7.4f}  {_dz['FR']:+7.4f}"
                f"  {_dz['RL']:+7.4f}  {_dz['RR']:+7.4f}",
                end="\r"
            )

            if PRINT_JOINTS and POSTURE_ENABLED:
                print()
                for j, v in sorted(physical.items()):
                    print(f"    {j:12s}: {v:6.1f}°")

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