import time
import math

from layer3.leg_ik import solve_leg_ik
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# ---------------- CONFIG ----------------

LEG = "FL"          # test ONE leg only
BASE_X = 0.00
BASE_Y = 0.06       # hip-local, left leg
BASE_Z = -0.18      # stance height

AMP_X = 0.1        # 2 cm forward/back
FREQ = 0.25         # Hz (slow, safe)

DT = 0.03           # 33 Hz update


# ---------------- SERVO MAP ----------------

CHANNELS = {
    "FL_COXA":  COXA["FL"],
    "FL_THIGH": THIGHS["TFL"],
    "FL_WRIST": WRISTS["WFL"],
}


# ---------------- MAIN LOOP ----------------

t = 0.0

print("Running SINGLE LEG X-AXIS TEST (CTRL+C to stop)")

try:
    while True:
        # Smooth X motion
        x = BASE_X + AMP_X * math.sin(2 * math.pi * FREQ * t)
        y = BASE_Y
        z = BASE_Z

        # ---- IK ----
        coxa, thigh, wrist = solve_leg_ik(x, y, z, LEG)

        deltas = {
            "FL_COXA":  coxa,
            "FL_THIGH": thigh,
            "FL_WRIST": wrist,
        }

        # ---- CONVENTIONS + NORMALIZE ----
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)

        # ---- SEND TO SERVOS ----
        for joint, ch in CHANNELS.items():
            set_servo_angle(ch, physical[joint])

        t += DT
        time.sleep(DT)

except KeyboardInterrupt:
    print("\nStopped.")
