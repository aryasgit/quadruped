"""
TEST — Layer 6 DIAGONAL ARC WALK (DROP-IN)
=========================================

Purpose:
- Use the SAME perfect arc generator
- Move only diagonal legs at a time
- Other diagonal stays planted
- No FSM
- No posture
- No IMU
- Minimal change from original demo

This is the SIMPLEST real gait.
"""

import time

# ---------------- Layer 6 ----------------
from layer6.gait_generator import _leg_trajectory

# ---------------- Layer 3 ----------------
from layer3.leg_ik import solve_all_legs

# ---------------- Layer 2 ----------------
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

# ---------------- Hardware ----------------
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# -------------------------------------------------
# CONFIG
# -------------------------------------------------

FREQ = 0.40            # gait speed (higher = faster)
DT = 0.03

STEP_LENGTH = 0.12    # SAFE for your geometry
STEP_HEIGHT = 0.030
DUTY = 0.70           # longer stance = stability

STANCE_X = 0.0
STANCE_Y = 0.065
STANCE_Z = -0.17

# -------------------------------------------------
# COXA PRELOAD (ANTI-SLIP)
# -------------------------------------------------

COXA_BIAS = {
    "FL": -3.0,   # toe in
    "FR": +3.0,   # toe in
    "RL": -3.0,   # toe out
    "RR": +3.0,   # toe out
}

# -------------------------------------------------
# Servo map
# -------------------------------------------------

CHANNELS = {
    "FL_COXA":  COXA["FL"],
    "FL_THIGH": THIGHS["TFL"],
    "FL_WRIST": WRISTS["WFL"],

    "FR_COXA":  COXA["FR"],
    "FR_THIGH": THIGHS["TFR"],
    "FR_WRIST": WRISTS["WFR"],

    "RL_COXA":  COXA["RL"],
    "RL_THIGH": THIGHS["TRL"],
    "RL_WRIST": WRISTS["WRL"],

    "RR_COXA":  COXA["RR"],
    "RR_THIGH": THIGHS["TRR"],
    "RR_WRIST": WRISTS["WRR"],
}

# -------------------------------------------------
# Diagonal groups
# -------------------------------------------------

DIAG_A = ("FL", "RR")
DIAG_B = ("FR", "RL")

# -------------------------------------------------
# MAIN
# -------------------------------------------------

print("[TEST] Layer 6 — DIAGONAL ARC WALK")
print("FL+RR → FR+RL → repeat")
print("CTRL+C to stop")

t0 = time.time()

try:
    while True:
        t = time.time() - t0
        cycle = (t * FREQ) % 1.0

        # Which diagonal is active?
        if cycle < 0.5:
            active = DIAG_A
            local_phase = cycle * 2.0     # 0 → 1
        else:
            active = DIAG_B
            local_phase = (cycle - 0.5) * 2.0

        dx, dz = _leg_trajectory(
            local_phase,
            STEP_LENGTH,
            STEP_HEIGHT,
            DUTY,
        )

        feet = {}

        for leg in ("FL", "FR", "RL", "RR"):
            y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y

            if leg in active:
                # swing diagonal
                feet[leg] = (
                    STANCE_X + dx,
                    y,
                    STANCE_Z + dz,
                )
            else:
                # stance diagonal (PLANTED)
                feet[leg] = (
                    STANCE_X,
                    y,
                    STANCE_Z,
                )

        deltas = solve_all_legs(feet)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)
        # Apply static COXA bias ONCE
        for leg, bias in COXA_BIAS.items():
            physical[f"{leg}_COXA"] += bias


        for joint, ch in CHANNELS.items():
            set_servo_angle(ch, physical[joint])

        time.sleep(DT)

except KeyboardInterrupt:
    print("\n[TEST] Stopped cleanly 🫡")
