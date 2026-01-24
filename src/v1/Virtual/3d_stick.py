import math
import numpy as np
import matplotlib.pyplot as plt

# ==================================================
# CONFIG — CHANGE THESE FREELY
# ==================================================

# Servo angles in DEGREES (270° capable)
# Treat these as ABSOLUTE commands
SERVO = {
    # SHOULDERS (ab/ad)
    "SHOULDER_FL": 0.0,
    "SHOULDER_FR": 0.0,
    "SHOULDER_RL": -15.0,
    "SHOULDER_RR": 15.0,

    # MID LIMBS (hip pitch)
    "MID_FLM": 120.0,
    "MID_FRM": 120.0,
    "MID_RLM": 140.0,
    "MID_RRM": 140.0,

    # FEET (knee)
    "FOOT_FLF": -80.0,
    "FOOT_FRF": -80.0,
    "FOOT_RLF": -70.0,
    "FOOT_RRF": -70.0,
}

# ==================================================
# GEOMETRY (approximate but consistent)
# ==================================================
BODY_LENGTH = 0.20
BODY_WIDTH  = 0.12
BODY_HEIGHT = 0.06

L_SHOULDER = 0.00
L_THIGH    = 0.09
L_SHIN     = 0.09

DEG = math.pi / 180.0

# ==================================================
# LEG BASE POSITIONS
# ==================================================
BASE = {
    "FL": np.array([+BODY_LENGTH/2, +BODY_WIDTH/2, BODY_HEIGHT]),
    "FR": np.array([+BODY_LENGTH/2, -BODY_WIDTH/2, BODY_HEIGHT]),
    "RL": np.array([-BODY_LENGTH/2, +BODY_WIDTH/2, BODY_HEIGHT]),
    "RR": np.array([-BODY_LENGTH/2, -BODY_WIDTH/2, BODY_HEIGHT]),
}

# ==================================================
# KINEMATICS
# ==================================================
def leg_points(base, shoulder_deg, hip_deg, knee_deg):
    shoulder = shoulder_deg * DEG
    hip      = hip_deg * DEG
    knee     = knee_deg * DEG

    # Shoulder ab/ad (yaw)
    dir_xy = np.array([
        math.cos(shoulder),
        math.sin(shoulder),
        0.0
    ])

    hip_pt = base + dir_xy * L_SHOULDER

    # Hip pitch
    thigh = np.array([
        dir_xy[0] * math.cos(hip),
        dir_xy[1] * math.cos(hip),
        -math.sin(hip)
    ]) * L_THIGH

    knee_pt = hip_pt + thigh

    # Knee pitch
    shin = np.array([
        dir_xy[0] * math.cos(hip + knee),
        dir_xy[1] * math.cos(hip + knee),
        -math.sin(hip + knee)
    ]) * L_SHIN

    foot = knee_pt + shin
    return base, hip_pt, knee_pt, foot

# ==================================================
# DRAW
# ==================================================
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")

ax.set_xlim(-0.25, 0.25)
ax.set_ylim(-0.25, 0.25)
ax.set_zlim(-0.30, 0.10)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.view_init(elev=25, azim=-60)

# Draw body
body_pts = [
    BASE["FL"], BASE["FR"], BASE["RR"], BASE["RL"], BASE["FL"]
]

ax.plot(
    [p[0] for p in body_pts],
    [p[1] for p in body_pts],
    [p[2] for p in body_pts],
    "k-", linewidth=2
)

# Draw legs
for leg in ["FL", "FR", "RL", "RR"]:
    sh = SERVO[f"SHOULDER_{leg}"]
    mid = SERVO[f"MID_{leg}M"]
    knee = SERVO[f"FOOT_{leg}F"]

    p0, p1, p2, p3 = leg_points(BASE[leg], sh, mid, knee)

    ax.plot(
        [p0[0], p1[0], p2[0], p3[0]],
        [p0[1], p1[1], p2[1], p3[1]],
        [p0[2], p1[2], p2[2], p3[2]],
        linewidth=3
    )

    ax.scatter(p3[0], p3[1], p3[2], s=40)

plt.title("Quadruped Kinematic Sandbox\n(edit SERVO dict and re-run)")
plt.show()
