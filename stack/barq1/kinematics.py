"""
Layer 2 — KINEMATICS (leg FK/IK + body IK)
==========================================

Analytic kinematics for the BARQ v1 quadruped, mirroring the URDF
structure exactly (stack/urdf/barq_v1.urdf.xacro), in the spirit of the
spotMicro research (mike4192/spot_micro_kinematics).

Frames & conventions (right-handed, matching the URDF):
  body/base frame: x forward, y left, z up, origin mid-body at axle height.
  leg root: at the coxa (shoulder) joint; axes parallel to body axes.
  q1 = coxa  (URDF *_shoulder, rotation about +x)
  q2 = thigh (URDF *_leg,      rotation about +y)
  q3 = knee  (URDF *_foot,     rotation about +y, relative to thigh)
  q1=q2=q3=0  ->  leg straight down, foot at (0, side*HIP_LINK, -(L2+L3)).

The leg chain (from URDF): Rx(q1) -> translate (0, side*HIP_LINK, 0)
-> Ry(q2) -> translate (0,0,-UPPER_LEG) -> Ry(q3) -> translate (0,0,-LOWER_LEG).

Knee branch: q3 <= 0 (knee folds backward), matching the URDF foot joint
limit [-2.6, 0.1].

Pure python + math; no I/O, no state.
"""

import math

from barq1.geometry import BODY_LENGTH, BODY_WIDTH, HIP_LINK, UPPER_LEG, LOWER_LEG

LEGS = ("FL", "FR", "RL", "RR")

# +1 = left (hip link toward +y), -1 = right
SIDE = {"FL": +1, "FR": -1, "RL": +1, "RR": -1}

# Coxa joint position in the body frame (URDF shoulder joint origins)
LEG_ROOT = {
    "FL": (+BODY_LENGTH / 2, +BODY_WIDTH / 2, 0.0),
    "FR": (+BODY_LENGTH / 2, -BODY_WIDTH / 2, 0.0),
    "RL": (-BODY_LENGTH / 2, +BODY_WIDTH / 2, 0.0),
    "RR": (-BODY_LENGTH / 2, -BODY_WIDTH / 2, 0.0),
}

URDF_LEG_NAME = {"FL": "front_left", "FR": "front_right",
                 "RL": "rear_left", "RR": "rear_right"}

# URDF joint limits (rad): joint suffix -> (lower, upper)
LIMITS = {"shoulder": (-0.548, 0.548), "leg": (-2.666, 1.548), "foot": (-2.6, 0.1)}


class Unreachable(ValueError):
    """Foot target outside the leg workspace."""


# ---------------------------------------------------------------------------
# Single leg
# ---------------------------------------------------------------------------

def leg_fk(q1, q2, q3, side):
    """Joint angles -> foot position in the leg-root frame."""
    sx = -UPPER_LEG * math.sin(q2) - LOWER_LEG * math.sin(q2 + q3)
    sz = -UPPER_LEG * math.cos(q2) - LOWER_LEG * math.cos(q2 + q3)
    d = side * HIP_LINK
    return (
        sx,
        d * math.cos(q1) - sz * math.sin(q1),
        d * math.sin(q1) + sz * math.cos(q1),
    )


def leg_ik(x, y, z, side):
    """Foot position in the leg-root frame -> (q1, q2, q3).

    Raises Unreachable if the target is outside the workspace. Always
    returns the knee-backward branch (q3 <= 0), per the URDF limits.
    """
    d = side * HIP_LINK
    r_sq = y * y + z * z
    if r_sq < d * d:
        raise Unreachable(f"lateral target inside hip-link circle: {(x, y, z)}")
    sz = -math.sqrt(r_sq - d * d)          # leg plane reach (negative: below)
    q1 = math.atan2(z, y) - math.atan2(sz, d)

    cos_q3 = (x * x + sz * sz - UPPER_LEG**2 - LOWER_LEG**2) / (2 * UPPER_LEG * LOWER_LEG)
    if not -1.0 <= cos_q3 <= 1.0:
        raise Unreachable(f"target outside leg reach: {(x, y, z)}")
    q3 = -math.acos(cos_q3)

    u, v = -x, -sz
    q2 = math.atan2(u, v) - math.atan2(
        LOWER_LEG * math.sin(q3), UPPER_LEG + LOWER_LEG * math.cos(q3)
    )
    # normalize q1/q2 into (-pi, pi]
    q1 = math.atan2(math.sin(q1), math.cos(q1))
    q2 = math.atan2(math.sin(q2), math.cos(q2))
    return q1, q2, q3


def within_limits(q1, q2, q3, margin=0.0):
    """True if all three angles respect the URDF joint limits."""
    for q, key in ((q1, "shoulder"), (q2, "leg"), (q3, "foot")):
        lo, hi = LIMITS[key]
        if not lo + margin <= q <= hi - margin:
            return False
    return True


# ---------------------------------------------------------------------------
# Whole body
# ---------------------------------------------------------------------------

def _rot_rpy(roll, pitch, yaw):
    """Body rotation matrix R = Rz(yaw) @ Ry(pitch) @ Rx(roll) (rows)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _mat_t_vec(R, v):
    """R^T @ v (world -> body rotation)."""
    return tuple(R[0][i] * v[0] + R[1][i] * v[1] + R[2][i] * v[2] for i in range(3))


def _mat_vec(R, v):
    return tuple(sum(R[i][j] * v[j] for j in range(3)) for i in range(3))


def stance_feet_world(height, x_shift=0.0, y_shift=0.0):
    """Neutral stance: each foot on the ground directly under its thigh
    axis, with the body at `height`. Optional uniform body-frame shifts
    move every foot the same way (i.e. shift the body over the feet)."""
    feet = {}
    for leg in LEGS:
        rx, ry, _ = LEG_ROOT[leg]
        feet[leg] = (rx + x_shift, ry + SIDE[leg] * HIP_LINK + y_shift, 0.0)
    return feet


def body_ik(feet_world, body_xyz=(0.0, 0.0, 0.155), body_rpy=(0.0, 0.0, 0.0)):
    """World-frame foot targets + desired body pose -> joint angles.

    Returns {leg: (q1, q2, q3)}. Raises Unreachable if any leg cannot
    reach its target.
    """
    R = _rot_rpy(*body_rpy)
    angles = {}
    for leg, foot_w in feet_world.items():
        rel = tuple(foot_w[i] - body_xyz[i] for i in range(3))
        foot_b = _mat_t_vec(R, rel)
        root = LEG_ROOT[leg]
        local = tuple(foot_b[i] - root[i] for i in range(3))
        angles[leg] = leg_ik(*local, side=SIDE[leg])
    return angles


def body_fk(angles, body_xyz=(0.0, 0.0, 0.155), body_rpy=(0.0, 0.0, 0.0)):
    """Joint angles + body pose -> world-frame foot positions (verification)."""
    R = _rot_rpy(*body_rpy)
    feet = {}
    for leg, (q1, q2, q3) in angles.items():
        local = leg_fk(q1, q2, q3, SIDE[leg])
        root = LEG_ROOT[leg]
        foot_b = tuple(local[i] + root[i] for i in range(3))
        feet[leg] = tuple(_mat_vec(R, foot_b)[i] + body_xyz[i] for i in range(3))
    return feet


def to_urdf_joints(angles):
    """{leg: (q1,q2,q3)} -> {urdf_joint_name: angle} for sim/RViz."""
    out = {}
    for leg, (q1, q2, q3) in angles.items():
        n = URDF_LEG_NAME[leg]
        out[f"{n}_shoulder"] = q1
        out[f"{n}_leg"] = q2
        out[f"{n}_foot"] = q3
    return out
