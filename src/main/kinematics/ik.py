"""
L3 — INVERSE KINEMATICS (pure)
==============================

Foot target (hip-local x,y,z, meters) -> joint angles as DELTAS from stand
(degrees). Same 3-DOF SpotMicro model and angle conventions as the verified
old stack, reimplemented without numpy and fully domain-clamped so it can
never crash the control loop.

No hardware. No printing. No global state beyond the once-computed stand
reference. Unit-testable.

Contract:
    solve_leg(x, y, z, leg)  -> (coxaΔ, thighΔ, wristΔ) degrees, + Reach status
    solve_all(foot_targets)  -> {joint_name: Δdeg}
"""

import math
from config.robot_spec import (
    LINK_COXA, LINK_THIGH, LINK_WRIST, PHI,
    LEG_ID, RIGHT_LEGS, LEGS, stance_foot,
)

_L1, _L2, _L3 = LINK_COXA, LINK_THIGH, LINK_WRIST
_REACH = _L2 + _L3


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _point_to_rad(p1, p2):
    """Angle from +p1 toward +p2 in [0, 2pi)."""
    return math.atan2(p2, p1) % (2 * math.pi)


def _angle_corrector(t1, t2, t3, is_right):
    """Robot-specific offsets — verbatim equivalent of the old angle_corrector."""
    a1, a2, a3 = t1, t2, t3
    a2 -= 1.5 * math.pi
    if is_right:
        theta_1 = a1 - math.pi
        theta_2 = a2 + math.radians(45)
    else:
        theta_1 = a1 - 2 * math.pi if a1 > math.pi else a1
        theta_2 = -a2 - math.radians(45)
    theta_3 = -a3 + math.radians(45)
    return theta_1, theta_2, theta_3


def _leg_ik_calc(x, y, z, is_right):
    """
    Closed-form 3-DOF solve in the leg's local frame.
    Returns (theta1, theta2, theta3) radians (post angle-correction), and a
    boolean `reachable` (False if the target had to be clamped to the workspace).
    """
    reachable = True

    # ---- coxa (theta_1) : rotation about x in the YZ plane ----
    len_A = math.hypot(y, z)
    len_A = max(len_A, _L1 + 1e-6)
    a_1 = _point_to_rad(y, z)
    ratio = _clamp((math.sin(PHI) * _L1) / max(len_A, 1e-6), -1.0, 1.0)
    a_2 = math.asin(ratio)
    a_3 = math.pi - a_2 - PHI

    if is_right:
        theta_1 = a_1 - a_3
    else:
        theta_1 = a_1 + a_3
        if theta_1 >= 2 * math.pi:
            theta_1 -= 2 * math.pi

    # ---- project into the leg plane (rotate by -R about x) ----
    j2y = _L1 * math.cos(theta_1)
    j2z = _L1 * math.sin(theta_1)
    vx, vy, vz = x - 0.0, y - j2y, z - j2z

    R = (theta_1 - PHI - math.pi / 2) if is_right else (theta_1 + PHI - math.pi / 2)
    cR, sR = math.cos(R), math.sin(R)
    # RotMatrix3D([-R,0,0]) * v  ->  rotation about x by -R
    x_ = vx
    z_ = -sR * vy + cR * vz

    len_B = math.hypot(x_, z_)
    len_B = max(len_B, 1e-6)
    if len_B >= _REACH:
        len_B = _REACH * 0.99999
        reachable = False

    # ---- thigh / wrist (theta_2, theta_3) ----
    b_1 = _point_to_rad(x_, z_)
    b_2 = math.acos(_clamp((_L2 ** 2 + len_B ** 2 - _L3 ** 2) / (2 * _L2 * len_B), -1.0, 1.0))
    b_3 = math.acos(_clamp((_L2 ** 2 + _L3 ** 2 - len_B ** 2) / (2 * _L2 * _L3), -1.0, 1.0))

    theta_2 = b_1 - b_2
    theta_3 = math.pi - b_3

    t1, t2, t3 = _angle_corrector(theta_1, theta_2, theta_3, is_right)
    return t1, t2, t3, reachable


# ---------------------------------------------------------------------
# Stand reference (computed once)
# ---------------------------------------------------------------------
_STAND_REF = {}
for _leg in LEGS:
    _sx, _sy, _sz = stance_foot(_leg)
    _t1, _t2, _t3, _ = _leg_ik_calc(_sx, _sy, _sz, _leg in RIGHT_LEGS)
    _STAND_REF[_leg] = (math.degrees(_t1), math.degrees(_t2), math.degrees(_t3))


def solve_leg(x, y, z, leg):
    """
    Returns ((coxaΔ, thighΔ, wristΔ) in degrees, reachable: bool).
    Deltas are relative to the stand pose.
    """
    t1, t2, t3, reachable = _leg_ik_calc(x, y, z, leg in RIGHT_LEGS)
    ref = _STAND_REF[leg]
    delta = (
        math.degrees(t1) - ref[0],
        math.degrees(t2) - ref[1],
        math.degrees(t3) - ref[2],
    )
    return delta, reachable


def solve_all(foot_targets):
    """
    foot_targets: {leg: (x, y, z)} -> ({joint_name: Δdeg}, all_reachable: bool)
    """
    out = {}
    all_reachable = True
    for leg, (x, y, z) in foot_targets.items():
        (c, t, w), reachable = solve_leg(x, y, z, leg)
        out[f"{leg}_COXA"] = c
        out[f"{leg}_THIGH"] = t
        out[f"{leg}_WRIST"] = w
        all_reachable = all_reachable and reachable
    return out, all_reachable
