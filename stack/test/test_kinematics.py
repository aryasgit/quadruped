"""Unit tests for barq1.kinematics — FK/IK consistency, symmetry, limits."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from barq1.geometry import HIP_LINK, LOWER_LEG, UPPER_LEG
from barq1.kinematics import (
    LEGS,
    SIDE,
    Unreachable,
    body_fk,
    body_ik,
    leg_fk,
    leg_ik,
    stance_feet_world,
    to_urdf_joints,
    within_limits,
)

TOL = 1e-9


def test_zero_pose():
    for side in (+1, -1):
        x, y, z = leg_fk(0, 0, 0, side)
        assert abs(x) < TOL
        assert abs(y - side * HIP_LINK) < TOL
        assert abs(z + (UPPER_LEG + LOWER_LEG)) < TOL
        q = leg_ik(x, y, z, side)
        assert all(abs(v) < 1e-6 for v in q)


def test_fk_ik_roundtrip_grid():
    n = checked = 0
    for side in (+1, -1):
        for x in (-0.12, -0.06, 0.0, 0.06, 0.12):
            for y in (-0.04, 0.0, 0.04, 0.08):
                for z in (-0.21, -0.17, -0.13, -0.09):
                    n += 1
                    try:
                        q = leg_ik(x, y * side, z, side)
                    except Unreachable:
                        continue
                    p = leg_fk(*q, side)
                    assert max(abs(p[0] - x), abs(p[1] - y * side), abs(p[2] - z)) < TOL
                    assert q[2] <= 0.1 + TOL  # knee-backward branch
                    checked += 1
    assert checked >= 100, f"only {checked}/{n} grid points reachable — workspace wrong?"


def test_unreachable_raises():
    with pytest.raises(Unreachable):
        leg_ik(0.0, HIP_LINK, -0.30, +1)  # beyond full leg extension
    with pytest.raises(Unreachable):
        leg_ik(0.0, 0.01, 0.0, +1)  # inside the hip-link circle


def test_stance_symmetry():
    h = 0.155
    angles = body_ik(stance_feet_world(h), body_xyz=(0, 0, h))
    for leg in LEGS:
        q1, q2, q3 = angles[leg]
        assert abs(q1) < 1e-9, f"{leg} coxa should be 0 in neutral stance"
        assert within_limits(q1, q2, q3)
    # all four legs identical in q2/q3
    q2s = {round(angles[leg][1], 12) for leg in LEGS}
    q3s = {round(angles[leg][2], 12) for leg in LEGS}
    assert len(q2s) == 1 and len(q3s) == 1
    # independent derivation (triangle / law of cosines, vertical reach R=h)
    alpha = math.acos((UPPER_LEG**2 + h**2 - LOWER_LEG**2) / (2 * UPPER_LEG * h))
    gamma = math.acos((UPPER_LEG**2 + LOWER_LEG**2 - h**2) / (2 * UPPER_LEG * LOWER_LEG))
    assert abs(angles["FL"][1] - alpha) < 1e-9
    assert abs(angles["FL"][2] + (math.pi - gamma)) < 1e-9


def test_body_pose_roundtrip():
    """body_ik then body_fk must reproduce the world foot targets exactly,
    across body translations and rotations."""
    feet = stance_feet_world(0.155)
    for xyz, rpy in [
        ((0, 0, 0.155), (0, 0, 0)),
        ((0.02, -0.01, 0.17), (0, 0, 0)),
        ((0, 0, 0.15), (0.12, 0, 0)),
        ((0, 0, 0.15), (0, -0.10, 0)),
        ((0.01, 0.01, 0.16), (0.08, 0.06, 0.10)),
    ]:
        angles = body_ik(feet, body_xyz=xyz, body_rpy=rpy)
        feet_back = body_fk(angles, body_xyz=xyz, body_rpy=rpy)
        for leg in LEGS:
            err = max(abs(feet_back[leg][i] - feet[leg][i]) for i in range(3))
            assert err < TOL, f"{leg} {xyz} {rpy}: {err}"


def test_stance_envelope_within_limits():
    """Heights and body shifts we intend to use must respect URDF limits."""
    for h in (0.12, 0.14, 0.155, 0.17, 0.19):
        for dx in (-0.03, 0.0, 0.03):
            for dy in (-0.03, 0.0, 0.03):
                angles = body_ik(stance_feet_world(h), body_xyz=(dx, dy, h))
                for leg in LEGS:
                    assert within_limits(*angles[leg]), (h, dx, dy, leg, angles[leg])


def test_deep_crouch_exceeds_thigh_limit():
    """Documents the workspace boundary: a 0.10 m crouch with a 4 cm body
    shift needs q2 > 1.548 rad (URDF thigh limit) — lie-down sequences must
    use foot x-offsets instead, as spotMicro's motion config does."""
    angles = body_ik(stance_feet_world(0.10), body_xyz=(0.04, 0.0, 0.10))
    assert not all(within_limits(*angles[leg]) for leg in LEGS)


def test_left_right_mirror():
    """Mirroring a target across y must mirror q1 and keep q2/q3."""
    qL = leg_ik(0.05, +0.09, -0.16, +1)
    qR = leg_ik(0.05, -0.09, -0.16, -1)
    assert abs(qL[0] + qR[0]) < TOL
    assert abs(qL[1] - qR[1]) < TOL
    assert abs(qL[2] - qR[2]) < TOL


def test_to_urdf_joints():
    angles = body_ik(stance_feet_world(0.155))
    j = to_urdf_joints(angles)
    assert len(j) == 12
    assert abs(j["front_left_shoulder"]) < 1e-9
    assert j["rear_right_foot"] == angles["RR"][2]
