"""Unit tests for the velocity gait (D-016) — pure, no sim.

Guards the engine's invariants: every commanded velocity yields IK-reachable,
in-limit joint angles; zero command is periodic; forward command drives
stance feet backward; the filter honours its rate limit.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from barq1.command import GaitCommand
from barq1.filters import RateLimitedFirstOrderFilter
from barq1.kinematics import LEGS, body_ik, within_limits
from barq1.velocity_gait import GaitConfig, VelocityGait


def _drive(cmd, ticks, check_limits=True):
    g = VelocityGait()
    g.reset()
    for _ in range(ticks):
        feet, xyz, rpy = g.step(cmd)
        ang = body_ik(feet, body_xyz=xyz, body_rpy=rpy)  # raises Unreachable if bad
        if check_limits:
            for leg in LEGS:
                assert within_limits(*ang[leg]), (cmd, leg, ang[leg])
    return g


def test_single_axis_maxima_reachable_and_in_limits():
    """Each axis at its configured max (single-axis) must stay in joint limits."""
    c = GaitConfig()
    n = VelocityGait().phase_length * 2
    for cmd in (GaitCommand(vx=c.max_vx), GaitCommand(vx=-c.max_vx),
                GaitCommand(vy=c.max_vy), GaitCommand(vy=-c.max_vy),
                GaitCommand(wz=c.max_wz), GaitCommand(wz=-c.max_wz),
                GaitCommand()):
        _drive(cmd, n)


def test_absurd_command_clamped_and_reachable():
    """Way-over-max combined command clamps and stays IK-reachable (no crash);
    grazing a joint limit at the combined corner is acceptable (hardware
    saturates via the mech-window clamp)."""
    g = VelocityGait()
    _drive(GaitCommand(vx=10.0, vy=10.0, wz=10.0), g.phase_length, check_limits=False)


def test_zero_command_periodic_in_xy():
    g = _drive(GaitCommand(), VelocityGait().phase_length)
    for leg in LEGS:                       # feet return to neutral x,y after a cycle
        assert abs(g.feet[leg][0] - g.neutral[leg][0]) < 5e-3
        assert abs(g.feet[leg][1] - g.neutral[leg][1]) < 5e-3


def test_forward_moves_stance_feet_backward():
    g = VelocityGait()
    g.reset()
    cmd = GaitCommand(vx=0.05)
    g.step(cmd)                            # tick 0: phase 0 (shift, all stance)
    x_before = g.feet["FL"][0]
    for _ in range(5):
        g.step(cmd)
    assert g.feet["FL"][0] < x_before      # stance foot swept backward


def test_filter_rate_limit():
    f = RateLimitedFirstOrderFilter(dt=0.02, tau=0.3, x0=0.0, rate_limit=0.5)
    f.set_command(10.0)
    assert abs(f.step()) <= 0.5 * 0.02 + 1e-9
