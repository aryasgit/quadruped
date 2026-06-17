"""Unit tests for the controller FSM (D-017) — pure, no sim."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from barq1.command import GaitCommand
from barq1.controller import Controller, IDLE_HEIGHT
from barq1.kinematics import LEGS, body_ik, within_limits


def _run(ctrl, cmd, secs):
    n = int(round(secs / ctrl.dt))
    last = None
    for _ in range(n):
        feet, xyz, rpy = ctrl.step(cmd)
        ang = body_ik(feet, body_xyz=xyz, body_rpy=rpy)   # reachable?
        for leg in LEGS:
            assert within_limits(*ang[leg]), (cmd.state, leg, ang[leg])
        last = (feet, xyz, rpy)
    return last


def test_idle_lowers_then_stand_raises():
    c = Controller(start_state="stand")
    _, xyz, _ = _run(c, GaitCommand(state="idle"), 3.0)
    assert abs(xyz[2] - IDLE_HEIGHT) < 0.01          # reached idle height
    _, xyz, _ = _run(c, GaitCommand(state="stand"), 3.0)
    assert abs(xyz[2] - c.stand_h) < 0.01            # back to stand height


def test_stand_does_not_drift():
    c = Controller(start_state="stand")
    _run(c, GaitCommand(state="stand"), 1.0)         # settle
    feet, xyz, _ = _run(c, GaitCommand(state="stand"), 3.0)
    for leg in LEGS:                                 # feet pinned at neutral
        assert abs(feet[leg][0] - c.neutral[leg][0]) < 1e-6
        assert abs(feet[leg][1] - c.neutral[leg][1]) < 1e-6
    assert xyz[0] == 0.0 and xyz[1] == 0.0           # no body x/y drift


def test_walk_request_stands_first_then_cycles():
    c = Controller(start_state="idle")
    # from idle, request walk: must raise to stand height before the gait runs
    _run(c, GaitCommand(state="walk", vx=0.02), 0.3)
    assert c.state != "walk"                          # still rising, not walking
    _run(c, GaitCommand(state="walk", vx=0.02), 3.0)
    assert c.state == "walk"                          # now walking


def test_full_sequence_stays_reachable():
    c = Controller(start_state="stand")
    for cmd, secs in ((GaitCommand(state="stand", roll=0.1), 1.0),
                      (GaitCommand(state="walk", vx=0.024, wz=0.1), 3.0),
                      (GaitCommand(state="stand"), 1.0),
                      (GaitCommand(state="idle"), 2.0),
                      (GaitCommand(state="stand"), 2.0)):
        _run(c, cmd, secs)
