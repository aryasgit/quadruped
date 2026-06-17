"""Unit tests for the teleop left-stick 4-quadrant mapping (drive.py)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from barq1.velocity_gait import GaitConfig
from teleop.drive import stick_to_velocity

C = GaitConfig()


def test_cardinal_directions():
    # full forward / back / left / right
    assert stick_to_velocity(0.0, 1.0, C) == (C.max_vx, 0.0)     # front
    assert stick_to_velocity(0.0, -1.0, C) == (-C.max_vx, 0.0)   # back
    assert stick_to_velocity(-1.0, 0.0, C) == (0.0, C.max_vy)    # left  (+vy)
    assert stick_to_velocity(1.0, 0.0, C) == (0.0, -C.max_vy)    # right (-vy)


def test_proportional_speed():
    # half push -> half speed
    vx, vy = stick_to_velocity(0.0, 0.5, C)
    assert abs(vx - 0.5 * C.max_vx) < 1e-9 and vy == 0.0
    # tiny push -> tiny speed
    vx, _ = stick_to_velocity(0.0, 0.1, C)
    assert abs(vx - 0.1 * C.max_vx) < 1e-9


def test_dominant_axis_is_cardinal():
    # mostly-forward with slight left -> pure forward (no diagonal)
    vx, vy = stick_to_velocity(0.3, 0.9, C)
    assert vy == 0.0 and vx > 0
    # mostly-left with slight forward -> pure strafe
    vx, vy = stick_to_velocity(-0.9, 0.3, C)
    assert vx == 0.0 and vy > 0


def test_zero_stick_is_zero():
    assert stick_to_velocity(0.0, 0.0, C) == (0.0, 0.0)
