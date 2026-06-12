"""Unit tests for barq1.servo_map with a synthetic calibration."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from barq1.servo_map import CalibrationError, ServoMap
from barq1.servos import SERVOS


def synthetic_calib():
    """Mid-window zero, nominal slope, sign alternating with mount."""
    cal = {}
    for n, s in SERVOS.items():
        cal[n] = {
            "zero_ticks": (s.mech_lo + s.mech_hi) / 2,
            "slope_ticks_per_deg": -1.59 if s.inverted else 1.59,
        }
    return cal


def test_zero_angle_maps_to_zero_ticks():
    m = ServoMap(synthetic_calib())
    for n, s in SERVOS.items():
        assert m.ticks_for(n, 0.0) == round((s.mech_lo + s.mech_hi) / 2)


def test_slope_direction_and_inverse_roundtrip():
    m = ServoMap(synthetic_calib())
    for n, s in SERVOS.items():
        q = math.radians(10)
        t = m.ticks_for(n, q)
        mid = (s.mech_lo + s.mech_hi) / 2
        assert (t > mid) == (not s.inverted)
        # inverse recovers the angle (within tick quantization)
        assert abs(math.degrees(m.angle_for(n, t)) - 10) < 0.5


def test_clamps_to_mech_window():
    m = ServoMap(synthetic_calib())
    for n, s in SERVOS.items():
        assert m.ticks_for(n, math.radians(+500)) in (s.mech_lo, s.mech_hi)
        assert m.ticks_for(n, math.radians(-500)) in (s.mech_lo, s.mech_hi)


def test_to_ticks_channels():
    m = ServoMap(synthetic_calib())
    ticks = m.to_ticks({leg: (0.0, 0.0, 0.0) for leg in ("FL", "FR", "RL", "RR")})
    assert len(ticks) == 12
    assert set(ticks) == {s.channel for s in SERVOS.values()}


def test_missing_servo_raises():
    cal = synthetic_calib()
    del cal["FL_coxa"]
    with pytest.raises(CalibrationError):
        ServoMap(cal)


def test_bad_slope_warns():
    cal = synthetic_calib()
    cal["FL_coxa"]["slope_ticks_per_deg"] = 0.1
    m = ServoMap(cal)
    assert any("FL_coxa" in w for w in m.warnings)
