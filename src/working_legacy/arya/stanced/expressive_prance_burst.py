"""
High-impact expressive demo stance/trick for the Arya stack.

Motion: ExpressivePranceBurst
- Anticipation crouch + breathing preload
- Aggressive hip-driven sway burst (coxa personality)
- Weight shift then front-right paw tap/stomp sequence
- Curiosity tilt hold with micro-corrections
- Controlled damped recovery to neutral
"""

from __future__ import annotations

import math
import time
from typing import Callable, Dict, Tuple

from ik.solver import solve_all_legs, _STAND_Y, _STAND_Z
from joints.conventions import apply_joint_conventions
from joints.space import normalize_all
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS

Feet = Dict[str, Tuple[float, float, float]]


class ExpressivePranceBurst:
    """Lifelike, high-energy demo trick with strong coxa expressiveness."""

    DT = 0.02  # 50 Hz

    SERVO_MAP = {
        "FL_COXA": COXA["FL"], "FL_THIGH": THIGHS["TFL"], "FL_WRIST": WRISTS["WFL"],
        "FR_COXA": COXA["FR"], "FR_THIGH": THIGHS["TFR"], "FR_WRIST": WRISTS["WFR"],
        "RL_COXA": COXA["RL"], "RL_THIGH": THIGHS["TRL"], "RL_WRIST": WRISTS["WRL"],
        "RR_COXA": COXA["RR"], "RR_THIGH": THIGHS["TRR"], "RR_WRIST": WRISTS["WRR"],
    }

    def execute(self, state=None, actuators=None):
        neutral = self._neutral_feet()

        # 1) Build-up / anticipation (slow, elastic preload)
        self._segment(0.70, lambda p: self._anticipation_pose(neutral, p), state, actuators)

        # 2) Peak action burst (fast hip sway + asymmetry)
        self._segment(1.60, lambda p: self._sway_burst_pose(neutral, p), state, actuators)

        # 3) Shift CoM left before paw lift (stability-first transition)
        shifted_left = self._weight_shift_left(neutral)
        self._transition(self._sway_burst_pose(neutral, 1.0), shifted_left, 0.45, state, actuators)

        # 4) Front-right paw rapid taps (3 grounded legs + dynamic body compensation)
        self._segment(1.25, lambda p: self._paw_tap_pose(shifted_left, p), state, actuators)

        # 5) Curiosity tilt hold with tiny alive corrections
        self._segment(0.90, lambda p: self._curiosity_hold_pose(shifted_left, p), state, actuators)

        # 6) Recovery with damped oscillation back to stand
        self._segment(0.85, lambda p: self._recovery_pose(neutral, p), state, actuators)
        self._send_feet(neutral, actuators)

    # ---------- pose generators ----------
    @staticmethod
    def _neutral_feet() -> Feet:
        return {
            "FL": (0.0, _STAND_Y, _STAND_Z),
            "FR": (0.0, -_STAND_Y, _STAND_Z),
            "RL": (0.0, _STAND_Y, _STAND_Z),
            "RR": (0.0, -_STAND_Y, _STAND_Z),
        }

    def _anticipation_pose(self, base: Feet, p: float) -> Feet:
        e = self._ease_in_out(p)
        breath = 0.004 * math.sin(2.0 * math.pi * (1.15 * p))

        feet = dict(base)
        # Down + slight rear load (pre-jump illusion without instability)
        z_drop = -0.012 * e + breath
        front_pull = -0.015 * e
        rear_push = 0.013 * e

        feet["FL"] = (front_pull, _STAND_Y + 0.004, _STAND_Z + z_drop)
        feet["FR"] = (front_pull + 0.003, -_STAND_Y + 0.002, _STAND_Z + z_drop + 0.001)
        feet["RL"] = (rear_push, _STAND_Y + 0.006, _STAND_Z + z_drop - 0.003)
        feet["RR"] = (rear_push - 0.002, -_STAND_Y + 0.004, _STAND_Z + z_drop - 0.002)
        return feet

    def _sway_burst_pose(self, base: Feet, p: float) -> Feet:
        # Build-up -> strong burst -> partial decay
        env = self._burst_envelope(p)
        phase = 2.0 * math.pi * (2.2 * p)

        sway_y = 0.020 * env * math.sin(phase)
        twist_x = 0.028 * env * math.sin(phase + 0.45)  # strong coxa-driven character
        roll_z = 0.008 * env * math.sin(phase + math.pi / 2)

        feet = dict(base)
        feet["FL"] = (+twist_x * 0.7, _STAND_Y + sway_y + 0.003, _STAND_Z + roll_z)
        feet["FR"] = (+twist_x * 0.9, -_STAND_Y + sway_y * 0.85, _STAND_Z - roll_z * 0.8)
        feet["RL"] = (-twist_x, _STAND_Y - sway_y * 0.95 + 0.004, _STAND_Z - roll_z)
        feet["RR"] = (-twist_x * 0.8, -_STAND_Y - sway_y + 0.002, _STAND_Z + roll_z * 0.9)
        return feet

    @staticmethod
    def _weight_shift_left(base: Feet) -> Feet:
        return {
            "FL": (0.006, _STAND_Y + 0.018, _STAND_Z - 0.006),
            "FR": (0.010, -_STAND_Y + 0.024, _STAND_Z - 0.001),
            "RL": (0.012, _STAND_Y + 0.015, _STAND_Z - 0.010),
            "RR": (0.008, -_STAND_Y + 0.020, _STAND_Z - 0.005),
        }

    def _paw_tap_pose(self, shifted: Feet, p: float) -> Feet:
        feet = dict(shifted)

        # 4 taps with non-linear sharp attack / smooth release
        taps = 4.0
        cyc = (p * taps) % 1.0
        lift = self._tap_profile(cyc)

        # Support tripod micro compensation (alive wobble correction)
        comp = 0.005 * math.sin(2.0 * math.pi * (p * taps + 0.2))
        feet["FL"] = (shifted["FL"][0] + comp, shifted["FL"][1] + 0.002, shifted["FL"][2] - 0.003)
        feet["RL"] = (shifted["RL"][0] + comp * 0.7, shifted["RL"][1] + 0.001, shifted["RL"][2] - 0.004)
        feet["RR"] = (shifted["RR"][0] - comp * 0.6, shifted["RR"][1], shifted["RR"][2] - 0.002)

        # Paw tap: quick up + quick down (appears energetic, stays safe)
        fr_x = shifted["FR"][0] + 0.028 + 0.008 * math.sin(2.0 * math.pi * p)
        fr_y = shifted["FR"][1] - 0.006
        fr_z = shifted["FR"][2] + 0.050 * lift  # lift toward body (less negative z)
        feet["FR"] = (fr_x, fr_y, fr_z)
        return feet

    def _curiosity_hold_pose(self, shifted: Feet, p: float) -> Feet:
        # Asymmetric tilt + tiny autonomous corrections
        d = 0.003 * math.sin(2.0 * math.pi * 2.4 * p)
        d2 = 0.002 * math.sin(2.0 * math.pi * 3.1 * p + 0.8)
        return {
            "FL": (shifted["FL"][0] + 0.012, shifted["FL"][1] + 0.007, shifted["FL"][2] - 0.004 + d),
            "FR": (shifted["FR"][0] + 0.030, shifted["FR"][1] - 0.009, shifted["FR"][2] + 0.018 + d2),
            "RL": (shifted["RL"][0] - 0.010, shifted["RL"][1] + 0.004, shifted["RL"][2] - 0.006 - d2),
            "RR": (shifted["RR"][0] - 0.006, shifted["RR"][1] - 0.002, shifted["RR"][2] - 0.002 - d),
        }

    def _recovery_pose(self, neutral: Feet, p: float) -> Feet:
        e = self._ease_out(p)
        damp = math.exp(-3.0 * p)
        osc = 0.008 * damp * math.sin(2.0 * math.pi * 3.0 * p)

        feet = dict(neutral)
        feet["FL"] = (osc * 0.8 * (1.0 - e), _STAND_Y + osc * 0.6, _STAND_Z - 0.004 * (1.0 - e))
        feet["FR"] = (osc * 0.5 * (1.0 - e), -_STAND_Y + osc * 0.4, _STAND_Z + 0.002 * (1.0 - e))
        feet["RL"] = (-osc * 0.9 * (1.0 - e), _STAND_Y - osc * 0.5, _STAND_Z - 0.006 * (1.0 - e))
        feet["RR"] = (-osc * 0.7 * (1.0 - e), -_STAND_Y - osc * 0.3, _STAND_Z - 0.003 * (1.0 - e))
        return feet

    # ---------- execution helpers ----------
    def _segment(self, duration: float, pose_fn: Callable[[float], Feet], state, actuators):
        steps = max(2, int(duration / self.DT))
        for i in range(steps + 1):
            if self._should_abort(state):
                return
            p = i / steps
            self._send_feet(pose_fn(p), actuators)
            time.sleep(self.DT)

    def _transition(self, a: Feet, b: Feet, duration: float, state, actuators):
        steps = max(2, int(duration / self.DT))
        for i in range(steps + 1):
            if self._should_abort(state):
                return
            t = self._ease_in_out(i / steps)
            pose = {}
            for leg in a:
                ax, ay, az = a[leg]
                bx, by, bz = b[leg]
                pose[leg] = (
                    ax + (bx - ax) * t,
                    ay + (by - ay) * t,
                    az + (bz - az) * t,
                )
            self._send_feet(pose, actuators)
            time.sleep(self.DT)

    def _send_feet(self, feet: Feet, actuators):
        deltas = solve_all_legs(feet)
        conv = apply_joint_conventions(deltas)
        physical = normalize_all(conv)

        # Preserve control-stack separation: use actuator abstraction if available.
        if actuators is not None:
            if hasattr(actuators, "apply_joint_angles"):
                actuators.apply_joint_angles(physical)
                return
            if hasattr(actuators, "set_joint_angles"):
                actuators.set_joint_angles(physical)
                return
            if hasattr(actuators, "write"):
                actuators.write(physical)
                return

        for joint, angle in physical.items():
            channel = self.SERVO_MAP.get(joint)
            if channel is not None:
                set_servo_angle(channel, angle)

    @staticmethod
    def _should_abort(state) -> bool:
        if state is None:
            return False
        return bool(getattr(state, "stop_requested", False) or getattr(state, "abort", False))

    @staticmethod
    def _ease_in_out(t: float) -> float:
        t = max(0.0, min(1.0, t))
        return t * t * (3.0 - 2.0 * t)

    @staticmethod
    def _ease_out(t: float) -> float:
        t = max(0.0, min(1.0, t))
        return 1.0 - (1.0 - t) ** 3

    @staticmethod
    def _burst_envelope(t: float) -> float:
        # Quick rise (anticipation release), strong middle, soft fall.
        rise = 1.0 / (1.0 + math.exp(-18.0 * (t - 0.18)))
        decay = math.exp(-0.65 * max(0.0, t - 0.55) * 4.0)
        return max(0.0, min(1.0, rise * decay))

    @staticmethod
    def _tap_profile(cycle_t: float) -> float:
        # Burst + decay foot lift profile in [0,1].
        if cycle_t < 0.18:
            return (cycle_t / 0.18) ** 0.45  # fast attack
        release = (cycle_t - 0.18) / 0.82
        return max(0.0, (1.0 - release) ** 1.8)


__all__ = ["ExpressivePranceBurst"]
