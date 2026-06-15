"""
Open-loop servo model for simulation (D-008, D-009, D-015).
==========================================================

The real interface is command-only: the Jetson writes a pulse width, the
DS3240MG's internal controller does the rest, and NOTHING comes back.
PyBullet's position controller with torque and velocity caps is a fair
stand-in for that internal loop; the caps are what make sim trajectories
transfer — an infinitely strong sim servo would "validate" moves the real
robot cannot do.

DS3240MG (Q-002 pending datasheet confirmation):
  stall torque   40 kg.cm = 3.92 N.m @ 7.4 V
  speed          ~0.15 s / 60 deg     -> ~7.0 rad/s no-load
Sim uses a derated continuous torque and slightly conservative velocity.

FIDELITY (D-015): PyBullet's POSITION_CONTROL models the servo's internal
response, but the sim used to feed it raw float radians — bypassing the
hardware actuation boundary (the PCA9685 can only place a servo on a
discrete tick grid, and commands arrive a PWM frame late). `JointActuator`
restores that boundary so the sim drives exactly the angle the real servo
could reach. The hardware path does the equivalent in
`barq1/servo_map.ticks_for()` + the PCA9685.
"""

import math
from collections import deque

from barq1.truths import PULSE_MIN, PULSE_MAX, SERVO_TRAVEL_DEG, PWM_FREQ_HZ

STALL_TORQUE_NM = 3.92
MAX_TORQUE_NM = 3.0        # continuous derate
MAX_VELOCITY_RAD_S = 6.5
POSITION_GAIN = 0.30       # pybullet kp — stiff hobby-servo-ish tracking
VELOCITY_GAIN = 1.0

# Hardware actuation-boundary truth: the PCA9685 places a servo only on a
# discrete tick grid — 270 deg spread over (535-106)=429 ticks => 0.629 deg
# (0.0110 rad) minimum step. The grid PHASE (absolute zero) is set per-servo
# by calibration on hardware; only the SPACING is a fixed hardware truth, and
# spacing is what caps reachable precision — so a nominal grid anchored at 0
# faithfully models the resolution limit without needing calibration data.
TICK_RAD = math.radians(SERVO_TRAVEL_DEG / (PULSE_MAX - PULSE_MIN))
PWM_FRAME_S = 1.0 / PWM_FREQ_HZ      # 0.02 s — one command transport frame


def quantize_to_tick_grid(angle_rad):
    """Snap a joint angle to the PCA9685 tick resolution."""
    return round(angle_rad / TICK_RAD) * TICK_RAD


class JointActuator:
    """The per-joint hardware actuation boundary (D-015): travel clamp +
    tick-resolution quantization + command transport delay (PWM frame + I2C
    latency). With this in the path the sim cannot pass an angle the robot
    can't physically hit — critical because no joint feedback exists to catch
    such errors on hardware (D-009)."""

    def __init__(self, lo, hi, delay_frames=1, quantize=True):
        self.lo, self.hi = lo, hi
        self.delay = max(0, int(delay_frames))
        self.quantize = quantize
        self._pipe = deque([0.0] * self.delay, maxlen=self.delay) if self.delay else None

    def _shape(self, target):
        t = min(self.hi, max(self.lo, target))
        if self.quantize:
            t = min(self.hi, max(self.lo, quantize_to_tick_grid(t)))
        return t

    def reset(self, angle):
        """Seed the delay buffer to a known pose (spawn/teleport)."""
        a = self._shape(angle)
        if self.delay:
            self._pipe = deque([a] * self.delay, maxlen=self.delay)
        return a

    def command(self, target):
        """Return the angle the joint is actually driven to THIS frame."""
        t = self._shape(target)
        if self.delay == 0:
            return t
        out = self._pipe[0]          # value commanded `delay` frames ago
        self._pipe.append(t)         # maxlen deque drops the oldest
        return out


def command_position(p, body_id, joint_index, target_rad):
    """Command one joint exactly like the hardware: a position, nothing else."""
    p.setJointMotorControl2(
        bodyUniqueId=body_id,
        jointIndex=joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_rad,
        positionGain=POSITION_GAIN,
        velocityGain=VELOCITY_GAIN,
        force=MAX_TORQUE_NM,
        maxVelocity=MAX_VELOCITY_RAD_S,
    )
