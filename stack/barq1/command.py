"""
Layer 2.6 — GAIT COMMAND (D-016)
================================

The single command object the controller consumes — mirrors spotMicro's
Command (command.h): body-frame velocity + body-pose setpoints + a desired
discrete state. Our frame: x forward, y left, z up (so vy is +left, wz is
+CCW), versus spotMicro's x-fwd / y-up / z-left.
"""

from dataclasses import dataclass

STATES = ("idle", "stand", "walk")


@dataclass
class GaitCommand:
    # continuous setpoints
    vx: float = 0.0        # forward velocity   [m/s], + forward
    vy: float = 0.0        # lateral velocity   [m/s], + left
    wz: float = 0.0        # yaw rate           [rad/s], + CCW (turn left)
    roll: float = 0.0      # body posture roll  [rad]
    pitch: float = 0.0     # body posture pitch [rad]
    yaw: float = 0.0       # body posture yaw   [rad] (static offset, not turn)
    height: float = 0.155  # body height        [m]
    # desired discrete state (drives the controller FSM)
    state: str = "stand"   # one of STATES

    def clamped(self, max_vx, max_vy, max_wz):
        c = lambda v, m: max(-m, min(m, v))
        return GaitCommand(c(self.vx, max_vx), c(self.vy, max_vy),
                           c(self.wz, max_wz), self.roll, self.pitch, self.yaw,
                           self.height, self.state)

    @property
    def is_moving(self):
        return abs(self.vx) > 1e-4 or abs(self.vy) > 1e-4 or abs(self.wz) > 1e-4
