"""
Layer 3.5 — CONTROLLER FSM (D-017)
==================================

The single brain that turns a GaitCommand into a 50 Hz motion frame
(feet, body_xyz, body_rpy) for body_ik — the one entry point the sim,
teleop, and the hardware runtime all call (D-013). Replaces the legacy
hardcoded controller entirely.

States (driven by cmd.state):
  idle   — body lowered to a safe rest crouch, feet planted, no cycling
  stand  — body at stand height, feet planted, body posture (roll/pitch/yaw)
           tracked from the command
  walk   — the velocity gait (D-016) runs

Transitions are always smooth: body height and posture pass through
rate-limited first-order filters (spotMicro's primitive, barq1/filters.py),
and on leaving walk the feet ease back to neutral. The gait only runs once
the body is actually at stand height, so a "walk" request from idle stands
up first, then walks. Standing/idle never cycle the legs, so there is no
idle drift (the open-loop artifact seen with the raw gait at zero command).
"""

from barq1.command import GaitCommand
from barq1.filters import RateLimitedFirstOrderFilter, Vec3Filter
from barq1.kinematics import LEGS
from barq1.velocity_gait import GaitConfig, VelocityGait

IDLE_HEIGHT = 0.10        # rest crouch (reachable without body shift)
HEIGHT_TAU = 0.25
HEIGHT_RATE = 0.10        # m/s body-height ramp limit
RPY_TAU = 0.15
RPY_RATE = 1.0            # rad/s posture ramp limit
FOOT_RECENTER_RATE = 0.12  # m/s foot ease-to-neutral when leaving walk
AT_HEIGHT_TOL = 0.004      # m — "arrived at stand height"


class Controller:
    def __init__(self, dt=0.02, gait_cfg: GaitConfig = None, start_state="idle"):
        self.dt = dt
        self.gait = VelocityGait(gait_cfg)
        self.stand_h = self.gait.cfg.stand_height
        self.neutral = {leg: list(self.gait.neutral[leg]) for leg in LEGS}
        self.state = start_state
        h0 = self.stand_h if start_state in ("stand", "walk") else IDLE_HEIGHT
        self.h = RateLimitedFirstOrderFilter(dt, HEIGHT_TAU, h0, HEIGHT_RATE)
        self.rpy = Vec3Filter(dt, RPY_TAU, (0.0, 0.0, 0.0), RPY_RATE)
        self.feet = {leg: list(self.neutral[leg]) for leg in LEGS}
        self._gait_active = False

    def _ease_feet_to_neutral(self, dt):
        lim = FOOT_RECENTER_RATE * dt
        for leg in LEGS:
            for i in range(3):
                d = self.neutral[leg][i] - self.feet[leg][i]
                self.feet[leg][i] += max(-lim, min(lim, d))

    def step(self, cmd: GaitCommand, dt=None):
        dt = dt or self.dt
        desired = cmd.state if cmd.state in ("idle", "stand", "walk") else "stand"

        # 1) body height: smooth ramp toward the state's target
        h_target = IDLE_HEIGHT if desired == "idle" else self.stand_h
        self.h.set_command(h_target)
        h = self.h.step()
        at_stand = abs(h - self.stand_h) < AT_HEIGHT_TOL

        # 2) body posture: tracked only while standing/walking and up to height
        if desired in ("stand", "walk") and at_stand:
            rpy = self.rpy.run((cmd.roll, cmd.pitch, cmd.yaw))
        else:
            rpy = self.rpy.run((0.0, 0.0, 0.0))

        # 3) feet: cycle the gait only while walking, up to height, AND given a
        #    nonzero velocity command. Walk mode with no input stays DORMANT
        #    (hold stance, no leg cycling / body weave) so it never drifts.
        walking = desired == "walk" and at_stand and cmd.is_moving
        if walking:
            if not self._gait_active:
                self.gait.reset()          # restart cleanly from neutral
                self._gait_active = True
            feet, body_xyz, _ = self.gait.step(cmd, dt)
            self.feet = {leg: list(feet[leg]) for leg in LEGS}
            bx, by = body_xyz[0], body_xyz[1]
            self.state = "walk"
        else:
            self._gait_active = False
            self._ease_feet_to_neutral(dt)
            bx, by = 0.0, 0.0
            if at_stand:
                self.state = "walk" if desired == "walk" else \
                    (desired if desired in ("stand", "idle") else self.state)
            elif desired == "idle" and h <= IDLE_HEIGHT + AT_HEIGHT_TOL:
                self.state = "idle"

        feet_out = {leg: tuple(self.feet[leg]) for leg in LEGS}
        return feet_out, (bx, by, h), rpy

    @property
    def settled(self):
        """True when not mid-transition (feet ~neutral or actively walking)."""
        if self.state == "walk":
            return True
        return all(abs(self.feet[l][i] - self.neutral[l][i]) < 1e-3
                   for l in LEGS for i in range(3))
