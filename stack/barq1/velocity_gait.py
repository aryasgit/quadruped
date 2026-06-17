"""
Layer 3 — VELOCITY-COMMANDED GAIT (D-016)
=========================================

Port of spotMicro's walk controller (spot_micro_walk.cpp) into our frame
(x fwd, y left, z up; spotMicro is x-fwd / y-up / z-left). A stateful
generator: feed it a GaitCommand (vx, vy, wz) each 50 Hz tick and it emits
the next motion frame (feet, body_xyz, body_rpy) for body_ik — the same
contract the sim and hardware runtime already consume (D-013).

Unlike the fixed pre-baked crawl (barq1/gait.py, kept intact), this is
continuous and velocity-parameterized, so it supports forward, lateral
(strafe), and yaw (turning) motion from one engine.

8-phase static gait (one leg ever in swing): even phases shift the body
over the upcoming support tripod, odd phases swing one leg. Swing order
RR -> FR -> RL -> FL (spotMicro's rb -> rf -> lb -> lf).

  stance controller  — planted feet move backward at the commanded velocity
                       and rotate by the yaw rate, propelling the body.
  swing controller   — the free foot arcs (triangular height) to a
                       velocity-scaled touchdown ahead of neutral.
  body shift         — the body eases toward the centroid of the three
                       stance feet so the COM stays inside the support
                       triangle while a leg is up.
"""

import math
from dataclasses import dataclass

from barq1.command import GaitCommand
from barq1.kinematics import LEGS, stance_feet_world

# Which leg swings in each odd phase (even phases are all-stance body shifts).
_SWING_BY_PHASE = {1: "RR", 3: "FR", 5: "RL", 7: "FL"}


@dataclass
class GaitConfig:
    dt: float = 0.02            # 50 Hz
    stand_height: float = 0.155
    swing_ticks: int = 10       # ticks per swing phase  (0.20 s) — brisk cadence
    shift_ticks: int = 6        # ticks per body-shift phase (0.12 s); cycle 1.28 s
    z_clearance: float = 0.045  # swing foot lift [m]
    alpha: float = 0.5          # fwd/back stride centering (spotMicro)
    beta: float = 0.5           # yaw stride centering (spotMicro)
    h_tau: float = 0.02         # stance foot ground-restore time constant
    # body-shift magnitudes (spotMicro-derived, sim-tuned for margin: D-016)
    fwd_shift: float = 0.040    # shift forward when a REAR leg swings
    back_shift: float = 0.005   # shift back when a FRONT leg swings
    side_shift: float = 0.022   # shift toward the stance side
    shift_gain: float = 0.20    # per-frame easing toward the shift target
    # Speed clamps = the STEADY-STATE joint-safe envelope (scanned at this
    # cadence; the velocity ramp below keeps the from-standstill transient
    # inside limits too). Brisk cadence + ramp lifted these ~2.5x over the
    # original from-reset-limited values (D-016 tuning, see 05 2026-06-15).
    max_vx: float = 0.05
    max_vy: float = 0.05
    max_wz: float = 0.22
    # velocity ramp rates (per second) — ease into/out of motion smoothly so
    # walk-start never sweeps a foot past its joint limit.
    accel_vx: float = 0.08
    accel_vy: float = 0.08
    accel_wz: float = 0.6


def _clamp(v, m):
    return max(-m, min(m, v))


def _ramp(cur, target, step):
    return cur + max(-step, min(step, target - cur))


class VelocityGait:
    def __init__(self, cfg: GaitConfig = None):
        self.cfg = cfg or GaitConfig()
        self.neutral = {leg: list(f) for leg, f in
                        stance_feet_world(self.cfg.stand_height).items()}
        self.reset()

    def reset(self):
        self.feet = {leg: list(self.neutral[leg]) for leg in LEGS}
        self.body = [0.0, 0.0, self.cfg.stand_height]
        self.tick = 0
        self._v = [0.0, 0.0, 0.0]   # ramped (vx, vy, wz)
        # build the 8-phase schedule: [shift, swing] x4
        self._durs, self._swing_leg = [], []
        for idx in range(8):
            if idx % 2 == 0:
                self._durs.append(self.cfg.shift_ticks)
                self._swing_leg.append(None)
            else:
                self._durs.append(self.cfg.swing_ticks)
                self._swing_leg.append(_SWING_BY_PHASE[idx])
        self.phase_length = sum(self._durs)
        self.stance_ticks = self.phase_length - self.cfg.swing_ticks

    # -- phase clock --------------------------------------------------------

    def _phase(self):
        t = self.tick % self.phase_length
        acc = 0
        for i, d in enumerate(self._durs):
            if t < acc + d:
                return i, t - acc
            acc += d
        return 7, 0

    # -- per-leg controllers (spotMicro math, our frame) --------------------

    def _stance(self, leg, vx, vy, wz, dt):
        x, y, z = self.feet[leg]
        # planted feet sweep opposite the commanded yaw so the body turns +wz
        th = -wz * dt                      # rotate planted foot about body up (+z)
        c, s = math.cos(th), math.sin(th)
        xr, yr = x * c - y * s, x * s + y * c
        xr -= vx * dt
        yr -= vy * dt
        z += (1.0 / self.cfg.h_tau) * (0.0 - z) * dt   # restore to ground
        return [xr, yr, z]

    def _swing(self, leg, prop, vx, vy, wz, dt):
        cfg = self.cfg
        nx, ny, _ = self.neutral[leg]
        th = cfg.beta * self.stance_ticks * dt * wz   # touchdown ahead in turn dir
        c, s = math.cos(th), math.sin(th)
        tdx = nx * c - ny * s + cfg.alpha * self.stance_ticks * dt * vx
        tdy = nx * s + ny * c + cfg.alpha * self.stance_ticks * dt * vy
        if prop < 0.5:
            h = (prop / 0.5) * cfg.z_clearance
        else:
            h = cfg.z_clearance * (1.0 - (prop - 0.5) / 0.5)
        x, y, z = self.feet[leg]
        time_left = max(dt, dt * cfg.swing_ticks * (1.0 - prop))
        x += (tdx - x) / time_left * dt
        y += (tdy - y) / time_left * dt
        return [x, y, h]

    def _body_shift(self, idx, dt):
        # leg that is (or is next) in swing -> shift over the opposite tripod
        swing_leg = self._swing_leg[idx] if idx % 2 == 1 else self._swing_leg[(idx + 1) % 8]
        if swing_leg is None:
            tx, ty = 0.0, 0.0
        else:
            rear = swing_leg in ("RR", "RL")
            right = swing_leg in ("RR", "FR")
            tx = self.cfg.fwd_shift if rear else -self.cfg.back_shift
            ty = self.cfg.side_shift if right else -self.cfg.side_shift
        bx = self.body[0] + (tx - self.body[0]) * self.cfg.shift_gain
        by = self.body[1] + (ty - self.body[1]) * self.cfg.shift_gain
        return [bx, by, self.cfg.stand_height]

    # -- public step --------------------------------------------------------

    def step(self, cmd: GaitCommand, dt=None):
        cfg = self.cfg
        dt = dt or cfg.dt
        # ramp the internal velocity toward the clamped command (smooth start)
        self._v[0] = _ramp(self._v[0], _clamp(cmd.vx, cfg.max_vx), cfg.accel_vx * dt)
        self._v[1] = _ramp(self._v[1], _clamp(cmd.vy, cfg.max_vy), cfg.accel_vy * dt)
        self._v[2] = _ramp(self._v[2], _clamp(cmd.wz, cfg.max_wz), cfg.accel_wz * dt)
        vx, vy, wz = self._v

        idx, sub = self._phase()
        swing_leg = self._swing_leg[idx]
        for leg in LEGS:
            if leg == swing_leg:
                self.feet[leg] = self._swing(leg, sub / max(1, cfg.swing_ticks),
                                             vx, vy, wz, dt)
            else:
                self.feet[leg] = self._stance(leg, vx, vy, wz, dt)
        self.body = self._body_shift(idx, dt)
        self.tick += 1

        feet = {leg: tuple(self.feet[leg]) for leg in LEGS}
        return (feet, (self.body[0], self.body[1], cmd.height),
                (cmd.roll, cmd.pitch, cmd.yaw))

    def cycle_time(self):
        return self.phase_length * self.cfg.dt
