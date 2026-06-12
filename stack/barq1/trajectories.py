"""
Layer 2.5 — TRAJECTORIES (D-013)
================================

Pure motion-frame generators: the SINGLE source of body/feet motion for
both the simulator and the hardware runtime. A frame is
(feet_world, body_xyz, body_rpy); generators yield frames at DT = 1/50 s —
the PCA9685 PWM frame rate, so one frame == one hardware command.

No I/O, no physics, no state beyond the generator's own.
"""

import math

from barq1.kinematics import LEGS, stance_feet_world

DT = 0.02            # 50 Hz command frames
STAND_H = 0.155      # spotMicro default stand height
CROUCH_H = 0.125


def _n(seconds):
    return max(1, int(round(seconds / DT)))


def smoothstep(s):
    s = min(1.0, max(0.0, s))
    return s * s * (3 - 2 * s)


def hold(feet, xyz, rpy, seconds):
    for _ in range(_n(seconds)):
        yield dict(feet), xyz, rpy


def move_body(feet, xyz0, xyz1, rpy0, rpy1, seconds, ease=None):
    """Linear (default) body-pose interpolation with feet planted."""
    n = _n(seconds)
    for i in range(1, n + 1):
        s = ease(i / n) if ease else i / n
        xyz = tuple(a + (b - a) * s for a, b in zip(xyz0, xyz1))
        rpy = tuple(a + (b - a) * s for a, b in zip(rpy0, rpy1))
        yield dict(feet), xyz, rpy


def stance_ramp(h0=CROUCH_H, h1=STAND_H, seconds=2.0):
    feet = stance_feet_world(h1)
    yield from move_body(feet, (0, 0, h0), (0, 0, h1), (0, 0, 0), (0, 0, 0), seconds)


def pose_sweep(h=STAND_H, roll_amp=0.15, pitch_amp=0.12, yaw_amp=0.15,
               h_amp=0.02, period=4.0):
    """One sinusoidal period per axis (roll, pitch, yaw), then a height bob."""
    feet = stance_feet_world(h)
    n = _n(period)
    for idx, amp in ((0, roll_amp), (1, pitch_amp), (2, yaw_amp)):
        for i in range(n):
            ang = amp * math.sin(2 * math.pi * i / n)
            rpy = [0.0, 0.0, 0.0]
            rpy[idx] = ang
            yield dict(feet), (0, 0, h), tuple(rpy)
    for i in range(n):
        yield dict(feet), (0, 0, h + h_amp * math.sin(2 * math.pi * i / n)), (0, 0, 0)


def swing_foot(p0, p1, s, clearance):
    """Swing-leg foot path: smoothstep ground travel, sinusoidal lift."""
    e = smoothstep(s)
    return (p0[0] + (p1[0] - p0[0]) * e,
            p0[1] + (p1[1] - p0[1]) * e,
            p0[2] + (p1[2] - p0[2]) * e + clearance * math.sin(math.pi * s))


def weight_shift_lift(h=STAND_H, shift=(-0.030, -0.035), lift_leg="FL",
                      lift_height=0.040, phase_s=1.0, hold_s=1.0):
    """Shift onto the support tripod, lift one leg, hold, set down, recenter."""
    feet = stance_feet_world(h)
    yield from move_body(feet, (0, 0, h), (shift[0], shift[1], h),
                         (0, 0, 0), (0, 0, 0), phase_s)
    p0 = feet[lift_leg]
    up = (p0[0], p0[1], lift_height)
    n = _n(phase_s)
    for i in range(1, n + 1):
        f = dict(feet)
        f[lift_leg] = tuple(a + (b - a) * (i / n) for a, b in zip(p0, up))
        yield f, (shift[0], shift[1], h), (0, 0, 0)
    f_up = dict(feet); f_up[lift_leg] = up
    yield from hold(f_up, (shift[0], shift[1], h), (0, 0, 0), hold_s)
    for i in range(1, n + 1):
        f = dict(feet)
        f[lift_leg] = tuple(a + (b - a) * (i / n) for a, b in zip(up, p0))
        yield f, (shift[0], shift[1], h), (0, 0, 0)
    yield from move_body(feet, (shift[0], shift[1], h), (0, 0, h),
                         (0, 0, 0), (0, 0, 0), phase_s)


def chain(*gens):
    for g in gens:
        yield from g
