"""
Layer 3 — CRAWL GAIT
====================

Quasi-static 6-phase crawl (spotMicro-style creep), built on the proven
weight-shift primitive (05_RESEARCH_LOG 2026-06-12: 33 mm tripod margin):

  per cycle:  shift RIGHT (+advance) | swing RL | swing FL |
              shift LEFT  (+advance) | swing RR | swing FR

The COM always sits inside the support tripod opposite the swinging leg.
One foot-step of `step_length` per leg per cycle; the body advances the
same distance during the two shift phases → static stability throughout.
Open-loop by design (D-009): frames only, no feedback consumed.
"""

import math
from dataclasses import dataclass

from barq1.kinematics import stance_feet_world
from barq1.trajectories import DT, _n, hold, move_body, swing_foot, STAND_H


@dataclass
class CrawlParams:
    stand_height: float = STAND_H
    step_length: float = 0.040   # m per leg per cycle (== body advance/cycle)
    clearance: float = 0.035     # swing foot lift height
    shift_y: float = 0.035       # lateral COM shift (proven margin at 0.035)
    shift_x: float = -0.030      # slight rearward bias onto the tripod
    shift_s: float = 0.9         # duration of each shift phase
    swing_s: float = 0.7         # duration of each leg swing


# swing order with the body shift that protects each swing
_SEQUENCE = (("right", "RL"), ("right", "FL"), ("left", "RR"), ("left", "FR"))


def crawl(cycles=3, p=CrawlParams()):
    """Yield frames for `cycles` complete crawl cycles, starting and ending
    in the centered neutral stance (body recentered, all feet down)."""
    h = p.stand_height
    feet = {leg: list(f) for leg, f in stance_feet_world(h).items()}
    body_x = 0.0

    def feet_now():
        return {leg: tuple(f) for leg, f in feet.items()}

    side_y = {"right": -p.shift_y, "left": +p.shift_y}
    cur_xyz = (0.0, 0.0, h)

    for _ in range(cycles):
        prev_side = None
        for side, leg in _SEQUENCE:
            if side != prev_side:
                # shift onto the new support side, advancing half a step
                body_x += p.step_length / 2
                tgt = (body_x + p.shift_x, side_y[side], h)
                yield from move_body(feet_now(), cur_xyz, tgt,
                                     (0, 0, 0), (0, 0, 0), p.shift_s)
                cur_xyz = tgt
                prev_side = side
            # swing one leg forward by a full step
            p0 = tuple(feet[leg])
            p1 = (p0[0] + p.step_length, p0[1], 0.0)
            n = _n(p.swing_s)
            for i in range(1, n + 1):
                f = feet_now()
                f[leg] = swing_foot(p0, p1, i / n, p.clearance)
                yield f, cur_xyz, (0, 0, 0)
            feet[leg] = list(p1)

    # recenter over the feet
    tgt = (body_x, 0.0, h)
    yield from move_body(feet_now(), cur_xyz, tgt, (0, 0, 0), (0, 0, 0), p.shift_s)
    yield from hold(feet_now(), tgt, (0, 0, 0), 0.5)


def expected_advance(cycles, p=CrawlParams()):
    return cycles * p.step_length


def cycle_time(p=CrawlParams()):
    return 2 * p.shift_s + 4 * p.swing_s
