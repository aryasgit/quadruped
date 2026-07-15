"""
L4 — FOOT TRAJECTORY SHAPES (pure)
==================================

One trot foot-path profile as a function of leg phase. Returns a normalized
"stride fraction" in [-0.5, +0.5] (position along the stride direction) and a
"lift fraction" in [0, 1] (vertical swing height).

Fixes the old velocity-DISCONTINUOUS touchdown: the swing horizontal profile is
a cubic Hermite whose velocity at touchdown matches the stance drag velocity, so
the foot plants moving at ground speed instead of scrubbing. This is a primary
contributor to the "slides in place" symptom on low-friction floors.

Pure math, no state.
"""

import math


def wrap_phase(p):
    return p % 1.0


def _hermite(sw, p0, p1, m0, m1):
    """Cubic Hermite interpolation, sw in [0,1]."""
    sw2 = sw * sw
    sw3 = sw2 * sw
    h00 = 2 * sw3 - 3 * sw2 + 1
    h10 = sw3 - 2 * sw2 + sw
    h01 = -2 * sw3 + 3 * sw2
    h11 = sw3 - sw2
    return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1


def trot_profile(phase, duty):
    """
    phase: leg phase in [0,1)
    duty:  stance fraction

    Returns (frac, lift):
      frac in [-0.5, +0.5]  -> position along the stride direction
                              (stance drags +0.5 -> -0.5; swing returns -0.5 -> +0.5)
      lift in [0, 1]        -> vertical lift (0 during stance)
    """
    if phase < duty:
        # STANCE: linear drag, foot planted, constant velocity.
        s = phase / duty
        return 0.5 - s, 0.0

    # SWING: return the foot to the front, in the air.
    sw = (phase - duty) / (1.0 - duty)

    # Velocity match: stance frac-velocity per unit phase is -1/duty.
    # Scale to the swing interval so touchdown velocity matches stance (no scrub).
    swing_span = 1.0 - duty
    m_touchdown = (-1.0 / duty) * swing_span
    m_liftoff = 0.0                      # foot has just left the ground

    frac = _hermite(sw, -0.5, 0.5, m_liftoff, m_touchdown)
    lift = math.sin(math.pi * sw)        # 0 at liftoff and touchdown, 1 at apex
    return frac, lift
