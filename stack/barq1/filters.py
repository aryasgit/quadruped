"""
Layer 2.6 — RATE-LIMITED FIRST-ORDER FILTER (D-016)
===================================================

Ported verbatim from spotMicro
(spot_micro_motion_cmd/src/rate_limited_first_order_filter/
rate_limited_first_order_filter.h), per D-014 (spotMicro is the design
guide). Used for smooth state transitions and command shaping:

    y[i] = (1 - a) * y[i-1] + a * u[i],   a = dt / (tau + dt)

then the per-step change is clamped to +/- rate_limit. Assumes a fixed dt.
"""

import math


class RateLimitedFirstOrderFilter:
    def __init__(self, dt, tau, x0=0.0, rate_limit=math.inf):
        self.dt = dt
        self.tau = tau
        self.alpha = dt / (tau + dt)
        self.state = x0
        self.cmd = x0
        self.rate_limit = rate_limit

    def set_command(self, cmd):
        self.cmd = cmd

    def reset(self, x0):
        self.state = x0
        self.cmd = x0

    def step(self):
        y = (1.0 - self.alpha) * self.state + self.alpha * self.cmd
        rate = (y - self.state) / self.dt
        if abs(rate) > self.rate_limit:
            y = self.state + (self.rate_limit if rate > 0 else -self.rate_limit) * self.dt
        self.state = y
        return y

    def run(self, cmd):
        self.set_command(cmd)
        return self.step()


class Vec3Filter:
    """Three independent rate-limited first-order filters (x, y, z)."""

    def __init__(self, dt, tau, x0=(0.0, 0.0, 0.0), rate_limit=math.inf):
        self.f = [RateLimitedFirstOrderFilter(dt, tau, v, rate_limit) for v in x0]

    def reset(self, x0):
        for fi, v in zip(self.f, x0):
            fi.reset(v)

    def run(self, cmd):
        return tuple(fi.run(c) for fi, c in zip(self.f, cmd))


if __name__ == "__main__":
    f = RateLimitedFirstOrderFilter(dt=0.02, tau=0.3, x0=0.0, rate_limit=0.06)
    f.set_command(1.0)
    ys = [round(f.step(), 4) for _ in range(10)]
    print("step response (tau=0.3, rl=0.06/s):", ys)
    assert ys[0] <= 0.06 * 0.02 + 1e-9, "rate limit should cap the first step"
    print("ok")
