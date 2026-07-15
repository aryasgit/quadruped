"""
L5 — COMMAND (the cross-layer spine)
====================================

One command struct flows io -> control -> gait. Command sources (web, CLI)
produce it; the loop consumes it; the gait is a pure function of it.

Also holds the analog mapping (deadzone + stick magnitude -> gait frequency and
step scale) and a smooth command envelope. Deadzone lives HERE, once.
"""

from dataclasses import dataclass, replace
from config import gait_params as GP


@dataclass(frozen=True)
class Command:
    fwd: float = 0.0        # -1..1  (+forward)
    strafe: float = 0.0     # -1..1  (+right)
    turn: float = 0.0       # -1..1  (+turn-left / CCW)
    height: str = GP.DEFAULT_HEIGHT
    mode: str = "WALK"      # WALK | IDLE | STABILITY

    def magnitude(self):
        return min(1.0, (self.fwd ** 2 + self.strafe ** 2 + self.turn ** 2) ** 0.5)

    def is_moving(self):
        return self.magnitude() > 1e-3


def apply_deadzone(v, dz=GP.DEADZONE):
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def clean(cmd: Command) -> Command:
    """Deadzone the analog axes."""
    return replace(
        cmd,
        fwd=apply_deadzone(cmd.fwd),
        strafe=apply_deadzone(cmd.strafe),
        turn=apply_deadzone(cmd.turn),
    )


def gait_freq(cmd: Command, gait: str = "trot") -> float:
    """Map command magnitude -> gait cycle frequency (Hz)."""
    m = cmd.magnitude()
    if m < 1e-3:
        return 0.0
    if gait == "crawl":
        return GP.CRAWL_FREQ_MIN + (GP.CRAWL_FREQ_MAX - GP.CRAWL_FREQ_MIN) * m
    return GP.FREQ_MIN + (GP.FREQ_MAX - GP.FREQ_MIN) * m


def height_z(cmd: Command) -> float:
    return GP.HEIGHT_MODES.get(cmd.height, GP.HEIGHT_MODES[GP.DEFAULT_HEIGHT])
