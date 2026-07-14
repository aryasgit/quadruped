"""
arya/stances/expressive_wiggle.py
===================================
ExpressiveWiggle — Full-body, coxa-driven hip-sway with figure-8 rear trajectory.

WHAT MAKES IT ALIVE:
  - Rear legs trace a 3D figure-8 (XYZ coupled) instead of a flat fore-aft twitch.
  - RL and RR move in the SAME lateral direction (both left, then both right) with a
    small phase offset (π/12) between them — the rear end sways as a UNIT, slightly
    rolling, exactly like a real dog tail-wag.
  - Z loading is COUPLED to Y sway: the loaded side gets shorter (body dips),
    unloaded side gets taller (body rises) → produces a real body roll.
  - Front legs follow rear with a π/3 phase lag at 30% amplitude — motion travels
    as a wave from rear to front, not all-at-once.
  - Amplitude envelope: 1.5-cycle ramp-in, 4-cycle peak, 1.5-cycle ramp-out.
  - Micro-wobble (7× frequency, env² weighted) overlaid on Z at peak amplitude —
    simulates the micro-corrections a real animal makes to stay balanced.

AMPLITUDE vs EXISTING wiggle():
  - Y coxa swing: ±0.022 m (existing: 0 — no coxa used at all)
  - Z roll:        ±0.013 m differential  (existing: 0)
  - X arc:         ±0.018 m per leg (existing: ±0.015 m, no coupling)
  - Front legs:    now participate (existing: completely static)

STABILITY:
  - All 4 feet on ground throughout, amplitude envelope prevents velocity spikes.
  - Y max: 0.092 m (RL peak splay), within coxa working range.
  - Z min: −0.193 m (29 mm margin from −0.22 m limit).
  - Front counter-sway at 30% rear amplitude keeps fore-CoM centred.
"""

import time
import math
import sys
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ik.solver import solve_all_legs, _STAND_Y, _STAND_Z
from joints.conventions import apply_joint_conventions
from joints.space import normalize_all
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS

# ── servo map ────────────────────────────────────────────────────
SERVO_MAP = {
    "FL_COXA":  COXA["FL"],   "FL_THIGH": THIGHS["TFL"],  "FL_WRIST": WRISTS["WFL"],
    "FR_COXA":  COXA["FR"],   "FR_THIGH": THIGHS["TFR"],  "FR_WRIST": WRISTS["WFR"],
    "RL_COXA":  COXA["RL"],   "RL_THIGH": THIGHS["TRL"],  "RL_WRIST": WRISTS["WRL"],
    "RR_COXA":  COXA["RR"],   "RR_THIGH": THIGHS["TRR"],  "RR_WRIST": WRISTS["WRR"],
}

_SX = 0.0
_SY = _STAND_Y    # 0.07 m
_SZ = _STAND_Z    # −0.18 m
_DT = 0.025       # 40 Hz


# ── shared helpers ───────────────────────────────────────────────

def _neutral_feet() -> dict:
    return {
        "FL": (_SX, +_SY, _SZ),
        "FR": (_SX, -_SY, _SZ),
        "RL": (_SX, +_SY, _SZ),
        "RR": (_SX, -_SY, _SZ),
    }


def _send(feet: dict) -> None:
    deltas = solve_all_legs(feet)
    conv   = apply_joint_conventions(deltas)
    phys   = normalize_all(conv)
    for joint, angle in phys.items():
        ch = SERVO_MAP.get(joint)
        if ch is not None:
            set_servo_angle(ch, angle)


def _lerp(a: dict, b: dict, t: float) -> dict:
    t = max(0.0, min(1.0, t))
    out = {}
    for leg in a:
        ax, ay, az = a[leg]
        bx, by, bz = b[leg]
        out[leg] = (ax + (bx - ax) * t,
                    ay + (by - ay) * t,
                    az + (bz - az) * t)
    return out


def _ease(t: float) -> float:
    """Cubic ease-in/out."""
    return t * t * (3.0 - 2.0 * t)


def _transition(start: dict, end: dict, duration: float) -> None:
    steps = max(1, int(duration / _DT))
    for i in range(steps + 1):
        _send(_lerp(start, end, _ease(i / steps)))
        time.sleep(_DT)


def stand() -> None:
    _send(_neutral_feet())
    time.sleep(0.2)


# ═════════════════════════════════════════════════════════════════
#  ExpressiveWiggle
# ═════════════════════════════════════════════════════════════════

class ExpressiveWiggle:
    """
    Whole-body hip-sway driven by coupled lateral/vertical/fore-aft
    coxa motion on rear legs, with phase-lagged front follow-through.
    """

    # ── rear leg primary motion ──────────────────────────────────
    PERIOD       = 0.68    # one full sway cycle, s  (snappier than existing)
    N_CYCLES     = 7       # total wiggle cycles
    RAMP_CYCLES  = 1.5     # cycles to ramp amplitude in and out

    AMP_Y_REAR   = 0.022   # lateral coxa swing amplitude, m
    AMP_Z_REAR   = 0.013   # height roll coupling amplitude, m
    AMP_X_REAR   = 0.018   # fore-aft figure-8 arc amplitude, m

    # RR lags RL by this angle — creates organic sequential feel
    RR_PHASE_LAG = math.pi / 12   # 15 degrees

    # ── front leg follow-through ─────────────────────────────────
    FRONT_Y_FRAC = 0.30    # fraction of rear Y for front legs
    FRONT_Z_FRAC = 0.26    # fraction of rear Z
    FRONT_X_FRAC = 0.22    # fraction of rear X
    FRONT_LAG    = math.pi / 3    # front follows rear by 60 degrees

    # ── micro-wobble overlaid at peak ────────────────────────────
    WOBBLE_AMP   = 0.0028  # Z wobble amplitude, m
    WOBBLE_MULT  = 7.0     # frequency = PERIOD / WOBBLE_MULT Hz

    def execute(self, state=None, actuators=None) -> None:
        neutral = _neutral_feet()

        # gentle pre-wiggle settle — one beat of anticipation
        pre = {k: (v[0], v[1], v[2] + 0.004) for k, v in neutral.items()}
        _transition(neutral, pre, duration=0.20)
        _transition(pre, neutral, duration=0.20)

        total     = self.N_CYCLES * self.PERIOD
        ramp_time = self.RAMP_CYCLES * self.PERIOD
        steps     = max(1, int(total / _DT))
        omega     = 2.0 * math.pi / self.PERIOD
        wobble_w  = omega * self.WOBBLE_MULT

        # track last frame so we can transition out cleanly
        last_feet = neutral.copy()

        for i in range(steps):
            t = i * _DT

            # ── amplitude envelope ────────────────────────────────
            if t < ramp_time:
                env = _ease(t / ramp_time)
            elif t > (total - ramp_time):
                env = _ease((total - t) / ramp_time)
            else:
                env = 1.0

            # ── primary sinusoids ─────────────────────────────────
            theta_rl = omega * t
            theta_rr = theta_rl + self.RR_PHASE_LAG   # RR 15° behind RL

            s_rl = math.sin(theta_rl)
            c_rl = math.cos(theta_rl)
            s_rr = math.sin(theta_rr)
            c_rr = math.cos(theta_rr)

            # ── rear legs: figure-8 XYZ trajectory ───────────────
            # Both Y values shift in the SAME direction (whole rear sways).
            # Z is ANTI-coupled: loaded side compresses (Z+), unloaded extends (Z−).
            # X arcs in opposite directions for the figure-8 component.
            rl_y = +_SY + self.AMP_Y_REAR * s_rl * env
            rl_z =  _SZ + self.AMP_Z_REAR * s_rl * env   # Z+ = shorter = body dips left
            rl_x = _SX  - self.AMP_X_REAR * c_rl * env   # arcs fwd at left peak

            rr_y = -_SY + self.AMP_Y_REAR * s_rr * env   # same lateral direction as RL
            rr_z =  _SZ - self.AMP_Z_REAR * s_rr * env   # opposite Z (right side rises)
            rr_x = _SX  + self.AMP_X_REAR * c_rr * env   # arcs back at left peak

            # ── front legs: phase-lagged follow-through ───────────
            phi_fl = theta_rl - self.FRONT_LAG
            phi_fr = theta_rr - self.FRONT_LAG
            env_f  = env * 0.82   # front slightly softer than rear

            fl_y = +_SY + self.AMP_Y_REAR * self.FRONT_Y_FRAC * math.sin(phi_fl) * env_f
            fl_z =  _SZ + self.AMP_Z_REAR * self.FRONT_Z_FRAC * math.sin(phi_fl) * env_f
            fl_x = _SX  - self.AMP_X_REAR * self.FRONT_X_FRAC * math.cos(phi_fl) * env_f

            fr_y = -_SY + self.AMP_Y_REAR * self.FRONT_Y_FRAC * math.sin(phi_fr) * env_f
            fr_z =  _SZ - self.AMP_Z_REAR * self.FRONT_Z_FRAC * math.sin(phi_fr) * env_f
            fr_x = _SX  + self.AMP_X_REAR * self.FRONT_X_FRAC * math.cos(phi_fr) * env_f

            # ── micro-wobble (env² so it only appears near peak) ──
            wobble = self.WOBBLE_AMP * math.sin(wobble_w * t) * (env ** 2)
            rl_z += wobble
            rr_z -= wobble

            last_feet = {
                "FL": (fl_x, fl_y, fl_z),
                "FR": (fr_x, fr_y, fr_z),
                "RL": (rl_x, rl_y, rl_z),
                "RR": (rr_x, rr_y, rr_z),
            }
            _send(last_feet)
            time.sleep(_DT)

        _transition(last_feet, neutral, duration=0.45)
