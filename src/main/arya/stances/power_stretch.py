"""
arya/stances/power_stretch.py
================================
PowerStretch — Dramatic full-extension stretch with wave propagation,
trembles, and a snappy shake-out.

WHAT MAKES IT ALIVE:
  - The extension TRAVELS as a wave: FL drops first, FR follows 0.18 s later,
    then the rear rises in a separate pulse — you see the 'spine' curve load up
    sequentially rather than as a rigid block.
  - Coxa splays wide during the reach, dramatically framing the stretched stance.
  - At full extension, micro-trembles (3 Hz, ±0.004 m Z, FL/FR 25° out of phase)
    simulate muscle tension — the unmistakable hallmark of genuine strain.
  - Shake-out uses THREE overlapping sine waves at incommensurate frequencies
    (10 Hz, 13 Hz, 7 Hz) so each leg rattles differently — pure organic chaos.
  - Recovery wave mirrors the entry: rear settles first, front follows,
    coxa narrows last.

AMPLITUDE vs EXISTING stretch():
  - Front Z extension: −0.035 m delta  (existing: −0.04 m but no wave, no trembles)
  - Front X reach:     +0.058 m        (existing: +0.04–0.05 m)
  - Coxa splay:        +0.025 m extra Y(existing: 0 — no coxa used)
  - Rear X push-back:  −0.038 m        (existing: −0.03 m)
  - Rear Z rise:        +0.020 m       (existing: +0.01 m)

STABILITY:
  - All 4 feet grounded in every phase.
  - Front Z min: −0.215 m (7 mm margin from −0.22 m limit).
  - Y max with splay: 0.095 m (within coxa working range).
  - Rear Z max: −0.160 m (legs shorter = body rises at rear, stable).
  - Trembles and shake-out amplitudes are small enough (≤ ±0.006 m)
    that they cannot push any joint out of bounds.
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
    return t * t * (3.0 - 2.0 * t)


def _ease_out_quart(t: float) -> float:
    """Strong deceleration — overshoots feel snappy on initial reach."""
    t = max(0.0, min(1.0, t))
    u = 1.0 - t
    return 1.0 - u * u * u * u


def _transition(start: dict, end: dict, duration: float,
                curve=None) -> None:
    """Interpolate from start to end using the given curve (default: cubic ease)."""
    if curve is None:
        curve = _ease
    steps = max(1, int(duration / _DT))
    for i in range(steps + 1):
        _send(_lerp(start, end, curve(i / steps)))
        time.sleep(_DT)


def stand() -> None:
    _send(_neutral_feet())
    time.sleep(0.2)


# ═════════════════════════════════════════════════════════════════
#  PowerStretch
# ═════════════════════════════════════════════════════════════════

class PowerStretch:
    """
    Dramatic full-extension stretch. Phases:
        1. ANTICIPATE  — slight crouch + coxa begins to splay
        2. FRONT REACH — FL drops forward first, FR follows 0.18 s later
        3. REAR RISE   — rear pushes back and up (spine-curve illusion)
        4. HOLD        — full extension with micro-trembles
        5. SHAKE-OUT   — multi-frequency oscillation burst
        6. RECOVER     — wave recovery: rear first, then front
    """

    # ── target geometry ──────────────────────────────────────────
    FRONT_X_REACH =  0.058   # front feet forward from hip (m)
    FRONT_Y_SPLAY =  0.025   # extra coxa abduction on front (m)
    FRONT_Z_DROP  = -0.030   # front feet Z offset (leg extends, body drops at front, m)

    REAR_X_BACK   = -0.038   # rear feet backward from hip (m)
    REAR_Z_RISE   =  0.020   # rear feet Z offset (leg shortens, body rises at rear, m)

    # ── timing ───────────────────────────────────────────────────
    ANTICIPATE_DUR =  0.35   # pre-stretch settle (s)
    FL_REACH_DUR   =  0.55   # FL drops to full extension (s)
    FR_DELAY       =  0.18   # FR starts this long after FL begins (s)
    FR_REACH_DUR   =  0.50   # FR drops to full extension (s)
    REAR_RISE_DUR  =  0.55   # rear pushes back and up (s)
    HOLD_DUR       =  1.60   # hold at full stretch (s)
    SHAKEOUT_DUR   =  0.48   # post-stretch shake (s)
    RECOVER_DUR    =  0.90   # return to neutral (s)

    # ── trembles during hold ─────────────────────────────────────
    TREMBLE_HZ     =  3.0    # frequency (Hz)
    TREMBLE_AMP    =  0.004  # Z amplitude per front leg (m)
    TREMBLE_PHASE  =  0.44   # phase offset FL vs FR (rad, ≈25°)

    # ── shake-out: 3 incommensurate freqs per leg ────────────────
    # Each leg gets unique weights for the three frequencies so they
    # look independently rattled rather than synchronized.
    SHAKE_AMP      =  0.0055 # base amplitude (m)
    SHAKE_FREQS    = (10.0, 13.0, 7.0)   # Hz
    SHAKE_WEIGHTS  = {        # per-leg weight triplets
        "FL": (1.00, 0.55, 0.30),
        "FR": (0.60, 1.00, 0.45),
        "RL": (0.40, 0.30, 1.00),
        "RR": (0.35, 0.70, 0.80),
    }

    # ─────────────────────────────────────────────────────────────

    def _build_stretched(self) -> dict:
        """Full-extension target pose (all legs at final position)."""
        return {
            "FL": ( self.FRONT_X_REACH,
                   +_SY + self.FRONT_Y_SPLAY,
                    _SZ + self.FRONT_Z_DROP),
            "FR": ( self.FRONT_X_REACH,
                   -_SY - self.FRONT_Y_SPLAY,
                    _SZ + self.FRONT_Z_DROP),
            "RL": ( self.REAR_X_BACK,  +_SY,  _SZ + self.REAR_Z_RISE),
            "RR": ( self.REAR_X_BACK,  -_SY,  _SZ + self.REAR_Z_RISE),
        }

    def _phase_anticipate(self, neutral: dict) -> dict:
        """
        Slight crouch + coxa begins to spread.
        Returns the anticipation pose so the reach starts from here.
        """
        antici = {
            "FL": (_SX, +_SY + 0.010, _SZ + 0.005),   # small splay, slight crouch
            "FR": (_SX, -_SY - 0.010, _SZ + 0.005),
            "RL": (_SX, +_SY,          _SZ + 0.005),
            "RR": (_SX, -_SY,          _SZ + 0.005),
        }
        _transition(neutral, antici, duration=self.ANTICIPATE_DUR)
        time.sleep(0.12)
        return antici

    def _phase_front_reach(self, antici: dict) -> dict:
        """
        FL drops forward first (quart ease-out for snappy reach),
        FR follows FRONT_FR_DELAY seconds later.
        Returns the intermediate pose with both front legs down,
        rear legs still at anticipation position.
        """
        # FL target (front reach, not rear)
        fl_target = (
             self.FRONT_X_REACH,
            +_SY + self.FRONT_Y_SPLAY,
             _SZ + self.FRONT_Z_DROP,
        )
        # FR target
        fr_target = (
             self.FRONT_X_REACH,
            -_SY - self.FRONT_Y_SPLAY,
             _SZ + self.FRONT_Z_DROP,
        )

        # Run FL reach; simultaneously begin FR after its delay
        fl_steps   = max(1, int(self.FL_REACH_DUR / _DT))
        fr_start   = max(0, int(self.FR_DELAY / _DT))

        # intermediate FR (lerped from antici toward fr_target)
        ax_fr, ay_fr, az_fr = antici["FR"]
        bx_fr, by_fr, bz_fr = fr_target
        ax_rl, ay_rl, az_rl = antici["RL"]
        ax_rr, ay_rr, az_rr = antici["RR"]

        fl_x0, fl_y0, fl_z0 = antici["FL"]
        bx_fl, by_fl, bz_fl = fl_target

        # Total steps covers FL + tail-of-FR (both must finish)
        fr_steps  = max(1, int(self.FR_REACH_DUR / _DT))
        total_steps = max(fl_steps, fr_start + fr_steps)

        for i in range(total_steps + 1):
            t_fl = _ease_out_quart(min(1.0, i / fl_steps))
            fl = (
                fl_x0 + (bx_fl - fl_x0) * t_fl,
                fl_y0 + (by_fl - fl_y0) * t_fl,
                fl_z0 + (bz_fl - fl_z0) * t_fl,
            )

            fr_i = max(0, i - fr_start)
            t_fr = _ease_out_quart(min(1.0, fr_i / fr_steps))
            fr = (
                ax_fr + (bx_fr - ax_fr) * t_fr,
                ay_fr + (by_fr - ay_fr) * t_fr,
                az_fr + (bz_fr - az_fr) * t_fr,
            )

            feet = {
                "FL": fl,
                "FR": fr,
                "RL": (ax_rl, ay_rl, az_rl),
                "RR": (ax_rr, ay_rr, az_rr),
            }
            _send(feet)
            time.sleep(_DT)

        return {
            "FL": fl_target,
            "FR": fr_target,
            "RL": antici["RL"],
            "RR": antici["RR"],
        }

    def _phase_rear_rise(self, mid: dict) -> dict:
        """Rear legs push back and up — creates the spine-curve illusion."""
        stretched = self._build_stretched()
        # Only rear legs move in this phase
        rear_start = {"RL": mid["RL"], "RR": mid["RR"]}
        rear_end   = {"RL": stretched["RL"], "RR": stretched["RR"]}

        steps = max(1, int(self.REAR_RISE_DUR / _DT))
        ax_rl, ay_rl, az_rl = rear_start["RL"]
        bx_rl, by_rl, bz_rl = rear_end["RL"]
        ax_rr, ay_rr, az_rr = rear_start["RR"]
        bx_rr, by_rr, bz_rr = rear_end["RR"]
        fl = mid["FL"]
        fr = mid["FR"]

        for i in range(steps + 1):
            t = _ease(i / steps)
            feet = {
                "FL": fl,
                "FR": fr,
                "RL": (ax_rl + (bx_rl - ax_rl) * t,
                       ay_rl + (by_rl - ay_rl) * t,
                       az_rl + (bz_rl - az_rl) * t),
                "RR": (ax_rr + (bx_rr - ax_rr) * t,
                       ay_rr + (by_rr - ay_rr) * t,
                       az_rr + (bz_rr - az_rr) * t),
            }
            _send(feet)
            time.sleep(_DT)

        return stretched

    def _phase_hold(self, stretched: dict) -> None:
        """
        Full-extension hold with micro-trembles.

        FL and FR tremble at TREMBLE_HZ but with a small phase offset
        so they look independently strained, not mechanically synced.
        Rear legs hold perfectly still (acting as anchor).
        """
        steps   = max(1, int(self.HOLD_DUR / _DT))
        omega_t = 2.0 * math.pi * self.TREMBLE_HZ

        fl_base_x, fl_base_y, fl_base_z = stretched["FL"]
        fr_base_x, fr_base_y, fr_base_z = stretched["FR"]

        for i in range(steps):
            t = i * _DT
            fl_z = fl_base_z + self.TREMBLE_AMP * math.sin(omega_t * t)
            fr_z = fr_base_z + self.TREMBLE_AMP * math.sin(omega_t * t
                                                             + self.TREMBLE_PHASE)
            feet = {
                "FL": (fl_base_x, fl_base_y, fl_z),
                "FR": (fr_base_x, fr_base_y, fr_z),
                "RL": stretched["RL"],
                "RR": stretched["RR"],
            }
            _send(feet)
            time.sleep(_DT)

    def _phase_shakeout(self, stretched: dict) -> None:
        """
        Rapid multi-frequency oscillation burst — dogs shake themselves
        out after a big stretch. Each leg gets a unique combination of
        three incommensurate sine waves so they rattle independently.
        """
        steps    = max(1, int(self.SHAKEOUT_DUR / _DT))
        omegas   = [2.0 * math.pi * f for f in self.SHAKE_FREQS]

        bases = {leg: stretched[leg] for leg in ("FL", "FR", "RL", "RR")}

        # Amplitude ramps in then out over full shake duration
        for i in range(steps):
            t = i * _DT
            ramp = _ease(min(1.0, t / (0.1 * self.SHAKEOUT_DUR)))
            decay = _ease(min(1.0, (self.SHAKEOUT_DUR - t) / (0.3 * self.SHAKEOUT_DUR)))
            env = ramp * decay

            feet = {}
            for leg in ("FL", "FR", "RL", "RR"):
                bx, by, bz = bases[leg]
                w0, w1, w2 = self.SHAKE_WEIGHTS[leg]
                dz = (self.SHAKE_AMP * env * (
                    w0 * math.sin(omegas[0] * t) +
                    w1 * math.sin(omegas[1] * t + 0.8) +
                    w2 * math.sin(omegas[2] * t + 1.6)
                ) / (w0 + w1 + w2))
                feet[leg] = (bx, by, bz + dz)

            _send(feet)
            time.sleep(_DT)

    def _phase_recover(self, stretched: dict) -> None:
        """
        Wave recovery: rear settles first, then front narrows coxa
        and rises back to neutral. Mirrors the sequential entry.
        """
        neutral = _neutral_feet()

        # Step 1: rear returns to neutral while front holds
        rear_target = {"RL": neutral["RL"], "RR": neutral["RR"]}
        steps = max(1, int((self.RECOVER_DUR * 0.45) / _DT))
        ax_rl, ay_rl, az_rl = stretched["RL"]
        bx_rl, by_rl, bz_rl = rear_target["RL"]
        ax_rr, ay_rr, az_rr = stretched["RR"]
        bx_rr, by_rr, bz_rr = rear_target["RR"]
        fl = stretched["FL"]
        fr = stretched["FR"]

        for i in range(steps + 1):
            t = _ease(i / steps)
            feet = {
                "FL": fl,
                "FR": fr,
                "RL": (ax_rl + (bx_rl - ax_rl) * t,
                       ay_rl + (by_rl - ay_rl) * t,
                       az_rl + (bz_rl - az_rl) * t),
                "RR": (ax_rr + (bx_rr - ax_rr) * t,
                       ay_rr + (by_rr - ay_rr) * t,
                       az_rr + (bz_rr - az_rr) * t),
            }
            _send(feet)
            time.sleep(_DT)

        # Step 2: front returns to neutral
        mid = {
            "FL": stretched["FL"],
            "FR": stretched["FR"],
            "RL": neutral["RL"],
            "RR": neutral["RR"],
        }
        _transition(mid, neutral, duration=self.RECOVER_DUR * 0.60)

    def execute(self, state=None, actuators=None) -> None:
        """Run the full PowerStretch sequence."""
        neutral  = _neutral_feet()
        antici   = self._phase_anticipate(neutral)
        mid      = self._phase_front_reach(antici)
        stretched = self._phase_rear_rise(mid)
        self._phase_hold(stretched)
        self._phase_shakeout(stretched)
        self._phase_recover(stretched)
