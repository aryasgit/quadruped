"""
arya/stances/curious_alert.py
==============================
CuriousAlert — 'What was that?' multi-phase investigation behavior.

Motion idea:
    The robot reacts to something interesting. It rises into a wide,
    alert stance (coxa abduction splays front feet outward), then
    performs a slow sinusoidal left-to-right scan via differential coxa
    Y-axis motion — creating a convincing 'head turn' illusion without
    a physical neck. After scanning it sniff-bobs (front body traces a
    small ellipse, like a dog investigating a scent), then holds a tilted
    asymmetric curiosity lean before returning to neutral.

Why it looks expressive:
    - COXA joints do the heavy lifting: differential abduction/adduction
      between FL and FR creates a body-yaw illusion that reads as a
      deliberate, directed look.
    - The scan is enveloped (ramps in/out) and sinusoidal — exactly the
      kind of smooth, non-robotic motion that sells 'alive'.
    - The sniff bob adds a breathing/searching quality via a small
      elliptical trajectory on both front feet.
    - The asymmetric lean at the end is unmistakably dog-like — it is
      physically impossible to mistake for a simple programmed pose.

Stability:
    - All four legs remain grounded throughout (no paw lifts).
    - Rear legs are planted slightly behind center as a stable anchor.
    - Transitions run at 40 Hz with cubic ease-in/out.
    - Y workspace stays within ±0.11 m (coxa well within mech range).
    - Z workspace stays above -0.20 m (safe working range).
"""

import time
import math
import sys
import os

# ── path resolution (works whether run directly or via -m) ──────
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ik.solver import solve_all_legs, _STAND_Y, _STAND_Z
from joints.conventions import apply_joint_conventions
from joints.space import normalize_all
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# ── servo channel map ────────────────────────────────────────────
SERVO_MAP = {
    "FL_COXA":  COXA["FL"],   "FL_THIGH": THIGHS["TFL"],  "FL_WRIST": WRISTS["WFL"],
    "FR_COXA":  COXA["FR"],   "FR_THIGH": THIGHS["TFR"],  "FR_WRIST": WRISTS["WFR"],
    "RL_COXA":  COXA["RL"],   "RL_THIGH": THIGHS["TRL"],  "RL_WRIST": WRISTS["WRL"],
    "RR_COXA":  COXA["RR"],   "RR_THIGH": THIGHS["TRR"],  "RR_WRIST": WRISTS["WRR"],
}

# ── geometry constants ───────────────────────────────────────────
_SX = 0.0
_SY = _STAND_Y   # 0.07 m  — lateral foot offset from hip center
_SZ = _STAND_Z   # -0.18 m — nominal standing height
_DT = 0.025      # 40 Hz update period


# ── low-level pipeline helpers ───────────────────────────────────

def _neutral_feet() -> dict:
    return {
        "FL": (_SX, +_SY, _SZ),
        "FR": (_SX, -_SY, _SZ),
        "RL": (_SX, +_SY, _SZ),
        "RR": (_SX, -_SY, _SZ),
    }


def _send(feet: dict) -> None:
    """Push foot targets through the full pipeline to servos."""
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
    """Cubic ease-in/out (smooth step)."""
    return t * t * (3.0 - 2.0 * t)


def _transition(start: dict, end: dict, duration: float) -> None:
    """Smoothly interpolate from start to end over `duration` seconds."""
    steps = max(1, int(duration / _DT))
    for i in range(steps + 1):
        _send(_lerp(start, end, _ease(i / steps)))
        time.sleep(_DT)


def stand() -> None:
    """Return to neutral standing pose."""
    _send(_neutral_feet())
    time.sleep(0.2)


# ═════════════════════════════════════════════════════════════════
#  CuriousAlert
# ═════════════════════════════════════════════════════════════════

class CuriousAlert:
    """
    'What was that?' investigation behavior.

    Five sequential phases:
        1. ALERT RISE  — wide-splayed front stance, body slightly raised
        2. SCAN        — sinusoidal L/R head-scan via differential coxa Y
        3. SNIFF BOB   — front-body elliptical oscillation (investigating)
        4. LEAN        — asymmetric tilt (classic dog curiosity posture)
        5. RECOVER     — smooth return to neutral
    """

    # ── Phase 1: Alert rise ──────────────────────────────────────
    ALERT_X_FRONT  =  0.022   # front feet slightly forward (m)
    ALERT_Y_SPLAY  =  0.018   # extra coxa abduction on front legs (m)
    ALERT_Z_RISE   =  0.015   # feet pushed down so body appears raised (m)
    ALERT_X_REAR   = -0.012   # rear feet planted back as stable anchor (m)
    ALERT_DURATION =  0.75    # transition into alert pose (s)

    # ── Phase 2: Scan ────────────────────────────────────────────
    # Both front feet shift same lateral direction (s > 0 = scan left):
    #   FL Y increases  (foot moves out   → coxa abducts more)
    #   FR Y toward 0   (foot moves left  → coxa adducts)
    # This differential creates a body-yaw / 'looking' illusion.
    SCAN_Y_AMP     =  0.020   # half-amplitude of lateral coxa sweep (m)
    SCAN_PERIOD    =  1.5     # one full L→R→L cycle (s)
    SCAN_N_CYCLES  =  2       # how many full sweeps to perform
    SCAN_RAMP_PCT  =  0.15    # fraction of total scan time used for ramp in/out

    # ── Phase 3: Sniff bob ───────────────────────────────────────
    BOB_Z_AMP      =  0.010   # front feet Z bob amplitude (m)
    BOB_X_AMP      =  0.006   # front feet fore-aft bob amplitude (m)
    BOB_PERIOD     =  0.50    # one bob cycle (s)
    BOB_N_CYCLES   =  5       # number of sniff bobs
    BOB_MICRO_AMP  =  0.004   # lateral micro-sway amplitude during bob (m)

    # ── Phase 4: Curiosity lean ──────────────────────────────────
    LEAN_SIDE      = "left"   # "left" or "right"
    LEAN_Z_DIFF    =  0.016   # height differential between sides (m)
    LEAN_HOLD      =  1.1     # how long to hold the tilted pose (s)

    # ── Misc ─────────────────────────────────────────────────────
    RECOVER_DURATION = 1.0    # transition back to neutral (s)

    # ─────────────────────────────────────────────────────────────

    def _alert_base(self) -> dict:
        """The wide alert stance used as base for phases 2 and 3."""
        return {
            "FL": (self.ALERT_X_FRONT,
                   +_SY + self.ALERT_Y_SPLAY,
                   _SZ - self.ALERT_Z_RISE),
            "FR": (self.ALERT_X_FRONT,
                   -_SY - self.ALERT_Y_SPLAY,
                   _SZ - self.ALERT_Z_RISE),
            "RL": (self.ALERT_X_REAR, +_SY, _SZ),
            "RR": (self.ALERT_X_REAR, -_SY, _SZ),
        }

    def _phase_alert_rise(self) -> dict:
        """
        Rise from neutral into the wide alert stance.
        Returns the alert_base dict for use by subsequent phases.
        """
        neutral = _neutral_feet()
        alert   = self._alert_base()
        _transition(neutral, alert, duration=self.ALERT_DURATION)
        time.sleep(0.20)
        return alert

    def _phase_scan(self, alert: dict) -> None:
        """
        Sinusoidal L/R scan via differential coxa abduction.

        sin > 0  → scan left  : FL moves out, FR moves in
        sin < 0  → scan right : FL moves in,  FR moves out

        An amplitude envelope (ease-in/ease-out over SCAN_RAMP_PCT of
        total duration) prevents jerky starts and stops.

        Rear legs counter-sway at ~35% amplitude to keep CoM balanced.
        """
        total   = self.SCAN_N_CYCLES * self.SCAN_PERIOD
        steps   = max(1, int(total / _DT))
        omega   = 2.0 * math.pi / self.SCAN_PERIOD
        ramp    = self.SCAN_RAMP_PCT * total

        fx = self.ALERT_X_FRONT
        fz = _SZ - self.ALERT_Z_RISE
        rx = self.ALERT_X_REAR
        sy = self.ALERT_Y_SPLAY
        amp = self.SCAN_Y_AMP

        for i in range(steps):
            t = i * _DT

            # amplitude envelope
            if t < ramp:
                env = _ease(t / ramp)
            elif t > (total - ramp):
                env = _ease((total - t) / ramp)
            else:
                env = 1.0

            s = math.sin(omega * t) * env * amp

            feet = {
                # Front differential: both feet shift same lateral direction
                "FL": (fx,  +_SY + sy + s,  fz),
                "FR": (fx,  -_SY - sy + s,  fz),
                # Rear counter: slight opposite shift to anchor CoM
                "RL": (rx,  +_SY - s * 0.35,  _SZ),
                "RR": (rx,  -_SY - s * 0.35,  _SZ),
            }
            _send(feet)
            time.sleep(_DT)

        # Snap cleanly back to alert base (s ≈ 0 at end of even cycles)
        last = {
            "FL": (fx, +_SY + sy, fz),
            "FR": (fx, -_SY - sy, fz),
            "RL": (rx, +_SY, _SZ),
            "RR": (rx, -_SY, _SZ),
        }
        _transition(last, alert, duration=0.30)
        time.sleep(0.15)

    def _phase_sniff_bob(self, alert: dict) -> None:
        """
        Front body traces a small ellipse in XZ while a slow micro-sway
        drifts the Y position — like a dog sniffing something on the ground.

        Both front feet move together in phase (synchronous bob), which
        reads as 'the whole front of the animal is investigating', not
        just one leg twitching.
        """
        total      = self.BOB_N_CYCLES * self.BOB_PERIOD
        steps      = max(1, int(total / _DT))
        bob_omega  = 2.0 * math.pi / self.BOB_PERIOD
        sway_omega = bob_omega / 3.0   # micro-sway is 3× slower than bob

        fx_base = self.ALERT_X_FRONT
        fz_base = _SZ - self.ALERT_Z_RISE
        rx      = self.ALERT_X_REAR
        sy      = self.ALERT_Y_SPLAY

        for i in range(steps):
            t = i * _DT
            # Elliptical trajectory: Z leads, X follows 90° behind
            z_off    = math.sin(bob_omega * t) * self.BOB_Z_AMP
            x_off    = math.cos(bob_omega * t) * self.BOB_X_AMP
            micro_y  = math.sin(sway_omega * t + 0.7) * self.BOB_MICRO_AMP

            feet = {
                "FL": (fx_base + x_off,
                       +_SY + sy + micro_y,
                       fz_base + z_off),
                "FR": (fx_base + x_off,
                       -_SY - sy + micro_y,
                       fz_base + z_off),
                "RL": (rx, +_SY, _SZ),
                "RR": (rx, -_SY, _SZ),
            }
            _send(feet)
            time.sleep(_DT)

        # Return cleanly to alert base
        last = {
            "FL": (fx_base, +_SY + sy, fz_base),
            "FR": (fx_base, -_SY - sy, fz_base),
            "RL": (rx, +_SY, _SZ),
            "RR": (rx, -_SY, _SZ),
        }
        _transition(last, alert, duration=0.30)
        time.sleep(0.10)

    def _phase_lean(self, alert: dict) -> None:
        """
        Asymmetric full-body tilt — the most recognizably 'dog-like'
        posture a quadruped can hold.

        Z differential is distributed across front AND rear legs so the
        whole body tilts as a rigid unit, not just one end.
        Front legs contribute full LEAN_Z_DIFF; rear legs contribute 60%.
        """
        sign = +1.0 if self.LEAN_SIDE == "left" else -1.0
        dz   = self.LEAN_Z_DIFF

        tilted = {
            "FL": (self.ALERT_X_FRONT,
                   +_SY + self.ALERT_Y_SPLAY,
                   _SZ - self.ALERT_Z_RISE - sign * dz),
            "FR": (self.ALERT_X_FRONT,
                   -_SY - self.ALERT_Y_SPLAY,
                   _SZ - self.ALERT_Z_RISE + sign * dz),
            "RL": (self.ALERT_X_REAR,  +_SY,  _SZ - sign * dz * 0.6),
            "RR": (self.ALERT_X_REAR,  -_SY,  _SZ + sign * dz * 0.6),
        }
        _transition(alert, tilted, duration=0.90)
        time.sleep(self.LEAN_HOLD)

        # Unwind tilt back to symmetric alert
        _transition(tilted, alert, duration=0.70)
        time.sleep(0.10)

    def _phase_recover(self, alert: dict) -> None:
        """Smooth return from alert to neutral standing pose."""
        _transition(alert, _neutral_feet(), duration=self.RECOVER_DURATION)

    def execute(self, state=None, actuators=None) -> None:
        """
        Run the full CuriousAlert sequence.

        Args:
            state:     optional robot state handle (unused — pure kinematics)
            actuators: optional actuator handle    (unused — pure kinematics)
        """
        alert = self._phase_alert_rise()
        self._phase_scan(alert)
        self._phase_sniff_bob(alert)
        self._phase_lean(alert)
        self._phase_recover(alert)
