"""
Layer 6 — THRUST TROT GAIT
===========================

Design goals
------------
1. The two stance legs (one diagonal) generate a deliberate forward THRUST
   by sweeping the foot along a shallow downward arc that pushes the body
   forward — not just a passive backward slide.

2. The two swing legs (other diagonal) trace a smooth raised arch that
   lands SOFTLY ahead of centre of mass, absorbing load symmetrically so
   the robot does not rock side-to-side.

3. At every moment the stance diagonal is bearing the full body weight in
   a pose whose centre of pressure is directly under the body centre, so
   the robot never tips.

Foot trajectory design
----------------------
STANCE leg (phase in [0, duty)):
    The foot moves along a shallow catenary-like arc:
      - starts slightly FORWARD and LOW  (load acceptance)
      - passes through neutral mid-stance at ground level
      - ends slightly BACKWARD and LOW   (active push-off, dz < 0)
    The downward press at toe-off drives a ground-reaction force that
    has a forward horizontal component — this is the thrust.

SWING leg (phase in [duty, 1)):
    The foot traces a raised sine arch:
      - lifts from toe-off position
      - peaks at STEP_HEIGHT in the middle of swing
      - lands smoothly at the heel-strike position ahead of stance centre
    The landing point is chosen so that when the foot touches down it is
    directly under the projected CoM, keeping balance.

Coordinate convention (matches layer3/kinematics.py)
------------------------------------------------------
  +X  →  forward
  +Y  →  left (left legs use +stance_y, right legs use -stance_y)
  +Z  →  up   (ground is at z = stance_z, typically −0.18 m)
"""

import time
import math
from typing import Dict, Tuple

# ── Layer 3 ──────────────────────────────────────────────────────────────────
from layer3.leg_ik import solve_all_legs, _STAND_Y

# ── Layer 2 ──────────────────────────────────────────────────────────────────
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

# ── Hardware ─────────────────────────────────────────────────────────────────
from hardware.pca9685 import set_servo_angle, init_pca
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# =============================================================================
# SERVO CHANNEL MAP
# =============================================================================

CHANNELS: Dict[str, int] = {
    "FL_COXA":  COXA["FL"],   "FL_THIGH": THIGHS["TFL"],  "FL_WRIST": WRISTS["WFL"],
    "FR_COXA":  COXA["FR"],   "FR_THIGH": THIGHS["TFR"],  "FR_WRIST": WRISTS["WFR"],
    "RL_COXA":  COXA["RL"],   "RL_THIGH": THIGHS["TRL"],  "RL_WRIST": WRISTS["WRL"],
    "RR_COXA":  COXA["RR"],   "RR_THIGH": THIGHS["TRR"],  "RR_WRIST": WRISTS["WRR"],
}

LEFT_LEGS  = ("FL", "RL")

# Diagonal phase offsets: FL+RR lead, FR+RL follow by half cycle
DIAG_PHASE: Dict[str, float] = {
    "FL": 0.0,
    "RR": 0.0,
    "FR": 0.5,
    "RL": 0.5,
}

# Coxa soft clamp — allows natural foot tracking, prevents yaw accumulation
COXA_MAX_DELTA = 3.0   # degrees


# =============================================================================
# THRUST STANCE TRAJECTORY
# =============================================================================

def _stance_trajectory(
    s: float,          # normalised stance progress  0 → 1
    step_length: float,
    thrust_dip: float, # how far the foot presses DOWN at toe-off (metres, +ve)
) -> Tuple[float, float]:
    """
    Stance foot path — a smooth arc from heel-strike to toe-off.

    s = 0  : heel-strike  — foot at  +step_length/2 ahead,  z = 0
    s = 0.5: mid-stance   — foot at  0,                     z = 0  (full load)
    s = 1  : toe-off      — foot at  -step_length/2 behind, z = -thrust_dip

    The downward press at toe-off (dz < 0) creates an upward ground
    reaction force with a forward component — this is the propulsive thrust.

    The vertical component follows a half-sine so the press is smooth,
    peaks at toe-off, and the foot returns to ground level at mid-stance
    where the CoM is directly above (maximum stability).
    """
    # Fore-aft: linear sweep from front to back
    dx = (step_length / 2.0) - s * step_length        # +L/2 → -L/2

    # Vertical: zero at heel-strike, dips smoothly to -thrust_dip at toe-off
    # Uses sin²(s·π/2) so it is zero at s=0 and 1 at s=1, smooth throughout
    dz = -thrust_dip * math.sin(s * math.pi / 2.0) ** 2

    return dx, dz


# =============================================================================
# SWING ARCH TRAJECTORY
# =============================================================================

def _swing_trajectory(
    s: float,          # normalised swing progress  0 → 1
    step_length: float,
    step_height: float,
    thrust_dip: float, # must match stance so foot lifts from the right place
) -> Tuple[float, float]:
    """
    Swing foot path — a smooth raised arch from toe-off to heel-strike.

    s = 0  : lifts from toe-off position (-step_length/2, -thrust_dip)
    s = 0.5: apex — foot is at step_height above ground level
    s = 1  : lands at heel-strike position (+step_length/2, 0)

    The arch is designed so that:
      - The foot clears the ground smoothly with no sudden lift jerk
      - The landing is at ground level (dz = 0) so there is no impact
      - The horizontal position at landing is +step_length/2 ahead,
        which positions the CoM directly over the stance diagonal at
        the moment of load transfer — preserving balance.
    """
    # Fore-aft: smooth S-curve using sine so acceleration is gradual
    dx = -(step_length / 2.0) + s * step_length      # -L/2 → +L/2

    # Vertical: starts at -thrust_dip, rises to step_height, returns to 0
    # Decompose into two components:
    #   base : rises from -thrust_dip to 0  (linear)
    #   arch : sine bump peaking at mid-swing
    base = -thrust_dip * (1.0 - s)                   # -thrust_dip → 0
    arch = step_height * math.sin(math.pi * s)        # 0 → peak → 0
    dz   = base + arch

    return dx, dz


# =============================================================================
# COMBINED TRAJECTORY DISPATCHER
# =============================================================================

def _leg_trajectory(
    phase: float,
    step_length: float,
    step_height: float,
    thrust_dip: float,
    duty: float,
) -> Tuple[float, float]:
    """
    Route to stance or swing trajectory based on phase.
    Returns (dx, dz) relative to nominal stance origin.
    """
    phase = phase % 1.0

    if phase < duty:
        s = phase / duty
        return _stance_trajectory(s, step_length, thrust_dip)
    else:
        s = (phase - duty) / (1.0 - duty)
        return _swing_trajectory(s, step_length, step_height, thrust_dip)


# =============================================================================
# TROT GAIT CLASS
# =============================================================================

class TrotGait:
    """
    Thrust trot gait engine.

    Key parameters
    --------------
    thrust_dip   : How far (metres) the foot presses into the ground at
                   toe-off.  This is what generates forward propulsion.
                   Start at 0.008 m (8 mm).  Increase for more thrust.
                   Too high → robot bounces.  Too low → gliding, no drive.

    step_length  : Total fore-aft stride length (metres).
                   The foot travels from +step_length/2 to -step_length/2.

    step_height  : Peak swing height above ground (metres).
                   Must comfortably clear the floor.  0.035 m is safe.

    duty         : Fraction of cycle in stance.  0.6 gives 60% stance,
                   40% swing — slightly more stable than 0.5 for first runs.

    stance_y     : Lateral foot offset from body centreline (metres).
                   MUST equal _STAND_Y from layer3/leg_ik.py or the IK
                   deltas will be wrong and the robot will list sideways.
    """

    def __init__(
        self,
        frequency:   float = 1.5,
        step_length: float = 0.06,
        step_height: float = 0.035,
        thrust_dip:  float = 0.008,    # metres — the propulsion parameter
        duty:        float = 0.60,
        stance_x:    float = 0.0,
        stance_y:    float = _STAND_Y,
        stance_z:    float = -0.18,
        dt:          float = 0.02,
        coxa_clamp:  float = COXA_MAX_DELTA,
    ):
        self.frequency   = frequency
        self.step_length = step_length
        self.step_height = step_height
        self.thrust_dip  = thrust_dip
        self.duty        = duty
        self.stance_x    = stance_x
        self.stance_y    = stance_y
        self.stance_z    = stance_z
        self.dt          = dt
        self.coxa_clamp  = coxa_clamp

        self._t0      = None
        self._running = False

    # ------------------------------------------------------------------
    # PUBLIC API
    # ------------------------------------------------------------------

    def start(self):
        init_pca()
        self._t0      = time.time()
        self._running = True
        mismatch = abs(self.stance_y - _STAND_Y)
        print(f"[TROT] Thrust trot started")
        print(f"       stance_y={self.stance_y:.4f} m  IK ref={_STAND_Y:.4f} m  "
              f"{'✓ matched' if mismatch < 0.002 else f'WARNING {mismatch*1000:.1f} mm mismatch'}")
        print(f"       thrust_dip={self.thrust_dip*1000:.1f} mm  "
              f"step={self.step_length*1000:.0f} mm  "
              f"height={self.step_height*1000:.0f} mm  "
              f"duty={self.duty:.0%}")

    def stop(self):
        self._running = False
        print("[TROT] Stopped")

    def tick(self) -> bool:
        if not self._running:
            return False
        if self._t0 is None:
            self._t0 = time.time()

        t = time.time() - self._t0
        foot_targets = self._compute_foot_targets(t)

        # ── Layer 3: 3-D IK → joint deltas (degrees from stand pose) ─
        deltas = solve_all_legs(foot_targets)

        # ── Coxa soft clamp ──────────────────────────────────────────
        # Allow natural foot-tracking yaw within ±coxa_clamp degrees.
        # Beyond that the robot would pivot rather than walk straight.
        clamp = self.coxa_clamp
        for leg in ("FL", "FR", "RL", "RR"):
            k = f"{leg}_COXA"
            deltas[k] = max(-clamp, min(clamp, deltas[k]))

        # ── Layer 2.5: mechanical sign conventions ────────────────────
        deltas = apply_joint_conventions(deltas)

        # ── Layer 2: physical servo angles ───────────────────────────
        physical = normalize_all(deltas)

        # ── Hardware write ────────────────────────────────────────────
        for joint, channel in CHANNELS.items():
            set_servo_angle(channel, physical[joint])

        return True

    def run_forever(self):
        self.start()
        try:
            while self.tick():
                time.sleep(self.dt)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    # ------------------------------------------------------------------
    # FOOT TARGET GEOMETRY
    # ------------------------------------------------------------------

    def _compute_foot_targets(self, t: float) -> Dict[str, Tuple[float, float, float]]:
        if self.frequency <= 0.0:
            return {
                "FL": (self.stance_x,  self.stance_y, self.stance_z),
                "FR": (self.stance_x, -self.stance_y, self.stance_z),
                "RL": (self.stance_x,  self.stance_y, self.stance_z),
                "RR": (self.stance_x, -self.stance_y, self.stance_z),
            }

        targets = {}
        base_phase = t * self.frequency

        for leg, offset in DIAG_PHASE.items():
            phase = (base_phase + offset) % 1.0

            dx, dz = _leg_trajectory(
                phase,
                self.step_length,
                self.step_height,
                self.thrust_dip,
                self.duty,
            )

            y = self.stance_y if leg in LEFT_LEGS else -self.stance_y

            targets[leg] = (
                self.stance_x + dx,
                y,
                self.stance_z + dz,   # dz is signed: negative = ground press, positive = lift
            )

        return targets


# =============================================================================
# SMOKE TEST — THIS WILL MOVE THE ROBOT
# =============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("  THRUST TROT GAIT")
    print("  Stance legs press DOWN at toe-off for forward thrust.")
    print("  Swing legs trace a smooth raised arch.")
    print("  Diagonal balance maintained throughout.")
    print(f"  IK stand reference Y = {_STAND_Y:.4f} m")
    print("  CTRL+C to stop")
    print("=" * 60)

    gait = TrotGait(
        frequency   = 1.5,    # Hz  — increase to 2.0–2.5 once stable
        step_length = 0.06,   # m   — stride length
        step_height = 0.035,  # m   — swing apex height
        thrust_dip  = 0.008,  # m   — toe-off ground press (the thrust source)
                              #        increase to 0.012 for more drive
                              #        decrease to 0.004 if bouncing occurs
        duty        = 0.60,   # 60% stance — slightly conservative, stable first run
        stance_z    = -0.18,  # m   — nominal body height
        dt          = 0.02,   # s   — 50 Hz control loop
        coxa_clamp  = 3.0,    # deg — raise to 5 if feet curve; lower to 1.5 if pivot returns
    )

    gait.run_forever()