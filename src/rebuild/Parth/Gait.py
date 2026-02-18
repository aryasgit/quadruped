"""
Layer 6 — TROT GAIT GENERATOR
==============================

A proper diagonal trot gait using Layer 3 IK.

TROT CONTRACT:
- FL + RR swing together  (Diagonal A, phase = 0.0)
- FR + RL swing together  (Diagonal B, phase = 0.5)
- At any moment, exactly one diagonal is in swing, one in stance
- Foot trajectories are computed in body-frame meters
- IK is solved fresh every tick via layer3/leg_ik.py
- No posture correction, no IMU, no FSM — pure geometry

COORDINATE CONVENTION (matches layer3/kinematics.py):
  +X  → forward
  +Y  → left
  -Z  → down (ground)

USAGE:
    python -m layer6.trot_gait          # smoke test, moves robot
    from layer6.trot_gait import TrotGait
"""

import time
import math
from typing import Dict, Tuple

# ── Layer 3 ──────────────────────────────────────────────────────────────────
from layer3.leg_ik import solve_all_legs        # pure IK, returns deltas

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
    "FL_COXA":  COXA["FL"],
    "FL_THIGH": THIGHS["TFL"],
    "FL_WRIST": WRISTS["WFL"],

    "FR_COXA":  COXA["FR"],
    "FR_THIGH": THIGHS["TFR"],
    "FR_WRIST": WRISTS["WFR"],

    "RL_COXA":  COXA["RL"],
    "RL_THIGH": THIGHS["TRL"],
    "RL_WRIST": WRISTS["WRL"],

    "RR_COXA":  COXA["RR"],
    "RR_THIGH": THIGHS["TRR"],
    "RR_WRIST": WRISTS["WRR"],
}


# =============================================================================
# TROT DIAGONAL GROUPS
# =============================================================================
#
#  Diagonal A: FL (front-left)  + RR (rear-right)  — phase offset 0.0
#  Diagonal B: FR (front-right) + RL (rear-left)   — phase offset 0.5
#
#  This is the classic trot: two legs on opposite corners move together.
#  At duty = 0.5, stance and swing are perfectly balanced.

DIAG_PHASE: Dict[str, float] = {
    "FL": 0.0,   # Diagonal A
    "RR": 0.0,   # Diagonal A
    "FR": 0.5,   # Diagonal B
    "RL": 0.5,   # Diagonal B
}

LEFT_LEGS  = ("FL", "RL")
RIGHT_LEGS = ("FR", "RR")


# =============================================================================
# FOOT TRAJECTORY
# =============================================================================

def _foot_trajectory(
    phase: float,
    step_length: float,
    step_height: float,
    duty: float,
) -> Tuple[float, float]:
    """
    Given a normalised phase [0, 1), return (dx, dz) relative to stance.

    Stance phase  [0, duty)       — foot sweeps backward on the ground.
    Swing phase   [duty, 1.0)     — foot lifts in a sine arc and sweeps forward.

    dx is along the body X axis (forward = +).
    dz is vertical offset (up = +, so dz ≥ 0 during swing).
    """
    phase = phase % 1.0

    if phase < duty:
        # STANCE: linear sweep from +half to -half step length
        s = phase / duty                               # 0 → 1
        dx = (step_length / 2.0) - s * step_length    # +L/2 → -L/2
        dz = 0.0
    else:
        # SWING: cosine sweep forward + sine arc upward
        s = (phase - duty) / (1.0 - duty)             # 0 → 1
        dx = -(step_length / 2.0) + s * step_length   # -L/2 → +L/2
        dz = step_height * math.sin(math.pi * s)      # 0 → peak → 0

    return dx, dz


# =============================================================================
# TROT GAIT CLASS
# =============================================================================

class TrotGait:
    """
    Stateful trot gait engine.

    Call tick() in a loop.  Call stop() to cleanly halt.

    Parameters
    ----------
    frequency    : float  — step cycles per second (Hz).  1.5 is a safe start.
    step_length  : float  — meters per step (fore-aft).
    step_height  : float  — meters peak swing height.
    duty         : float  — fraction of cycle spent in stance.
                            0.5 = balanced trot (recommended).
    stance_x     : float  — nominal foot X position (body frame, meters).
    stance_y     : float  — nominal foot |Y| offset from centreline (meters).
    stance_z     : float  — nominal foot Z position (body frame, meters, -ve = down).
    dt           : float  — control loop period (seconds).
    """

    def __init__(
        self,
        frequency:   float = 1.5,
        step_length: float = 0.06,
        step_height: float = 0.030,
        duty:        float = 0.50,
        stance_x:    float = 0.0,
        stance_y:    float = 0.080,
        stance_z:    float = -0.17,
        dt:          float = 0.02,
    ):
        self.frequency   = frequency
        self.step_length = step_length
        self.step_height = step_height
        self.duty        = duty
        self.stance_x    = stance_x
        self.stance_y    = stance_y
        self.stance_z    = stance_z
        self.dt          = dt

        self._t0      = None   # wall-clock reference set on first tick
        self._running = False

    # ------------------------------------------------------------------
    # PUBLIC API
    # ------------------------------------------------------------------

    def start(self):
        """Initialise hardware and begin gait."""
        init_pca()
        self._t0      = time.time()
        self._running = True
        print("[TROT] Gait started — diagonal trot (FL+RR / FR+RL)")

    def stop(self):
        """Mark gait as stopped (does not move servos)."""
        self._running = False
        print("[TROT] Gait stopped")

    def tick(self) -> bool:
        """
        Compute one control step and write to servos.

        Returns True while running, False after stop() is called.
        """
        if not self._running:
            return False

        if self._t0 is None:
            self._t0 = time.time()

        t = time.time() - self._t0
        foot_targets = self._compute_foot_targets(t)

        # ── Layer 3 IK ───────────────────────────────────────────────
        #  solve_all_legs() takes body-frame foot positions and returns
        #  joint angle deltas (degrees) from the stand reference pose.
        deltas = solve_all_legs(foot_targets)

        # ── Layer 2.5 — joint conventions (sign flips) ────────────────
        deltas = apply_joint_conventions(deltas)

        # ── Layer 2 — normalise to physical servo angles ──────────────
        physical = normalize_all(deltas)

        # ── Hardware write ────────────────────────────────────────────
        for joint, channel in CHANNELS.items():
            set_servo_angle(channel, physical[joint])

        return True

    def run_forever(self):
        """
        Blocking loop.  Handles timing and KeyboardInterrupt cleanly.
        """
        self.start()
        try:
            while self.tick():
                time.sleep(self.dt)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    # ------------------------------------------------------------------
    # INTERNAL — foot target geometry
    # ------------------------------------------------------------------

    def _compute_foot_targets(
        self,
        t: float,
    ) -> Dict[str, Tuple[float, float, float]]:
        """
        Return absolute body-frame foot positions for all four legs at time t.

        Coordinate convention:
          x  — forward (+)
          y  — lateral: +LEFT for FL/RL, -RIGHT for FR/RR
          z  — vertical: negative = below body origin

        Each leg's phase is driven by the shared clock plus its diagonal offset.
        """
        if self.frequency <= 0.0:
            # Static stance — no motion
            return {
                "FL": (self.stance_x,  self.stance_y, self.stance_z),
                "FR": (self.stance_x, -self.stance_y, self.stance_z),
                "RL": (self.stance_x,  self.stance_y, self.stance_z),
                "RR": (self.stance_x, -self.stance_y, self.stance_z),
            }

        foot_targets = {}
        base_phase = t * self.frequency   # continuous phase counter

        for leg, phase_offset in DIAG_PHASE.items():
            leg_phase = (base_phase + phase_offset) % 1.0

            dx, dz = _foot_trajectory(
                leg_phase,
                self.step_length,
                self.step_height,
                self.duty,
            )

            # Y sign: left legs +Y, right legs -Y (body frame)
            y = self.stance_y if leg in LEFT_LEGS else -self.stance_y

            foot_targets[leg] = (
                self.stance_x + dx,
                y,
                self.stance_z + dz,
            )

        return foot_targets


# =============================================================================
# SMOKE TEST — THIS WILL MOVE THE ROBOT
# =============================================================================

if __name__ == "__main__":
    print("=" * 55)
    print("  TROT GAIT SMOKE TEST")
    print("  Diagonal A (FL+RR) and Diagonal B (FR+RL)")
    print("  alternate with 0.5 phase offset.")
    print("  CTRL+C to stop cleanly.")
    print("=" * 55)

    gait = TrotGait(
        frequency   = 1.5,    # Hz  — start slow, increase to ~2.5
        step_length = 0.06,   # m   — conservative first run
        step_height = 0.030,  # m
        duty        = 0.50,   # balanced trot
        stance_y    = 0.080,  # m
        stance_z    = -0.17,  # m
        dt          = 0.02,   # 50 Hz control loop
    )

    gait.run_forever()