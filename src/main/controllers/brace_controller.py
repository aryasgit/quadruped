"""
controllers/brace_controller.py — Shove Detection & Brace Response
====================================================================

PURPOSE:
  Detects external shoves/pushes via gyro rate spikes from the IMU
  and outputs per-leg dZ corrections (metres) that the main walk
  controller adds to foot targets before IK.

  This module NEVER touches servos, IK, or conventions.
  It is a pure computation module: IMU in → dZ offsets out.

DETECTION METHOD:
  Gyro rates (deg/s) from the MPU-6050 are the fastest indicator
  of an external disturbance.  Accelerometer-derived angles are
  sluggish (complementary filter lag).  Raw gyro rate spikes
  arrive within one sample (~20ms) of impact.

  1. Low-pass filter the gyro rates to get a "calm baseline"
     (very slow α — walks through but shoves stand out)
  2. Subtract baseline from raw → residual (the transient)
  3. If |residual| > ENTRY threshold → shove detected (with hysteresis)
  4. Compute TARGET dZ proportional to residual (INSTANT, not integrated)
  5. Smooth first-order attack toward target (prevents mechanical shock)
  6. Exponential decay back to zero when residual drops below EXIT threshold

RESPONSE MODEL (v2 — direct proportional):
  Old (v1): dz += gain * residual * dt   → integration, ~1mm per tick, painfully slow
  New (v2): target = gain * residual      → instant, 12mm for a light 10°/s touch

  Attack: first-order filter snaps to target in ~60ms (50% per tick)
  Decay:  exponential return to zero, half-life ~139ms

SIGN CONVENTIONS (from imu.py):
  roll  > 0 = tilt LEFT   →  left side dropping  → extend left, retract right
  pitch > 0 = nose UP     →  front rising         → retract front, extend rear

  roll_rate  > 0 = rotating toward left  (left side going down)
  pitch_rate > 0 = rotating nose up      (front going up)

  Correction opposes the disturbance:
    roll_rate > 0 (falling left)  → dz_left  negative (extend), dz_right positive (retract)
    pitch_rate > 0 (nose rising)  → dz_front positive (retract), dz_rear  negative (extend)

OUTPUT CONTRACT:
  offsets = brace.update(imu)
  → returns {"FL": dz, "FR": dz, "RL": dz, "RR": dz}  in metres
  → main controller does: foot_z = nominal_z + offsets[leg]
  → positive dZ = foot moves UP (leg retracts / shortens)
  → negative dZ = foot moves DOWN (leg extends / lengthens)

INTEGRATION INTO MAIN CONTROLLER:
  from controllers.brace_controller import BraceController

  brace = BraceController()
  # in the loop:
  offsets = brace.update(imu)
  for leg in feet:
      x, y, z = feet[leg]
      feet[leg] = (x, y, z + offsets[leg])
  # then IK as normal
"""

import time
import math
from dataclasses import dataclass, field
from typing import Dict


LEGS = ("FL", "FR", "RL", "RR")


# =====================================================================
# TUNING PARAMETERS
# =====================================================================

# --- Detection thresholds (with hysteresis) --------------------------
#
# ENTRY threshold: residual must exceed this to trigger shove.
# EXIT  threshold: residual must drop below this to release.
# Gap between them prevents rapid on/off jitter at boundary.
#
# 8 deg/s catches a light finger tap on the chassis.
# Lower to 5 if still too insensitive; raise if you get false
# triggers during normal walking.

ROLL_SHOVE_THRESHOLD  = 8.0    # deg/s — more sensitive
PITCH_SHOVE_THRESHOLD = 6.0    # deg/s — more sensitive

ROLL_EXIT_THRESHOLD   = 6.0     # deg/s — exit (less jitter)
PITCH_EXIT_THRESHOLD  = 4.5     # deg/s — exit (less jitter)

# --- Baseline extraction ---------------------------------------------
#
# Very slow low-pass on gyro rates.  Tracks intentional motion
# (walking, turning) over seconds, but does NOT absorb fast
# transients (shoves).
#
# Time constant = -DT / ln(1 - α) ≈ DT / α ≈ 0.02/0.008 = 2.5s
# After a 2.5s sustained turn the baseline absorbs it.
# A 200ms shove barely moves the baseline (~8% absorbed).

BASELINE_ALPHA = 0.008

# --- Response: DIRECT PROPORTIONAL -----------------------------------
#
# Metres of dZ per deg/s of residual gyro rate.
# This sets the INSTANTANEOUS target correction magnitude.
#
#   Light touch  ~10 deg/s → 0.0012 × 10 = 0.012m = 12mm  ← clearly felt
#   Medium push  ~30 deg/s → 0.0012 × 30 = 0.036m = 36mm  ← strong brace
#   Hard shove   ~60 deg/s → 0.0012 × 60 = 0.072m → clamped to 40mm

ROLL_GAIN  = 0.0008     # m per deg/s — moderate
PITCH_GAIN = 0.0007     # m per deg/s — moderate

# --- Attack smoothing ------------------------------------------------
#
# First-order filter rate toward target correction.
# Prevents instantaneous servo jumps (mechanical shock).
#
# Effective per-tick alpha = ATTACK_RATE × dt.
# At 25/s, dt=0.02: alpha = 0.50 → 50% of gap closed per tick.
#   1 tick  (20ms): 50% of target
#   2 ticks (40ms): 75%
#   3 ticks (60ms): 87.5%
#   5 ticks (100ms): 97%

ATTACK_RATE = 10.0      # 1/s — slower approach to target

# --- Decay -----------------------------------------------------------
#
# Exponential return to zero when shove ends (residual < exit threshold).
# Half-life = ln(2) / DECAY_RATE ≈ 0.693 / 5.0 ≈ 139ms

DECAY_RATE = 5.0        # 1/s

# --- Limits ----------------------------------------------------------

MAX_DZ = 0.040          # max correction per leg (m) — 40mm, within IK workspace
MIN_DZ = 0.0012         # larger deadband to suppress micro-vibrations

# --- Timing ----------------------------------------------------------

DT = 0.02               # nominal loop period (50 Hz)


# =====================================================================
# STATE SNAPSHOT — for telemetry / debugging
# =====================================================================

@dataclass
class BraceState:
    """Immutable snapshot of controller state for the tester to read."""
    # Raw IMU
    roll: float = 0.0
    pitch: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0

    # Baseline (filtered gyro rates)
    roll_rate_baseline: float = 0.0
    pitch_rate_baseline: float = 0.0

    # Residuals (raw - baseline)
    roll_residual: float = 0.0
    pitch_residual: float = 0.0

    # Detection flags
    roll_shove: bool = False
    pitch_shove: bool = False

    # Per-leg dZ output
    dz: Dict[str, float] = field(default_factory=lambda: {l: 0.0 for l in LEGS})

    # Timing
    dt_actual: float = 0.02
    loop_hz: float = 50.0

    # Shove event counter (total since reset)
    shove_count: int = 0


# =====================================================================
# BRACE CONTROLLER
# =====================================================================

class BraceController:
    """
    Gyro-rate impulse detector + direct proportional brace response.

    Stateful — call update() every tick at ~50 Hz.
    Thread-safe: NO. Call from one thread only (the main servo loop).
    """

    def __init__(self):
        # Per-leg accumulated offsets
        self._dz = {leg: 0.0 for leg in LEGS}

        # Gyro rate baselines (low-pass filtered)
        self._roll_rate_base  = 0.0
        self._pitch_rate_base = 0.0

        # Timing
        self._last_time = None

        # Detection state (hysteresis flags)
        self._roll_shove  = False
        self._pitch_shove = False
        self._shove_count = 0

        # Last snapshot for tester
        self._state = BraceState()

    def reset(self):
        """Zero all state. Call before a new run or after mode change."""
        self._dz = {leg: 0.0 for leg in LEGS}
        self._roll_rate_base  = 0.0
        self._pitch_rate_base = 0.0
        self._last_time = None
        self._roll_shove  = False
        self._pitch_shove = False
        self._shove_count = 0
        self._state = BraceState()

    def update(self, imu) -> Dict[str, float]:
        """
        Read IMU, detect shoves, compute dZ corrections.

        Args:
            imu: IMUFilter instance (must have .update() → roll, pitch, roll_rate, pitch_rate)

        Returns:
            dict {leg: dz_metres} — add to foot Z before IK
        """
        now = time.time()
        dt = DT
        if self._last_time is not None:
            dt = max(0.005, min(0.1, now - self._last_time))
        self._last_time = now

        # ----------------------------------------------------------
        # 1. Read IMU
        # ----------------------------------------------------------
        roll, pitch, roll_rate, pitch_rate = imu.update()

        # ----------------------------------------------------------
        # 2. Update baselines (very slow low-pass on gyro rates)
        #    Tracks walking/turning gradually.  Shoves appear as
        #    sharp residuals because the baseline barely moves.
        # ----------------------------------------------------------
        alpha = min(1.0, BASELINE_ALPHA * dt / DT)
        self._roll_rate_base  += alpha * (roll_rate  - self._roll_rate_base)
        self._pitch_rate_base += alpha * (pitch_rate - self._pitch_rate_base)

        roll_residual  = roll_rate  - self._roll_rate_base
        pitch_residual = pitch_rate - self._pitch_rate_base

        # ----------------------------------------------------------
        # 3. Hysteresis shove detection
        #    Enter on ENTRY threshold, exit on (lower) EXIT threshold.
        #    Prevents rapid toggling when residual hovers near boundary.
        # ----------------------------------------------------------
        if self._roll_shove:
            if abs(roll_residual) < ROLL_EXIT_THRESHOLD:
                self._roll_shove = False
        else:
            if abs(roll_residual) > ROLL_SHOVE_THRESHOLD:
                self._roll_shove = True
                self._shove_count += 1

        if self._pitch_shove:
            if abs(pitch_residual) < PITCH_EXIT_THRESHOLD:
                self._pitch_shove = False
        else:
            if abs(pitch_residual) > PITCH_SHOVE_THRESHOLD:
                self._pitch_shove = True
                self._shove_count += 1

        # ----------------------------------------------------------
        # 4. Compute per-leg TARGET dZ (direct proportional)
        #    Target is the desired correction RIGHT NOW for the
        #    current residual.  No integration — instant response.
        # ----------------------------------------------------------
        for leg in LEGS:
            target = 0.0

            if self._roll_shove:
                # roll_residual > 0 = falling left → extend left (neg dz), retract right (pos dz)
                roll_dir = -1.0 if leg in ("FL", "RL") else +1.0
                target += roll_dir * ROLL_GAIN * roll_residual

            if self._pitch_shove:
                # pitch_residual > 0 = nose up → retract front (pos dz), extend rear (neg dz)
                pitch_dir = +1.0 if leg in ("FL", "FR") else -1.0
                target += pitch_dir * PITCH_GAIN * pitch_residual

            # Clamp target to workspace
            target = max(-MAX_DZ, min(MAX_DZ, target))

            # ----------------------------------------------------------
            # 5. Attack toward target / decay toward zero
            # ----------------------------------------------------------
            if abs(target) > MIN_DZ:
                # Active correction — smooth first-order approach to target
                # Clamp alpha to [0, 1] to prevent overshoot on dt spikes
                attack_alpha = min(1.0, ATTACK_RATE * dt)
                self._dz[leg] += attack_alpha * (target - self._dz[leg])
            else:
                # No active correction on this leg — decay toward zero
                self._dz[leg] *= math.exp(-DECAY_RATE * dt)
                if abs(self._dz[leg]) < MIN_DZ:
                    self._dz[leg] = 0.0

            # Final safety clamp
            self._dz[leg] = max(-MAX_DZ, min(MAX_DZ, self._dz[leg]))

        # ----------------------------------------------------------
        # 6. Build telemetry snapshot
        # ----------------------------------------------------------
        self._state = BraceState(
            roll=roll,
            pitch=pitch,
            roll_rate=roll_rate,
            pitch_rate=pitch_rate,
            roll_rate_baseline=self._roll_rate_base,
            pitch_rate_baseline=self._pitch_rate_base,
            roll_residual=roll_residual,
            pitch_residual=pitch_residual,
            roll_shove=self._roll_shove,
            pitch_shove=self._pitch_shove,
            dz=dict(self._dz),
            dt_actual=dt,
            loop_hz=1.0 / dt if dt > 0 else 0.0,
            shove_count=self._shove_count,
        )

        return dict(self._dz)

    @property
    def state(self) -> BraceState:
        """Last computed state snapshot — read-only for telemetry."""
        return self._state

    @property
    def active(self) -> bool:
        """True if any offset is non-zero (robot is bracing or recovering)."""
        return any(abs(v) > MIN_DZ for v in self._dz.values())