"""
Parth/Trot.py — DIAGONAL TROT WITH IMU STABILIZATION
=====================================================

WRITTEN FROM SCRATCH.  Uses the proven servo pipeline:
    solve_all_legs → apply_joint_conventions → normalize_all

This is the same pipeline used in layer6/test_walk_minimal_V2.py,
which is the ONLY walk that moves hardware correctly.

Trot pattern:
    Diagonal A  (FL + RR)  phase = 0.0
    Diagonal B  (FR + RL)  phase = 0.5

Features:
    - Bezier cubic swing trajectory (smooth foot arcs)
    - IMU roll / pitch compensation (optional, default ON)
    - Coxa roll stabilization
    - Per-leg stance trim (replaces the lateral_bias hack)
    - CSV diagnostic logging (optional)

Run:
    cd src/rebuild
    python -m Parth.Trot
"""

import time
import math
import csv
from typing import Dict, Tuple

# ── layers that are proven to work together ─────
from layer3.leg_ik import solve_all_legs, _STAND_Y
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

# ── hardware ────────────────────────────────────
from hardware.pca9685 import set_servo_angle, init_pca
from hardware.absolute_truths import COXA, THIGHS, WRISTS, COXA_MECH
from hardware.imu import init_mpu, calibrate, IMUFilter


# ================================================================
#  SERVO CHANNEL MAP  (straight from absolute_truths)
# ================================================================
CHANNELS: Dict[str, int] = {
    "FL_COXA":  COXA["FL"],   "FL_THIGH": THIGHS["TFL"],  "FL_WRIST": WRISTS["WFL"],
    "FR_COXA":  COXA["FR"],   "FR_THIGH": THIGHS["TFR"],  "FR_WRIST": WRISTS["WFR"],
    "RL_COXA":  COXA["RL"],   "RL_THIGH": THIGHS["TRL"],  "RL_WRIST": WRISTS["WRL"],
    "RR_COXA":  COXA["RR"],   "RR_THIGH": THIGHS["TRR"],  "RR_WRIST": WRISTS["WRR"],
}


# ================================================================
#  BODY GEOMETRY  (meters — matches kinematics.py)
# ================================================================
BODY_LENGTH = 0.208
BODY_WIDTH  = 0.078

# signed position of each hip relative to body centre
HIP_X = {"FL": +BODY_LENGTH/2, "FR": +BODY_LENGTH/2,
          "RL": -BODY_LENGTH/2, "RR": -BODY_LENGTH/2}
HIP_Y = {"FL": +BODY_WIDTH/2,  "FR": -BODY_WIDTH/2,
          "RL": +BODY_WIDTH/2,  "RR": -BODY_WIDTH/2}


# ================================================================
#  IMU GAINS  (tuned from trot_log.csv analysis, iteration 2)
#
#  v1 gains (2.0/2.5/18.0) → positive-feedback, roll ±12°
#  v2 gains (0.5/0.6) + derivative → rate terms injected ±73mm Z
#     noise because raw gyro rates hit ±134°/s.
#  v3: REMOVE derivative entirely.  Add EMA low-pass on angles.
#     Proportional-only with filtered angles is stable and gentle.
# ================================================================
PITCH_GAIN     = 0.9      # Z compensation per pitch degree (was 0.6; raised to fix pitch bias)
ROLL_GAIN      = 0.6      # Z compensation per roll  degree
COXA_ROLL_GAIN = 2.0      # coxa degrees per radian roll

# Yaw trim via DIFFERENTIAL STRIDE LENGTH (meters).
# Positive → right legs stride longer → counter-clockwise correction.
# Negative → left  legs stride longer → clockwise correction.
# Manual override — auto-calc from coxa asymmetry was unreliable after recal.
# Tune: if robot rotates RIGHT, increase; if LEFT, decrease.
YAW_STRIDE_TRIM = 0.005   # metres  (start conservative, tune with --log)

# EMA smoothing factor for IMU angles (0 = no smoothing, 1 = no filter)
# At 33Hz with alpha=0.3: tau≈0.07s, cutoff≈2.3Hz — smoother output,
# still passes 1.2Hz trot signal at ~80%.
IMU_EMA_ALPHA  = 0.3

DEADBAND_DEG   = 1.0      # ignore IMU noise below this
MAX_COMP_DEG   = 5.0      # clamp extreme angles


# ================================================================
#  PER-LEG STANCE TRIM  (meters — adjust if robot drifts)
# ================================================================
#  + = outward (Y) or forward (X)
TRIM_Y = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}
TRIM_X = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}


# ================================================================
#  TRAJECTORY FUNCTIONS
# ================================================================

def _bezier_swing(s: float, length: float, height: float) -> Tuple[float, float]:
    """Cubic Bézier from rear-ground to front-ground via a high arc.
       s runs 0 → 1.   Returns (dx, dz)."""
    # control points: start, high-rear, high-front, end
    p0x, p0z = -length / 2,  0.0
    p1x, p1z = -length / 4,  height * 1.4   # overshoot keeps foot clear
    p2x, p2z =  length / 4,  height * 1.4
    p3x, p3z =  length / 2,  0.0

    u = 1.0 - s
    dx = u**3*p0x + 3*u**2*s*p1x + 3*u*s**2*p2x + s**3*p3x
    dz = u**3*p0z + 3*u**2*s*p1z + 3*u*s**2*p2z + s**3*p3z
    return dx, dz


def _stance_sweep(s: float, length: float) -> Tuple[float, float]:
    """Linear ground push-back.  s runs 0 → 1.  Returns (dx, 0)."""
    return length / 2 - s * length, 0.0


def _foot_offset(phase: float, length: float, height: float,
                 duty: float) -> Tuple[float, float]:
    """Combined stance + swing.  phase wraps 0 → 1."""
    phase = phase % 1.0
    if phase < duty:
        return _stance_sweep(phase / duty, length)
    s = (phase - duty) / (1.0 - duty)
    return _bezier_swing(s, length, height)


# ================================================================
#  TROT CLASS
# ================================================================

class Trot:
    """
    IMU-stabilised diagonal trot.

    The IK → servo pipeline is IDENTICAL to test_walk_minimal_V2.py:
        solve_all_legs  →  apply_joint_conventions  →  normalize_all
    No extra mirroring, no coxa-lock override.
    """

    def __init__(
        self,
        frequency:   float = 1.2,     # Hz  (was 1.5; slower = smoother)
        step_length: float = 0.05,    # m   (was 0.06; shorter = less pitch disturbance)
        step_height: float = 0.025,   # m   (was 0.03; lower = less vertical rocking)
        duty:        float = 0.55,    # stance fraction (was 0.50; >0.5 = brief 4-foot overlap)
        stance_x:    float = 0.0,
        stance_y:    float = 0.080,   # wider than _STAND_Y for stability
        stance_z:    float = -0.17,
        dt:          float = 0.02,    # control period (50 Hz)
        use_imu:     bool  = True,
        log_csv:     bool  = False,
    ):
        self.freq       = frequency
        self.step_len   = step_length
        self.step_ht    = step_height
        self.duty       = duty
        self.sx         = stance_x
        self.sy         = stance_y
        self.sz         = stance_z
        self.dt         = dt
        self.use_imu    = use_imu
        self.log_csv    = log_csv

        # IMU runtime state
        self._imu: IMUFilter = None
        self._ref_roll  = None
        self._ref_pitch = None
        self._filt_roll  = 0.0    # EMA-filtered roll
        self._filt_pitch = 0.0    # EMA-filtered pitch
        self._prev_tick = 0.0     # for measuring actual dt

        # timing
        self._t0       = 0.0
        self._running  = False

        # logging
        self._csv_f    = None
        self._csv_w    = None

    # ────────────────────────────────────────────
    #  start / stop
    # ────────────────────────────────────────────

    def start(self):
        init_pca()

        if self.use_imu:
            print("[TROT] Initialising IMU …")
            init_mpu()
            calib = calibrate()
            self._imu = IMUFilter(calib, dt=self.dt)
            self._ref_roll = None
            self._ref_pitch = None
            print("[TROT] IMU ready")

        if self.log_csv:
            self._csv_f = open("Parth/trot_log.csv", "w", newline="")
            self._csv_w = csv.writer(self._csv_f)
            self._csv_w.writerow(["t", "roll", "pitch", "roll_rate", "pitch_rate",
                                  "FL_x","FL_y","FL_z",
                                  "FR_x","FR_y","FR_z",
                                  "RL_x","RL_y","RL_z",
                                  "RR_x","RR_y","RR_z"])

        self._t0 = time.time()
        self._prev_tick = 0.0
        self._running = True
        print(f"[TROT] GO  freq={self.freq} len={self.step_len} "
              f"ht={self.step_ht} duty={self.duty} imu={self.use_imu}")

    def stop(self):
        self._running = False
        if self._csv_f:
            self._csv_f.close()
            self._csv_f = None
        print("[TROT] Stopped")

    # ────────────────────────────────────────────
    #  IMU helper
    # ────────────────────────────────────────────

    def _get_imu(self) -> Tuple[float, float, float, float]:
        """Return (roll_deg, pitch_deg, roll_rate, pitch_rate)
        relative to boot reference.  Roll & pitch are EMA-filtered."""
        if self._imu is None:
            return 0.0, 0.0, 0.0, 0.0

        roll, pitch, roll_rate, pitch_rate = self._imu.update()

        # lock reference on first call
        if self._ref_roll is None:
            self._ref_roll  = roll
            self._ref_pitch = pitch
            return 0.0, 0.0, 0.0, 0.0

        roll  -= self._ref_roll
        pitch -= self._ref_pitch
        pitch  = -pitch                       # IMU convention flip
        pitch_rate = -pitch_rate

        # deadband
        if abs(roll)  < DEADBAND_DEG: roll  = 0.0
        if abs(pitch) < DEADBAND_DEG: pitch = 0.0

        # clamp
        roll  = max(-MAX_COMP_DEG, min(MAX_COMP_DEG, roll))
        pitch = max(-MAX_COMP_DEG, min(MAX_COMP_DEG, pitch))

        # EMA low-pass — smooths jitter without adding much lag
        a = IMU_EMA_ALPHA
        self._filt_roll  = a * roll  + (1.0 - a) * self._filt_roll
        self._filt_pitch = a * pitch + (1.0 - a) * self._filt_pitch

        return self._filt_roll, self._filt_pitch, roll_rate, pitch_rate

    # ────────────────────────────────────────────
    #  one control tick
    # ────────────────────────────────────────────

    def tick(self) -> bool:
        if not self._running:
            return False

        now = time.time()
        t = now - self._t0
        base_phase = t * self.freq

        # ── measure actual dt for IMU filter accuracy ──
        actual_dt = now - self._prev_tick if self._prev_tick else self.dt
        self._prev_tick = now
        if self._imu:
            self._imu.dt = actual_dt    # keep filter in sync with reality

        # ── IMU ──
        if self.use_imu:
            roll, pitch, roll_rate, pitch_rate = self._get_imu()
        else:
            roll, pitch, roll_rate, pitch_rate = 0.0, 0.0, 0.0, 0.0
        roll_r  = math.radians(roll)
        pitch_r = math.radians(pitch)

        # ── foot targets ──
        feet: Dict[str, Tuple[float, float, float]] = {}

        for leg in ("FL", "FR", "RL", "RR"):

            # diagonal sync: FL+RR together, FR+RL offset by 0.5
            if leg in ("FL", "RR"):
                phase = base_phase
            else:
                phase = base_phase + 0.5

            # per-leg stride length: apply yaw trim
            # positive YAW_STRIDE_TRIM → right legs stride longer → clockwise
            if leg in ("FR", "RR"):
                leg_step_len = self.step_len + YAW_STRIDE_TRIM
            else:
                leg_step_len = self.step_len - YAW_STRIDE_TRIM

            dx, dz = _foot_offset(phase, leg_step_len,
                                  self.step_ht, self.duty)

            # lateral sign convention:  left legs +Y, right legs -Y
            if leg in ("FL", "RL"):
                y =  self.sy + TRIM_Y[leg]
            else:
                y = -self.sy + TRIM_Y[leg]

            x = self.sx + TRIM_X[leg] + dx
            z = self.sz + dz

            # ── IMU Z compensation (proportional only) ──
            if self.use_imu:
                dz_p = -PITCH_GAIN * HIP_X[leg] * math.sin(pitch_r)
                dz_r = -ROLL_GAIN  * HIP_Y[leg] * math.sin(roll_r)
                z += dz_p + dz_r

            feet[leg] = (x, y, z)

        # ── IK  →  conventions  →  normalise  (PROVEN pipeline) ──
        deltas   = solve_all_legs(feet)
        deltas   = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)

        # ── coxa roll stabilisation (after normalize) ──
        if self.use_imu:
            for leg in ("FL", "RL"):
                physical[f"{leg}_COXA"] += COXA_ROLL_GAIN * roll_r
            for leg in ("FR", "RR"):
                physical[f"{leg}_COXA"] -= COXA_ROLL_GAIN * roll_r

        # ── actuate ──
        for joint, ch in CHANNELS.items():
            set_servo_angle(ch, physical[joint])

        # ── log ──
        if self._csv_w:
            row = [f"{t:.3f}", f"{roll:.2f}", f"{pitch:.2f}",
                   f"{roll_rate:.2f}", f"{pitch_rate:.2f}"]
            for leg in ("FL", "FR", "RL", "RR"):
                row.extend(f"{v:.4f}" for v in feet[leg])
            self._csv_w.writerow(row)

        return True

    # ────────────────────────────────────────────
    #  run helpers
    # ────────────────────────────────────────────

    def run_forever(self):
        self.start()
        try:
            while self.tick():
                time.sleep(self.dt)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def run_seconds(self, seconds: float):
        self.start()
        end = time.time() + seconds
        try:
            while self.tick() and time.time() < end:
                time.sleep(self.dt)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


# ================================================================
#  MAIN
# ================================================================

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="SpotMicro diagonal trot")
    ap.add_argument("--no-imu", action="store_true",
                    help="Disable IMU stabilisation")
    ap.add_argument("--log",    action="store_true",
                    help="Write trot_log.csv")
    ap.add_argument("--freq",   type=float, default=1.2)
    ap.add_argument("--len",    type=float, default=0.05,
                    help="step length (m)")
    ap.add_argument("--ht",     type=float, default=0.025,
                    help="step height (m)")
    ap.add_argument("--duty",   type=float, default=0.55)
    ap.add_argument("--sz",     type=float, default=-0.17,
                    help="stance Z (m)")
    ap.add_argument("--time",   type=float, default=0,
                    help="run for N seconds (0 = forever)")
    args = ap.parse_args()

    trot = Trot(
        frequency   = args.freq,
        step_length = args.len,
        step_height = args.ht,
        duty        = args.duty,
        stance_z    = args.sz,
        use_imu     = not args.no_imu,
        log_csv     = args.log,
    )

    if args.time > 0:
        trot.run_seconds(args.time)
    else:
        trot.run_forever()
