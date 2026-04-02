"""
Layer 6 — MULTI-DIRECTIONAL WALK + POSTURE CONTROLLER
======================================================

Two-mode controller with terrain-adaptive gait:

  IDLE  — No input.  Posture PID levels the body continuously.
          Terminal shows stabilisation spinner + roll/pitch.

  WALK  — Feedforward terrain offsets from IMU geometry every frame.
          Per-leg step height scales to available Z travel.
          PID trim window at cycle boundaries for residual correction.
          D-pad: single cycle at default freq → trim → wait.
          Analog: proportional speed, trim at phase-wrap if error > threshold.

Architecture:
  ┌─ IMU Thread ──────────────────────────┐
  │  Reads sensor → filters → publishes   │
  │  roll / pitch at ~100 Hz.             │
  │  NEVER touches servos.                │
  │  Pauses when posture_step needs the   │
  │  raw IMU object (idle, PID trim).     │
  └───────────────┬───────────────────────┘
                  │  thread-safe roll, pitch
                  ▼
  ┌─ Main Thread (sole servo owner) ──────┐
  │  IDLE:  posture_step(feet, imu)       │
  │  WALK:  feedforward offsets + gait    │
  │         PID trim at cycle boundary    │
  └───────────────────────────────────────┘

Controls — see banner printed at startup.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time
import math
import tty
import termios
import select
import threading

try:
    import pygame
except ImportError:
    pygame = None

# ---------------- Layer 6 ----------------
from gait.generator import _leg_trajectory

# ---------------- Layer 3 ----------------
from ik.solver import solve_all_legs

# ---------------- Layer 2 ----------------
from joints.conventions import apply_joint_conventions
from joints.space import normalize_all

# ---------------- Hardware ----------------
from hardware.pca9685 import init_pca, set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS
from hardware.imu import init_mpu, calibrate, IMUFilter

# ---------------- Stability controller ----------------
from joints.stability_controller import posture_step, reset_reference, _dz

# ---------------- Tricks ----------------
from stance.tricks import (
    stand, shake, bow, wiggle, pushups,
    bheek, high_five, sit, stretch,
    tilt_dance, combo
)


# =================================================================
#  GAIT CONFIG
# =================================================================

FREQ = 1.5
DT   = 0.02        # 50 Hz main loop

STEP_LENGTH = 0.06
STEP_HEIGHT = 0.030
DUTY        = 0.80

LATERAL_STEP_LENGTH = 0.04
LATERAL_STEP_HEIGHT = 0.020
LATERAL_FREQ        = 1.5

STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

# =================================================================
#  RAMP CONFIG
# =================================================================

RAMP_TIME = 0.15
RAMP_MIN  = 0.05

# =================================================================
#  ANALOG JOYSTICK CONFIG
# =================================================================

ANALOG_DEADZONE = 0.15
ANALOG_FREQ_MIN = 0.6
ANALOG_FREQ_MAX = 2.8
ANALOG_STEP_MIN = 0.25
ANALOG_STEP_MAX = 1.0

AXIS_LX = 0       # Left stick X  → strafe
AXIS_LY = 1       # Left stick Y  → fwd/back (inverted)
AXIS_RX = 3       # Right stick X → turn

ANALOG_RAMP_UP   = 5.0
ANALOG_RAMP_DOWN = 4.0

# =================================================================
#  BODY SHIFT CONFIG
# =================================================================

BODY_SHIFT_FWD  = 0.010
BODY_SHIFT_LAT  = 0.008
BODY_SHIFT_TURN = 0.006

# =================================================================
#  TERRAIN / POSTURE CONFIG
# =================================================================

# Distance from body centre to hip along the X (front–rear) axis.
# Used by the feedforward to convert pitch into front/rear Z offset.
HIP_X_OFFSET = 0.060       # metres — measure on your robot

# Mechanical Z limits (metres, in foot-frame convention)
Z_LIMIT_COMPRESSED = -0.10  # most folded  (leg shortest)
Z_LIMIT_EXTENDED   = -0.22  # most extended (leg longest)

# Step-height scale clamps
STEP_H_SCALE_MIN = 0.30     # never shrink step below 30 %
STEP_H_SCALE_MAX = 1.20     # allow up to 120 % on extended legs

# PID trim at cycle boundaries
PID_TRIM_THRESHOLD = 2.0    # degrees — only trim if |error| > this
PID_TRIM_ITERS     = 3      # posture_step calls per trim window
PID_TRIM_DT        = 0.02   # seconds per iteration → ~60 ms window

# Low-pass filter coefficient for feedforward during walking.
# Strips gait vibration from IMU, keeps terrain slope.
# Lower = smoother but more lag.  0.10–0.20 is a good range.
IMU_LPF_ALPHA = 0.12

# Maximum terrain offset (metres) — clamp so IK never sees crazy values
TERRAIN_DZ_MAX = 0.045

# =================================================================
#  KEY / DIRECTION / TRICK MAPS
# =================================================================

KEY_MAP = {
    'w': "forward",   's': "backward",
    'a': "left",      'd': "right",
    'q': "turn_left", 'e': "turn_right",
    'c': "height",
    '1': "shake",     '2': "bow",       '3': "wiggle",
    '4': "pushups",   '5': "bheek",     '6': "high_five",
    '7': "sit",       '8': "stretch",   '9': "tilt_dance",
    '0': "combo",
    'x': "quit",
}

FORWARD_BACKWARD = ("forward", "backward")
LEFT_RIGHT       = ("left", "right")
TURN             = ("turn_left", "turn_right")

TURN_STEP_LENGTH = 0.04
TURN_STEP_HEIGHT = 0.025
TURN_FREQ        = 1.5

TRICK_MAP = {
    "shake": shake,       "bow": bow,
    "wiggle": wiggle,     "pushups": pushups,
    "bheek": bheek,       "high_five": high_five,
    "sit": sit,           "stretch": stretch,
    "tilt_dance": tilt_dance, "combo": combo,
}

# =================================================================
#  HEIGHT MODES
# =================================================================

HEIGHT_MODES = [
    ("HIGH",   -0.19),
    ("NORMAL", -0.18),
    ("LOW",    -0.15),
    ("CROUCH", -0.12),
]
HEIGHT_TRANSITION = 0.3

# =================================================================
#  COXA PRELOAD / CHANNELS
# =================================================================

COXA_DELTA_BIAS = {
    "FL": +1.5, "FR": +1.5,
    "RL": +1.5, "RR": +1.5,
}

CHANNELS = {}
for _leg in ("FL", "FR", "RL", "RR"):
    CHANNELS[f"{_leg}_COXA"]  = COXA[_leg]
    CHANNELS[f"{_leg}_THIGH"] = THIGHS[f"T{_leg}"]
    CHANNELS[f"{_leg}_WRIST"] = WRISTS[f"W{_leg}"]

DIAG_A = ("FL", "RR")

STAND_FEET = {
    "FL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
    "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
}

LEGS = ("FL", "FR", "RL", "RR")
ZERO_OFFSETS = {leg: 0.0 for leg in LEGS}

# Terminal spinner frames
SPINNER = ('◐', '◓', '◑', '◒')


# #################################################################
#  IMU THREAD
# #################################################################

class IMUThread:
    """
    Continuously reads the IMU at ~100 Hz and publishes filtered
    roll / pitch.  NEVER touches servos.

    Call .pause() before anything else uses the raw IMU object
    (posture_step, calibrate) so I2C accesses don't collide.
    Call .resume() afterwards.
    """

    def __init__(self, imu_filter: IMUFilter):
        self._imu     = imu_filter
        self._lock    = threading.Lock()
        self._roll    = 0.0
        self._pitch   = 0.0
        self._running = True
        self._paused  = False
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self._running:
            if not self._paused:
                try:
                    r, p, _, _ = self._imu.update()
                    with self._lock:
                        self._roll  = r
                        self._pitch = p
                except Exception:
                    pass
            time.sleep(0.01)       # ~100 Hz

    @property
    def roll(self):
        with self._lock:
            return self._roll

    @property
    def pitch(self):
        with self._lock:
            return self._pitch

    def pause(self):
        self._paused = True
        time.sleep(0.02)           # let in-flight read finish

    def resume(self):
        self._paused = False

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)

    @property
    def imu(self):
        """Raw IMUFilter handle — only use while thread is paused."""
        return self._imu


# #################################################################
#  TERRAIN OFFSET FUNCTIONS
# #################################################################

def clamp_offset(dz: float) -> float:
    return max(-TERRAIN_DZ_MAX, min(TERRAIN_DZ_MAX, dz))


def compute_terrain_offsets(roll_deg: float, pitch_deg: float) -> dict:
    """
    Feedforward: geometric per-leg Z offset from body tilt.

    On a tilted surface the ground under each foot is at a different
    height relative to the body centre.

        roll  > 0  →  body tilts RIGHT  →  left feet lower, right higher
        pitch > 0  →  body tilts NOSE-UP →  front feet higher, rear lower

    Returns {leg: dz_metres}.  Positive = ground is higher under that
    foot → leg must be shorter (more compressed).
    """
    roll_rad  = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)

    dr = STANCE_Y     * math.tan(roll_rad)
    dp = HIP_X_OFFSET * math.tan(pitch_rad)

    return {
        "FL": clamp_offset(+dr - dp),
        "FR": clamp_offset(-dr - dp),
        "RL": clamp_offset(+dr + dp),
        "RR": clamp_offset(-dr + dp),
    }


def compute_step_scales(stance_z: float, terrain_offsets: dict) -> dict:
    """
    Per-leg step-height scale based on remaining Z travel.

    A compressed leg (positive offset, stance_z less negative) has less
    room above for a swing arc → step_height reduced.  An extended leg
    has more room → scale ≥ 1.0.

    Nominal (flat ground):
        FL swing arc ──╮  ╭──            all legs same step_height
                       ╰──╯

    On incline (front compressed):
        FL ─╮╭─   short arc, low lift   step_h × 0.6
        RR ──╮  ╭──  full arc           step_h × 1.0
             ╰──╯
    """
    nominal_range = abs(stance_z - Z_LIMIT_COMPRESSED)
    if nominal_range < 0.001:
        return {leg: 1.0 for leg in LEGS}

    scales = {}
    for leg in LEGS:
        adjusted_z = stance_z + terrain_offsets.get(leg, 0.0)
        available  = abs(adjusted_z - Z_LIMIT_COMPRESSED)
        raw_scale  = available / nominal_range
        scales[leg] = max(STEP_H_SCALE_MIN, min(STEP_H_SCALE_MAX, raw_scale))
    return scales


# #################################################################
#  CORE HELPERS
# #################################################################

def wrap_phase(p: float) -> float:
    w = p % 1.0
    return 0.0 if w >= 1.0 else w


def ramp_factor(elapsed: float, total_duration: float) -> float:
    if total_duration <= 2 * RAMP_TIME:
        raw = math.sin(math.pi * elapsed / total_duration)
        return max(RAMP_MIN, raw)
    if elapsed < RAMP_TIME:
        raw = math.sin((math.pi / 2) * (elapsed / RAMP_TIME))
    elif elapsed > total_duration - RAMP_TIME:
        remaining = total_duration - elapsed
        raw = math.sin((math.pi / 2) * (remaining / RAMP_TIME))
    else:
        raw = 1.0
    return max(RAMP_MIN, raw)


def apply_ramp(feet: dict, ramp: float, stance_z: float = STANCE_Z) -> dict:
    stand_ref = {
        "FL": (STANCE_X,  STANCE_Y, stance_z),
        "FR": (STANCE_X, -STANCE_Y, stance_z),
        "RL": (STANCE_X,  STANCE_Y, stance_z),
        "RR": (STANCE_X, -STANCE_Y, stance_z),
    }
    ramped = {}
    for leg in feet:
        sx, sy, sz = stand_ref[leg]
        fx, fy, fz = feet[leg]
        dx, dy, dz = fx - sx, fy - sy, fz - sz
        ramped[leg] = (sx + dx * ramp, sy + dy * ramp, sz + dz * ramp)
    return ramped


def stand_at_z(stance_z: float) -> dict:
    return {
        "FL": (STANCE_X,  STANCE_Y, stance_z),
        "FR": (STANCE_X, -STANCE_Y, stance_z),
        "RL": (STANCE_X,  STANCE_Y, stance_z),
        "RR": (STANCE_X, -STANCE_Y, stance_z),
    }


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


# #################################################################
#  TRAJECTORY GENERATORS
# #################################################################

def _lateral_trajectory(phase, step_length, step_height, duty):
    phase = max(0.0, min(phase, 0.9999999))
    if phase < duty:
        s = phase / duty
        dy = +step_length / 2 - s * step_length
        return dy, 0.0
    s = (phase - duty) / (1.0 - duty)
    dy = -step_length / 2 + s * step_length
    dz = step_height * math.sin(math.pi * s)
    return dy, dz


def compute_feet_directional(phase, direction, stance_z=STANCE_Z,
                             terrain_offsets=None):
    """Route D-pad / keyboard strings through the unified engine."""
    _t = terrain_offsets
    if direction == "forward":
        return compute_feet_analog(phase,  1.0, 0.0, 0.0, 1.0, stance_z, _t)
    elif direction == "backward":
        return compute_feet_analog(phase, -1.0, 0.0, 0.0, 1.0, stance_z, _t)
    elif direction == "left":
        return compute_feet_analog(phase, 0.0, -1.0, 0.0, 1.0, stance_z, _t)
    elif direction == "right":
        return compute_feet_analog(phase, 0.0,  1.0, 0.0, 1.0, stance_z, _t)
    elif direction == "turn_left":
        return compute_feet_analog(phase, 0.0, 0.0, -1.0, 1.0, stance_z, _t)
    elif direction == "turn_right":
        return compute_feet_analog(phase, 0.0, 0.0,  1.0, 1.0, stance_z, _t)
    else:
        return stand_at_z(stance_z)


# #################################################################
#  ANALOG BLENDED FOOT COMPUTATION  (terrain-aware)
# #################################################################

def compute_feet_analog(phase: float,
                        fwd: float, strafe: float, turn: float,
                        step_scale: float,
                        stance_z: float = STANCE_Z,
                        terrain_offsets: dict = None) -> dict:
    """
    Blended foot targets driven by three analog axes.

    When terrain_offsets is provided each leg's gait arc is:
      • Vertically shifted to match the ground plane under that foot.
      • Step height scaled so compressed legs take shorter arcs and
        extended legs take normal/taller arcs.

    Args:
        phase:            gait phase 0–1
        fwd / strafe / turn:  axis values -1..+1
        step_scale:       overall step-length multiplier (stick magnitude)
        stance_z:         current stance height
        terrain_offsets:  {leg: dz_metres} or None for flat-ground
    """
    if terrain_offsets is None:
        terrain_offsets = ZERO_OFFSETS

    step_scales = compute_step_scales(stance_z, terrain_offsets)

    # Body shift (lean into movement)
    shift_x = -fwd    * BODY_SHIFT_FWD
    shift_y = -strafe * BODY_SHIFT_LAT + -turn * BODY_SHIFT_TURN

    feet = {}

    for leg in LEGS:
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        h_scale   = step_scales[leg]
        leg_z     = stance_z + terrain_offsets[leg]

        dx_total = 0.0
        dy_total = 0.0
        dz_max   = 0.0

        # --- Forward / backward ---
        if abs(fwd) > 0.01:
            dx_fb, dz_fb = _leg_trajectory(
                leg_phase,
                STEP_LENGTH * step_scale * abs(fwd),
                STEP_HEIGHT * h_scale,
                DUTY,
            )
            dx_fb *= (1.0 if fwd > 0 else -1.0)
            dx_total += dx_fb
            dz_max = max(dz_max, dz_fb)

        # --- Strafe ---
        if abs(strafe) > 0.01:
            s_phase = leg_phase
            if strafe < 0:
                s_phase = wrap_phase(1.0 - s_phase)
            dy_s, dz_s = _lateral_trajectory(
                s_phase,
                LATERAL_STEP_LENGTH * step_scale * abs(strafe),
                LATERAL_STEP_HEIGHT * h_scale,
                DUTY,
            )
            y_mult = +1 if leg in ("FR", "RR") else -1
            dy_s *= y_mult
            dy_total += dy_s
            dz_max = max(dz_max, dz_s)

        # --- Turn ---
        if abs(turn) > 0.01:
            dx_t, dz_t = _leg_trajectory(
                leg_phase,
                TURN_STEP_LENGTH * step_scale * abs(turn),
                TURN_STEP_HEIGHT * h_scale,
                DUTY,
            )
            turn_sign = 1.0 if turn > 0 else -1.0
            if leg in ("FL", "RL"):
                dx_t = +dx_t * turn_sign
            else:
                dx_t = -dx_t * turn_sign
            dx_total += dx_t
            dz_max = max(dz_max, dz_t)

        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (
            STANCE_X + dx_total + shift_x,
            base_y   + dy_total + shift_y,
            leg_z    + dz_max,
        )

    return feet


# #################################################################
#  SERVO PIPELINE
# #################################################################

def apply_coxa_bias(deltas: dict) -> dict:
    biased = deltas.copy()
    for leg, bias in COXA_DELTA_BIAS.items():
        key = f"{leg}_COXA"
        if key in biased:
            biased[key] += bias
    return biased


def send_to_servos(physical: dict):
    for joint, ch in CHANNELS.items():
        set_servo_angle(ch, physical[joint])


def execute_step(feet: dict) -> bool:
    """Full pipeline: IK → coxa bias → conventions → normalize → servos."""
    try:
        deltas = solve_all_legs(feet)
        deltas = apply_coxa_bias(deltas)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)
        send_to_servos(physical)
        return True
    except Exception as e:
        print(f"\n[WARN] Pipeline error: {e}")
        return False


def send_physical(physical: dict):
    """
    For posture_step output — already fully processed angles.
    Do NOT pass through execute_step (would double-apply conventions).
    """
    for joint, angle in physical.items():
        ch = CHANNELS.get(joint)
        if ch is not None:
            set_servo_angle(ch, angle)


# #################################################################
#  PID TRIM WINDOW
# #################################################################

def run_pid_trim(imu_thread: IMUThread, stance_z: float,
                 n_iters: int = PID_TRIM_ITERS) -> dict:
    """
    Pause IMU thread, run posture_step for n_iters at stand,
    snapshot _dz, resume thread, re-establish stand through the
    normal (coxa-biased) pipeline, return the snapshot.

    Total wall time ≈ n_iters × PID_TRIM_DT  (~60 ms default).
    """
    feet = stand_at_z(stance_z)

    imu_thread.pause()
    for _ in range(n_iters):
        physical = posture_step(feet, imu_thread.imu)
        send_physical(physical)
        time.sleep(PID_TRIM_DT)
    snapshot = {leg: _dz[leg] for leg in LEGS}
    imu_thread.resume()

    # Re-establish stand via normal pipeline (includes coxa bias)
    # so the first walking frame doesn't jerk from missing bias.
    execute_step(feet)
    return snapshot


def should_trim(roll: float, pitch: float) -> bool:
    return abs(roll) > PID_TRIM_THRESHOLD or abs(pitch) > PID_TRIM_THRESHOLD


# #################################################################
#  SINGLE-CYCLE EXECUTION  (D-pad / keyboard)
# #################################################################

def execute_single_cycle(direction: str, stance_z: float = STANCE_Z,
                         terrain_offsets: dict = None):
    if direction in LEFT_RIGHT:
        freq = LATERAL_FREQ
        num_cycles = 2
        label = f"{direction.upper()} (lateral x2)"
    elif direction in TURN:
        freq = TURN_FREQ
        num_cycles = 1
        label = f"{direction.upper().replace('_', ' ')}"
    else:
        freq = FREQ
        num_cycles = 1
        label = direction.upper()

    print(f"  Executing: {label}", end="", flush=True)

    cycle_start = time.time()
    single_cycle_duration = 1.0 / freq
    total_duration = single_cycle_duration * num_cycles

    while True:
        loop_start = time.time()
        elapsed = loop_start - cycle_start
        if elapsed >= total_duration:
            break

        phase = (elapsed / single_cycle_duration) % 1.0
        feet = compute_feet_directional(phase, direction, stance_z,
                                        terrain_offsets)
        ramp = ramp_factor(elapsed, total_duration)
        feet = apply_ramp(feet, ramp, stance_z)
        execute_step(feet)

        progress = int((elapsed / total_duration) * 10)
        ramp_pct = int(ramp * 100)
        print(
            f"\r  Executing: {label} "
            f"[{'=' * progress}{' ' * (10 - progress)}] "
            f"ramp:{ramp_pct:3d}%",
            end="", flush=True,
        )

        loop_elapsed = time.time() - loop_start
        sleep_time = max(0, DT - loop_elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)

    print(
        f"\r  Executing: {label} "
        f"[==========] ramp:  0% Done"
    )
    execute_step(stand_at_z(stance_z))


# #################################################################
#  HEIGHT TRANSITION
# #################################################################

def transition_height(from_z: float, to_z: float):
    t0 = time.time()
    while time.time() - t0 < HEIGHT_TRANSITION:
        t = (time.time() - t0) / HEIGHT_TRANSITION
        execute_step(stand_at_z(lerp(from_z, to_z, t)))
        time.sleep(DT)
    execute_step(stand_at_z(to_z))


# #################################################################
#  ANALOG STICK HELPERS
# #################################################################

def apply_deadzone(value: float, deadzone: float = ANALOG_DEADZONE) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def read_sticks(controller) -> tuple:
    raw_lx = controller.get_axis(AXIS_LX)
    raw_ly = controller.get_axis(AXIS_LY)
    raw_rx = controller.get_axis(AXIS_RX)
    return apply_deadzone(-raw_ly), apply_deadzone(raw_lx), apply_deadzone(raw_rx)


def stick_magnitude(fwd: float, strafe: float, turn: float) -> float:
    return min(1.0, math.sqrt(fwd * fwd + strafe * strafe + turn * turn))


# #################################################################
#  GAMEPAD POLLING
# #################################################################

def poll_dpad(controller):
    hat = controller.get_hat(0)
    if hat == (0,  1): return 'w'
    if hat == (0, -1): return 's'
    if hat == (-1, 0): return 'q'
    if hat == ( 1, 0): return 'e'
    return None


def poll_buttons(controller):
    lt = controller.get_axis(2)
    rt = controller.get_axis(5)
    if lt > 0.7:                 return '3'
    if rt > 0.7:                 return '2'
    if controller.get_button(4): return '1'
    if controller.get_button(5): return '5'
    if controller.get_button(1): return '4'
    if controller.get_button(0): return '9'
    if controller.get_button(3): return 'c'
    if controller.get_button(2): return 'x'
    if controller.get_button(6): return '7'
    if controller.get_button(7): return '6'
    return None


def _wait_button_release(controller):
    while True:
        pygame.event.pump()
        pressed = any(controller.get_button(i)
                      for i in range(controller.get_numbuttons()))
        if abs(controller.get_axis(2)) > 0.5 or abs(controller.get_axis(5)) > 0.5:
            pressed = True
        if not pressed:
            break
        time.sleep(0.02)


# #################################################################
#  KEYBOARD INPUT  (non-blocking with select)
# #################################################################

def poll_keyboard_char(timeout: float = 0.0):
    """
    Non-blocking keyboard read.  Terminal must already be in raw mode.
    Returns a single character or None if no key within timeout.
    """
    if select.select([sys.stdin], [], [], timeout)[0]:
        ch = sys.stdin.read(1)
        if ch == '\x03':
            raise KeyboardInterrupt
        return ch.lower()
    return None


# #################################################################
#  TERMINAL DISPLAY HELPERS
# #################################################################

def _status_idle(roll, pitch, spinner_idx, height_name):
    sp = SPINNER[spinner_idx % len(SPINNER)]
    print(
        f"\r  {sp} STABILISING [{height_name}]  "
        f"roll:{roll:+6.2f}°  pitch:{pitch:+6.2f}°  "
        f"dz[FL:{_dz['FL']:+.3f} FR:{_dz['FR']:+.3f} "
        f"RL:{_dz['RL']:+.3f} RR:{_dz['RR']:+.3f}]   ",
        end="", flush=True,
    )


def _status_walk_analog(fwd, strafe, turn, mag, freq, ramp_level,
                        roll, pitch, height_name, terrain_offsets):
    bar = int(mag * 10)
    print(
        f"\r  WALK [{height_name}] "
        f"F:{fwd:+.2f} S:{strafe:+.2f} T:{turn:+.2f} "
        f"|{'█' * bar}{'·' * (10 - bar)}| "
        f"{freq:.1f}Hz ramp:{ramp_level:.0%} "
        f"r:{roll:+5.1f}° p:{pitch:+5.1f}° "
        f"t[{terrain_offsets['FL']:+.3f},{terrain_offsets['RR']:+.3f}]   ",
        end="", flush=True,
    )


def _status_trim():
    print(
        f"\r  ◈ PID TRIM "
        f"                                                            ",
        end="", flush=True,
    )


# #################################################################
#  MAIN — GAMEPAD
# #################################################################

def main_gamepad(controller, gamepad_name: str, imu_thread: IMUThread):
    print(f"[INPUT] Gamepad detected: {gamepad_name}")
    print("        Analog sticks + D-pad + posture controller.\n")
    print("Analog:  L-Stick Y = Fwd/Back   L-Stick X = Strafe   R-Stick X = Turn")
    print("D-Pad:   Up/Down = Fwd/Back     Left/Right = Turn L/R  (single cycle)")
    print("Buttons: A=TiltDance B=Pushups X=Quit Y=Height")
    print("         LB=Shake RB=Bheek LT=Wiggle RT=Bow M1=Sit M2=HiFive")
    print()
    print(f"Analog range: {ANALOG_FREQ_MIN:.1f}–{ANALOG_FREQ_MAX:.1f} Hz  |  "
          f"Deadzone: {ANALOG_DEADZONE:.2f}")
    print(f"Posture:  PID trim threshold={PID_TRIM_THRESHOLD:.1f}°  "
          f"window={PID_TRIM_ITERS * PID_TRIM_DT * 1000:.0f}ms  "
          f"IMU LPF α={IMU_LPF_ALPHA:.2f}")
    _print_common_info()

    # ---- state ----
    height_index    = 1
    current_z       = HEIGHT_MODES[height_index][1]
    phase           = 0.0
    ramp_level      = 0.0
    moving          = False
    last_fwd        = 0.0
    last_strafe     = 0.0
    last_turn       = 0.0

    # Terrain tracking
    terrain_offsets = dict(ZERO_OFFSETS)
    walk_roll       = 0.0        # low-passed IMU for feedforward
    walk_pitch      = 0.0
    base_roll       = 0.0        # roll/pitch at last PID trim
    base_pitch      = 0.0
    pid_snapshot    = dict(ZERO_OFFSETS)

    spinner_idx     = 0

    # ---- init ----
    print("[INIT] Moving to stand pose...")
    execute_step(STAND_FEET)
    time.sleep(0.5)
    reset_reference()
    print("[INIT] Ready — entering posture-stabilised idle")
    print("-" * 70)

    try:
        while True:
            loop_top = time.time()
            pygame.event.pump()

            # ====================================================
            #  1. BUTTONS — quit / height / tricks
            # ====================================================
            btn = poll_buttons(controller)
            if btn is not None:
                cmd = KEY_MAP.get(btn)

                if cmd == "quit":
                    print("\n[QUIT] Exiting...")
                    break

                if cmd == "height":
                    old_z = current_z
                    height_index = (height_index + 1) % len(HEIGHT_MODES)
                    new_name, new_z = HEIGHT_MODES[height_index]
                    print(f"\n  Height → {new_name} (Z={new_z:.2f}m)")
                    transition_height(old_z, new_z)
                    current_z = new_z
                    phase = 0.0
                    ramp_level = 0.0
                    moving = False
                    terrain_offsets = dict(ZERO_OFFSETS)
                    pid_snapshot = dict(ZERO_OFFSETS)
                    _wait_button_release(controller)
                    continue

                if cmd in TRICK_MAP:
                    if moving:
                        execute_step(stand_at_z(current_z))
                        moving = False
                        ramp_level = 0.0
                        phase = 0.0
                    print(f"\n  Trick: {cmd.upper()}")
                    try:
                        TRICK_MAP[cmd]()
                    except Exception as e:
                        print(f"  [WARN] Trick error: {e}")
                    stand()
                    _wait_button_release(controller)
                    continue

            # ====================================================
            #  2. D-PAD — single cycle → PID trim → wait
            # ====================================================
            dpad = poll_dpad(controller)
            if dpad is not None:
                if moving:
                    execute_step(stand_at_z(current_z))
                    moving = False
                    ramp_level = 0.0
                    phase = 0.0

                cmd = KEY_MAP[dpad]
                print()
                execute_single_cycle(cmd, current_z, terrain_offsets)

                # PID trim after cycle completes (robot is at stand)
                _status_trim()
                pid_snapshot = run_pid_trim(imu_thread, current_z)
                terrain_offsets = dict(pid_snapshot)
                base_roll  = imu_thread.roll
                base_pitch = imu_thread.pitch

                # Wait for d-pad release
                while True:
                    pygame.event.pump()
                    if controller.get_hat(0) == (0, 0):
                        break
                    time.sleep(0.02)
                time.sleep(0.05)
                continue

            # ====================================================
            #  3. ANALOG STICKS
            # ====================================================
            fwd, strafe, turn = read_sticks(controller)
            mag = stick_magnitude(fwd, strafe, turn)

            if mag > 0.01:
                # ---- walking ----
                last_fwd, last_strafe, last_turn = fwd, strafe, turn

                # Low-pass IMU for terrain feedforward
                r, p = imu_thread.roll, imu_thread.pitch
                walk_roll  += IMU_LPF_ALPHA * (r - walk_roll)
                walk_pitch += IMU_LPF_ALPHA * (p - walk_pitch)

                # Active terrain offsets = PID baseline + feedforward delta
                # PID snapshot captures validated correction at last trim.
                # Feedforward delta tracks slope changes since that trim.
                delta_ff = compute_terrain_offsets(
                    walk_roll  - base_roll,
                    walk_pitch - base_pitch,
                )
                for leg in LEGS:
                    terrain_offsets[leg] = clamp_offset(
                        pid_snapshot[leg] + delta_ff[leg]
                    )

                # Gait parameters from stick magnitude
                freq       = lerp(ANALOG_FREQ_MIN, ANALOG_FREQ_MAX, mag)
                step_scale = lerp(ANALOG_STEP_MIN,  ANALOG_STEP_MAX,  mag)
                ramp_level = min(1.0, ramp_level + ANALOG_RAMP_UP * DT)

                # Advance phase — detect cycle boundary for PID trim
                prev_phase = phase
                phase = wrap_phase(phase + freq * DT)
                cycle_wrapped = (phase < prev_phase) and moving

                # Compute terrain-adapted feet
                feet = compute_feet_analog(
                    phase, fwd, strafe, turn,
                    step_scale, current_z, terrain_offsets,
                )
                feet = apply_ramp(feet, ramp_level, current_z)
                execute_step(feet)
                moving = True

                # PID trim at cycle boundary if error exceeds threshold
                if cycle_wrapped and should_trim(walk_roll, walk_pitch):
                    _status_trim()
                    pid_snapshot = run_pid_trim(imu_thread, current_z)
                    terrain_offsets = dict(pid_snapshot)
                    base_roll  = imu_thread.roll
                    base_pitch = imu_thread.pitch

                # Status line
                height_name = HEIGHT_MODES[height_index][0]
                _status_walk_analog(
                    fwd, strafe, turn, mag, freq, ramp_level,
                    walk_roll, walk_pitch, height_name, terrain_offsets,
                )

            elif moving:
                # ---- stick released: ramp out ----
                ramp_level -= ANALOG_RAMP_DOWN * DT
                if ramp_level <= RAMP_MIN:
                    execute_step(stand_at_z(current_z))
                    moving = False
                    ramp_level = 0.0
                    phase = 0.0
                    print(
                        f"\r  RAMP-OUT → IDLE"
                        f"                                                    ",
                    )
                else:
                    freq = lerp(ANALOG_FREQ_MIN, ANALOG_FREQ_MAX, 0.3)
                    phase = wrap_phase(phase + freq * DT)
                    feet = compute_feet_analog(
                        phase, last_fwd, last_strafe, last_turn,
                        ANALOG_STEP_MIN, current_z, terrain_offsets,
                    )
                    feet = apply_ramp(feet, ramp_level, current_z)
                    execute_step(feet)

            else:
                # ============================================
                #  4. IDLE — posture controller owns servos
                # ============================================
                imu_thread.pause()
                feet = stand_at_z(current_z)
                physical = posture_step(feet, imu_thread.imu)
                send_physical(physical)
                imu_thread.resume()

                # Capture latest offsets so walk starts from correct baseline
                pid_snapshot = {leg: _dz[leg] for leg in LEGS}
                terrain_offsets = dict(pid_snapshot)
                base_roll  = imu_thread.roll
                base_pitch = imu_thread.pitch
                walk_roll  = base_roll
                walk_pitch = base_pitch

                # Spinner display
                spinner_idx += 1
                height_name = HEIGHT_MODES[height_index][0]
                _status_idle(imu_thread.roll, imu_thread.pitch,
                             spinner_idx, height_name)

            # ---- timing ----
            elapsed = time.time() - loop_top
            time.sleep(max(0, DT - elapsed))

    except KeyboardInterrupt:
        print("\n\n[INTERRUPT] Caught Ctrl+C")

    finally:
        print("[SHUTDOWN] Returning to stand...")
        execute_step(STAND_FEET)
        imu_thread.stop()
        print("[SHUTDOWN] Complete")


# #################################################################
#  MAIN — KEYBOARD  (with idle posture via non-blocking select)
# #################################################################

def main_keyboard(imu_thread: IMUThread):
    if pygame is None:
        print("[INPUT] pygame not installed — using keyboard controls.\n")
    else:
        print("[INPUT] No gamepad detected — using keyboard controls.\n")

    print("Keyboard Controls:")
    print("  W = Forward    S = Backward   A = Strafe L   D = Strafe R")
    print("  Q = Turn Left  E = Turn Right C = Height     X = Quit")
    print("  1–0 = Tricks")
    print()
    print(f"Posture: PID trim threshold={PID_TRIM_THRESHOLD:.1f}°  "
          f"IMU LPF α={IMU_LPF_ALPHA:.2f}")
    _print_common_info()

    print("[INIT] Moving to stand pose...")
    execute_step(STAND_FEET)
    time.sleep(0.5)
    reset_reference()
    print("[INIT] Ready — entering posture-stabilised idle")
    print("-" * 70)

    height_index    = 1
    current_z       = HEIGHT_MODES[height_index][1]
    terrain_offsets = dict(ZERO_OFFSETS)
    pid_snapshot    = dict(ZERO_OFFSETS)
    base_roll       = 0.0
    base_pitch      = 0.0
    spinner_idx     = 0

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)

    try:
        while True:
            # Non-blocking key check — timeout = DT gives one idle iteration
            key = poll_keyboard_char(timeout=DT)

            if key is None:
                # ---- IDLE: posture correction ----
                imu_thread.pause()
                physical = posture_step(stand_at_z(current_z), imu_thread.imu)
                send_physical(physical)
                imu_thread.resume()

                pid_snapshot = {leg: _dz[leg] for leg in LEGS}
                terrain_offsets = dict(pid_snapshot)
                base_roll  = imu_thread.roll
                base_pitch = imu_thread.pitch

                spinner_idx += 1
                height_name = HEIGHT_MODES[height_index][0]
                _status_idle(imu_thread.roll, imu_thread.pitch,
                             spinner_idx, height_name)
                continue

            # ---- process key ----
            if key not in KEY_MAP:
                continue

            command = KEY_MAP[key]

            if command == "quit":
                print("\n[QUIT] Exiting...")
                break

            elif command == "height":
                old_z = current_z
                height_index = (height_index + 1) % len(HEIGHT_MODES)
                new_name, new_z = HEIGHT_MODES[height_index]
                print(f"\n  Height → {new_name} (Z={new_z:.2f}m)")
                transition_height(old_z, new_z)
                current_z = new_z
                terrain_offsets = dict(ZERO_OFFSETS)
                pid_snapshot = dict(ZERO_OFFSETS)

            elif command in TRICK_MAP:
                print(f"\n  Trick: {command.upper()}")
                try:
                    TRICK_MAP[command]()
                except Exception as e:
                    print(f"  [WARN] Trick error: {e}")
                stand()

            else:
                print()
                execute_single_cycle(command, current_z, terrain_offsets)

                # PID trim after cycle
                _status_trim()
                pid_snapshot = run_pid_trim(imu_thread, current_z)
                terrain_offsets = dict(pid_snapshot)
                base_roll  = imu_thread.roll
                base_pitch = imu_thread.pitch

    except KeyboardInterrupt:
        print("\n\n[INTERRUPT] Caught Ctrl+C")

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("[SHUTDOWN] Returning to stand...")
        execute_step(STAND_FEET)
        imu_thread.stop()
        print("[SHUTDOWN] Complete")


# #################################################################
#  COMMON PRINT
# #################################################################

def _print_common_info():
    print()
    print(f"Terrain: HIP_X={HIP_X_OFFSET*1000:.0f}mm  "
          f"Z_limits=[{Z_LIMIT_COMPRESSED:.2f}, {Z_LIMIT_EXTENDED:.2f}]  "
          f"max_dz={TERRAIN_DZ_MAX*1000:.0f}mm")
    print(f"Ramp: {RAMP_TIME*1000:.0f}ms sine ease")
    print()
    print("Height modes:")
    for name, z in HEIGHT_MODES:
        marker = " <-- default" if name == "NORMAL" else ""
        print(f"  {name:8s}: Z={z:.2f}m{marker}")
    print()
    print("-" * 70)


# #################################################################
#  ENTRY POINT
# #################################################################

def main():
    print("=" * 70)
    print("  WALK CONTROLLER  —  Analog + D-Pad + Posture Stabilisation")
    print("=" * 70)
    print()

    # ---- hardware init ----
    print("[INIT] PCA9685...")
    init_pca()

    print("[INIT] MPU6050...")
    init_mpu()

    print("[INIT] Calibrating IMU — keep robot FLAT and STILL...")
    calib = calibrate(samples=200)
    imu_filter = IMUFilter(calib)

    # ---- start IMU thread ----
    imu_thread = IMUThread(imu_filter)
    print("[INIT] IMU thread started (100 Hz)")
    print()

    # ---- detect input method ----
    use_gamepad = False
    controller  = None
    gamepad_name = ""

    if pygame is not None:
        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                controller = pygame.joystick.Joystick(0)
                controller.init()
                use_gamepad = True
                gamepad_name = controller.get_name()
        except Exception:
            pass

    if use_gamepad:
        main_gamepad(controller, gamepad_name, imu_thread)
    else:
        main_keyboard(imu_thread)


if __name__ == "__main__":
    main()