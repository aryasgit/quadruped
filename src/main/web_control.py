"""
super_controller_v1.py — Unified Main, Brace, and Stability Controller
=======================================================================

Modes:
- Normal: Main controller for walking/tricks.
          Brace controller runs when idle.
          When a command arrives during brace, robot returns to neutral
          stand before executing.
- Stability: Activated by 'p' (keyboard) or button 9 (gamepad).
             All other control suspended. IMU posture correction runs.
             Auto-exits after MAX_STABILITY_TIME seconds.
             Any input shows exit prompt; press 'y' to confirm.
             After exit: smooth lerp back to stand -> normal mode.

Changes from original:
  - reset_pid  replaced with reset_reference (canonical name)
  - STAND_FEET deleted; stand_at_z(STANCE_Z) used everywhere
  - strict_stand() now takes stance_z parameter (no height snap)
  - smooth_stand() lerps _sc._dz to zero — no jerk on stability exit
  - Watchdog: os.execv removed; uses recovery_requested Event instead
  - Trick thread race fixed via _trick_stop Event
  - TrickRunner class: centralised pre/post hooks for all tricks
  - Stability mode: MAX_STABILITY_TIME auto-exit added
  - Phase continuity: phase saved/restored across stability transitions
  - BraceSuspended context manager: brace always clean entering/leaving stability
"""

import sys
import os
import time
import threading
import math
import tty
import termios
import select
from contextlib import contextmanager
import socket
from web_remote import WebControlHub

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import pygame
except ImportError:
    pygame = None

# --- Hardware / pipeline ---
from gait.generator import _leg_trajectory
from ik.solver import solve_all_legs
from joints.conventions import apply_joint_conventions
from joints.space import normalize_all
from hardware.pca9685 import init_pca, set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS
from hardware.imu import init_mpu, calibrate, IMUFilter

# --- Tricks ---
from stance.tricks import (
    stand, shake, bow, wiggle, pushups,
    bheek, high_five, sit, stretch, tilt_dance, combo
)
import stance.tricks as stance_tricks   # for cooperative trick abort

# --- Sub-controllers ---
from controllers.brace_controller import BraceController
from controllers.stability_controller import posture_step, reset_reference

# Access _dz dict live for smooth_stand lerp.
# Import as module so reassignment inside reset_reference is always visible.
import controllers.stability_controller as _sc

# NOTE: imu and brace are NOT initialised here — they require hardware
# init (init_pca / init_mpu / calibrate) which must run inside main().


# =====================================================================
# GAIT CONFIG
# =====================================================================

FREQ             = 1.3
DT               = 0.0134        # 50 Hz

STEP_LENGTH      = 0.080
STEP_HEIGHT      = 0.030
DUTY             = 0.70

# Backward gait tuning: keep support polygon safer and reduce tipping risk.
BACKWARD_STEP_LENGTH_SCALE = 0.86
BACKWARD_STEP_HEIGHT_SCALE = 1.08

# Motion-only body pitch bias.
# Positive values lean the body forward in this controller frame.
MOTION_PITCH_BIAS = 0.006
BACKWARD_PITCH_BIAS = 0.010

LATERAL_STEP_LENGTH = 0.03
LATERAL_STEP_HEIGHT = 0.018
LATERAL_FREQ        = 1.2

STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

RAMP_TIME = 0.15
RAMP_MIN  = 0.05

ANALOG_DEADZONE   = 0.15
ANALOG_FREQ_MIN   = 0.5
ANALOG_FREQ_MAX   = 2.2
ANALOG_STEP_MIN   = 0.20
ANALOG_STEP_MAX   = 0.8

AXIS_LX = 0
AXIS_LY = 1
AXIS_RX = 3

ANALOG_RAMP_UP   = 5.0
ANALOG_RAMP_DOWN = 4.0

BODY_SHIFT_FWD  = 0.010
BODY_SHIFT_LAT  = 0.008
BODY_SHIFT_TURN = 0.012

TURN_STEP_LENGTH = 0.07
TURN_STEP_HEIGHT = 0.032
TURN_FREQ        = 1.2

HEIGHT_TRANSITION = 0.3

HEIGHT_MODES = [
    ("HIGH",   -0.19),
    ("NORMAL", -0.18),
    ("LOW",    -0.15),
    ("CROUCH", -0.12),
]

COXA_DELTA_BIAS = {
    "FL": +1.5, "FR": +1.5, "RL": +1.5, "RR": +1.5,
}

# Dynamic lateral tilt via coxa offsets.
# Positive strafe command means move right; we apply opposite-side body tilt.
LATERAL_COXA_OPPOSE_GAIN = 2.5

DIAG_A = ("FL", "RR")

WATCHDOG_TIMEOUT = 5.0          # seconds — reduced from 15; os.execv is gone so safe to be aggressive
MAX_STABILITY_TIME = 60.0       # seconds — stability auto-exit
SMOOTH_STAND_FRAMES = 30        # frames for smooth_stand lerp (30 * 0.02s = 0.6s)

watchdog_last_heartbeat = [time.time()]

# Watchdog signals main loop to recover cleanly — no process restart.
recovery_requested = threading.Event()

# Trick thread abort signal — prevents race between trick thread and main thread on I2C bus.
_trick_stop = threading.Event()


# =====================================================================
# KEY / BUTTON MAP
# =====================================================================

KEY_MAP = {
    'w': "forward",    's': "backward",
    'a': "left",       'd': "right",
    'q': "turn_left",  'e': "turn_right",
    'c': "height",
    '1': "shake",      '2': "bow",       '3': "wiggle",
    '4': "pushups",    '5': "bheek",     '6': "high_five",
    '7': "sit",        '8': "stretch",   '9': "tilt_dance",
    '0': "combo",
    'x': "quit",
}

FORWARD_BACKWARD = ("forward", "backward")
LEFT_RIGHT       = ("left", "right")
TURN             = ("turn_left", "turn_right")

TRICK_MAP = {
    "shake": shake,     "bow": bow,         "wiggle": wiggle,
    "pushups": pushups, "bheek": bheek,     "high_five": high_five,
    "sit": sit,         "stretch": stretch, "tilt_dance": tilt_dance,
    "combo": combo,
}

# Stability toggle — not in KEY_MAP by design (no conflict with tricks/movement)
STABILITY_KEY            = 'p'
STABILITY_GAMEPAD_BUTTON = 9    # Back/Select

# =====================================================================
# SERVO CHANNEL MAP
# =====================================================================

CHANNELS = {}
for _leg in ("FL", "FR", "RL", "RR"):
    CHANNELS[f"{_leg}_COXA"]  = COXA[_leg]
    CHANNELS[f"{_leg}_THIGH"] = THIGHS[f"T{_leg}"]
    CHANNELS[f"{_leg}_WRIST"] = WRISTS[f"W{_leg}"]


# =====================================================================
# WATCHDOG
# =====================================================================
# FIX #5: No longer restarts the process with os.execv.
# Sets recovery_requested instead — main loop handles clean recovery.
# IMU calibration is NOT re-run. Robot recovers in-place.

def watchdog_thread():
    while True:
        time.sleep(1.0)
        elapsed = time.time() - watchdog_last_heartbeat[0]
        if elapsed > WATCHDOG_TIMEOUT:
            print(f"\n[WATCHDOG] Main loop unresponsive for {elapsed:.1f}s — requesting in-place recovery.")
            recovery_requested.set()
            # Do NOT call os.execv — robot stays live, main loop handles it.

def get_lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()

# =====================================================================
# MATH / GAIT HELPERS
# =====================================================================

def wrap_phase(p):
    w = p % 1.0
    return 0.0 if w >= 1.0 else w


def ramp_factor(elapsed, total_duration):
    if total_duration <= 2 * RAMP_TIME:
        return max(RAMP_MIN, math.sin(math.pi * elapsed / total_duration))
    if elapsed < RAMP_TIME:
        raw = math.sin((math.pi / 2) * (elapsed / RAMP_TIME))
    elif elapsed > total_duration - RAMP_TIME:
        raw = math.sin((math.pi / 2) * ((total_duration - elapsed) / RAMP_TIME))
    else:
        raw = 1.0
    return max(RAMP_MIN, raw)


def apply_ramp(feet, ramp, stance_z=STANCE_Z):
    ref = stand_at_z(stance_z)
    return {
        leg: (
            ref[leg][0] + (feet[leg][0] - ref[leg][0]) * ramp,
            ref[leg][1] + (feet[leg][1] - ref[leg][1]) * ramp,
            ref[leg][2] + (feet[leg][2] - ref[leg][2]) * ramp,
        )
        for leg in feet
    }


def stand_at_z(stance_z):
    return {
        "FL": (STANCE_X,  STANCE_Y, stance_z),
        "FR": (STANCE_X, -STANCE_Y, stance_z),
        "RL": (STANCE_X,  STANCE_Y, stance_z),
        "RR": (STANCE_X, -STANCE_Y, stance_z),
    }


def lerp(a, b, t):
    return a + (b - a) * t


def apply_motion_pitch_bias(feet, pitch_bias):
    if abs(pitch_bias) < 1e-9:
        return feet

    return {
        leg: (
            x,
            y,
            z + (pitch_bias if leg in ("FL", "FR") else -pitch_bias),
        )
        for leg, (x, y, z) in feet.items()
    }


# =====================================================================
# SERVO PIPELINE
# =====================================================================

def apply_coxa_bias(deltas, strafe_cmd=0.0):
    biased = deltas.copy()
    for leg, bias in COXA_DELTA_BIAS.items():
        key = f"{leg}_COXA"
        if key in biased:
            biased[key] += bias

    # Dynamic coxa bias to oppose lateral motion (left/right stride).
    # strafe_cmd in [-1, +1]: -1 = left, +1 = right.
    s = max(-1.0, min(1.0, strafe_cmd))
    if abs(s) > 0.01:
        for leg in ("FL", "FR", "RL", "RR"):
            key = f"{leg}_COXA"
            if key not in biased:
                continue
            side = +1.0 if leg in ("FL", "RL") else -1.0
            biased[key] += -s * side * LATERAL_COXA_OPPOSE_GAIN

    return biased


def send_to_servos(physical):
    for joint, ch in CHANNELS.items():
        set_servo_angle(ch, physical[joint])


def execute_step(feet, strafe_cmd=0.0):
    # If trick thread is being aborted, suppress servo writes from the dying thread.
    if _trick_stop.is_set():
        return False
    try:
        deltas = solve_all_legs(feet)
        deltas = apply_coxa_bias(deltas, strafe_cmd)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)
        send_to_servos(physical)
        return True
    except Exception as e:
        print(f"\n[WARN] Pipeline error: {e}")
        return False


# FIX #4: stance_z parameter — no snap to STANCE_Z if in LOW/CROUCH.
def strict_stand(stance_z=STANCE_Z):
    """Apply stand pose firmly. Use for mode transitions that need a hard reset."""
    print("[POSE] Strict stand pose...")
    for _ in range(10):
        execute_step(stand_at_z(stance_z))
        time.sleep(0.03)


def smooth_stand(stance_z=STANCE_Z, n_frames=SMOOTH_STAND_FRAMES):
    """
    Lerp stability controller dZ offsets back to zero, then settle at stand.
    Call this after stability mode exits — prevents servo jerk.
    If no stability offsets are active (e.g. watchdog recovery), runs as a
    plain slow stand which still prevents abrupt position commands.
    """
    # Snapshot current posture offsets from stability controller module-level dict.
    # _sc._dz is the live dict — safe to read even if reset_reference was called early
    # (values will just be zero, making this a no-op lerp).
    start_dz = {leg: _sc._dz[leg] for leg in ("FL", "FR", "RL", "RR")}

    for i in range(n_frames):
        t = i / n_frames
        feet = {
            leg: (x, y, z + start_dz[leg] * (1.0 - t))
            for leg, (x, y, z) in stand_at_z(stance_z).items()
        }
        execute_step(feet)
        time.sleep(DT)

    execute_step(stand_at_z(stance_z))


def run_trick_with_timeout(trick_func, timeout=10.0):
    """
    Run a trick with a timeout.

    BUG FIX: the abort now actually stops the trick. Tricks write through
    stance.tricks._send (not execute_step), so the old _trick_stop gate on
    execute_step never suppressed a runaway trick. We now raise the cooperative
    abort flag in stance.tricks, which makes _send a no-op, so a timed-out trick
    thread stops commanding servos and the main thread safely takes over. The
    bus lock also makes any brief overlap electrically harmless.
    """
    stance_tricks.clear_abort()
    _trick_stop.clear()
    result = [None]

    def target():
        try:
            trick_func()
            result[0] = True
        except Exception as e:
            print(f"[ERROR] Trick failed: {e}")
            result[0] = False

    t = threading.Thread(target=target, daemon=True)
    t.start()
    # BUG FIX: keep the watchdog heartbeat alive while the trick runs. The old
    # code did a single blocking join(timeout); tricks longer than
    # WATCHDOG_TIMEOUT (5s) would trip a spurious in-place recovery mid-trick.
    deadline = time.time() + timeout
    while t.is_alive() and time.time() < deadline:
        watchdog_last_heartbeat[0] = time.time()
        t.join(0.2)
    watchdog_last_heartbeat[0] = time.time()

    if t.is_alive():
        print("[WARN] Trick timed out — aborting trick thread.")
        stance_tricks.request_abort()   # trick's _send calls become no-ops
        _trick_stop.set()
        time.sleep(DT * 2)              # let any in-flight frame drain
        _trick_stop.clear()
        # NOTE: abort stays set until the next trick starts (clear_abort above),
        # so the zombie thread cannot resume writing servos.
        return False

    stance_tricks.clear_abort()
    _trick_stop.clear()
    return result[0]


# =====================================================================
# TRICK RUNNER — centralised pre/post hooks
# =====================================================================
# NEW: eliminates scattered pre/post logic in both keyboard and gamepad loops.
# Every trick goes through: stand at current_z → brace.reset → trick → stand().

class TrickRunner:
    """
    Wraps every trick call with consistent pre/post behaviour.

    Pre-hook : always execute stand_at_z(current_z) and reset brace,
               regardless of whether the robot was moving.
    Post-hook: call stand() to guarantee clean pose after trick.
    """

    def __init__(self, brace: BraceController, timeout: float = 10.0):
        self._brace   = brace
        self._timeout = timeout

    def run(self, trick_name: str, current_z: float) -> bool:
        func = TRICK_MAP.get(trick_name)
        if func is None:
            print(f"[TRICK] Unknown trick: {trick_name}")
            return False

        # --- pre ---
        execute_step(stand_at_z(current_z))
        self._brace.reset()
        print(f"\n  Trick: {trick_name.upper()}")

        # --- execute ---
        result = run_trick_with_timeout(func, self._timeout)

        # --- post ---
        stand()
        return result


# =====================================================================
# BRACE SUSPENDED CONTEXT MANAGER
# =====================================================================
# NEW: guarantees brace is reset on both normal exit AND exception from stability.
# Replaces the scattered brace.reset() calls before/after stability blocks.

@contextmanager
def BraceSuspended(brace: BraceController):
    """
    Context manager that resets brace on entry and exit.
    Use around any block where brace should not accumulate state
    (stability mode, tricks that transition the full body).

    Usage:
        with BraceSuspended(brace):
            run_stability_mode(...)
    """
    brace.reset()
    try:
        yield
    finally:
        brace.reset()


# =====================================================================
# TRAJECTORY GENERATORS
# =====================================================================

def _lateral_trajectory(phase, step_length, step_height, duty):
    phase = max(0.0, min(phase, 0.9999999))
    if phase < duty:
        s = phase / duty
        return +step_length / 2 - s * step_length, 0.0
    s = (phase - duty) / (1.0 - duty)
    return -step_length / 2 + s * step_length, step_height * math.sin(math.pi * s)


def should_mirror_diagonal_order(strafe, turn):
    """
    Mirror swing-order for mirrored lateral/yaw commands.
    - right / turn_right: FR/RL first (default)
    - left  / turn_left : FL/RR first (mirrored)
    """
    lat_mag = abs(strafe)
    yaw_mag = abs(turn)

    if lat_mag < 0.01 and yaw_mag < 0.01:
        return False

    if lat_mag >= yaw_mag:
        return strafe < -0.01
    return turn < -0.01


def compute_feet_analog(phase, fwd, strafe, turn, step_scale, stance_z=STANCE_Z):
    shift_x = -fwd    * BODY_SHIFT_FWD
    shift_y = -strafe * BODY_SHIFT_LAT - turn * BODY_SHIFT_TURN
    mirror_order = should_mirror_diagonal_order(strafe, turn)
    feet = {}
    for leg in ("FL", "FR", "RL", "RR"):
        if mirror_order:
            leg_phase = wrap_phase(phase + 0.5) if leg in DIAG_A else phase
        else:
            leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        dx_total = dy_total = 0.0
        dz_max = 0.0

        if abs(fwd) > 0.01:
            backward = fwd < 0.0
            fwd_len_scale = BACKWARD_STEP_LENGTH_SCALE if backward else 1.0
            fwd_height_scale = BACKWARD_STEP_HEIGHT_SCALE if backward else 1.0
            dx_fb, dz_fb = _leg_trajectory(
                leg_phase,
                STEP_LENGTH * step_scale * abs(fwd) * fwd_len_scale,
                STEP_HEIGHT * fwd_height_scale,
                DUTY,
            )
            dx_fb *= (1.0 if fwd > 0 else -1.0)
            dx_total += dx_fb
            dz_max = max(dz_max, dz_fb)

        if abs(strafe) > 0.01:
            s_phase = wrap_phase(1.0 - leg_phase) if strafe < 0 else leg_phase
            dy_s, dz_s = _lateral_trajectory(
                s_phase, LATERAL_STEP_LENGTH * step_scale * abs(strafe),
                LATERAL_STEP_HEIGHT, DUTY)
            dy_s *= +1 if leg in ("FR", "RR") else -1
            dy_total += dy_s
            dz_max = max(dz_max, dz_s)

        if abs(turn) > 0.01:
            dx_t, dz_t = _leg_trajectory(
                leg_phase, TURN_STEP_LENGTH * step_scale * abs(turn),
                TURN_STEP_HEIGHT, DUTY)
            turn_sign = 1.0 if turn > 0 else -1.0
            dx_t = +dx_t * turn_sign if leg in ("FL", "RL") else -dx_t * turn_sign
            dx_total += dx_t
            dz_max = max(dz_max, dz_t)

        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (
            STANCE_X + dx_total + shift_x,
            base_y   + dy_total + shift_y,
            stance_z + dz_max,
        )

    motion_mag = min(1.0, math.sqrt(fwd * fwd + strafe * strafe + turn * turn))
    pitch_bias = MOTION_PITCH_BIAS * motion_mag
    if fwd < -0.01:
        pitch_bias += BACKWARD_PITCH_BIAS * min(1.0, abs(fwd))

    feet = apply_motion_pitch_bias(feet, pitch_bias)
    return feet


def compute_feet_directional(phase, direction, stance_z=STANCE_Z):
    if direction == "forward":      return compute_feet_analog(phase,  1.0, 0.0, 0.0, 1.0, stance_z)
    elif direction == "backward":   return compute_feet_analog(phase, -1.0, 0.0, 0.0, 1.0, stance_z)
    elif direction == "left":       return compute_feet_analog(phase, 0.0, -1.0, 0.0, 1.0, stance_z)
    elif direction == "right":      return compute_feet_analog(phase, 0.0,  1.0, 0.0, 1.0, stance_z)
    elif direction == "turn_left":  return compute_feet_analog(phase, 0.0, 0.0, -1.0, 1.0, stance_z)
    elif direction == "turn_right": return compute_feet_analog(phase, 0.0, 0.0,  1.0, 1.0, stance_z)
    else:                           return stand_at_z(stance_z)


# =====================================================================
# SINGLE-CYCLE EXECUTION
# =====================================================================

def execute_single_cycle(direction, stance_z=STANCE_Z):
    if direction in LEFT_RIGHT:
        freq, num_cycles, label = LATERAL_FREQ, 2, f"{direction.upper()} (lateral x2)"
    elif direction in TURN:
        freq, num_cycles, label = TURN_FREQ, 1, direction.upper().replace('_', ' ')
    else:
        freq, num_cycles, label = FREQ, 1, direction.upper()

    print(f"  Executing: {label}", end="", flush=True)
    cycle_start = time.time()
    single_cycle_duration = 1.0 / freq
    total_duration = single_cycle_duration * num_cycles

    lateral_cmd = -1.0 if direction == "left" else (1.0 if direction == "right" else 0.0)

    while True:
        loop_start = time.time()
        elapsed = loop_start - cycle_start
        if elapsed >= total_duration:
            break
        phase = (elapsed / single_cycle_duration) % 1.0
        feet = apply_ramp(
            compute_feet_directional(phase, direction, stance_z),
            ramp_factor(elapsed, total_duration), stance_z)
        execute_step(feet, lateral_cmd)
        progress = int((elapsed / total_duration) * 10)
        print(
            f"\r  Executing: {label} "
            f"[{'='*progress}{' '*(10-progress)}] "
            f"ramp:{int(ramp_factor(elapsed, total_duration)*100):3d}%",
            end="", flush=True)
        time.sleep(max(0, DT - (time.time() - loop_start)))

    print(f"\r  Executing: {label} [==========] ramp:  0% Done")
    execute_step(stand_at_z(stance_z))


def transition_height(from_z, to_z):
    t0 = time.time()
    while time.time() - t0 < HEIGHT_TRANSITION:
        execute_step(stand_at_z(lerp(from_z, to_z, (time.time() - t0) / HEIGHT_TRANSITION)))
        time.sleep(DT)
    execute_step(stand_at_z(to_z))


# =====================================================================
# ANALOG STICK HELPERS
# =====================================================================

def apply_deadzone(value, deadzone=ANALOG_DEADZONE):
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def read_sticks(controller):
    return (
        apply_deadzone(-controller.get_axis(AXIS_LY)),
        apply_deadzone( controller.get_axis(AXIS_LX)),
        apply_deadzone( controller.get_axis(AXIS_RX)),
    )


def stick_magnitude(fwd, strafe, turn):
    return min(1.0, math.sqrt(fwd*fwd + strafe*strafe + turn*turn))


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
    if lt > 0.7:                   return '3'
    if rt > 0.7:                   return '2'
    if controller.get_button(4):   return '1'
    if controller.get_button(5):   return '5'
    if controller.get_button(1):   return '4'
    if controller.get_button(0):   return '9'
    if controller.get_button(3):   return 'c'
    if controller.get_button(2):   return 'x'
    if controller.get_button(6):   return '7'
    if controller.get_button(7):   return '6'
    return None


def _wait_button_release(controller, timeout_s=1.5):
    """
    Wait for gamepad inputs to settle after a command button press.

    Uses the same trigger thresholds as poll_buttons() and includes a timeout
    so noisy axes cannot deadlock the main loop.
    """
    t0 = time.time()
    while True:
        pygame.event.pump()
        any_pressed = any(controller.get_button(i) for i in range(controller.get_numbuttons()))

        # Match press semantics used in poll_buttons(): triggers count only when
        # pushed high, not by absolute magnitude (some pads rest near -1.0).
        lt = controller.get_axis(2)
        rt = controller.get_axis(5)
        if lt > 0.7 or rt > 0.7:
            any_pressed = True

        if not any_pressed:
            break

        if time.time() - t0 > timeout_s:
            print("[INPUT] Release wait timeout; continuing.")
            break

        time.sleep(0.02)


# =====================================================================
# KEYBOARD INPUT HELPERS
# =====================================================================

def get_key_blocking():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    if ch == '\x03':
        raise KeyboardInterrupt
    return ch.lower()


def get_key_timeout(timeout_s):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if ready:
            ch = sys.stdin.read(1)
            if ch == '\x03':
                raise KeyboardInterrupt
            return ch.lower()
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# =====================================================================
# STABILITY MODE
# =====================================================================
# CHANGES:
#   - Takes stance_z so stand commands match current height mode
#   - MAX_STABILITY_TIME auto-exit added
#   - reset_reference() only at entry; smooth_stand in CALLER handles exit
#   - Comment added above send_to_servos — posture_step output is already physical
#   - brace.reset() removed — BraceSuspended context manager handles it in caller

def run_stability_mode(imu, use_gamepad, controller, stance_z=STANCE_Z):
    """
    Runs stability controller exclusively until exit is confirmed or timeout.

    stance_z  : current height mode Z — stand commands use this, not STANCE_Z
    Returns   : nothing. Caller calls smooth_stand(stance_z) then reset_reference().
    """
    print("\n[STABILITY] Stability mode ACTIVE — IMU posture control running.")
    print(f"[STABILITY] Auto-exit after {MAX_STABILITY_TIME:.0f}s.")
    if use_gamepad:
        print("[STABILITY] Press any button/stick to show exit prompt.")
        print("[STABILITY] Then press button 9 to confirm exit.")
    else:
        print("[STABILITY] Press any key to show exit prompt, then 'y' to confirm.")

    # Clean entry — zero any stale posture state from previous sessions.
    reset_reference()

    t_enter = time.time()

    while True:
        watchdog_last_heartbeat[0] = time.time()
        loop_top = time.time()

        # --- Auto-exit on timeout ---
        if time.time() - t_enter > MAX_STABILITY_TIME:
            print(f"\n[STABILITY] Auto-timeout after {MAX_STABILITY_TIME:.0f}s. Exiting.")
            break

        # --- Run stability controller ---
        if imu is not None:
            try:
                # posture_step returns fully-processed physical angles.
                # DO NOT pipe through execute_step — that would double-apply conventions.
                physical = posture_step(stand_at_z(stance_z), imu)
                send_to_servos(physical)
            except Exception as e:
                print(f"\n[WARN] Stability step error: {e}")
                execute_step(stand_at_z(stance_z))
        else:
            execute_step(stand_at_z(stance_z))

        # --- Check for exit input ---
        if use_gamepad:
            pygame.event.pump()
            any_input = any(
                controller.get_button(i) for i in range(controller.get_numbuttons()))
            fwd, strafe, turn = read_sticks(controller)
            if stick_magnitude(fwd, strafe, turn) > 0.3:
                any_input = True

            if any_input:
                _wait_button_release(controller)
                print("\n[STABILITY] Input detected. Press button 9 within 5s to confirm exit...")
                t_wait = time.time()
                confirmed = False
                while time.time() - t_wait < 5.0:
                    pygame.event.pump()
                    execute_step(stand_at_z(stance_z))
                    if controller.get_button(STABILITY_GAMEPAD_BUTTON):
                        confirmed = True
                        _wait_button_release(controller)
                        break
                    time.sleep(0.05)
                if confirmed:
                    print("[STABILITY] Confirmed. Exiting stability mode.")
                    break
                else:
                    print("[STABILITY] Not confirmed. Staying in stability mode.")
        else:
            key = get_key_timeout(DT)
            if key is not None:
                print(
                    f"\n[STABILITY] Input ('{key}') detected. "
                    f"Press 'y' to exit stability mode: ",
                    end="", flush=True)
                confirm = get_key_timeout(5.0)
                if confirm == 'y':
                    print("Y")
                    print("[STABILITY] Confirmed. Exiting stability mode.")
                    break
                else:
                    shown = confirm.upper() if confirm else "(timeout)"
                    print(shown)
                    print("[STABILITY] Cancelled. Staying in stability mode.")

        time.sleep(max(0, DT - (time.time() - loop_top)))

    # NOTE: do NOT call reset_reference() here.
    # Caller reads _sc._dz via smooth_stand() to lerp offsets to zero first,
    # then calls reset_reference() to clean up state.
    print("[STABILITY] Stability mode deactivated.")

def run_stability_mode_web(imu, web, stance_z=STANCE_Z):
    print("\n[STABILITY] Stability mode ACTIVE — IMU posture control running.")
    print(f"[STABILITY] Auto-exit after {MAX_STABILITY_TIME:.0f}s.")
    print("[STABILITY] Press any button or move a stick to show exit prompt.")
    print("[STABILITY] Then press P to confirm exit.")

    reset_reference()
    t_enter = time.time()

    while True:
        watchdog_last_heartbeat[0] = time.time()
        loop_top = time.time()

        if time.time() - t_enter > MAX_STABILITY_TIME:
            print(f"\n[STABILITY] Auto-timeout after {MAX_STABILITY_TIME:.0f}s. Exiting.")
            break

        if imu is not None:
            try:
                physical = posture_step(stand_at_z(stance_z), imu)
                send_to_servos(physical)
            except Exception as e:
                print(f"\n[WARN] Stability step error: {e}")
                execute_step(stand_at_z(stance_z))
        else:
            execute_step(stand_at_z(stance_z))

        # Browser input handling
        if web.has_activity():
            web.wait_for_idle()
            print("\n[STABILITY] Input detected. Press P within 5s to confirm exit...")
            t_wait = time.time()
            confirmed = False

            while time.time() - t_wait < 5.0:
                execute_step(stand_at_z(stance_z))
                cmd = web.poll_command()
                if cmd == STABILITY_KEY:
                    confirmed = True
                    break
                time.sleep(0.05)

            if confirmed:
                print("[STABILITY] Confirmed. Exiting stability mode.")
                break
            else:
                print("[STABILITY] Not confirmed. Staying in stability mode.")

        time.sleep(max(0, DT - (time.time() - loop_top)))

    print("[STABILITY] Stability mode deactivated.")

# =====================================================================
# GAMEPAD MAIN LOOP
# =====================================================================

def main_web(imu, brace, web):
    print("[INPUT] Web remote mode.")
    print("[INFO] Open the phone browser at the shown URL.")
    print("[INFO] P = stability mode, X = quit.")
    print("Move: left stick = fwd/strafe, right stick = turn")
    print("Buttons: 1-0 tricks, C height, W/A/S/D/Q/E single steps")
    print("-" * 55)

    height_index = 1
    current_z = HEIGHT_MODES[height_index][1]
    phase = 0.0
    ramp_level = 0.0
    moving = False
    last_fwd = last_strafe = last_turn = 0.0

    trick_runner = TrickRunner(brace)
    brace_was_active = False

    execute_step(stand_at_z(current_z))
    time.sleep(0.5)
    print("[INIT] Ready!\n")

    try:
        while True:
            watchdog_last_heartbeat[0] = time.time()

            if recovery_requested.is_set():
                print("\n[WATCHDOG] In-place recovery: returning to stand...")
                smooth_stand(current_z)
                reset_reference()
                phase = 0.0
                ramp_level = 0.0
                moving = False
                recovery_requested.clear()
                watchdog_last_heartbeat[0] = time.time()
                continue

            loop_top = time.time()

            # Priority 1: queued button command from phone
            cmd = web.poll_command()

            if cmd == STABILITY_KEY:
                saved_phase = phase
                with BraceSuspended(brace):
                    run_stability_mode_web(imu, web, current_z)

                smooth_stand(current_z)
                reset_reference()
                phase = saved_phase
                ramp_level = 0.0
                moving = False
                continue

            if cmd is not None and cmd in KEY_MAP:
                mapped = KEY_MAP[cmd]

                if brace_was_active:
                    print("[BRACE] Returning to neutral before command...")
                    brace.reset()
                    for _ in range(5):
                        execute_step(stand_at_z(current_z))
                        time.sleep(0.03)

                if mapped == "quit":
                    print("\n[QUIT] Exiting...")
                    break

                elif mapped == "height":
                    old_z = current_z
                    height_index = (height_index + 1) % len(HEIGHT_MODES)
                    new_name, new_z = HEIGHT_MODES[height_index]
                    print(f"  Height -> {new_name} (Z={new_z:.2f}m)")
                    transition_height(old_z, new_z)
                    current_z = new_z
                    phase = 0.0
                    ramp_level = 0.0
                    moving = False

                elif mapped in TRICK_MAP:
                    trick_runner.run(mapped, current_z)
                    moving = False
                    ramp_level = 0.0
                    phase = 0.0

                else:
                    # single-cycle movements
                    execute_single_cycle(mapped, current_z)

                time.sleep(0.1)
                continue

            # Priority 2: continuous analog sticks
            fwd, strafe, turn = web.read_sticks()
            mag = stick_magnitude(fwd, strafe, turn)

            if mag > 0.01:
                last_fwd, last_strafe, last_turn = fwd, strafe, turn
                freq = lerp(ANALOG_FREQ_MIN, ANALOG_FREQ_MAX, mag)
                step_scale = lerp(ANALOG_STEP_MIN, ANALOG_STEP_MAX, mag)
                ramp_level = min(1.0, ramp_level + ANALOG_RAMP_UP * DT)
                phase = wrap_phase(phase + freq * DT)

                feet = apply_ramp(
                    compute_feet_analog(phase, fwd, strafe, turn, step_scale, current_z),
                    ramp_level,
                    current_z,
                )
                execute_step(feet, strafe)
                moving = True
                brace_was_active = False

                bar = int(mag * 10)
                print(
                    f"\r  ANALOG [{HEIGHT_MODES[height_index][0]}] "
                    f"F:{fwd:+.2f} S:{strafe:+.2f} T:{turn:+.2f} "
                    f"|{'|' * bar}{'.' * (10 - bar)}| "
                    f"{freq:.1f}Hz ramp:{ramp_level:.0%}   ",
                    end="",
                    flush=True
                )

            elif moving:
                ramp_level -= ANALOG_RAMP_DOWN * DT
                if ramp_level <= RAMP_MIN:
                    execute_step(stand_at_z(current_z))
                    moving = False
                    ramp_level = 0.0
                    phase = 0.0
                    print(f"\r  ANALOG [STAND]                                              ")
                else:
                    freq = lerp(ANALOG_FREQ_MIN, ANALOG_FREQ_MAX, 0.3)
                    phase = wrap_phase(phase + freq * DT)
                    feet = apply_ramp(
                        compute_feet_analog(
                            phase, last_fwd, last_strafe, last_turn,
                            ANALOG_STEP_MIN, current_z
                        ),
                        ramp_level,
                        current_z,
                    )
                    execute_step(feet, last_strafe)

            else:
                # idle: brace controller
                if imu is not None:
                    offsets = brace.update(imu)
                    brace_was_active = brace.active
                    brace_feet = {
                        leg: (x, y, z + offsets[leg])
                        for leg, (x, y, z) in stand_at_z(current_z).items()
                    }
                    execute_step(brace_feet)
                else:
                    execute_step(stand_at_z(current_z))
                    brace_was_active = False

            time.sleep(max(0, DT - (time.time() - loop_top)))

    except KeyboardInterrupt:
        print("\n[INTERRUPT] Caught Ctrl+C")
    finally:
        print("[SHUTDOWN] Returning to stand...")
        execute_step(stand_at_z(STANCE_Z))
        print("[SHUTDOWN] Done.")

        
def main_gamepad(controller, gamepad_name, imu, brace):
    print(f"[INPUT] Gamepad: {gamepad_name}")
    print("[INFO] Button 9 (Back/Select) = stability mode toggle.\n")
    print("Analog sticks: Left Y=Fwd/Back  Left X=Strafe  Right X=Turn")
    print("D-Pad: single-cycle movement   Buttons: tricks / height / quit")
    print("-" * 55)

    height_index = 1
    current_z    = HEIGHT_MODES[height_index][1]
    phase        = 0.0
    ramp_level   = 0.0
    moving       = False
    last_fwd = last_strafe = last_turn = 0.0

    trick_runner = TrickRunner(brace)

    execute_step(stand_at_z(current_z))
    time.sleep(0.5)
    print("[INIT] Ready!\n")

    try:
        while True:
            watchdog_last_heartbeat[0] = time.time()

            # --- Watchdog recovery (FIX #5: replaces os.execv restart) ---
            if recovery_requested.is_set():
                print("\n[WATCHDOG] In-place recovery: returning to stand...")
                smooth_stand(current_z)
                reset_reference()
                phase = 0.0; ramp_level = 0.0; moving = False
                recovery_requested.clear()
                watchdog_last_heartbeat[0] = time.time()
                continue

            loop_top = time.time()
            pygame.event.pump()

            # --- Stability mode trigger ---
            if controller.get_button(STABILITY_GAMEPAD_BUTTON):
                _wait_button_release(controller)
                if moving:
                    execute_step(stand_at_z(current_z))
                    moving = False; ramp_level = 0.0

                # Save phase for continuity — restoring avoids foot-slam on first walk frame.
                saved_phase = phase

                with BraceSuspended(brace):
                    run_stability_mode(imu, True, controller, current_z)

                # Smooth exit: lerp stability dZ offsets to zero, THEN reset state.
                smooth_stand(current_z)
                reset_reference()

                # Restore phase continuity.
                phase = saved_phase
                ramp_level = 0.0
                moving = False
                continue

            # --- Trick / height / quit buttons ---
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
                    print(f"\n  Height -> {new_name} (Z={new_z:.2f}m)")
                    transition_height(old_z, new_z)
                    current_z = new_z
                    phase = 0.0; ramp_level = 0.0; moving = False
                    _wait_button_release(controller)
                    continue

                if cmd in TRICK_MAP:
                    trick_runner.run(cmd, current_z)
                    moving = False; ramp_level = 0.0; phase = 0.0
                    _wait_button_release(controller)
                    continue

            # --- D-pad single cycle ---
            dpad = poll_dpad(controller)
            if dpad is not None:
                if moving:
                    execute_step(stand_at_z(current_z))
                    moving = False; ramp_level = 0.0; phase = 0.0
                execute_single_cycle(KEY_MAP[dpad], current_z)
                while controller.get_hat(0) != (0, 0):
                    pygame.event.pump()
                    time.sleep(0.02)
                continue

            # --- Analog sticks ---
            fwd, strafe, turn = read_sticks(controller)
            mag = stick_magnitude(fwd, strafe, turn)

            if mag > 0.01:
                last_fwd, last_strafe, last_turn = fwd, strafe, turn
                freq       = lerp(ANALOG_FREQ_MIN, ANALOG_FREQ_MAX, mag)
                step_scale = lerp(ANALOG_STEP_MIN,  ANALOG_STEP_MAX,  mag)
                ramp_level = min(1.0, ramp_level + ANALOG_RAMP_UP * DT)
                phase = wrap_phase(phase + freq * DT)
                feet = apply_ramp(
                    compute_feet_analog(phase, fwd, strafe, turn, step_scale, current_z),
                    ramp_level, current_z)
                execute_step(feet, strafe)
                moving = True
                bar = int(mag * 10)
                print(
                    f"\r  ANALOG [{HEIGHT_MODES[height_index][0]}] "
                    f"F:{fwd:+.2f} S:{strafe:+.2f} T:{turn:+.2f} "
                    f"|{'|'*bar}{'.'*(10-bar)}| {freq:.1f}Hz ramp:{ramp_level:.0%}   ",
                    end="", flush=True)

            elif moving:
                ramp_level -= ANALOG_RAMP_DOWN * DT
                if ramp_level <= RAMP_MIN:
                    execute_step(stand_at_z(current_z))
                    moving = False; ramp_level = 0.0; phase = 0.0
                    print(f"\r  ANALOG [STAND]                                              ")
                else:
                    freq = lerp(ANALOG_FREQ_MIN, ANALOG_FREQ_MAX, 0.3)
                    phase = wrap_phase(phase + freq * DT)
                    feet = apply_ramp(
                        compute_feet_analog(
                            phase, last_fwd, last_strafe, last_turn,
                            ANALOG_STEP_MIN, current_z),
                        ramp_level, current_z)
                    execute_step(feet, last_strafe)

            else:
                # IDLE: brace controller active
                if imu is not None:
                    offsets = brace.update(imu)
                    brace_feet = {
                        leg: (x, y, z + offsets[leg])
                        for leg, (x, y, z) in stand_at_z(current_z).items()
                    }
                    execute_step(brace_feet)
                else:
                    execute_step(stand_at_z(current_z))

            time.sleep(max(0, DT - (time.time() - loop_top)))

    except KeyboardInterrupt:
        print("\n[INTERRUPT] Caught Ctrl+C")
    finally:
        print("[SHUTDOWN] Returning to stand...")
        execute_step(stand_at_z(STANCE_Z))
        print("[SHUTDOWN] Done.")


# =====================================================================
# KEYBOARD MAIN LOOP
# =====================================================================

def main_keyboard(imu, brace):
    print("[INPUT] Keyboard mode.")
    print("[INFO] 'p' = stability mode toggle.  'x' = quit.")
    print("Move: W/S=Fwd/Back  A/D=Strafe  Q/E=Turn  C=Height")
    print("Tricks: 1=Shake 2=Bow 3=Wiggle 4=Pushups 5=Bheek")
    print("        6=HiFive 7=Sit 8=Stretch 9=TiltDance 0=Combo")
    print("-" * 55)

    execute_step(stand_at_z(STANCE_Z))
    time.sleep(0.5)
    print("[INIT] Ready!\n")

    height_index = 1
    current_z    = HEIGHT_MODES[height_index][1]
    phase        = 0.0
    trick_runner = TrickRunner(brace)

    try:
        while True:
            watchdog_last_heartbeat[0] = time.time()

            # --- Watchdog recovery ---
            if recovery_requested.is_set():
                print("\n[WATCHDOG] In-place recovery: returning to stand...")
                smooth_stand(current_z)
                reset_reference()
                phase = 0.0
                recovery_requested.clear()
                watchdog_last_heartbeat[0] = time.time()
                continue

            mode_name = HEIGHT_MODES[height_index][0]
            brace_str = "brace+imu" if imu is not None else "no imu"
            print(f"\nWaiting [{mode_name}] ({brace_str}): ", end="", flush=True)

            # Non-blocking key wait: run brace steps between polls.
            key = None
            brace_was_active = False
            while key is None:
                watchdog_last_heartbeat[0] = time.time()
                key = get_key_timeout(DT)
                if key is None:
                    if imu is not None:
                        offsets = brace.update(imu)
                        # FIX #3: brace.active property now exists in BraceController
                        if brace.active:
                            brace_was_active = True
                        brace_feet = {
                            leg: (x, y, z + offsets[leg])
                            for leg, (x, y, z) in stand_at_z(current_z).items()
                        }
                        execute_step(brace_feet)
                    else:
                        execute_step(stand_at_z(current_z))

            print(key.upper())

            # --- Stability mode ---
            if key == STABILITY_KEY:
                saved_phase = phase

                with BraceSuspended(brace):
                    run_stability_mode(imu, False, None, current_z)

                smooth_stand(current_z)
                reset_reference()
                phase = saved_phase
                continue

            if key not in KEY_MAP:
                print(f"  Unknown key '{key}'.")
                continue

            command = KEY_MAP[key]

            # Return to neutral if brace was compensating before executing command.
            if brace_was_active:
                print("[BRACE] Returning to neutral before command...")
                brace.reset()
                for _ in range(5):
                    execute_step(stand_at_z(current_z))
                    time.sleep(0.03)

            if command == "quit":
                print("\n[QUIT] Exiting...")
                break

            elif command == "height":
                old_z = current_z
                height_index = (height_index + 1) % len(HEIGHT_MODES)
                new_name, new_z = HEIGHT_MODES[height_index]
                print(f"  Height -> {new_name} (Z={new_z:.2f}m)")
                transition_height(old_z, new_z)
                current_z = new_z

            elif command in TRICK_MAP:
                trick_runner.run(command, current_z)

            else:
                execute_single_cycle(command, current_z)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INTERRUPT] Caught Ctrl+C")
    finally:
        print("[SHUTDOWN] Returning to stand...")
        execute_step(stand_at_z(STANCE_Z))
        print("[SHUTDOWN] Done.")


# =====================================================================
# ENTRY POINT
# =====================================================================

def main():
    print("=" * 55)
    print("  SUPER CONTROLLER  (Main + Brace + Stability + Web)")
    print("=" * 55)

    # BUG FIX: guard hardware init. A hiccup here used to crash with a raw
    # traceback before the server/watchdog even existed; now it fails cleanly.
    try:
        print("[INIT] PCA9685 servo driver...")
        init_pca()

        print("[INIT] MPU6050 IMU...")
        init_mpu()

        print("[INIT] Calibrating IMU — keep robot FLAT and STILL...")
        calib = calibrate(samples=200)
        imu = IMUFilter(calib)
        print("[INIT] IMU ready.")
    except Exception as e:
        print(f"[INIT][FATAL] Hardware init failed: {e}")
        print("[INIT] Check I2C wiring/power (PCA9685 @0x40, MPU6050 @0x68 on bus 7).")
        return

    brace = BraceController()

    web = WebControlHub(deadzone=ANALOG_DEADZONE)
    web.start(host="0.0.0.0", port=8000)

    ip = get_lan_ip()
    print(f"[WEB] Open this on your phone:")
    print(f"[WEB] http://{ip}:8000")
    print("[WEB] Keep the phone and Jetson on the same Wi-Fi network.")

    watchdog_last_heartbeat[0] = time.time()
    threading.Thread(target=watchdog_thread, daemon=True).start()

    main_web(imu, brace, web)


if __name__ == "__main__":
    main()