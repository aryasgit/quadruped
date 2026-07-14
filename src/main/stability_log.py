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

Logging additions (PATCH):
  - StabilityLogger imported from tools.stability_logger
  - run_stability_mode() gains optional logger= parameter
  - logger.start() called at stability entry
  - logger.record(imu) called after every posture_step tick
  - logger.save() called in callers (gamepad + keyboard) after mode exits
  - sc_module=_sc passed at logger construction so _dz is snapshotted live
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

# --- Sub-controllers ---
from controllers.brace_controller import BraceController
from controllers.stability_controller import posture_step, reset_reference

# Access _dz dict live for smooth_stand lerp.
import controllers.stability_controller as _sc

# ── PATCH: stability data logger ──────────────────────────────────────
from tools.stability_logger import StabilityLogger
# ──────────────────────────────────────────────────────────────────────


# =====================================================================
# GAIT CONFIG
# =====================================================================

FREQ             = 1.3
DT               = 0.02        # 50 Hz

STEP_LENGTH      = 0.10
STEP_HEIGHT      = 0.045
DUTY             = 0.80

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

DIAG_A = ("FL", "RR")

WATCHDOG_TIMEOUT = 5.0
MAX_STABILITY_TIME = 60.0
SMOOTH_STAND_FRAMES = 30

watchdog_last_heartbeat = [time.time()]
recovery_requested = threading.Event()
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

STABILITY_KEY            = 'p'
STABILITY_GAMEPAD_BUTTON = 9


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

def watchdog_thread():
    while True:
        time.sleep(1.0)
        elapsed = time.time() - watchdog_last_heartbeat[0]
        if elapsed > WATCHDOG_TIMEOUT:
            print(f"\n[WATCHDOG] Main loop unresponsive for {elapsed:.1f}s — requesting in-place recovery.")
            recovery_requested.set()


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


# =====================================================================
# SERVO PIPELINE
# =====================================================================

def apply_coxa_bias(deltas):
    biased = deltas.copy()
    for leg, bias in COXA_DELTA_BIAS.items():
        key = f"{leg}_COXA"
        if key in biased:
            biased[key] += bias
    return biased


def send_to_servos(physical):
    for joint, ch in CHANNELS.items():
        set_servo_angle(ch, physical[joint])


def execute_step(feet):
    if _trick_stop.is_set():
        return False
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


def strict_stand(stance_z=STANCE_Z):
    print("[POSE] Strict stand pose...")
    for _ in range(10):
        execute_step(stand_at_z(stance_z))
        time.sleep(0.03)


def smooth_stand(stance_z=STANCE_Z, n_frames=SMOOTH_STAND_FRAMES):
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
    t.join(timeout)

    if t.is_alive():
        print("[WARN] Trick timed out — aborting trick thread.")
        _trick_stop.set()
        time.sleep(DT * 2)
        _trick_stop.clear()
        return False

    _trick_stop.clear()
    return result[0]


# =====================================================================
# TRICK RUNNER
# =====================================================================

class TrickRunner:
    def __init__(self, brace: BraceController, timeout: float = 10.0):
        self._brace   = brace
        self._timeout = timeout

    def run(self, trick_name: str, current_z: float) -> bool:
        func = TRICK_MAP.get(trick_name)
        if func is None:
            print(f"[TRICK] Unknown trick: {trick_name}")
            return False
        execute_step(stand_at_z(current_z))
        self._brace.reset()
        print(f"\n  Trick: {trick_name.upper()}")
        result = run_trick_with_timeout(func, self._timeout)
        stand()
        return result


# =====================================================================
# BRACE SUSPENDED CONTEXT MANAGER
# =====================================================================

@contextmanager
def BraceSuspended(brace: BraceController):
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


def compute_feet_analog(phase, fwd, strafe, turn, step_scale, stance_z=STANCE_Z):
    shift_x = -fwd    * BODY_SHIFT_FWD
    shift_y = -strafe * BODY_SHIFT_LAT - turn * BODY_SHIFT_TURN
    feet = {}
    for leg in ("FL", "FR", "RL", "RR"):
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        dx_total = dy_total = 0.0
        dz_max = 0.0

        if abs(fwd) > 0.01:
            dx_fb, dz_fb = _leg_trajectory(
                leg_phase, STEP_LENGTH * step_scale * abs(fwd), STEP_HEIGHT, DUTY)
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

    while True:
        loop_start = time.time()
        elapsed = loop_start - cycle_start
        if elapsed >= total_duration:
            break
        phase = (elapsed / single_cycle_duration) % 1.0
        feet = apply_ramp(
            compute_feet_directional(phase, direction, stance_z),
            ramp_factor(elapsed, total_duration), stance_z)
        execute_step(feet)
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


def _wait_button_release(controller):
    while True:
        pygame.event.pump()
        any_pressed = any(controller.get_button(i) for i in range(controller.get_numbuttons()))
        if abs(controller.get_axis(2)) > 0.5 or abs(controller.get_axis(5)) > 0.5:
            any_pressed = True
        if not any_pressed:
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
# STABILITY COUNTDOWN
# =====================================================================

# Big block digits — each is a list of 5 strings, 5 chars wide.
_BIG_DIGITS = {
    '5': [
        " ███ ",
        " █   ",
        " ███ ",
        "   █ ",
        " ███ ",
    ],
    '4': [
        " █ █ ",
        " █ █ ",
        " ███ ",
        "   █ ",
        "   █ ",
    ],
    '3': [
        " ███ ",
        "   █ ",
        " ███ ",
        "   █ ",
        " ███ ",
    ],
    '2': [
        " ███ ",
        "   █ ",
        " ███ ",
        " █   ",
        " ███ ",
    ],
    '1': [
        "  █  ",
        "  █  ",
        "  █  ",
        "  █  ",
        "  █  ",
    ],
    'G': [
        " ███ ",
        " █   ",
        " █ █ ",
        " █ █ ",
        " ███ ",
    ],
    'O': [
        " ███ ",
        " █ █ ",
        " █ █ ",
        " █ █ ",
        " ███ ",
    ],
    '!': [
        "  █  ",
        "  █  ",
        "  █  ",
        "     ",
        "  █  ",
    ],
}

def _print_big(chars, prefix=''):
    """Print a string of characters in large block font, all on one line."""
    rows = [''] * 5
    for ch in chars:
        glyph = _BIG_DIGITS.get(ch, ['      '] * 5)
        for i in range(5):
            rows[i] += glyph[i] + '  '
    sys.stdout.write('\n')
    for row in rows:
        sys.stdout.write(prefix + row + '\n')
    sys.stdout.write('\n')
    sys.stdout.flush()


STABILITY_COUNTDOWN_SECS = 5   # change to 3 if you want a shorter wait


def stability_countdown(imu, stance_z, countdown=STABILITY_COUNTDOWN_SECS):
    """
    Counts down from `countdown` to GO, printing large block digits.
    The robot continues standing and running posture_step on each tick
    so it is actively stable when you give the sync knock.

    Logger has NOT started yet — the knock will be the first sharp event.
    """
    print("\n" + "─" * 44)
    print("  STAND BY — STABILITY MODE STARTING IN:")
    print("─" * 44)

    for n in range(countdown, 0, -1):
        _print_big(str(n), prefix='    ')

        # Hold this digit on screen for 1 second, running servo loop throughout.
        t0 = time.time()
        while time.time() - t0 < 1.0:
            watchdog_last_heartbeat[0] = time.time()
            loop_top = time.time()
            if imu is not None:
                try:
                    physical = posture_step(stand_at_z(stance_z), imu)
                    send_to_servos(physical)
                except Exception:
                    execute_step(stand_at_z(stance_z))
            else:
                execute_step(stand_at_z(stance_z))
            elapsed = time.time() - loop_top
            time.sleep(max(0, DT - elapsed))

    _print_big('GO!', prefix='    ')
    print("─" * 44)
    print("  LOGGING STARTED — GIVE SYNC KNOCK NOW")
    print("─" * 44 + "\n")


# =====================================================================
# STABILITY MODE  (PATCHED — logger= parameter added)
# =====================================================================

def run_stability_mode(
    imu,
    use_gamepad,
    controller,
    stance_z=STANCE_Z,
    logger=None,                    # ── PATCH: optional StabilityLogger ──
):
    """
    Runs stability controller exclusively until exit is confirmed or timeout.

    stance_z : current height mode Z
    logger   : StabilityLogger instance, or None to skip logging.
               Pass a pre-constructed StabilityLogger — this function calls
               logger.start() at entry and records every tick, but does NOT
               call logger.save(); the caller is responsible for that so it
               can handle the returned path.
    """
    print("\n[STABILITY] Stability mode triggered.")
    if use_gamepad:
        print("[STABILITY] Press any button/stick to show exit prompt.")
        print("[STABILITY] Then press button 9 to confirm exit.")
    else:
        print("[STABILITY] Press any key to show exit prompt, then 'y' to confirm.")

    # ── Countdown: robot stays live, logger has not started yet ───────
    # Robot is actively stabilising during the countdown so it is settled
    # before you give the sync knock. The knock is the first sharp event
    # in the log — used to align video to JSON in extract_frames.py.
    stability_countdown(imu, stance_z)
    # ──────────────────────────────────────────────────────────────────

    reset_reference()

    # ── PATCH: start logger clock — AFTER countdown ───────────────────
    if logger is not None:
        logger.start()
    print(f"\n[STABILITY] Stability mode ACTIVE — logging running.")
    print(f"[STABILITY] Auto-exit after {MAX_STABILITY_TIME:.0f}s.")
    # ──────────────────────────────────────────────────────────────────

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

                # ── PATCH: log roll/pitch + dZ offsets after every tick ────
                # imu filter has been updated inside posture_step, so
                # imu.roll / imu.pitch are current-frame values here.
                if logger is not None:
                    logger.record(imu)
                # ──────────────────────────────────────────────────────────

            except Exception as e:
                print(f"\n[WARN] Stability step error: {e}")
                execute_step(stand_at_z(stance_z))
        else:
            execute_step(stand_at_z(stance_z))
            # Note: no logging branch when imu is None — nothing useful to record.

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

    print("[STABILITY] Stability mode deactivated.")


# =====================================================================
# GAMEPAD MAIN LOOP  (PATCHED — logger constructed + saved)
# =====================================================================

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

                saved_phase = phase

                # ── PATCH: construct logger for this stability session ─────
                logger = StabilityLogger(
                    output_dir="logs",
                    stance_z=current_z,
                    sc_module=_sc,
                )
                # ──────────────────────────────────────────────────────────

                with BraceSuspended(brace):
                    run_stability_mode(
                        imu, True, controller, current_z,
                        logger=logger,              # ── PATCH ──
                    )

                smooth_stand(current_z)
                reset_reference()

                # ── PATCH: flush log to disk after mode exits cleanly ──────
                json_path, csv_path = logger.save()
                print(f"[LOGGER] JSON → {json_path}")
                print(f"[LOGGER] CSV  → {csv_path}")
                # ──────────────────────────────────────────────────────────

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
                execute_step(feet)
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
                    execute_step(feet)

            else:
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
# KEYBOARD MAIN LOOP  (PATCHED — logger constructed + saved)
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

            key = None
            brace_was_active = False
            while key is None:
                watchdog_last_heartbeat[0] = time.time()
                key = get_key_timeout(DT)
                if key is None:
                    if imu is not None:
                        offsets = brace.update(imu)
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

                # ── PATCH: construct logger for this stability session ─────
                logger = StabilityLogger(
                    output_dir="logs",
                    stance_z=current_z,
                    sc_module=_sc,
                )
                # ──────────────────────────────────────────────────────────

                with BraceSuspended(brace):
                    run_stability_mode(
                        imu, False, None, current_z,
                        logger=logger,              # ── PATCH ──
                    )

                smooth_stand(current_z)
                reset_reference()

                # ── PATCH: flush log to disk ───────────────────────────────
                json_path, csv_path = logger.save()
                print(f"[LOGGER] JSON → {json_path}")
                print(f"[LOGGER] CSV  → {csv_path}")
                # ──────────────────────────────────────────────────────────

                phase = saved_phase
                continue

            if key not in KEY_MAP:
                print(f"  Unknown key '{key}'.")
                continue

            command = KEY_MAP[key]

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
    print("  SUPER CONTROLLER  (Main + Brace + Stability)")
    print("=" * 55)

    print("[INIT] PCA9685 servo driver...")
    init_pca()

    print("[INIT] MPU6050 IMU...")
    init_mpu()

    print("[INIT] Calibrating IMU — keep robot FLAT and STILL...")
    calib = calibrate(samples=200)
    imu = IMUFilter(calib)
    print("[INIT] IMU ready.")

    brace = BraceController()

    use_gamepad = False
    controller = None
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

    watchdog_last_heartbeat[0] = time.time()
    threading.Thread(target=watchdog_thread, daemon=True).start()

    if use_gamepad:
        main_gamepad(controller, gamepad_name, imu, brace)
    else:
        main_keyboard(imu, brace)


if __name__ == "__main__":
    main()