"""
TEST — Layer 6 MULTI-DIRECTIONAL WALK (Analog Joystick Edition)
================================================================

Single-cycle directional walking with keyboard or gamepad control.
Automatically detects gamepad; falls back to keyboard if none found.

Keyboard Controls:
  W/S   = Forward / Backward
  A/D   = Left strafe / Right strafe
  Q/E   = Turn left / Turn right
  C     = Cycle height mode
  1-0   = Tricks
  X     = Quit

Gamepad Controls (Analog):
  Left Stick Y        = Forward / Backward  (proportional speed)
  Left Stick X        = Strafe Left / Right  (proportional speed)
  Right Stick X       = Turn Left / Right    (proportional speed)
  Sticks combine for diagonal / arc movement.

Gamepad Controls (Digital):
  D-Pad Up/Down       = Forward / Backward   (default frequency, single cycle)
  D-Pad Left/Right    = Turn Left / Turn Right (default frequency, single cycle)
  A/B/X/Y, LB/RB, LT/RT, M1/M2 = Tricks & controls

Analog stick deflection maps to walk speed:
  Small deflection  → slow, short steps
  Full deflection   → fast, full-length steps
  Release stick     → smooth ramp-out to stand
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time
import math
import tty
import termios

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
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS

# ---------------- Tricks ----------------
from stance.tricks import (
    stand, shake, bow, wiggle, pushups,
    bheek, high_five, sit, stretch,
    tilt_dance, combo
)


# -------------------------------------------------
# GAIT CONFIG (defaults for D-pad / keyboard)
# -------------------------------------------------

FREQ = 1.5
DT = 0.02  # 50 Hz

STEP_LENGTH = 0.06
STEP_HEIGHT = 0.030
DUTY = 0.80

LATERAL_STEP_LENGTH = 0.04
LATERAL_STEP_HEIGHT = 0.020
LATERAL_FREQ = 1.5

STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

# -------------------------------------------------
# RAMP CONFIG
# -------------------------------------------------

RAMP_TIME = 0.15
RAMP_MIN  = 0.05

# -------------------------------------------------
# ANALOG JOYSTICK CONFIG
# -------------------------------------------------

ANALOG_DEADZONE   = 0.15     # Ignore stick drift below this
ANALOG_FREQ_MIN   = 0.6      # Hz at minimum deflection
ANALOG_FREQ_MAX   = 2.8      # Hz at full tilt
ANALOG_STEP_MIN   = 0.25     # Step-length multiplier at min deflection
ANALOG_STEP_MAX   = 1.0      # Step-length multiplier at full tilt

# Stick axes (Xbox-style layout — adjust if your pad differs)
AXIS_LX = 0   # Left stick X  → strafe
AXIS_LY = 1   # Left stick Y  → forward/back (inverted: up = -1)
AXIS_RX = 3   # Right stick X → turn

# Ramp-in / ramp-out rate for analog mode (units: per second)
ANALOG_RAMP_UP   = 5.0       # 0→1 in 0.2 s
ANALOG_RAMP_DOWN = 4.0       # 1→0 in 0.25 s

# -------------------------------------------------
# DIRECTION CONFIG (keyboard / D-pad)
# -------------------------------------------------

KEY_MAP = {
    'w': "forward",
    's': "backward",
    'a': "left",
    'd': "right",
    'q': "turn_left",
    'e': "turn_right",
    'c': "height",

    '1': "shake",
    '2': "bow",
    '3': "wiggle",
    '4': "pushups",
    '5': "bheek",
    '6': "high_five",
    '7': "sit",
    '8': "stretch",
    '9': "tilt_dance",
    '0': "combo",

    'x': "quit",
}

FORWARD_BACKWARD = ("forward", "backward")
LEFT_RIGHT       = ("left", "right")
TURN             = ("turn_left", "turn_right")

TURN_STEP_LENGTH = 0.04
TURN_STEP_HEIGHT = 0.025
TURN_FREQ        = 1.5

# -------------------------------------------------
# TRICK MAP
# -------------------------------------------------

TRICK_MAP = {
    "shake": shake,
    "bow": bow,
    "wiggle": wiggle,
    "pushups": pushups,
    "bheek": bheek,
    "high_five": high_five,
    "sit": sit,
    "stretch": stretch,
    "tilt_dance": tilt_dance,
    "combo": combo,
}

# -------------------------------------------------
# HEIGHT MODES
# -------------------------------------------------

HEIGHT_MODES = [
    ("HIGH",   -0.19),
    ("NORMAL", -0.18),
    ("LOW",    -0.15),
    ("CROUCH", -0.12),
]
HEIGHT_TRANSITION = 0.3

# -------------------------------------------------
# COXA PRELOAD
# -------------------------------------------------

COXA_DELTA_BIAS = {
    "FL": +1.5,
    "FR": +1.5,
    "RL": +1.5,
    "RR": +1.5,
}

# -------------------------------------------------
# Servo channel map
# -------------------------------------------------

CHANNELS = {}
for _leg in ("FL", "FR", "RL", "RR"):
    CHANNELS[f"{_leg}_COXA"]  = COXA[_leg]
    CHANNELS[f"{_leg}_THIGH"] = THIGHS[f"T{_leg}"]
    CHANNELS[f"{_leg}_WRIST"] = WRISTS[f"W{_leg}"]

# -------------------------------------------------
# Diagonal groups
# -------------------------------------------------

DIAG_A = ("FL", "RR")

# -------------------------------------------------
# Stand pose
# -------------------------------------------------

STAND_FEET = {
    "FL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
    "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
}


# =====================================================================
# HELPERS (unchanged from original)
# =====================================================================

def wrap_phase(p: float) -> float:
    return p - math.floor(p)


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


# =====================================================================
# TRAJECTORY GENERATORS
# =====================================================================

def _lateral_trajectory(phase, step_length, step_height, duty):
    if phase < duty:
        s = phase / duty
        dy = +step_length / 2 - s * step_length
        return dy, 0.0
    s = (phase - duty) / (1.0 - duty)
    dy = -step_length / 2 + s * step_length
    dz = step_height * math.sin(math.pi * s)
    return dy, dz


# ---------- D-pad / keyboard trajectory computers (unchanged) ----------

def compute_feet_forward_backward(phase, direction, stance_z=STANCE_Z):
    x_mult = 1 if direction == "forward" else -1
    feet = {}
    for leg in ("FL", "FR", "RL", "RR"):
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        dx, dz = _leg_trajectory(leg_phase, STEP_LENGTH, STEP_HEIGHT, DUTY)
        dx *= x_mult
        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X + dx, base_y, stance_z + dz)
    return feet


def compute_feet_lateral(phase, direction, stance_z=STANCE_Z):
    feet = {}
    for leg in ("FL", "FR", "RL", "RR"):
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        if direction == "left":
            leg_phase = wrap_phase(1.0 - leg_phase)
        dy, dz = _lateral_trajectory(leg_phase, LATERAL_STEP_LENGTH, LATERAL_STEP_HEIGHT, DUTY)
        y_mult = +1 if leg in ("FR", "RR") else -1
        dy *= y_mult
        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X, base_y + dy, stance_z + dz)
    return feet


def compute_feet_turn(phase, direction, stance_z=STANCE_Z):
    turn_mult = 1 if direction == "turn_left" else -1
    feet = {}
    for leg in ("FL", "FR", "RL", "RR"):
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        dx, dz = _leg_trajectory(leg_phase, TURN_STEP_LENGTH, TURN_STEP_HEIGHT, DUTY)
        if leg in ("FL", "RL"):
            dx = -dx * turn_mult
        else:
            dx = +dx * turn_mult
        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X + dx, base_y, stance_z + dz)
    return feet


def compute_feet_directional(phase, direction, stance_z=STANCE_Z):
    if direction in FORWARD_BACKWARD:
        return compute_feet_forward_backward(phase, direction, stance_z)
    elif direction in LEFT_RIGHT:
        return compute_feet_lateral(phase, direction, stance_z)
    elif direction in TURN:
        return compute_feet_turn(phase, direction, stance_z)
    else:
        return stand_at_z(stance_z)


# =====================================================================
# ANALOG BLENDED FOOT COMPUTATION
# =====================================================================

def compute_feet_analog(phase: float,
                        fwd: float, strafe: float, turn: float,
                        step_scale: float,
                        stance_z: float = STANCE_Z) -> dict:
    """
    Blended foot targets driven by three analog axes.

    Each axis contributes dx / dy / dz independently per leg.
    The vertical lift (dz) takes the MAX across all active
    components so the foot clears the ground for any combination.

    Args:
        phase:      current gait phase 0-1
        fwd:        forward(+) / backward(-) magnitude, -1..+1
        strafe:     right(+) / left(-) magnitude, -1..+1
        turn:       turn-right(+) / turn-left(-), -1..+1
        step_scale: 0..1 overall step-length multiplier from stick magnitude
        stance_z:   current stance height
    """
    feet = {}

    for leg in ("FL", "FR", "RL", "RR"):
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)

        dx_total = 0.0
        dy_total = 0.0
        dz_max   = 0.0

        # --- Forward / backward component ---
        if abs(fwd) > 0.01:
            dx_fb, dz_fb = _leg_trajectory(
                leg_phase,
                STEP_LENGTH * step_scale * abs(fwd),
                STEP_HEIGHT,
                DUTY,
            )
            dx_fb *= (1.0 if fwd > 0 else -1.0)
            dx_total += dx_fb
            dz_max = max(dz_max, dz_fb)

        # --- Strafe component ---
        if abs(strafe) > 0.01:
            s_phase = leg_phase
            # Left strafe = negative strafe value → reverse phase
            if strafe < 0:
                s_phase = wrap_phase(1.0 - s_phase)

            dy_s, dz_s = _lateral_trajectory(
                s_phase,
                LATERAL_STEP_LENGTH * step_scale * abs(strafe),
                LATERAL_STEP_HEIGHT,
                DUTY,
            )
            # Right legs mirror Y
            y_mult = +1 if leg in ("FR", "RR") else -1
            dy_s *= y_mult
            dy_total += dy_s
            dz_max = max(dz_max, dz_s)

        # --- Turn component ---
        if abs(turn) > 0.01:
            dx_t, dz_t = _leg_trajectory(
                leg_phase,
                TURN_STEP_LENGTH * step_scale * abs(turn),
                TURN_STEP_HEIGHT,
                DUTY,
            )
            # turn > 0 = turn right (clockwise):
            #   left legs push forward, right legs push backward
            # turn < 0 = turn left (counter-clockwise):
            #   left legs push backward, right legs push forward
            turn_sign = 1.0 if turn > 0 else -1.0
            if leg in ("FL", "RL"):
                dx_t = +dx_t * turn_sign
            else:
                dx_t = -dx_t * turn_sign
            dx_total += dx_t
            dz_max = max(dz_max, dz_t)

        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (
            STANCE_X + dx_total,
            base_y   + dy_total,
            stance_z + dz_max,
        )

    return feet


# =====================================================================
# SERVO PIPELINE (unchanged)
# =====================================================================

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


# =====================================================================
# D-PAD SINGLE-CYCLE EXECUTION (unchanged)
# =====================================================================

def execute_single_cycle(direction: str, stance_z: float = STANCE_Z):
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
        feet = compute_feet_directional(phase, direction, stance_z)
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


# =====================================================================
# HEIGHT TRANSITION (unchanged)
# =====================================================================

def transition_height(from_z: float, to_z: float):
    def set_all_z(z):
        execute_step(stand_at_z(z))

    t0 = time.time()
    while time.time() - t0 < HEIGHT_TRANSITION:
        t = (time.time() - t0) / HEIGHT_TRANSITION
        z = lerp(from_z, to_z, t)
        set_all_z(z)
        time.sleep(DT)
    set_all_z(to_z)


# =====================================================================
# ANALOG STICK HELPERS
# =====================================================================

def apply_deadzone(value: float, deadzone: float = ANALOG_DEADZONE) -> float:
    """Apply deadzone with rescaling so output is 0 at edge of deadzone."""
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    # Rescale so deadzone edge → 0, ±1 → ±1
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def read_sticks(controller) -> tuple:
    """
    Read analog sticks and return (fwd, strafe, turn).

    fwd:    +1 = full forward,  -1 = full backward
    strafe: +1 = full right,    -1 = full left
    turn:   +1 = turn right,    -1 = turn left
    """
    raw_lx = controller.get_axis(AXIS_LX)
    raw_ly = controller.get_axis(AXIS_LY)
    raw_rx = controller.get_axis(AXIS_RX)

    fwd    = apply_deadzone(-raw_ly)   # Invert: stick-up is negative
    strafe = apply_deadzone(raw_lx)
    turn   = apply_deadzone(raw_rx)

    return fwd, strafe, turn


def stick_magnitude(fwd: float, strafe: float, turn: float) -> float:
    """Combined magnitude, clamped to 1.0."""
    return min(1.0, math.sqrt(fwd * fwd + strafe * strafe + turn * turn))


# =====================================================================
# GAMEPAD BUTTON / D-PAD POLLING (non-blocking)
# =====================================================================

def poll_dpad(controller) -> str | None:
    """Return d-pad direction key or None."""
    hat = controller.get_hat(0)
    if hat == (0,  1): return 'w'   # Up    → forward
    if hat == (0, -1): return 's'   # Down  → backward
    if hat == (-1, 0): return 'q'   # Left  → turn left
    if hat == ( 1, 0): return 'e'   # Right → turn right
    return None


def poll_buttons(controller) -> str | None:
    """Return key for first pressed button, or None."""
    lt = controller.get_axis(2)
    rt = controller.get_axis(5)
    if lt > 0.7:                    return '3'   # LT → Wiggle
    if rt > 0.7:                    return '2'   # RT → Bow
    if controller.get_button(4):    return '1'   # LB → Shake
    if controller.get_button(5):    return '5'   # RB → Bheek
    if controller.get_button(1):    return '4'   # B  → Pushups
    if controller.get_button(0):    return '9'   # A  → Tilt Dance
    if controller.get_button(3):    return 'c'   # Y  → Height
    if controller.get_button(2):    return 'x'   # X  → Quit
    if controller.get_button(6):    return '7'   # M1 → Sit
    if controller.get_button(7):    return '6'   # M2 → High Five
    return None


# =====================================================================
# INPUT: Keyboard (terminal, unchanged)
# =====================================================================

def get_key_from_keyboard():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '\x03':
        raise KeyboardInterrupt
    return ch.lower()


# =====================================================================
# MAIN — GAMEPAD (analog polling loop)
# =====================================================================

def main_gamepad(controller, gamepad_name: str):
    print(f"[INPUT] Gamepad detected: {gamepad_name}")
    print("        Analog sticks for proportional movement.\n")
    print("Analog Controls:")
    print("  Left Stick Y       = Forward / Backward (proportional)")
    print("  Left Stick X       = Strafe Left / Right (proportional)")
    print("  Right Stick X      = Turn Left / Right   (proportional)")
    print()
    print("D-Pad (single cycle at default frequency):")
    print("  Up/Down            = Forward / Backward")
    print("  Left/Right         = Turn Left / Turn Right")
    print()
    print("Buttons:")
    print("  A = Tilt Dance     B = Pushups    X = Quit")
    print("  Y = Cycle Height   LB = Shake     RB = Bheek")
    print("  LT = Wiggle        RT = Bow")
    print("  M1 = Sit           M2 = High Five")
    print()
    print(f"Analog range: {ANALOG_FREQ_MIN:.1f} – {ANALOG_FREQ_MAX:.1f} Hz  |  "
          f"Deadzone: {ANALOG_DEADZONE:.2f}")
    _print_common_info()

    # --- State ---
    height_index = 1
    current_z    = HEIGHT_MODES[height_index][1]
    phase        = 0.0
    ramp_level   = 0.0        # 0 = stopped, 1 = full amplitude
    moving       = False
    last_fwd     = 0.0
    last_strafe  = 0.0
    last_turn    = 0.0

    # Init stand
    print("[INIT] Moving to stand pose...")
    execute_step(STAND_FEET)
    time.sleep(0.5)
    print("[INIT] Ready!")
    print("-" * 55)

    try:
        while True:
            loop_top = time.time()
            pygame.event.pump()

            # ---- 1. Check quit / trick / height buttons ----
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
                    # Debounce: wait for button release
                    _wait_button_release(controller)
                    continue

                if cmd in TRICK_MAP:
                    # Stop any analog motion first
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

            # ---- 2. Check D-pad → single-cycle at default freq ----
            dpad = poll_dpad(controller)
            if dpad is not None:
                # Stop analog motion
                if moving:
                    execute_step(stand_at_z(current_z))
                    moving = False
                    ramp_level = 0.0
                    phase = 0.0

                cmd = KEY_MAP[dpad]
                print()
                execute_single_cycle(cmd, current_z)

                # Wait for d-pad release so it doesn't repeat
                while True:
                    pygame.event.pump()
                    if controller.get_hat(0) == (0, 0):
                        break
                    time.sleep(0.02)

                time.sleep(0.05)
                continue

            # ---- 3. Analog stick movement ----
            fwd, strafe, turn = read_sticks(controller)
            mag = stick_magnitude(fwd, strafe, turn)

            if mag > 0.01:
                # --- Moving ---
                last_fwd, last_strafe, last_turn = fwd, strafe, turn

                # Map magnitude → frequency & step scale
                freq      = lerp(ANALOG_FREQ_MIN, ANALOG_FREQ_MAX, mag)
                step_scale = lerp(ANALOG_STEP_MIN,  ANALOG_STEP_MAX,  mag)

                # Ramp in
                ramp_level = min(1.0, ramp_level + ANALOG_RAMP_UP * DT)

                # Advance phase
                phase = wrap_phase(phase + freq * DT)

                # Compute blended feet & apply ramp
                feet = compute_feet_analog(phase, fwd, strafe, turn,
                                           step_scale, current_z)
                feet = apply_ramp(feet, ramp_level, current_z)
                execute_step(feet)
                moving = True

                # Status line
                bar = int(mag * 10)
                mode_name = HEIGHT_MODES[height_index][0]
                print(
                    f"\r  ANALOG [{mode_name}] "
                    f"F:{fwd:+.2f} S:{strafe:+.2f} T:{turn:+.2f} "
                    f"|{'█' * bar}{'·' * (10 - bar)}| "
                    f"{freq:.1f}Hz  ramp:{ramp_level:.0%}   ",
                    end="", flush=True,
                )

            elif moving:
                # --- Stick released: ramp out ---
                ramp_level -= ANALOG_RAMP_DOWN * DT
                if ramp_level <= RAMP_MIN:
                    # Fully stopped
                    execute_step(stand_at_z(current_z))
                    moving = False
                    ramp_level = 0.0
                    phase = 0.0
                    print(
                        f"\r  ANALOG [STAND]"
                        f"                                              ",
                    )
                else:
                    # Still ramping out — keep last direction at reducing amplitude
                    freq = lerp(ANALOG_FREQ_MIN, ANALOG_FREQ_MAX, 0.3)
                    phase = wrap_phase(phase + freq * DT)
                    feet = compute_feet_analog(
                        phase, last_fwd, last_strafe, last_turn,
                        ANALOG_STEP_MIN, current_z,
                    )
                    feet = apply_ramp(feet, ramp_level, current_z)
                    execute_step(feet)

            # ---- Timing ----
            elapsed = time.time() - loop_top
            time.sleep(max(0, DT - elapsed))

    except KeyboardInterrupt:
        print("\n\n[INTERRUPT] Caught Ctrl+C")

    finally:
        print("[SHUTDOWN] Returning to stand...")
        execute_step(STAND_FEET)
        print("[SHUTDOWN] Complete")


def _wait_button_release(controller):
    """Block until all gamepad buttons are released."""
    while True:
        pygame.event.pump()
        any_pressed = False
        for i in range(controller.get_numbuttons()):
            if controller.get_button(i):
                any_pressed = True
                break
        # Also check triggers
        if abs(controller.get_axis(2)) > 0.5 or abs(controller.get_axis(5)) > 0.5:
            any_pressed = True
        if not any_pressed:
            break
        time.sleep(0.02)


# =====================================================================
# MAIN — KEYBOARD (unchanged blocking loop)
# =====================================================================

def main_keyboard():
    if pygame is None:
        print("[INPUT] pygame not installed — using keyboard controls.\n")
    else:
        print("[INPUT] No gamepad detected — using keyboard controls.\n")

    print("Keyboard Controls:")
    print("  W = Forward          S = Backward")
    print("  A = Left Strafe      D = Right Strafe")
    print("  Q = Turn Left        E = Turn Right")
    print("  C = Cycle Height     X = Quit")
    print()
    print("Tricks:")
    print("  1 = Shake      2 = Bow        3 = Wiggle")
    print("  4 = Pushups    5 = Bheek      6 = High Five")
    print("  7 = Sit        8 = Stretch    9 = Tilt Dance")
    print("  0 = Combo")
    _print_common_info()

    print("[INIT] Moving to stand pose...")
    execute_step(STAND_FEET)
    time.sleep(0.5)
    print("[INIT] Ready!")
    print("-" * 55)

    height_index = 1
    current_z = HEIGHT_MODES[height_index][1]

    try:
        while True:
            mode_name = HEIGHT_MODES[height_index][0]
            print(f"\nWaiting for command [{mode_name}]: ", end="", flush=True)

            key = get_key_from_keyboard()
            print(key.upper())

            if key not in KEY_MAP:
                print(f"  Unknown key '{key}'.")
                continue

            command = KEY_MAP[key]

            if command == "quit":
                print("\n[QUIT] Exiting...")
                break

            elif command == "height":
                old_z = current_z
                height_index = (height_index + 1) % len(HEIGHT_MODES)
                new_name, new_z = HEIGHT_MODES[height_index]
                print(f"  Height → {new_name} (Z={new_z:.2f}m)")
                transition_height(old_z, new_z)
                current_z = new_z

            elif command in TRICK_MAP:
                print(f"  Trick: {command.upper()}")
                try:
                    TRICK_MAP[command]()
                except Exception as e:
                    print(f"  [WARN] Trick error: {e}")
                stand()

            else:
                execute_single_cycle(command, current_z)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n[INTERRUPT] Caught Ctrl+C")

    finally:
        print("[SHUTDOWN] Returning to stand...")
        execute_step(STAND_FEET)
        print("[SHUTDOWN] Complete")


# =====================================================================
# COMMON PRINT
# =====================================================================

def _print_common_info():
    print()
    print("Motion:")
    print(f"  D-pad ramp in/out: {RAMP_TIME * 1000:.0f} ms (sine ease)")
    print()
    print("Height modes:")
    for name, z in HEIGHT_MODES:
        marker = " <-- default" if name == "NORMAL" else ""
        print(f"  {name:8s}: Z={z:.2f}m{marker}")
    print()
    print("-" * 55)


# =====================================================================
# ENTRY POINT
# =====================================================================

def main():
    print("=" * 55)
    print("  MULTI-DIRECTIONAL WALK CONTROLLER (Analog + D-Pad)")
    print("=" * 55)
    print()

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

    if use_gamepad:
        main_gamepad(controller, gamepad_name)
    else:
        main_keyboard()


if __name__ == "__main__":
    main()