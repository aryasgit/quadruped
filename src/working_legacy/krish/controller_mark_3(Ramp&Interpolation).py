"""
TEST — Layer 6 MULTI-DIRECTIONAL WALK
=====================================

Single-cycle directional walking with keyboard or gamepad control.
Automatically detects gamepad; falls back to keyboard if none found.

Keyboard Controls:
  W/S   = Forward / Backward
  A/D   = Left strafe / Right strafe
  Q/E   = Turn left / Turn right
  C     = Cycle height mode
  1-0   = Tricks
  X     = Quit

Gamepad Controls:
  D-Pad Up/Down     = Forward / Backward
  D-Pad Left/Right  = Turn Left / Turn Right
  A/B/X/Y, LB/RB, LT/RT, M1/M2 = Tricks & controls

After each input, robot executes ONE complete gait cycle,
then returns to stand and waits for next command.
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
# GAIT CONFIG
# -------------------------------------------------

FREQ = 1.5
DT = 0.02  # 50Hz

STEP_LENGTH = 0.06
STEP_HEIGHT = 0.030
DUTY = 0.80

# LATERAL-SPECIFIC CONFIG (smaller steps for stability)
LATERAL_STEP_LENGTH = 0.04   # Smaller lateral steps
LATERAL_STEP_HEIGHT = 0.020  # Lower lift
LATERAL_FREQ = 1.5           # Same as forward/backward

# Stance values
STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

# -------------------------------------------------
# RAMP CONFIG
# -------------------------------------------------
# Duration (in seconds) over which stride amplitude ramps
# from 0 → full at the start, and full → 0 at the end
# of each gait cycle.  This eliminates the position
# discontinuity that would otherwise jerk the servos.
RAMP_TIME = 0.15            # 150 ms ramp at each end
RAMP_MIN  = 0.05            # Floor so ramp never fully zeroes out
                             # (avoids division issues in IK)

# -------------------------------------------------
# DIRECTION CONFIG
# -------------------------------------------------

KEY_MAP = {
    'w': "forward",
    's': "backward",
    'a': "left",
    'd': "right",
    'q': "turn_left",
    'e': "turn_right",
    'c': "height",

    # tricks
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

# Direction categories
FORWARD_BACKWARD = ("forward", "backward")
LEFT_RIGHT = ("left", "right")
TURN = ("turn_left", "turn_right")

# Turn config
TURN_STEP_LENGTH = 0.04  # How much each leg moves during turn
TURN_STEP_HEIGHT = 0.025
TURN_FREQ = 1.5

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
# Height mode config (4 levels, cycled with C key)
# More negative = higher (legs more extended)
HEIGHT_MODES = [
    ("HIGH",   -0.19),   # Raised stance
    ("NORMAL", -0.18),   # Default stance (STANCE_Z)
    ("LOW",    -0.15),   # Lowered stance
    ("CROUCH", -0.12),   # Full crouch
]
HEIGHT_TRANSITION = 0.3  # Time to transition between heights


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
# Auto-generate servo channel map
# -------------------------------------------------

CHANNELS = {}
for _leg in ("FL", "FR", "RL", "RR"):
    CHANNELS[f"{_leg}_COXA"] = COXA[_leg]
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


# -------------------------------------------------
# Helper: phase wrapping
# -------------------------------------------------

def wrap_phase(p: float) -> float:
    return p - math.floor(p)


# -------------------------------------------------
# Helper: ramp factor
# -------------------------------------------------

def ramp_factor(elapsed: float, total_duration: float) -> float:
    """
    Returns a smooth 0→1→…→1→0 envelope over the cycle.

    Uses a sine-based ease-in / ease-out so that the first and
    last RAMP_TIME seconds taper gently rather than linearly.
    This produces smoother servo angle changes at cycle boundaries.

        elapsed = 0              → 0.0  (standstill)
        elapsed = RAMP_TIME      → 1.0  (full stride)
        elapsed = total - RAMP_T → 1.0  (full stride)
        elapsed = total          → 0.0  (standstill)
        middle portion           → 1.0

    The output is clamped to [RAMP_MIN, 1.0] so the multiplier
    never fully zeroes — avoids foot positions collapsing onto
    the body origin, which would cause IK singularities.
    """
    if total_duration <= 2 * RAMP_TIME:
        # Cycle is shorter than both ramps combined.
        # Use a single half-sine across the whole duration.
        raw = math.sin(math.pi * elapsed / total_duration)
        return max(RAMP_MIN, raw)

    if elapsed < RAMP_TIME:
        # Ease-in: sine curve 0 → 1
        raw = math.sin((math.pi / 2) * (elapsed / RAMP_TIME))
    elif elapsed > total_duration - RAMP_TIME:
        # Ease-out: sine curve 1 → 0
        remaining = total_duration - elapsed
        raw = math.sin((math.pi / 2) * (remaining / RAMP_TIME))
    else:
        raw = 1.0

    return max(RAMP_MIN, raw)


# -------------------------------------------------
# Helper: apply ramp to feet
# -------------------------------------------------

def apply_ramp(feet: dict, ramp: float, stance_z: float = STANCE_Z) -> dict:
    """
    Scale foot displacements (relative to neutral stand) by `ramp`.

    Each foot position is:
        foot = stand + displacement
    We scale only the displacement:
        foot_ramped = stand + displacement * ramp

    This gracefully tapers movement at the start and end of a cycle
    so that servos receive gradually changing angles instead of a
    sudden jump from the stand pose to mid-stride.
    """
    # Build stand reference at the given height
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

        # Displacement from stand
        dx = fx - sx
        dy = fy - sy
        dz = fz - sz

        # Scale displacement by ramp factor
        ramped[leg] = (
            sx + dx * ramp,
            sy + dy * ramp,
            sz + dz * ramp,
        )

    return ramped


# -------------------------------------------------
# Helper: compute foot targets with direction
# -------------------------------------------------

def _lateral_trajectory(phase: float, step_length: float, step_height: float, duty: float):
    """
    Lateral (Y-axis) trajectory generator.
    Same math as _leg_trajectory but for sideways movement.
    
    Returns: (dy, dz) - lateral offset and vertical lift
    """
    # STANCE phase: foot drags in -Y direction (for left strafe)
    if phase < duty:
        s = phase / duty
        dy = +step_length / 2 - s * step_length  # +step/2 → -step/2
        dz = 0.0
        return dy, dz
    
    # SWING phase: foot lifts and returns to start position
    s = (phase - duty) / (1.0 - duty)
    dy = -step_length / 2 + s * step_length  # -step/2 → +step/2
    dz = step_height * math.sin(math.pi * s)
    
    return dy, dz


def compute_feet_forward_backward(phase: float, direction: str, stance_z: float = STANCE_Z) -> dict:
    """
    Compute foot targets for forward/backward movement.
    Uses X-axis trajectory.
    """
    # Direction multiplier: forward=+1, backward=-1
    x_mult = 1 if direction == "forward" else -1
    
    feet = {}
    
    for leg in ("FL", "FR", "RL", "RR"):
        # Diagonal phase shift
        if leg in DIAG_A:
            leg_phase = phase
        else:
            leg_phase = wrap_phase(phase + 0.5)
        
        # Get X-axis trajectory
        dx, dz = _leg_trajectory(
            leg_phase,
            STEP_LENGTH,
            STEP_HEIGHT,
            DUTY,
        )
        
        # Apply direction
        dx = dx * x_mult
        
        # Base Y position (fixed for forward/backward)
        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        
        feet[leg] = (STANCE_X + dx, base_y, stance_z + dz)
    
    return feet


def compute_feet_lateral(phase: float, direction: str, stance_z: float = STANCE_Z) -> dict:
    """
    Compute foot targets for left/right strafe.

    For right strafe: all feet push in -Y (body moves +Y).
    Left legs get -dy, right legs get +dy so each side pushes outward.
    Left strafe: reverse the phase so the trajectory runs backwards.
    """
    feet = {}

    for leg in ("FL", "FR", "RL", "RR"):
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)

        if direction == "left":
            leg_phase = wrap_phase(1.0 - leg_phase)

        dy, dz = _lateral_trajectory(
            leg_phase,
            LATERAL_STEP_LENGTH,
            LATERAL_STEP_HEIGHT,
            DUTY,
        )

        # Right legs move opposite Y to left legs so all feet push outward.
        # Single explicit multiplier — no double flip.
        y_mult = +1 if leg in ("FR", "RR") else -1
        dy = dy * y_mult

        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X, base_y + dy, stance_z + dz)

    return feet


def compute_feet_turn(phase: float, direction: str, stance_z: float = STANCE_Z) -> dict:
    """
    Compute foot targets for turning in place.
    
    Turn left: Left legs push backward, right legs push forward
    Turn right: Left legs push forward, right legs push backward
    
    This creates rotation about the body center.
    """
    # Direction: turn_left = +1 (counter-clockwise), turn_right = -1 (clockwise)
    turn_mult = 1 if direction == "turn_left" else -1
    
    feet = {}
    
    for leg in ("FL", "FR", "RL", "RR"):
        # Diagonal phase shift
        if leg in DIAG_A:
            leg_phase = phase
        else:
            leg_phase = wrap_phase(phase + 0.5)
        
        # Get trajectory (reuse forward/backward trajectory for X motion)
        dx, dz = _leg_trajectory(
            leg_phase,
            TURN_STEP_LENGTH,
            TURN_STEP_HEIGHT,
            DUTY,
        )
        
        # Left legs vs right legs move opposite directions
        # For turn_left: left legs go backward (-dx), right legs go forward (+dx)
        if leg in ("FL", "RL"):  # Left side
            dx = -dx * turn_mult
        else:  # Right side (FR, RR)
            dx = +dx * turn_mult
        
        # Base Y position
        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        
        feet[leg] = (STANCE_X + dx, base_y, stance_z + dz)
    
    return feet


def compute_feet_directional(phase: float, direction: str, stance_z: float = STANCE_Z) -> dict:
    """
    Unified interface - routes to appropriate trajectory generator.
    """
    if direction in FORWARD_BACKWARD:
        return compute_feet_forward_backward(phase, direction, stance_z)
    elif direction in LEFT_RIGHT:
        return compute_feet_lateral(phase, direction, stance_z)
    elif direction in TURN:
        return compute_feet_turn(phase, direction, stance_z)
    else:
        # Return standing pose at current stance_z
        return {
            "FL": (STANCE_X,  STANCE_Y, stance_z),
            "FR": (STANCE_X, -STANCE_Y, stance_z),
            "RL": (STANCE_X,  STANCE_Y, stance_z),
            "RR": (STANCE_X, -STANCE_Y, stance_z),
        }


# -------------------------------------------------
# Helper: apply coxa bias
# -------------------------------------------------

def apply_coxa_bias(deltas: dict) -> dict:
    biased = deltas.copy()
    for leg, bias in COXA_DELTA_BIAS.items():
        key = f"{leg}_COXA"
        if key in biased:
            biased[key] += bias
    return biased


# -------------------------------------------------
# Helper: send to servos
# -------------------------------------------------

def send_to_servos(physical: dict):
    for joint, ch in CHANNELS.items():
        set_servo_angle(ch, physical[joint])


# -------------------------------------------------
# Helper: execute step
# -------------------------------------------------

def execute_step(feet: dict) -> bool:
    """Execute full pipeline. Returns True on success."""
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


# -------------------------------------------------
# Input: Gamepad (pygame)
# -------------------------------------------------

def get_key_blocking(controller):
    """Blocking gamepad input — polls controller until a button/hat is pressed."""
    while True:
        pygame.event.pump()

        # ---------------- D-PAD ----------------
        hat = controller.get_hat(0)

        if hat == (0, 1):
            return 'w'

        elif hat == (0, -1):
            return 's'

        elif hat == (-1, 0):
            return 'q'

        elif hat == (1, 0):
            return 'e'

        # ---------------- TRIGGERS ----------------

        lt = controller.get_axis(2)
        rt = controller.get_axis(5)

        if lt > 0.7:
            return '3'     # Bow

        if rt > 0.7:
            return '2'     # Stretch
        # ---------------- BUTTONS ----------------

        # LB
        if controller.get_button(4):
            return '1'

        # RB
        if controller.get_button(5):
            return '5'

        # B
        if controller.get_button(1):
            return '4'

        # A
        if controller.get_button(0):
            return '9'

        # Y
        if controller.get_button(3):
            return 'c'

        # X
        if controller.get_button(2):
            return 'x'


        # ---------------- M1 / M2 ----------------

        # M1
        if controller.get_button(6):
            return '7'

        # M2
        if controller.get_button(7):
            return '6'


        time.sleep(0.01)


# -------------------------------------------------
# Input: Keyboard (terminal)
# -------------------------------------------------

def get_key_from_keyboard():
    """Blocking keyboard input — reads a single keypress from terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # raw mode swallows Ctrl+C — re-raise it manually
    if ch == '\x03':
        raise KeyboardInterrupt

    return ch.lower()


# -------------------------------------------------
# Execute one complete gait cycle (with ramp)
# -------------------------------------------------

def execute_single_cycle(direction: str, stance_z: float = STANCE_Z):
    """
    Execute gait cycles in the given direction with smooth ramp-in
    and ramp-out at the start and end of the movement.

    The ramp scales foot displacement from the stand pose:
      - First RAMP_TIME seconds: amplitude grows 0 → 1  (ease in)
      - Middle portion:          amplitude stays at 1    (full stride)
      - Last RAMP_TIME seconds:  amplitude shrinks 1 → 0 (ease out)

    This means the servos receive gradually changing target angles
    instead of an abrupt jump from stand to mid-stride, producing
    smooth, jerk-free motion on start and stop.

    Forward/Backward: 1 cycle
    Left/Right:       2 cycles  (smaller steps, need more to cover distance)
    Turn:             1 cycle
    """
    # Select frequency and number of cycles based on direction type
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
        
        # Check if all cycles complete
        if elapsed >= total_duration:
            break
        
        # Compute phase (0 → 1 per cycle, wraps for multiple cycles)
        phase = (elapsed / single_cycle_duration) % 1.0
        
        # --- Compute raw foot targets ---
        feet = compute_feet_directional(phase, direction, stance_z)

        # --- Apply ramp envelope ---
        ramp = ramp_factor(elapsed, total_duration)
        feet = apply_ramp(feet, ramp, stance_z)

        # --- Send to servos ---
        execute_step(feet)
        
        # Progress indicator with ramp visualization
        progress = int((elapsed / total_duration) * 10)
        ramp_pct = int(ramp * 100)
        print(
            f"\r  Executing: {label} "
            f"[{'=' * progress}{' ' * (10 - progress)}] "
            f"ramp:{ramp_pct:3d}%",
            end="", flush=True
        )
        
        # Timing
        loop_elapsed = time.time() - loop_start
        sleep_time = max(0, DT - loop_elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    print(
        f"\r  Executing: {label} "
        f"[==========] ramp:  0% Done"
    )
    
    # Return to stand at current stance height
    stand_feet = {
        "FL": (STANCE_X,  STANCE_Y, stance_z),
        "FR": (STANCE_X, -STANCE_Y, stance_z),
        "RL": (STANCE_X,  STANCE_Y, stance_z),
        "RR": (STANCE_X, -STANCE_Y, stance_z),
    }
    execute_step(stand_feet)


# -------------------------------------------------
# Height transition
# -------------------------------------------------

def transition_height(from_z: float, to_z: float):
    """
    Smoothly transition between stance heights.
    """
    def lerp(a, b, t):
        return a + (b - a) * t
    
    def set_all_z(z):
        feet = {
            "FL": (STANCE_X,  STANCE_Y, z),
            "FR": (STANCE_X, -STANCE_Y, z),
            "RL": (STANCE_X,  STANCE_Y, z),
            "RR": (STANCE_X, -STANCE_Y, z),
        }
        execute_step(feet)
    
    t0 = time.time()
    while time.time() - t0 < HEIGHT_TRANSITION:
        t = (time.time() - t0) / HEIGHT_TRANSITION
        z = lerp(from_z, to_z, t)
        set_all_z(z)
        time.sleep(DT)
    set_all_z(to_z)


# -------------------------------------------------
# MAIN
# -------------------------------------------------

def main():
    # --------------------------------------------------
    # Detect input method: gamepad first, keyboard fallback
    # --------------------------------------------------
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

    # --------------------------------------------------
    # Banner
    # --------------------------------------------------
    print("=" * 55)
    print("  MULTI-DIRECTIONAL WALK CONTROLLER")
    print("=" * 55)
    print()

    if use_gamepad:
        print(f"[INPUT] Gamepad detected: {gamepad_name}")
        print("        Use gamepad buttons for movement and tricks.\n")
        print("Gamepad Controls:")
        print("  D-Pad Up/Down      = Forward / Backward")
        print("  D-Pad Left/Right   = Turn Left / Turn Right")
        print("  A                  = Tilt Dance")
        print("  B                  = Pushups")
        print("  X                  = Quit")
        print("  Y                  = Cycle Height")
        print("  LB                 = Shake")
        print("  RB                 = Bheek")
        print("  LT                 = Wiggle")
        print("  RT                 = Bow")
        print("  M1                 = Sit")
        print("  M2                 = High Five")
        input_fn = lambda: get_key_blocking(controller)
    else:
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
        input_fn = get_key_from_keyboard

    print()
    print("Motion:")
    print(f"  Ramp in/out: {RAMP_TIME * 1000:.0f} ms (sine ease)")
    print()
    print("Height modes:")
    for name, z in HEIGHT_MODES:
        marker = " <-- default" if name == "NORMAL" else ""
        print(f"  {name:8s}: Z={z:.2f}m{marker}")
    print()
    print("-" * 55)
    
    # Initialize to stand pose
    print("[INIT] Moving to stand pose...")
    execute_step(STAND_FEET)
    time.sleep(0.5)
    print("[INIT] Ready!")
    print("-" * 55)
    
    # Height mode state (start at NORMAL which is index 1)
    height_index = 1  # NORMAL
    current_z = HEIGHT_MODES[height_index][1]
    
    try:
        while True:
            print()
            mode_name = HEIGHT_MODES[height_index][0]
            print(f"Waiting for command [{mode_name}]: ", end="", flush=True)
            
            # Get input (gamepad or keyboard — transparent)
            key = input_fn()
            print(key.upper())  # Echo the key
            
            if key not in KEY_MAP:
                print(f"  Unknown key '{key}'. Use valid controls.")
                continue
            
            command = KEY_MAP[key]
            
            if command == "quit":
                print("\n[QUIT] Exiting...")
                break

            elif command == "height":
                old_z = current_z
                height_index = (height_index + 1) % len(HEIGHT_MODES)
                new_name, new_z = HEIGHT_MODES[height_index]
                print(f"  Changing to {new_name} (Z={new_z:.2f}m)...")
                transition_height(old_z, new_z)
                current_z = new_z

            elif command in TRICK_MAP:
                print(f"  Executing trick: {command.upper()}")

                try:
                    TRICK_MAP[command]()   # run trick
                except Exception as e:
                    print(f"  [WARN] Trick error: {e}")

                # return robot to stand afterwards
                stand()

            else:
                # Execute gait movement
                execute_single_cycle(command, current_z)
            
            # Brief pause at stand
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\n[INTERRUPT] Caught Ctrl+C")
    
    finally:
        # Ensure robot is at stand pose
        print("[SHUTDOWN] Returning to stand...")
        execute_step(STAND_FEET)
        print("[SHUTDOWN] Complete")


if __name__ == "__main__":
    main()