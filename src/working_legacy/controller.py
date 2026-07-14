"""
TEST — Layer 6 MULTI-DIRECTIONAL WALK
=====================================

Single-cycle directional walking with keyboard control.

Controls:
  W = Forward (1 cycle)
  S = Backward (1 cycle)
  A = Left strafe (1 cycle)
  D = Right strafe (1 cycle)
  Q = Quit

After each keypress, robot executes ONE complete gait cycle,
then returns to stand and waits for next command.
"""

import time
import math
import sys
import tty
import termios
import select

import threading

from arya.perception import run_perception, get_obstacle_state, set_robot_cmd
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
    # Applied in IK delta space, before sign flip in joint_conventions.
    # After sign × servo_dir both sides resolve to the same geometric direction.
    # +1.5 produces a geometric toe-in of ~1.5° on all four legs.
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
DIAG_B = ("FR", "RL")


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
            leg_phase = 1.0 - leg_phase

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
# Keyboard / gamepad input
# -------------------------------------------------
def get_key_keyboard():
    import tty, termios, arya.perception as perception
    perception.VERBOSE = False
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        perception.VERBOSE = True
    return key if key else None

def get_key_blocking(controller):
    import pygame
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
        # your logs show these mirror A/B events

        # M1
        if controller.get_button(6):
            return '7'

        # M2
        if controller.get_button(7):
            return '6'


        time.sleep(0.01)


# -------------------------------------------------
# Execute one complete gait cycle
# -------------------------------------------------

def execute_single_cycle(direction: str, stance_z: float = STANCE_Z):
    """
    Execute gait cycles in the given direction.
    Forward/Backward: 1 cycle
    Left/Right: 2 cycles
    Returns when complete.
    """
    set_robot_cmd(direction)
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
        obs = get_obstacle_state()
        if obs["zone"] == "stop":
            print(f"\n  [OBSTACLE] STOP — {obs['distance_m']:.2f}m")
            execute_step({
                "FL": (STANCE_X,  STANCE_Y, stance_z),
                "FR": (STANCE_X, -STANCE_Y, stance_z),
                "RL": (STANCE_X,  STANCE_Y, stance_z),
                "RR": (STANCE_X, -STANCE_Y, stance_z),
            })
            return
        loop_start = time.time()
        elapsed = loop_start - cycle_start
        
        # Check if all cycles complete
        if elapsed >= total_duration:
            break
        
        # Compute phase (0 → 1 per cycle, wraps for multiple cycles)
        phase = (elapsed / single_cycle_duration) % 1.0
        
        # Compute and execute
        feet = compute_feet_directional(phase, direction, stance_z)
        execute_step(feet)
        
        # Progress indicator (shows progress across all cycles)
        progress = int((elapsed / total_duration) * 10)
        print(f"\r  Executing: {label} [{'=' * progress}{' ' * (10-progress)}]", 
              end="", flush=True)
        
        # Timing
        loop_elapsed = time.time() - loop_start
        sleep_time = max(0, DT - loop_elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    print(f"\r  Executing: {label} [==========] Done")
    
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
    #import pygame
    #pygame.init()
    #pygame.joystick.init()
    #controller = pygame.joystick.Joystick(0)
    #controller.init()

    print("=" * 50)
    print("  MULTI-DIRECTIONAL WALK CONTROLLER")
    print("=" * 50)
    print()
    print("Controls:")
    print("  W = Forward")
    print("  S = Backward")
    print("  A = Left strafe")
    print("  D = Right strafe")
    print("  Q = Turn left")
    print("  E = Turn right")
    print("  C = Cycle height mode")

    print("  1 = Shake")
    print("  2 = Bow")
    print("  3 = Wiggle")
    print("  4 = Pushups")
    print("  5 = Bheek")
    print("  6 = High Five")
    print("  7 = Sit")
    print("  8 = Stretch")
    print("  9 = Tilt Dance")
    print("  0 = Combo")

    print("  X = Quit")
    print()
    print("Height modes:")
    for name, z in HEIGHT_MODES:
        marker = " <-- default" if name == "NORMAL" else ""
        print(f"  {name:8s}: Z={z:.2f}m{marker}")
    print()
    print("-" * 50)
    
    # Initialize to stand pose
    print("[INIT] Moving to stand pose...")
    execute_step(STAND_FEET)
    print("[INIT] Starting perception thread...")
    perc_thread = threading.Thread(target=run_perception, daemon=True)
    perc_thread.start()
    time.sleep(4.0)
    print("[INIT] Perception ready.")
    time.sleep(0.5)
    print("[INIT] Ready!")
    print("-" * 50)
    
    # Height mode state (start at NORMAL which is index 1)
    height_index = 1  # NORMAL
    current_z = HEIGHT_MODES[height_index][1]
    
    try:
        while True:
            print()
            mode_name = HEIGHT_MODES[height_index][0]
            print(f"Waiting for command (W/A/S/D/Q/E/C/X) [{mode_name}]: ", end="", flush=True)
            
            # Get keypress
            key = get_key_keyboard()
            if key is None:
                continue
            print(key.upper())  # Echo the key
            
            if key not in KEY_MAP:
                print(f"  Unknown key '{key}'. Use W/A/S/D/Q/E/C/X.")
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
        set_robot_cmd("idle")
        print("[SHUTDOWN] Complete")


if __name__ == "__main__":
    main()
