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

# ---------------- Layer 6 ----------------
from layer6.gait_generator import _leg_trajectory

# ---------------- Layer 3 ----------------
from layer3.leg_ik import solve_all_legs

# ---------------- Layer 2 ----------------
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

# ---------------- Hardware ----------------
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# -------------------------------------------------
# GAIT CONFIG
# -------------------------------------------------

FREQ = 1.5
DT = 0.02  # 50Hz

STEP_LENGTH = 0.06
STEP_HEIGHT = 0.030
DUTY = 0.80

# Stance values
STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18


# -------------------------------------------------
# DIRECTION CONFIG
# -------------------------------------------------

# (x_multiplier, y_multiplier)
DIRECTIONS = {
    "forward":  ( 1,  0),
    "backward": (-1,  0),
    "left":     ( 0,  1),
    "right":    ( 0, -1),
}

KEY_MAP = {
    'w': "forward",
    's': "backward",
    'a': "left",
    'd': "right",
    'q': "quit",
}


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

def compute_feet_directional(phase: float, direction: str) -> dict:
    """
    Compute foot targets for given phase and direction.
    """
    x_mult, y_mult = DIRECTIONS.get(direction, (1, 0))
    
    feet = {}
    
    for leg in ("FL", "FR", "RL", "RR"):
        # Diagonal phase shift
        if leg in DIAG_A:
            leg_phase = phase
        else:
            leg_phase = wrap_phase(phase + 0.5)
        
        # Get trajectory offset
        offset, dz = _leg_trajectory(
            leg_phase,
            STEP_LENGTH,
            STEP_HEIGHT,
            DUTY,
        )
        
        # Apply direction multipliers
        dx = offset * x_mult
        dy = offset * y_mult
        
        # Base Y position
        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        
        feet[leg] = (
            STANCE_X + dx,
            base_y + dy,
            STANCE_Z + dz,
        )
    
    return feet


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
# Keyboard input (Linux)
# -------------------------------------------------

def get_key_blocking():
    """Get single keypress (blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
        return key.lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# -------------------------------------------------
# Execute one complete gait cycle
# -------------------------------------------------

def execute_single_cycle(direction: str):
    """
    Execute exactly ONE gait cycle in the given direction.
    Returns when cycle is complete.
    """
    print(f"  Executing: {direction.upper()}", end="", flush=True)
    
    cycle_start = time.time()
    cycle_duration = 1.0 / FREQ  # Time for one complete cycle
    
    while True:
        loop_start = time.time()
        elapsed = loop_start - cycle_start
        
        # Check if cycle complete
        if elapsed >= cycle_duration:
            break
        
        # Compute phase (0 → 1 over cycle)
        phase = (elapsed / cycle_duration)
        
        # Compute and execute
        feet = compute_feet_directional(phase, direction)
        execute_step(feet)
        
        # Progress indicator
        progress = int(phase * 10)
        print(f"\r  Executing: {direction.upper()} [{'=' * progress}{' ' * (10-progress)}]", 
              end="", flush=True)
        
        # Timing
        loop_elapsed = time.time() - loop_start
        sleep_time = max(0, DT - loop_elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    print(f"\r  Executing: {direction.upper()} [==========] Done")
    
    # Return to stand
    execute_step(STAND_FEET)


# -------------------------------------------------
# MAIN
# -------------------------------------------------

def main():
    print("=" * 50)
    print("  MULTI-DIRECTIONAL WALK CONTROLLER")
    print("=" * 50)
    print()
    print("Controls:")
    print("  W = Forward")
    print("  S = Backward")
    print("  A = Left strafe")
    print("  D = Right strafe")
    print("  Q = Quit")
    print()
    print("-" * 50)
    
    # Initialize to stand pose
    print("[INIT] Moving to stand pose...")
    execute_step(STAND_FEET)
    time.sleep(0.5)
    print("[INIT] Ready!")
    print("-" * 50)
    
    try:
        while True:
            print()
            print("Waiting for command (W/A/S/D/Q): ", end="", flush=True)
            
            # Get keypress
            key = get_key_blocking()
            print(key.upper())  # Echo the key
            
            if key not in KEY_MAP:
                print(f"  Unknown key '{key}'. Use W/A/S/D/Q.")
                continue
            
            command = KEY_MAP[key]
            
            if command == "quit":
                print("\n[QUIT] Exiting...")
                break
            
            # Execute single cycle
            execute_single_cycle(command)
            
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
