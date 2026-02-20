"""
WALK CONTROLLER with IMU INTEGRATION
====================================

Combines:
- Multi-directional walking (W/A/S/D/Q/E)
- IMU-based posture control (active when idle)

Behavior:
- While waiting for command: balance controller runs continuously
- During walking cycle: balance controller pauses
- After walk cycle completes: balance controller resumes
"""

import time
import math
import sys
import tty
import termios
import select

# ---------------- Layer 6 ----------------
from layer6.gait_generator import _leg_trajectory

# ---------------- Layer 5 ----------------
from layer5.posture_controller import (
    posture_step,
    get_current_imu_state,
    compute_z_compensation,
    compute_coxa_compensation,
)

# ---------------- Layer 3 ----------------
from layer3.leg_ik import solve_all_legs

# ---------------- Layer 2 ----------------
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

# ---------------- Hardware ----------------
from hardware.pca9685 import set_servo_angle
from hardware.imu import IMUFilter, init_mpu, calibrate
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# -------------------------------------------------
# GAIT CONFIG
# -------------------------------------------------

FREQ = 1.5
DT = 0.02  # 50Hz

STEP_LENGTH = 0.06
STEP_HEIGHT = 0.030
DUTY = 0.80

# Lateral-specific config
LATERAL_STEP_LENGTH = 0.04
LATERAL_STEP_HEIGHT = 0.020
LATERAL_FREQ = 1.5

# Turn config
TURN_STEP_LENGTH = 0.04
TURN_STEP_HEIGHT = 0.025
TURN_FREQ = 1.5

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
    'x': "quit",
}

FORWARD_BACKWARD = ("forward", "backward")
LEFT_RIGHT = ("left", "right")
TURN = ("turn_left", "turn_right")


# -------------------------------------------------
# HEIGHT MODE CONFIG
# -------------------------------------------------

HEIGHT_MODES = [
    ("HIGH",   -0.20),
    ("NORMAL", -0.18),
    ("LOW",    -0.15),
    ("CROUCH", -0.12),
]
HEIGHT_TRANSITION = 0.3


# -------------------------------------------------
# SAFETY LIMITS (for feasibility checks)
# -------------------------------------------------

Z_MIN = -0.22   # Maximum leg extension (most negative)
Z_MAX = -0.10   # Minimum leg extension (least negative, folded)
MAX_TILT_FOR_CROUCH = 10.0   # Max roll/pitch (deg) allowed for CROUCH mode
MAX_TILT_FOR_LOW = 15.0      # Max roll/pitch (deg) allowed for LOW mode


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
    CHANNELS[f"{_leg}_COXA"] = COXA[_leg]
    CHANNELS[f"{_leg}_THIGH"] = THIGHS[f"T{_leg}"]
    CHANNELS[f"{_leg}_WRIST"] = WRISTS[f"W{_leg}"]


# -------------------------------------------------
# Diagonal groups
# -------------------------------------------------

DIAG_A = ("FL", "RR")
DIAG_B = ("FR", "RL")


# -------------------------------------------------
# Helpers
# -------------------------------------------------

def wrap_phase(p: float) -> float:
    return p - math.floor(p)


def get_stand_feet(stance_z: float) -> dict:
    """Get standing foot positions at given height."""
    return {
        "FL": (STANCE_X,  STANCE_Y, stance_z),
        "FR": (STANCE_X, -STANCE_Y, stance_z),
        "RL": (STANCE_X,  STANCE_Y, stance_z),
        "RR": (STANCE_X, -STANCE_Y, stance_z),
    }


def apply_z_compensation(feet: dict, z_comp: dict) -> dict:
    """Apply per-leg Z compensation from IMU to foot targets."""
    result = {}
    for leg, (x, y, z) in feet.items():
        result[leg] = (x, y, z + z_comp.get(leg, 0.0))
    return result


def check_z_feasibility(feet: dict, z_comp: dict) -> tuple:
    """
    Check if foot positions are within safe Z range.
    Returns: (is_feasible, reason_string)
    """
    for leg, (x, y, z) in feet.items():
        final_z = z + z_comp.get(leg, 0.0)
        if final_z < Z_MIN:
            return False, f"{leg} would be over-extended (Z={final_z:.3f} < {Z_MIN})"
        if final_z > Z_MAX:
            return False, f"{leg} would be over-folded (Z={final_z:.3f} > {Z_MAX})"
    return True, ""


def check_height_mode_feasibility(target_mode_name: str, roll_deg: float, pitch_deg: float) -> tuple:
    """
    Check if changing to a height mode is safe given current tilt.
    Returns: (is_feasible, reason_string)
    """
    max_tilt = max(abs(roll_deg), abs(pitch_deg))
    
    if target_mode_name == "CROUCH" and max_tilt > MAX_TILT_FOR_CROUCH:
        return False, f"Tilt too high ({max_tilt:.1f}°) for CROUCH mode (max {MAX_TILT_FOR_CROUCH}°)"
    
    if target_mode_name == "LOW" and max_tilt > MAX_TILT_FOR_LOW:
        return False, f"Tilt too high ({max_tilt:.1f}°) for LOW mode (max {MAX_TILT_FOR_LOW}°)"
    
    return True, ""


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
    """Execute IK pipeline without IMU. Returns True on success."""
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


def execute_step_with_compensation(feet: dict, z_comp: dict, coxa_comp: dict) -> bool:
    """
    Execute IK pipeline with static IMU compensation.
    Used during walking to maintain the IMU-compensated pose.
    """
    try:
        # Apply Z compensation to feet
        compensated_feet = apply_z_compensation(feet, z_comp)
        
        # Run IK
        deltas = solve_all_legs(compensated_feet)
        
        # Apply coxa compensation
        for leg, coxa_delta in coxa_comp.items():
            key = f"{leg}_COXA"
            if key in deltas:
                deltas[key] += coxa_delta
        
        deltas = apply_coxa_bias(deltas)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)
        send_to_servos(physical)
        return True
    except Exception as e:
        print(f"\n[WARN] Compensated pipeline error: {e}")
        return False


def execute_step_with_imu(feet: dict, imu: IMUFilter) -> bool:
    """Execute IK pipeline WITH IMU posture control. Returns True on success."""
    try:
        # posture_step does IK internally with IMU compensation
        deltas = posture_step(feet, imu)
        deltas = apply_coxa_bias(deltas)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)
        send_to_servos(physical)
        return True
    except Exception as e:
        print(f"\n[WARN] IMU pipeline error: {e}")
        return False


# -------------------------------------------------
# Trajectory generators
# -------------------------------------------------

def _lateral_trajectory(phase: float, step_length: float, step_height: float, duty: float):
    """Lateral (Y-axis) trajectory generator."""
    if phase < duty:
        s = phase / duty
        dy = +step_length / 2 - s * step_length
        dz = 0.0
        return dy, dz
    
    s = (phase - duty) / (1.0 - duty)
    dy = -step_length / 2 + s * step_length
    dz = step_height * math.sin(math.pi * s)
    return dy, dz


def compute_feet_forward_backward(phase: float, direction: str, stance_z: float) -> dict:
    x_mult = 1 if direction == "forward" else -1
    feet = {}
    
    for leg in ("FL", "FR", "RL", "RR"):
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        dx, dz = _leg_trajectory(leg_phase, STEP_LENGTH, STEP_HEIGHT, DUTY)
        dx = dx * x_mult
        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X + dx, base_y, stance_z + dz)
    
    return feet


def compute_feet_lateral(phase: float, direction: str, stance_z: float) -> dict:
    feet = {}
    
    for leg in ("FL", "FR", "RL", "RR"):
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        
        if direction == "left":
            leg_phase = 1.0 - leg_phase
        
        dy, dz = _lateral_trajectory(leg_phase, LATERAL_STEP_LENGTH, LATERAL_STEP_HEIGHT, DUTY)
        dy = dy * (-1)
        
        if leg in ("FR", "RR"):
            dy = -dy
        
        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X, base_y + dy, stance_z + dz)
    
    return feet


def compute_feet_turn(phase: float, direction: str, stance_z: float) -> dict:
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


def compute_feet_directional(phase: float, direction: str, stance_z: float) -> dict:
    if direction in FORWARD_BACKWARD:
        return compute_feet_forward_backward(phase, direction, stance_z)
    elif direction in LEFT_RIGHT:
        return compute_feet_lateral(phase, direction, stance_z)
    elif direction in TURN:
        return compute_feet_turn(phase, direction, stance_z)
    else:
        return get_stand_feet(stance_z)


# -------------------------------------------------
# Keyboard input (non-blocking)
# -------------------------------------------------

def key_available():
    """Check if a key is available (non-blocking)."""
    return select.select([sys.stdin], [], [], 0)[0]


def get_key_nonblocking():
    """Get key if available, else return None."""
    if not key_available():
        return None
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
        return key.lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def setup_terminal():
    """Setup terminal for non-blocking input."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old_settings


def restore_terminal(old_settings):
    """Restore terminal settings."""
    fd = sys.stdin.fileno()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# -------------------------------------------------
# Execute walking cycle (with captured IMU compensation)
# -------------------------------------------------

def execute_single_cycle(direction: str, stance_z: float, z_comp: dict = None, coxa_comp: dict = None):
    """
    Execute gait cycle with static IMU compensation.
    The compensation values are captured at the start of the cycle
    and held constant throughout (no live IMU updates during walk).
    """
    # Default to no compensation if not provided
    if z_comp is None:
        z_comp = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
    if coxa_comp is None:
        coxa_comp = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
    
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
    
    print(f"  Executing: {label} [IMU LOCKED]", end="", flush=True)
    
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
        
        # Apply captured IMU compensation
        execute_step_with_compensation(feet, z_comp, coxa_comp)
        
        progress = int((elapsed / total_duration) * 10)
        print(f"\r  Executing: {label} [{'=' * progress}{' ' * (10-progress)}]", 
              end="", flush=True)
        
        loop_elapsed = time.time() - loop_start
        sleep_time = max(0, DT - loop_elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    print(f"\r  Executing: {label} [==========] Done")
    
    # Return to stand (with compensation)
    execute_step_with_compensation(get_stand_feet(stance_z), z_comp, coxa_comp)


# -------------------------------------------------
# Height transition (with captured IMU compensation)
# -------------------------------------------------

def transition_height(from_z: float, to_z: float, z_comp: dict = None, coxa_comp: dict = None):
    """Smoothly transition between heights with captured IMU compensation."""
    if z_comp is None:
        z_comp = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
    if coxa_comp is None:
        coxa_comp = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
    
    def lerp(a, b, t):
        return a + (b - a) * t
    
    t0 = time.time()
    while time.time() - t0 < HEIGHT_TRANSITION:
        t = (time.time() - t0) / HEIGHT_TRANSITION
        z = lerp(from_z, to_z, t)
        execute_step_with_compensation(get_stand_feet(z), z_comp, coxa_comp)
        time.sleep(DT)
    execute_step_with_compensation(get_stand_feet(to_z), z_comp, coxa_comp)


# -------------------------------------------------
# MAIN with IMU balance loop
# -------------------------------------------------

def main():
    print("=" * 50)
    print("  WALK CONTROLLER + IMU BALANCE")
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
    print("  X = Quit")
    print()
    print("Height modes:")
    for name, z in HEIGHT_MODES:
        marker = " <-- default" if name == "NORMAL" else ""
        print(f"  {name:8s}: Z={z:.2f}m{marker}")
    print()
    print("IMU: Active when idle, compensation locked during walk")
    print(f"Safety: CROUCH blocked if tilt > {MAX_TILT_FOR_CROUCH}°")
    print(f"        LOW blocked if tilt > {MAX_TILT_FOR_LOW}°")
    print("-" * 50)
    
    # Initialize IMU
    print("[INIT] Starting IMU...")
    init_mpu()
    print("[INIT] Calibrating IMU (keep robot still)...")
    calib = calibrate()
    imu = IMUFilter(calib)
    time.sleep(0.3)
    
    # Initialize to stand
    print("[INIT] Moving to stand pose...")
    execute_step(get_stand_feet(STANCE_Z))
    time.sleep(0.5)
    
    # Prime the IMU reference by doing one posture step
    print("[INIT] Calibrating IMU reference...")
    for _ in range(10):
        execute_step_with_imu(get_stand_feet(STANCE_Z), imu)
        time.sleep(DT)
    
    print("[INIT] Ready! IMU balance active.")
    print("-" * 50)
    
    # State
    height_index = 1  # NORMAL
    current_z = HEIGHT_MODES[height_index][1]
    running = True
    current_roll = 0.0
    current_pitch = 0.0
    
    # Setup terminal for non-blocking input
    old_terminal = setup_terminal()
    
    try:
        last_status_time = 0
        
        while running:
            loop_start = time.time()
            
            # Get current IMU state (for display and feasibility checks)
            current_roll, current_pitch = get_current_imu_state(imu)
            
            # Check for keypress (non-blocking)
            key = get_key_nonblocking()
            
            if key is not None:
                # Process command
                print()  # New line after IMU status
                print(f"Key: {key.upper()}")
                
                if key not in KEY_MAP:
                    print(f"  Unknown key '{key}'. Use W/A/S/D/Q/E/C/X.")
                else:
                    command = KEY_MAP[key]
                    
                    if command == "quit":
                        print("[QUIT] Exiting...")
                        running = False
                        continue
                    
                    # Capture current IMU compensation BEFORE action
                    z_comp = compute_z_compensation(current_roll, current_pitch)
                    coxa_comp = compute_coxa_compensation(current_roll)
                    
                    if command == "height":
                        # Try to cycle to next height mode
                        next_index = (height_index + 1) % len(HEIGHT_MODES)
                        next_name, next_z = HEIGHT_MODES[next_index]
                        
                        # Check feasibility
                        feasible, reason = check_height_mode_feasibility(
                            next_name, current_roll, current_pitch
                        )
                        
                        if not feasible:
                            print(f"  [ACTION NOT ALLOWED] {reason}")
                        else:
                            # Also check Z range feasibility
                            test_feet = get_stand_feet(next_z)
                            z_feasible, z_reason = check_z_feasibility(test_feet, z_comp)
                            
                            if not z_feasible:
                                print(f"  [ACTION NOT ALLOWED] {z_reason}")
                            else:
                                old_z = current_z
                                height_index = next_index
                                print(f"  Changing to {next_name} (Z={next_z:.2f}m)...")
                                transition_height(old_z, next_z, z_comp, coxa_comp)
                                current_z = next_z
                    else:
                        # Movement command - check feasibility
                        # Check if the walk trajectory at max step would be safe
                        max_step_z_offset = STEP_HEIGHT  # Max Z change during walk
                        test_feet = get_stand_feet(current_z + max_step_z_offset)
                        z_feasible, z_reason = check_z_feasibility(test_feet, z_comp)
                        
                        if not z_feasible:
                            print(f"  [ACTION NOT ALLOWED] {z_reason}")
                        else:
                            # Execute walk cycle with captured IMU compensation
                            print(f"  IMU locked at: roll={current_roll:.1f}°, pitch={current_pitch:.1f}°")
                            execute_single_cycle(command, current_z, z_comp, coxa_comp)
                    
                    print()  # Blank line after action
            
            # IMU balance loop (runs when no command being executed)
            feet = get_stand_feet(current_z)
            execute_step_with_imu(feet, imu)
            
            # Status display (throttled to avoid spam)
            if time.time() - last_status_time > 0.5:
                mode_name = HEIGHT_MODES[height_index][0]
                print(f"\r[BALANCING] [{mode_name}] R:{current_roll:+5.1f}° P:{current_pitch:+5.1f}° | W/A/S/D/Q/E/C/X: ", 
                      end="", flush=True)
                last_status_time = time.time()
            
            # Timing
            loop_elapsed = time.time() - loop_start
            sleep_time = max(0, DT - loop_elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\n[INTERRUPT] Caught Ctrl+C")
    
    finally:
        # Restore terminal
        restore_terminal(old_terminal)
        
        # Return to stand
        print("[SHUTDOWN] Returning to stand...")
        execute_step(get_stand_feet(STANCE_Z))
        print("[SHUTDOWN] Complete")


if __name__ == "__main__":
    main()
