"""
TEST — Layer 6 DIAGONAL WALK + IMU POSTURE
==========================================

Combines:
- Diagonal trot gait from test_walk_minimal_V3
- IMU-based roll/pitch compensation from posture_controller

The posture logic is integrated directly (not using posture_step)
to avoid double wrist mirroring with joint_conventions.
"""

import time
import math
import csv
from datetime import datetime

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
from hardware.imu import init_mpu, calibrate, IMUFilter


# -------------------------------------------------
# GAIT CONFIG
# -------------------------------------------------

FREQ = 1.5
DT = 0.02  # Target 50Hz

STEP_LENGTH = 0.06
STEP_HEIGHT = 0.030
DUTY = 0.80

# Stance values — MUST MATCH layer3/leg_ik.py _STAND_* reference
STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

# Startup/shutdown config
STARTUP_DURATION = 2.0
SHUTDOWN_DURATION = 1.0


# -------------------------------------------------
# POSTURE CONFIG (from posture_controller.py)
# -------------------------------------------------

BODY_LENGTH = 0.208
BODY_WIDTH  = 0.078

LEG_X = {
    "FL": +BODY_LENGTH / 2,
    "FR": +BODY_LENGTH / 2,
    "RL": -BODY_LENGTH / 2,
    "RR": -BODY_LENGTH / 2,
}

LEG_Y = {
    "FL": +BODY_WIDTH / 2,
    "FR": -BODY_WIDTH / 2,
    "RL": +BODY_WIDTH / 2,
    "RR": -BODY_WIDTH / 2,
}

# Posture gains
PITCH_GAIN = 2.0
ROLL_GAIN  = 2.5
COXA_ROLL_GAIN = 18.0  # degrees per radian

# Limits
MAX_PITCH_DEG = 20.0
MAX_ROLL_DEG  = 20.0
DEADBAND_DEG  = 0.4

# Posture enable flag
POSTURE_ENABLED = True


# -------------------------------------------------
# COXA PRELOAD (in delta space, before conventions)
# -------------------------------------------------

COXA_DELTA_BIAS = {
    "FL": +1.5,
    "FR": +1.5,
    "RL": +1.5,
    "RR": +1.5,
}


# -------------------------------------------------
# LOGGING / DISPLAY CONFIG
# -------------------------------------------------

LOG_ENABLED = False
LOG_FILE = f"walk_imu_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

DISPLAY_HZ = 10  # Print status N times per second
DISPLAY_INTERVAL = 1.0 / DISPLAY_HZ


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
# Helper: phase wrapping
# -------------------------------------------------

def wrap_phase(p: float) -> float:
    return p - math.floor(p)


# -------------------------------------------------
# Helper: compute foot targets for gait
# -------------------------------------------------

def compute_feet(phase: float, freq_mult: float = 1.0) -> dict:
    """
    Compute foot targets for all legs.
    freq_mult: 0.0 = stand, 1.0 = full gait
    """
    feet = {}
    
    effective_step = STEP_LENGTH * freq_mult
    effective_height = STEP_HEIGHT * freq_mult
    
    for leg in ("FL", "FR", "RL", "RR"):
        # Diagonal phase shift
        if leg in DIAG_A:
            leg_phase = phase
        else:
            leg_phase = wrap_phase(phase + 0.5)
        
        dx, dz = _leg_trajectory(
            leg_phase,
            effective_step,
            effective_height,
            DUTY,
        )
        
        y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        
        feet[leg] = (
            STANCE_X + dx,
            y,
            STANCE_Z + dz,
        )
    
    return feet


# -------------------------------------------------
# Posture: IMU reference state
# -------------------------------------------------

_ref_roll = None
_ref_pitch = None


def reset_posture_reference():
    """Reset IMU reference (call when robot is level)."""
    global _ref_roll, _ref_pitch
    _ref_roll = None
    _ref_pitch = None


# -------------------------------------------------
# Posture: apply Z offsets based on IMU
# -------------------------------------------------

def apply_posture_z(feet: dict, imu: IMUFilter) -> tuple:
    """
    Apply roll/pitch compensation to foot Z positions.
    
    Returns:
        (modified_feet, roll_deg, pitch_deg)
    """
    global _ref_roll, _ref_pitch
    
    roll, pitch, _, _ = imu.update()
    
    # Lock reference on first call
    if _ref_roll is None:
        _ref_roll = roll
        _ref_pitch = pitch
        return feet, 0.0, 0.0
    
    # Relative angles
    roll = roll - _ref_roll
    pitch = pitch - _ref_pitch
    
    # IMU sign convention (your robot: front pitch negative)
    pitch = -pitch
    
    # Deadband
    if abs(roll) < DEADBAND_DEG:
        roll = 0.0
    if abs(pitch) < DEADBAND_DEG:
        pitch = 0.0
    
    # Clamp
    roll = max(min(roll, MAX_ROLL_DEG), -MAX_ROLL_DEG)
    pitch = max(min(pitch, MAX_PITCH_DEG), -MAX_PITCH_DEG)
    
    roll_r = math.radians(roll)
    pitch_r = math.radians(pitch)
    
    # Apply Z compensation
    modified = {}
    for leg, (x, y, z) in feet.items():
        lx = LEG_X[leg]
        ly = LEG_Y[leg]
        
        dz_pitch = -PITCH_GAIN * lx * math.sin(pitch_r)
        dz_roll = -ROLL_GAIN * ly * math.sin(roll_r)
        
        modified[leg] = (x, y, z + dz_pitch + dz_roll)
    
    return modified, roll, pitch


# -------------------------------------------------
# Posture: apply coxa roll stabilization to deltas
# -------------------------------------------------

def apply_coxa_roll_stabilization(deltas: dict, roll_deg: float) -> dict:
    """Add coxa angle adjustment for roll stabilization."""
    roll_r = math.radians(roll_deg)
    out = deltas.copy()
    
    for leg in ("FL", "RL"):
        out[f"{leg}_COXA"] += COXA_ROLL_GAIN * roll_r
    for leg in ("FR", "RR"):
        out[f"{leg}_COXA"] -= COXA_ROLL_GAIN * roll_r
    
    return out


# -------------------------------------------------
# Helper: apply coxa bias in delta space
# -------------------------------------------------

def apply_coxa_bias(deltas: dict) -> dict:
    """Apply coxa bias BEFORE joint conventions."""
    biased = deltas.copy()
    for leg, bias in COXA_DELTA_BIAS.items():
        key = f"{leg}_COXA"
        if key in biased:
            biased[key] += bias
    return biased


# -------------------------------------------------
# Helper: send commands to servos
# -------------------------------------------------

def send_to_servos(physical: dict):
    """Write physical angles to all servo channels."""
    for joint, ch in CHANNELS.items():
        set_servo_angle(ch, physical[joint])


# -------------------------------------------------
# Helper: execute full pipeline
# -------------------------------------------------

def execute_step(feet: dict, imu: IMUFilter = None) -> tuple:
    """
    Full pipeline: posture → IK → conventions → normalize → send.
    
    Returns:
        (physical_angles, roll_deg, pitch_deg) or (None, 0, 0) on failure
    """
    try:
        roll_deg = 0.0
        pitch_deg = 0.0
        
        # Posture compensation
        if POSTURE_ENABLED and imu is not None:
            feet, roll_deg, pitch_deg = apply_posture_z(feet, imu)
        
        # IK
        deltas = solve_all_legs(feet)
        
        # Coxa roll stabilization
        if POSTURE_ENABLED and imu is not None:
            deltas = apply_coxa_roll_stabilization(deltas, roll_deg)
        
        # Coxa bias
        deltas = apply_coxa_bias(deltas)
        
        # Conventions & normalize
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)
        
        # Send
        send_to_servos(physical)
        
        return physical, roll_deg, pitch_deg
    
    except Exception as e:
        print(f"\n[WARN] Pipeline error: {e}")
        return None, 0.0, 0.0


# -------------------------------------------------
# Display: format status line
# -------------------------------------------------

def format_status(t: float, phase: float, rate_hz: float, 
                  roll: float, pitch: float, feet: dict) -> str:
    """Format single-line status display."""
    # Determine swing diagonal
    if phase < 0.5:
        swing = "FL+RR"
    else:
        swing = "FR+RL"
    
    # Get lift heights (dz from stance)
    dz = {leg: (feet[leg][2] - STANCE_Z) * 100 for leg in feet}  # cm
    
    # Find which legs are lifted
    lifted = [f"{leg}={dz[leg]:+.1f}" for leg in ("FL", "FR", "RL", "RR") if dz[leg] > 0.5]
    lift_str = " ".join(lifted) if lifted else "none"
    
    return (f"t={t:5.1f}s | φ={phase:.2f} | {rate_hz:4.1f}Hz | "
            f"swing={swing} | R={roll:+5.1f}° P={pitch:+5.1f}° | lift: {lift_str}")


# -------------------------------------------------
# MAIN
# -------------------------------------------------

def main():
    print("=" * 60)
    print("  Layer 6 — DIAGONAL WALK + IMU POSTURE")
    print("=" * 60)
    print(f"Gait:    freq={FREQ}Hz, step={STEP_LENGTH}m, height={STEP_HEIGHT}m")
    print(f"Stance:  X={STANCE_X}, Y={STANCE_Y}, Z={STANCE_Z}")
    print(f"Posture: {'ENABLED' if POSTURE_ENABLED else 'DISABLED'}")
    print("-" * 60)
    
    # Initialize IMU
    print("[INIT] Initializing IMU...")
    init_mpu()
    
    print("[INIT] Calibrating IMU — keep robot still...")
    calib = calibrate(samples=200)
    imu = IMUFilter(calib, alpha=0.96, dt=DT)
    
    print("[INIT] IMU ready")
    print("-" * 60)
    print("CTRL+C to stop")
    print()
    
    # Logging setup
    log_data = []
    
    t0 = time.time()
    last_display = t0
    loop_count = 0
    
    try:
        while True:
            loop_start = time.time()
            t = loop_start - t0
            loop_count += 1
            
            # --- Startup ramp ---
            if t < STARTUP_DURATION:
                freq_mult = t / STARTUP_DURATION
            else:
                freq_mult = 1.0
            
            # --- Compute phase ---
            phase = wrap_phase(t * FREQ)
            
            # --- Compute foot targets ---
            feet = compute_feet(phase, freq_mult)
            
            # --- Execute pipeline ---
            physical, roll_deg, pitch_deg = execute_step(feet, imu)
            
            # --- Display status ---
            if t - last_display >= DISPLAY_INTERVAL and t >= STARTUP_DURATION:
                elapsed = time.time() - loop_start
                actual_hz = 1.0 / DT if elapsed < DT else 1.0 / elapsed
                
                status = format_status(t, phase, actual_hz, roll_deg, pitch_deg, feet)
                print(f"\r{status}", end="", flush=True)
                last_display = t
            elif t < STARTUP_DURATION:
                print(f"\r[STARTUP] {freq_mult*100:3.0f}%", end="", flush=True)
            
            # --- Logging ---
            if LOG_ENABLED and physical is not None:
                log_data.append({
                    "t": t,
                    "phase": phase,
                    "freq_mult": freq_mult,
                    "roll": roll_deg,
                    "pitch": pitch_deg,
                    **{f"foot_{k}_{ax}": v for k, pos in feet.items() 
                       for ax, v in zip("xyz", pos)},
                    **{f"phys_{k}": v for k, v in physical.items()},
                })
            
            # --- Timing ---
            elapsed = time.time() - loop_start
            sleep_time = max(0, DT - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n")
        print("-" * 60)
        print("[SHUTDOWN] Returning to stand...")
        
        # Graceful shutdown
        shutdown_start = time.time()
        while True:
            t_shutdown = time.time() - shutdown_start
            if t_shutdown >= SHUTDOWN_DURATION:
                break
            
            freq_mult = 1.0 - (t_shutdown / SHUTDOWN_DURATION)
            phase = wrap_phase((time.time() - t0) * FREQ)
            
            feet = compute_feet(phase, freq_mult)
            execute_step(feet, imu)
            
            print(f"\r[SHUTDOWN] {(1-freq_mult)*100:3.0f}%", end="", flush=True)
            time.sleep(DT)
        
        # Final stand
        stand_feet = {
            "FL": (STANCE_X, STANCE_Y, STANCE_Z),
            "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
            "RL": (STANCE_X, STANCE_Y, STANCE_Z),
            "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
        }
        execute_step(stand_feet, imu)
        
        print(f"\r[SHUTDOWN] Complete          ")
        print("-" * 60)
        
        # Save log
        if LOG_ENABLED and log_data:
            print(f"[LOG] Saving {len(log_data)} samples to {LOG_FILE}")
            with open(LOG_FILE, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=log_data[0].keys())
                writer.writeheader()
                writer.writerows(log_data)
            print("[LOG] Saved")
        
        print(f"[STATS] Ran for {t:.1f}s, {loop_count} loops")


if __name__ == "__main__":
    main()
