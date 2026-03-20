"""
TEST — Layer 6 DIAGONAL ARC WALK V3 (IMPROVED)
==============================================

Improvements over V2:
- Consistent stance parameters with IK reference
- Graceful startup ramp and shutdown
- Proper timing compensation for loop rate
- COXA bias applied in delta space (before conventions)
- Optional CSV logging for diagnostics
- Error handling for IK failures
- Auto-generated channel map
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


# -------------------------------------------------
# CONFIG — CONSISTENT WITH IK REFERENCE
# -------------------------------------------------

FREQ = 1.5
DT = 0.02  # Target 50Hz

STEP_LENGTH = 0.06
STEP_HEIGHT = 0.030
DUTY = 0.80  # Reduced from 0.87 for smoother swing

# Stance values — MUST MATCH layer3/leg_ik.py _STAND_* reference
STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

# Startup/shutdown config
STARTUP_DURATION = 2.0   # seconds to ramp up
SHUTDOWN_DURATION = 1.0  # seconds to return to stand

# -------------------------------------------------
# COXA PRELOAD (in delta space, before conventions)
# Positive = toe inward for stability
# -------------------------------------------------

COXA_DELTA_BIAS = {
    "FL": +1.5,  # Applied in IK delta space
    "FR": +1.5,
    "RL": +1.5,
    "RR": +1.5,
}

# -------------------------------------------------
# LOGGING CONFIG
# -------------------------------------------------

LOG_ENABLED = False
LOG_FILE = f"walk_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

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
# Helper: phase wrapping (avoids float errors)
# -------------------------------------------------

def wrap_phase(p: float) -> float:
    return p - math.floor(p)

# -------------------------------------------------
# Helper: compute foot targets for given phase & freq multiplier
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
# Helper: apply coxa bias in delta space
# -------------------------------------------------

def apply_coxa_bias(deltas: dict) -> dict:
    """Apply coxa bias BEFORE joint conventions (in IK delta space)."""
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
# Helper: compute and send (with error handling)
# -------------------------------------------------

def execute_step(feet: dict) -> dict:
    """
    IK solve -> conventions -> normalize -> send.
    Returns physical angles dict, or None on failure.
    """
    try:
        deltas = solve_all_legs(feet)
        deltas = apply_coxa_bias(deltas)
        deltas = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)
        send_to_servos(physical)
        return physical
    except Exception as e:
        print(f"[WARN] IK/servo error: {e}")
        return None

# -------------------------------------------------
# MAIN
# -------------------------------------------------

def main():
    print("[TEST] Layer 6 — DIAGONAL ARC WALK V3")
    print("FL+RR → FR+RL → repeat")
    print(f"Stance: X={STANCE_X}, Y={STANCE_Y}, Z={STANCE_Z}")
    print(f"Gait: freq={FREQ}Hz, step={STEP_LENGTH}m, height={STEP_HEIGHT}m, duty={DUTY}")
    print("CTRL+C to stop")
    print()
    
    # Logging setup
    log_data = []
    
    t0 = time.time()
    last_loop = t0
    
    try:
        while True:
            loop_start = time.time()
            t = loop_start - t0
            
            # --- Startup ramp ---
            if t < STARTUP_DURATION:
                freq_mult = t / STARTUP_DURATION
                print(f"\r[STARTUP] {freq_mult*100:.0f}%", end="", flush=True)
            else:
                freq_mult = 1.0
            
            # --- Compute phase ---
            phase = wrap_phase(t * FREQ)
            
            # --- Compute foot targets ---
            feet = compute_feet(phase, freq_mult)
            
            # --- Execute ---
            physical = execute_step(feet)
            
            # --- Logging ---
            if LOG_ENABLED and physical is not None:
                log_data.append({
                    "t": t,
                    "phase": phase,
                    "freq_mult": freq_mult,
                    **{f"foot_{k}_{ax}": v for k, pos in feet.items() for ax, v in zip("xyz", pos)},
                    **{f"phys_{k}": v for k, v in physical.items()},
                })
            
            # --- Timing compensation ---
            elapsed = time.time() - loop_start
            sleep_time = max(0, DT - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            last_loop = time.time()
    
    except KeyboardInterrupt:
        print("\n\n[SHUTDOWN] Returning to stand...")
        
        # Graceful shutdown: ramp back to stand
        shutdown_start = time.time()
        while True:
            t_shutdown = time.time() - shutdown_start
            if t_shutdown >= SHUTDOWN_DURATION:
                break
            
            # Ramp down
            freq_mult = 1.0 - (t_shutdown / SHUTDOWN_DURATION)
            phase = wrap_phase((time.time() - t0) * FREQ)
            
            feet = compute_feet(phase, freq_mult)
            execute_step(feet)
            
            time.sleep(DT)
        
        # Final stand pose
        stand_feet = {
            "FL": (STANCE_X, STANCE_Y, STANCE_Z),
            "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
            "RL": (STANCE_X, STANCE_Y, STANCE_Z),
            "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
        }
        execute_step(stand_feet)
        
        print("[SHUTDOWN] Complete")
        
        # Save log
        if LOG_ENABLED and log_data:
            print(f"[LOG] Saving {len(log_data)} samples to {LOG_FILE}")
            with open(LOG_FILE, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=log_data[0].keys())
                writer.writeheader()
                writer.writerows(log_data)
            print(f"[LOG] Saved")


if __name__ == "__main__":
    main()
