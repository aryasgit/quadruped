"""
controllers/brace_tester.py — Rich Terminal Visualizer for Brace Controller
=============================================================================

Run:
    cd /home/a/quadruped/src/main
    python3 controllers/brace_tester.py

What it does:
    1. Inits IMU + PCA9685, drives robot to stand
    2. Runs brace controller at 50 Hz
    3. Applies dZ corrections to legs in real time (robot physically braces)
    4. Renders a full-screen ANSI dashboard showing:
       - Roll/pitch angles with bar graphs
       - Gyro rates (raw, baseline, residual) with spike indicators
       - Shove detection status with flash alerts
       - Per-leg dZ bar chart (visual quadruped layout)
       - Live event log of detected shoves
       - Loop timing stats

Ctrl+C to exit cleanly.

Flags (edit before running):
    SERVOS_ENABLED = True   → actually move legs (set False for IMU-only dry run)
    LOG_CSV        = False  → dump data to CSV for post-analysis
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time
import math
import csv
from datetime import datetime
from collections import deque

# --- Hardware ---
from hardware.imu import init_mpu, calibrate, IMUFilter
from hardware.pca9685 import init_pca, set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS

# --- Pipeline ---
from ik.solver import solve_all_legs
from joints.conventions import apply_joint_conventions
from joints.space import normalize_all

# --- Brace controller ---
from controllers.brace_controller import (
    BraceController, BraceState,
    ROLL_SHOVE_THRESHOLD, PITCH_SHOVE_THRESHOLD,
    ROLL_EXIT_THRESHOLD, PITCH_EXIT_THRESHOLD,     # new
    ROLL_GAIN, PITCH_GAIN, DECAY_RATE, MAX_DZ,
    BASELINE_ALPHA, ATTACK_RATE,                    # new
)


# =====================================================================
# CONFIG
# =====================================================================

SERVOS_ENABLED = True       # False = dry run (IMU only, no servo output)
LOG_CSV        = False      # True = write timestamped CSV alongside terminal
DT             = 0.02       # 50 Hz

# Stand parameters
STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

STAND_FEET = {
    "FL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
    "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
}

LEGS = ("FL", "FR", "RL", "RR")

# Servo channel map
CHANNELS = {}
for _leg in LEGS:
    CHANNELS[f"{_leg}_COXA"]  = COXA[_leg]
    CHANNELS[f"{_leg}_THIGH"] = THIGHS[f"T{_leg}"]
    CHANNELS[f"{_leg}_WRIST"] = WRISTS[f"W{_leg}"]

# Event log ring buffer
EVENT_LOG_SIZE = 6


# =====================================================================
# ANSI HELPERS
# =====================================================================

class C:
    """ANSI color/style codes."""
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"

    RED     = "\033[91m"
    GREEN   = "\033[92m"
    YELLOW  = "\033[93m"
    BLUE    = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN    = "\033[96m"
    WHITE   = "\033[97m"
    GRAY    = "\033[90m"

    BG_RED    = "\033[41m"
    BG_GREEN  = "\033[42m"
    BG_YELLOW = "\033[43m"

    CLEAR_SCREEN = "\033[2J"
    HOME         = "\033[H"
    HIDE_CURSOR  = "\033[?25l"
    SHOW_CURSOR  = "\033[?25h"


def bar(value: float, max_val: float, width: int = 20,
        pos_color: str = C.GREEN, neg_color: str = C.RED) -> str:
    """Centered bar: negative goes left, positive goes right."""
    half = width // 2
    norm = max(-1.0, min(1.0, value / max_val)) if max_val > 0 else 0.0
    fill = int(abs(norm) * half)

    left  = " " * half
    right = " " * half

    if norm >= 0:
        right = "█" * fill + " " * (half - fill)
        return f"{C.DIM}{left}{C.RESET}│{pos_color}{right}{C.RESET}"
    else:
        pad = half - fill
        left = " " * pad + "█" * fill
        return f"{neg_color}{left}{C.RESET}│{C.DIM}{right}{C.RESET}"


def hbar(value: float, max_val: float, width: int = 12,
         color: str = C.CYAN) -> str:
    """Simple left-to-right bar (absolute value)."""
    norm = min(1.0, abs(value) / max_val) if max_val > 0 else 0.0
    fill = int(norm * width)
    return f"{color}{'█' * fill}{C.DIM}{'░' * (width - fill)}{C.RESET}"


def status_tag(active: bool, label_on: str, label_off: str) -> str:
    if active:
        return f"{C.BG_RED}{C.WHITE}{C.BOLD} {label_on} {C.RESET}"
    return f"{C.DIM} {label_off} {C.RESET}"


# =====================================================================
# SERVO HELPERS
# =====================================================================

def send_deltas(deltas: dict):
    """IK output → conventions → normalize → servos."""
    conv = apply_joint_conventions(deltas)
    phys = normalize_all(conv)
    for joint, angle in phys.items():
        ch = CHANNELS.get(joint)
        if ch is not None:
            set_servo_angle(ch, angle)


def apply_brace_and_send(base_feet: dict, offsets: dict):
    """Add brace dZ offsets to stand feet, solve IK, send to servos."""
    compensated = {}
    for leg, (x, y, z) in base_feet.items():
        compensated[leg] = (x, y, z + offsets.get(leg, 0.0))
    deltas = solve_all_legs(compensated)
    send_deltas(deltas)


def go_stand():
    deltas = solve_all_legs(STAND_FEET)
    send_deltas(deltas)


# =====================================================================
# RENDERER
# =====================================================================

class DashboardRenderer:
    """Builds the full terminal dashboard from a BraceState."""

    def __init__(self):
        self.events = deque(maxlen=EVENT_LOG_SIZE)
        self.t0 = time.time()
        self.frame = 0
        self.peak_roll_rate = 0.0
        self.peak_pitch_rate = 0.0

    def add_event(self, text: str):
        t = time.time() - self.t0
        self.events.append(f"{C.DIM}{t:7.2f}s{C.RESET}  {text}")

    def render(self, s: BraceState) -> str:
        self.frame += 1
        elapsed = time.time() - self.t0

        # Track peaks
        self.peak_roll_rate  = max(self.peak_roll_rate,  abs(s.roll_rate))
        self.peak_pitch_rate = max(self.peak_pitch_rate, abs(s.pitch_rate))

        lines = []
        W = 72  # terminal width

        # ── Header ──
        lines.append(f"{C.HOME}")
        lines.append(f"{C.BOLD}{C.CYAN}{'═' * W}{C.RESET}")
        lines.append(f"{C.BOLD}{C.CYAN}  BRACE CONTROLLER — LIVE DIAGNOSTICS{C.RESET}"
                      f"{C.DIM}{'':>20}{elapsed:7.1f}s  #{self.frame}{C.RESET}")
        lines.append(f"{C.BOLD}{C.CYAN}{'═' * W}{C.RESET}")

        # ── IMU Angles ──
        lines.append(f"")
        lines.append(f"  {C.BOLD}IMU ANGLES{C.RESET}")
        lines.append(f"    Roll  {s.roll:+7.2f}°  {bar(s.roll, 20.0, 30)}")
        lines.append(f"    Pitch {s.pitch:+7.2f}°  {bar(s.pitch, 20.0, 30)}")

        # ── Gyro Rates ──
        lines.append(f"")
        lines.append(f"  {C.BOLD}GYRO RATES{C.RESET}  (deg/s)"
                      f"           {C.DIM}raw      base     residual{C.RESET}")

        r_tag = status_tag(s.roll_shove,  "SHOVE", "calm ")
        p_tag = status_tag(s.pitch_shove, "SHOVE", "calm ")

        lines.append(
            f"    Roll  {s.roll_rate:+8.1f}  {s.roll_rate_baseline:+8.1f}"
            f"  {C.YELLOW if s.roll_shove else C.DIM}{s.roll_residual:+8.1f}{C.RESET}"
            f"  {r_tag}"
            f"  {hbar(s.roll_residual, ROLL_SHOVE_THRESHOLD * 3, 15, C.YELLOW if s.roll_shove else C.GRAY)}"
        )
        lines.append(
            f"    Pitch {s.pitch_rate:+8.1f}  {s.pitch_rate_baseline:+8.1f}"
            f"  {C.YELLOW if s.pitch_shove else C.DIM}{s.pitch_residual:+8.1f}{C.RESET}"
            f"  {p_tag}"
            f"  {hbar(s.pitch_residual, PITCH_SHOVE_THRESHOLD * 3, 15, C.YELLOW if s.pitch_shove else C.GRAY)}"
        )

        # ── Leg dZ Layout (quadruped top-down view) ──
        lines.append(f"")
        lines.append(f"  {C.BOLD}LEG dZ CORRECTIONS{C.RESET}  (mm)"
                      f"    {C.DIM}+ = retract   - = extend{C.RESET}")

        dz = s.dz
        max_disp = MAX_DZ * 1000  # mm

        def leg_cell(leg: str) -> str:
            val_mm = dz[leg] * 1000
            abs_mm = abs(val_mm)
            if abs_mm < 0.1:
                color = C.DIM
            elif val_mm > 0:
                color = C.RED      # retracting
            else:
                color = C.GREEN    # extending
            return f"{color}{leg} {val_mm:+6.1f}mm{C.RESET}  {hbar(val_mm, max_disp, 10, color)}"

        lines.append(f"")
        lines.append(f"      {C.DIM}FRONT{C.RESET}")
        lines.append(f"    {leg_cell('FL')}    {leg_cell('FR')}")
        lines.append(f"      {C.DIM}  ╔══════╗{C.RESET}")
        lines.append(f"      {C.DIM}  ║ BODY ║{C.RESET}")
        lines.append(f"      {C.DIM}  ╚══════╝{C.RESET}")
        lines.append(f"    {leg_cell('RL')}    {leg_cell('RR')}")
        lines.append(f"      {C.DIM}REAR{C.RESET}")

        # ── Stats row ──
        lines.append(f"")
        active_str = f"{C.GREEN}BRACING{C.RESET}" if (s.roll_shove or s.pitch_shove) else (
            f"{C.YELLOW}RECOVERING{C.RESET}" if any(abs(v) > 0.001 for v in dz.values()) else
            f"{C.DIM}IDLE{C.RESET}"
        )
        lines.append(
            f"  {C.BOLD}STATUS{C.RESET}  {active_str}"
            f"    Shoves: {C.BOLD}{s.shove_count}{C.RESET}"
            f"    Loop: {s.loop_hz:.0f} Hz"
            f"    dt: {s.dt_actual*1000:.1f}ms"
        )

        # ── Peaks ──
        lines.append(
            f"  {C.DIM}Peaks:  roll_rate={self.peak_roll_rate:.1f}°/s"
            f"   pitch_rate={self.peak_pitch_rate:.1f}°/s{C.RESET}"
        )

        # ── Config summary ──
        lines.append(f"")
        lines.append(
            f"  {C.DIM}Config:  R_thresh={ROLL_SHOVE_THRESHOLD:.0f}/{ROLL_EXIT_THRESHOLD:.0f}°/s"
            f"  P_thresh={PITCH_SHOVE_THRESHOLD:.0f}/{PITCH_EXIT_THRESHOLD:.0f}°/s"
            f"  R_gain={ROLL_GAIN:.4f}"
            f"  P_gain={PITCH_GAIN:.4f}"
            f"  attack={ATTACK_RATE:.0f}/s"
            f"  decay={DECAY_RATE:.0f}/s"
            f"  α={BASELINE_ALPHA:.3f}{C.RESET}"
        )

        servo_str = f"{C.GREEN}ON{C.RESET}" if SERVOS_ENABLED else f"{C.RED}OFF (DRY RUN){C.RESET}"
        lines.append(f"  {C.DIM}Servos: {servo_str}    max_dz={MAX_DZ*1000:.0f}mm{C.RESET}")

        # ── Event log ──
        lines.append(f"")
        lines.append(f"  {C.BOLD}EVENT LOG{C.RESET}")
        if self.events:
            for ev in self.events:
                lines.append(f"    {ev}")
        else:
            lines.append(f"    {C.DIM}(no shoves detected yet — push the robot!){C.RESET}")

        # ── Footer ──
        lines.append(f"")
        lines.append(f"  {C.DIM}Ctrl+C to exit{C.RESET}")

        # Pad remaining lines to avoid artifacts from previous frame
        while len(lines) < 38:
            lines.append(" " * W)

        return "\n".join(lines)


# =====================================================================
# CSV LOGGER
# =====================================================================

class CSVLogger:
    def __init__(self, enabled: bool):
        self.enabled = enabled
        self.writer = None
        self.file = None
        if enabled:
            fname = f"brace_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.file = open(fname, 'w', newline='')
            self.writer = csv.writer(self.file)
            self.writer.writerow([
                "t", "roll", "pitch", "roll_rate", "pitch_rate",
                "roll_base", "pitch_base", "roll_resid", "pitch_resid",
                "roll_shove", "pitch_shove",
                "dz_FL", "dz_FR", "dz_RL", "dz_RR",
            ])

    def log(self, t: float, s: BraceState):
        if not self.enabled:
            return
        self.writer.writerow([
            f"{t:.4f}", f"{s.roll:.3f}", f"{s.pitch:.3f}",
            f"{s.roll_rate:.2f}", f"{s.pitch_rate:.2f}",
            f"{s.roll_rate_baseline:.2f}", f"{s.pitch_rate_baseline:.2f}",
            f"{s.roll_residual:.2f}", f"{s.pitch_residual:.2f}",
            int(s.roll_shove), int(s.pitch_shove),
            f"{s.dz['FL']:.5f}", f"{s.dz['FR']:.5f}",
            f"{s.dz['RL']:.5f}", f"{s.dz['RR']:.5f}",
        ])

    def close(self):
        if self.file:
            self.file.close()


# =====================================================================
# MAIN
# =====================================================================

def main():
    print(f"{C.CLEAR_SCREEN}{C.HOME}")
    print(f"{C.BOLD}{C.CYAN}  BRACE CONTROLLER TESTER{C.RESET}")
    print(f"{'─' * 50}")

    # --- Hardware init ---
    print(f"  [INIT] PCA9685...")
    init_pca()

    print(f"  [INIT] MPU6050...")
    init_mpu()

    print(f"  [INIT] Calibrating IMU — keep robot {C.BOLD}FLAT and STILL{C.RESET}")
    calib = calibrate(samples=200)
    imu = IMUFilter(calib)

    # --- Stand ---
    if SERVOS_ENABLED:
        print(f"  [INIT] Moving to stand pose...")
        go_stand()
        time.sleep(0.5)
    else:
        print(f"  [INIT] {C.YELLOW}DRY RUN — servos disabled{C.RESET}")

    # --- Brace controller ---
    brace = BraceController()
    brace.reset()

    # --- Renderer + logger ---
    renderer = DashboardRenderer()
    logger = CSVLogger(LOG_CSV)

    # Hide cursor for clean rendering
    print(C.HIDE_CURSOR, end="", flush=True)
    print(C.CLEAR_SCREEN, end="", flush=True)

    t0 = time.time()
    prev_shove_count = 0

    try:
        while True:
            t_start = time.time()
            elapsed = t_start - t0

            # --- Run brace controller ---
            offsets = brace.update(imu)
            state = brace.state

            # --- Apply to servos ---
            if SERVOS_ENABLED:
                apply_brace_and_send(STAND_FEET, offsets)

            # --- Detect new shove events for log ---
            if state.shove_count > prev_shove_count:
                diff = state.shove_count - prev_shove_count
                for _ in range(diff):
                    axis = []
                    if state.roll_shove:
                        axis.append(f"ROLL {state.roll_residual:+.0f}°/s")
                    if state.pitch_shove:
                        axis.append(f"PITCH {state.pitch_residual:+.0f}°/s")
                    label = " + ".join(axis) if axis else "IMPULSE"
                    renderer.add_event(
                        f"{C.BOLD}{C.RED}⚡ SHOVE{C.RESET}  {label}"
                    )
                prev_shove_count = state.shove_count

            # --- Render dashboard ---
            frame = renderer.render(state)
            print(frame, end="", flush=True)

            # --- CSV ---
            logger.log(elapsed, state)

            # --- Timing ---
            loop_t = time.time() - t_start
            sleep = max(0.0, DT - loop_t)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        pass

    finally:
        print(C.SHOW_CURSOR, end="", flush=True)
        print(f"\n\n{C.BOLD}[EXIT]{C.RESET} Returning to stand...")
        if SERVOS_ENABLED:
            go_stand()
        logger.close()
        if LOG_CSV:
            print(f"  CSV log saved.")
        print(f"{C.BOLD}[EXIT]{C.RESET} Done.")


if __name__ == "__main__":
    main()