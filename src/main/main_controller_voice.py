"""
main_controller.py
==================
Layer 6: Multi-directional gait + tricks system.

INPUT MODES (selected at startup):
  1. Controller Mode  →  existing gamepad behavior (unchanged)
  2. Voice Mode       →  TCP socket server receives text commands from Mac

Voice pipeline:
  Mac mic → Whisper STT → mac_voice_sender.py → TCP → Jetson (this file)

Controls (Controller Mode):
  D-PAD Up/Down   = Forward / Backward
  D-PAD Left/Right = Turn left / Turn right
  LB              = Shake
  RB              = Bheek
  B               = Pushups
  A               = Tilt Dance
  Y               = Cycle height
  X               = Quit
  LT              = Wiggle
  RT              = Stretch
  M1              = Sit
  M2              = High Five

Voice commands:
  "barq move forward"   → forward
  "barq turn left"      → turn_left
  "barq do pushups"     → pushups
  "stop" / "halt"       → stand
  "quit" / "shutdown"   → exit
"""

import time
import math
import sys
import threading
import socket
import logging

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

# ─── LOGGING ───────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("ARYA")


# =============================================================================
# GAIT CONFIG
# =============================================================================

FREQ = 1.5
DT   = 0.02          # 50 Hz control loop

STEP_LENGTH  = 0.06
STEP_HEIGHT  = 0.030
DUTY         = 0.80

# Lateral-specific
LATERAL_STEP_LENGTH = 0.04
LATERAL_STEP_HEIGHT = 0.020
LATERAL_FREQ        = 1.5

# Stance
STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18

# Turn
TURN_STEP_LENGTH = 0.04
TURN_STEP_HEIGHT = 0.025
TURN_FREQ        = 1.5


# =============================================================================
# DIRECTION CATEGORIES
# =============================================================================

FORWARD_BACKWARD = ("forward", "backward")
LEFT_RIGHT       = ("left", "right")
TURN             = ("turn_left", "turn_right")

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

TRICK_MAP = {
    "shake":     shake,
    "bow":       bow,
    "wiggle":    wiggle,
    "pushups":   pushups,
    "bheek":     bheek,
    "high_five": high_five,
    "sit":       sit,
    "stretch":   stretch,
    "tilt_dance": tilt_dance,
    "combo":     combo,
}

HEIGHT_MODES = [
    ("HIGH",   -0.19),
    ("NORMAL", -0.18),
    ("LOW",    -0.15),
    ("CROUCH", -0.12),
]
HEIGHT_TRANSITION = 0.3


# =============================================================================
# COXA PRELOAD
# =============================================================================

COXA_DELTA_BIAS = {
    "FL": +1.5,
    "FR": +1.5,
    "RL": +1.5,
    "RR": +1.5,
}


# =============================================================================
# CHANNEL MAP
# =============================================================================

CHANNELS = {}
for _leg in ("FL", "FR", "RL", "RR"):
    CHANNELS[f"{_leg}_COXA"]  = COXA[_leg]
    CHANNELS[f"{_leg}_THIGH"] = THIGHS[f"T{_leg}"]
    CHANNELS[f"{_leg}_WRIST"] = WRISTS[f"W{_leg}"]

DIAG_A = ("FL", "RR")
DIAG_B = ("FR", "RL")

STAND_FEET = {
    "FL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
    "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
}


# =============================================================================
# TRAJECTORY HELPERS
# =============================================================================

def wrap_phase(p: float) -> float:
    return p - math.floor(p)


def _lateral_trajectory(phase, step_length, step_height, duty):
    if phase < duty:
        s  = phase / duty
        dy = +step_length / 2 - s * step_length
        return dy, 0.0
    s  = (phase - duty) / (1.0 - duty)
    dy = -step_length / 2 + s * step_length
    dz = step_height * math.sin(math.pi * s)
    return dy, dz


def compute_feet_forward_backward(phase, direction, stance_z=STANCE_Z):
    x_mult = 1 if direction == "forward" else -1
    feet   = {}
    for leg in ("FL", "FR", "RL", "RR"):
        lp       = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        dx, dz   = _leg_trajectory(lp, STEP_LENGTH, STEP_HEIGHT, DUTY)
        dx      *= x_mult
        base_y   = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X + dx, base_y, stance_z + dz)
    return feet


def compute_feet_lateral(phase, direction, stance_z=STANCE_Z):
    feet = {}
    for leg in ("FL", "FR", "RL", "RR"):
        lp = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        if direction == "left":
            lp = 1.0 - lp
        dy, dz  = _lateral_trajectory(lp, LATERAL_STEP_LENGTH, LATERAL_STEP_HEIGHT, DUTY)
        y_mult  = +1 if leg in ("FR", "RR") else -1
        dy     *= y_mult
        base_y  = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X, base_y + dy, stance_z + dz)
    return feet


def compute_feet_turn(phase, direction, stance_z=STANCE_Z):
    turn_mult = 1 if direction == "turn_left" else -1
    feet      = {}
    for leg in ("FL", "FR", "RL", "RR"):
        lp     = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        dx, dz = _leg_trajectory(lp, TURN_STEP_LENGTH, TURN_STEP_HEIGHT, DUTY)
        if leg in ("FL", "RL"):
            dx = -dx * turn_mult
        else:
            dx = +dx * turn_mult
        base_y    = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X + dx, base_y, stance_z + dz)
    return feet


def compute_feet_directional(phase, direction, stance_z=STANCE_Z):
    if direction in FORWARD_BACKWARD:
        return compute_feet_forward_backward(phase, direction, stance_z)
    elif direction in LEFT_RIGHT:
        return compute_feet_lateral(phase, direction, stance_z)
    elif direction in TURN:
        return compute_feet_turn(phase, direction, stance_z)
    return {
        "FL": (STANCE_X,  STANCE_Y, stance_z),
        "FR": (STANCE_X, -STANCE_Y, stance_z),
        "RL": (STANCE_X,  STANCE_Y, stance_z),
        "RR": (STANCE_X, -STANCE_Y, stance_z),
    }


# =============================================================================
# EXECUTION HELPERS
# =============================================================================

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
        deltas   = solve_all_legs(feet)
        deltas   = apply_coxa_bias(deltas)
        deltas   = apply_joint_conventions(deltas)
        physical = normalize_all(deltas)
        send_to_servos(physical)
        return True
    except Exception as e:
        print(f"\n[WARN] Pipeline error: {e}")
        return False


def execute_single_cycle(direction: str, stance_z: float = STANCE_Z):
    if direction in LEFT_RIGHT:
        freq, num_cycles = LATERAL_FREQ, 2
        label = f"{direction.upper()} (lateral x2)"
    elif direction in TURN:
        freq, num_cycles = TURN_FREQ, 1
        label = direction.upper().replace("_", " ")
    else:
        freq, num_cycles = FREQ, 1
        label = direction.upper()

    print(f"  Executing: {label}", end="", flush=True)

    cycle_start          = time.time()
    single_cycle_dur     = 1.0 / freq
    total_duration       = single_cycle_dur * num_cycles

    while True:
        loop_start = time.time()
        elapsed    = loop_start - cycle_start
        if elapsed >= total_duration:
            break
        phase = (elapsed / single_cycle_dur) % 1.0
        feet  = compute_feet_directional(phase, direction, stance_z)
        execute_step(feet)
        progress = int((elapsed / total_duration) * 10)
        print(f"\r  Executing: {label} [{'=' * progress}{' ' * (10 - progress)}]",
              end="", flush=True)
        sleep_time = max(0, DT - (time.time() - loop_start))
        if sleep_time > 0:
            time.sleep(sleep_time)

    print(f"\r  Executing: {label} [==========] Done")

    stand_feet = {
        "FL": (STANCE_X,  STANCE_Y, stance_z),
        "FR": (STANCE_X, -STANCE_Y, stance_z),
        "RL": (STANCE_X,  STANCE_Y, stance_z),
        "RR": (STANCE_X, -STANCE_Y, stance_z),
    }
    execute_step(stand_feet)


def transition_height(from_z: float, to_z: float):
    def lerp(a, b, t): return a + (b - a) * t
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
        set_all_z(lerp(from_z, to_z, t))
        time.sleep(DT)
    set_all_z(to_z)


# =============================================================================
# SHARED COMMAND EXECUTOR
# ─────────────────────────────────────────────────────────────────────────────
# Both controller_loop and voice_control_loop call this.
# Returns: "continue" | "quit"
# =============================================================================

def execute_command(command: str, height_state: dict) -> str:
    """
    Execute any normalized command against the current height state.

    height_state is a mutable dict with keys:
        "index"    : int  (current index into HEIGHT_MODES)
        "current_z": float

    Returns "quit" to signal loop exit, "continue" otherwise.
    """
    if command == "quit":
        return "quit"

    elif command == "stop":
        print("  [STOP] Returning to stand.")
        stand()
        return "continue"

    elif command == "height":
        old_z = height_state["current_z"]
        height_state["index"] = (height_state["index"] + 1) % len(HEIGHT_MODES)
        new_name, new_z        = HEIGHT_MODES[height_state["index"]]
        print(f"  Height → {new_name} (Z={new_z:.2f}m)")
        transition_height(old_z, new_z)
        height_state["current_z"] = new_z

    elif command in TRICK_MAP:
        print(f"  Trick: {command.upper()}")
        try:
            TRICK_MAP[command]()
        except Exception as e:
            print(f"  [WARN] Trick error: {e}")
        stand()

    elif command in (*FORWARD_BACKWARD, *LEFT_RIGHT, *TURN):
        execute_single_cycle(command, height_state["current_z"])

    else:
        print(f"  [WARN] Unknown command: '{command}'")

    return "continue"


# =============================================================================
# GAMEPAD INPUT  (Controller Mode — unchanged)
# =============================================================================

def get_key_blocking(controller):
    import pygame
    while True:
        pygame.event.pump()

        hat = controller.get_hat(0)
        if hat == (0,  1): return 'w'
        if hat == (0, -1): return 's'
        if hat == (-1, 0): return 'q'
        if hat == ( 1, 0): return 'e'

        lt = controller.get_axis(2)
        rt = controller.get_axis(5)
        if lt > 0.7: return '3'
        if rt > 0.7: return '2'

        if controller.get_button(4): return '1'
        if controller.get_button(5): return '5'
        if controller.get_button(1): return '4'
        if controller.get_button(0): return '9'
        if controller.get_button(3): return 'c'
        if controller.get_button(2): return 'x'
        if controller.get_button(6): return '7'
        if controller.get_button(7): return '6'

        time.sleep(0.01)


# =============================================================================
# VOICE INPUT LAYER
# =============================================================================

VOICE_PORT    = 9999
VOICE_TIMEOUT = 300.0    # seconds — socket idle before server resets


# ─── Command normalization ────────────────────────────────────────────────────

# Phrase → internal command. Checked in order (longer phrases first to
# avoid partial matches).
_VOICE_PHRASE_MAP = [
    # ── Movement ──────────────────────────────────────────────────
    ("move forward",    "forward"),
    ("go ahead",        "forward"),
    ("go forward",      "forward"),
    ("forward",         "forward"),

    ("move backward",   "backward"),
    ("go backward",     "backward"),
    ("go back",         "backward"),
    ("backward",        "backward"),
    ("back",            "backward"),

    ("strafe left",     "left"),
    ("move left",       "left"),
    ("go left",         "left"),
    ("left",            "left"),

    ("strafe right",    "right"),
    ("move right",      "right"),
    ("go right",        "right"),
    ("right",           "right"),

    ("turn left",       "turn_left"),
    ("rotate left",     "turn_left"),

    ("turn right",      "turn_right"),
    ("rotate right",    "turn_right"),

    # ── Tricks ────────────────────────────────────────────────────
    ("do pushups",      "pushups"),
    ("push ups",        "pushups"),
    ("pushups",         "pushups"),

    ("wiggle",          "wiggle"),
    ("do a bow",        "bow"),
    ("take a bow",      "bow"),
    ("bow",             "bow"),

    ("shake hands",     "shake"),
    ("shake",           "shake"),

    ("sit down",        "sit"),
    ("sit",             "sit"),

    ("stretch",         "stretch"),

    ("tilt dance",      "tilt_dance"),
    ("dance",           "tilt_dance"),

    ("combo",           "combo"),
    ("do combo",        "combo"),

    ("high five",       "high_five"),
    ("bheek",           "bheek"),

    # ── Height ────────────────────────────────────────────────────
    ("stand tall",      "height"),
    ("increase height", "height"),
    ("raise height",    "height"),
    ("change height",   "height"),

    # ── Stop / Exit ───────────────────────────────────────────────
    ("halt",            "stop"),
    ("stop",            "stop"),
    ("freeze",          "stop"),

    ("shutdown",        "quit"),
    ("shut down",       "quit"),
    ("exit",            "quit"),
    ("quit",            "quit"),
    ("power off",       "quit"),
]


def parse_voice_command(text: str) -> str:
    """
    Normalize raw speech text → internal command string.

    Steps:
      1. Lowercase + strip
      2. Remove wake word "barq" (and "hey barq", "ok barq")
      3. Match against _VOICE_PHRASE_MAP (longest match first)
      4. Return matched command or empty string on no match
    """
    if not text:
        return ""

    t = text.lower().strip()

    # Strip wake word variants
    for wake in ("hey barq", "ok barq", "barq"):
        if t.startswith(wake):
            t = t[len(wake):].strip()
            break

    # Remove filler phrases
    for filler in ("please", "can you", "could you", "i want you to", "now"):
        t = t.replace(filler, "").strip()

    # Collapse multiple spaces
    t = " ".join(t.split())

    # Match phrases — first match wins (list is ordered longest→shortest)
    for phrase, command in _VOICE_PHRASE_MAP:
        if phrase in t:
            return command

    return ""    # unrecognized


# ─── TCP socket server ────────────────────────────────────────────────────────

def build_voice_server(port: int = VOICE_PORT) -> socket.socket:
    """Create and bind the TCP server socket."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    print(f"[VOICE] TCP server listening on port {port}")
    return srv


def accept_client(srv: socket.socket) -> tuple[socket.socket, str]:
    """Block until Mac connects. Returns (conn, addr_str)."""
    print("[VOICE] Waiting for Mac to connect...")
    conn, addr = srv.accept()
    conn.settimeout(VOICE_TIMEOUT)
    addr_str = f"{addr[0]}:{addr[1]}"
    print(f"[VOICE] Mac connected from {addr_str}")
    return conn, addr_str


def readline_from_socket(conn: socket.socket, buf: list) -> str | None:
    """
    Read one newline-terminated line from the socket.
    buf is a mutable list[str] used as a persistent receive buffer.
    Returns the line (stripped) or None on disconnect/timeout.
    """
    while True:
        if "\n" in buf[0]:
            line, buf[0] = buf[0].split("\n", 1)
            return line.strip()
        try:
            chunk = conn.recv(1024).decode("utf-8", errors="replace")
            if not chunk:
                return None       # clean disconnect
            buf[0] += chunk
        except socket.timeout:
            print("[VOICE] Socket timeout — no command received.")
            return None
        except OSError:
            return None


# ─── Debounce ─────────────────────────────────────────────────────────────────

_DEBOUNCE_WINDOW = 1.5     # seconds

_last_voice_command   = ""
_last_voice_time      = 0.0


def _voice_debounced(command: str) -> bool:
    """Returns True if duplicate within debounce window."""
    global _last_voice_command, _last_voice_time
    now = time.time()
    if command == _last_voice_command and (now - _last_voice_time) < _DEBOUNCE_WINDOW:
        return True
    _last_voice_command = command
    _last_voice_time    = now
    return False


# ─── Voice control loop ───────────────────────────────────────────────────────

def voice_control_loop():
    """
    Main loop for Voice Mode.

    - Runs TCP server on VOICE_PORT
    - Receives raw text from Mac
    - Parses → internal command
    - Routes through execute_command() — same backend as controller
    - Auto-reconnects if Mac disconnects
    """
    height_state = {"index": 1, "current_z": HEIGHT_MODES[1][1]}   # NORMAL

    print("[INIT] Moving to stand pose...")
    execute_step(STAND_FEET)
    time.sleep(0.5)
    print("[INIT] Ready for voice commands.")
    print()

    srv = build_voice_server(VOICE_PORT)

    try:
        while True:
            # ── Wait for Mac connection ───────────────────────────
            try:
                conn, addr = accept_client(srv)
            except KeyboardInterrupt:
                break

            recv_buf = [""]    # mutable buffer for readline

            try:
                while True:
                    raw = readline_from_socket(conn, recv_buf)

                    if raw is None:
                        print(f"[VOICE] Mac disconnected. Waiting for reconnect...")
                        break

                    if not raw:
                        continue

                    # ── Parse ─────────────────────────────────────
                    command = parse_voice_command(raw)
                    log.info(f'[VOICE] "{raw}" → {command or "UNRECOGNIZED"}')

                    if not command:
                        print(f"  [VOICE] Unrecognized: \"{raw}\"")
                        continue

                    if _voice_debounced(command):
                        print(f"  [VOICE] Debounced: {command}")
                        continue

                    # ── Execute ───────────────────────────────────
                    result = execute_command(command, height_state)
                    if result == "quit":
                        print("[VOICE] Quit command received.")
                        conn.close()
                        return        # exit voice_control_loop entirely

            except KeyboardInterrupt:
                conn.close()
                break
            finally:
                try:
                    conn.close()
                except Exception:
                    pass

    finally:
        srv.close()
        print("[VOICE] Server closed.")
        print("[SHUTDOWN] Returning to stand...")
        execute_step(STAND_FEET)
        print("[SHUTDOWN] Complete.")


# =============================================================================
# CONTROLLER LOOP  (original behavior — unchanged)
# =============================================================================

def controller_loop():
    import pygame
    pygame.init()
    pygame.joystick.init()
    controller = pygame.joystick.Joystick(0)
    controller.init()

    print("=" * 50)
    print("  MULTI-DIRECTIONAL WALK CONTROLLER")
    print("=" * 50)
    print()
    print("Controls:")
    print("  D-PAD Up/Down   = Forward / Backward")
    print("  D-PAD L/R       = Turn left / Turn right")
    print("  LB = Shake     RB = Bheek")
    print("  B  = Pushups   A  = Tilt Dance")
    print("  Y  = Height    X  = Quit")
    print("  LT = Wiggle    RT = Stretch")
    print("  M1 = Sit       M2 = High Five")
    print()
    print("Height modes:")
    for name, z in HEIGHT_MODES:
        marker = " <-- default" if name == "NORMAL" else ""
        print(f"  {name:8s}: Z={z:.2f}m{marker}")
    print()
    print("-" * 50)

    print("[INIT] Moving to stand pose...")
    execute_step(STAND_FEET)
    time.sleep(0.5)
    print("[INIT] Ready!")
    print("-" * 50)

    height_state = {"index": 1, "current_z": HEIGHT_MODES[1][1]}

    try:
        while True:
            mode_name = HEIGHT_MODES[height_state["index"]][0]
            print(f"\nWaiting for input [{mode_name}]: ", end="", flush=True)

            key = get_key_blocking(controller)
            print(key.upper())

            if key not in KEY_MAP:
                print(f"  Unknown key '{key}'.")
                continue

            command = KEY_MAP[key]
            result  = execute_command(command, height_state)
            if result == "quit":
                print("\n[QUIT] Exiting...")
                break

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n[INTERRUPT] Caught Ctrl+C")
    finally:
        print("[SHUTDOWN] Returning to stand...")
        execute_step(STAND_FEET)
        print("[SHUTDOWN] Complete.")


# =============================================================================
# MAIN — mode selection
# =============================================================================

def main():
    print()
    print("=" * 40)
    print("  ARYA  —  Quadruped Control System")
    print("=" * 40)
    print()
    print("  Select Mode:")
    print("    1.  Controller  (gamepad)")
    print("    2.  Voice       (TCP / Mac pipeline)")
    print()

    while True:
        choice = input("  Enter 1 or 2: ").strip()
        if choice in ("1", "2"):
            break
        print("  Invalid. Enter 1 or 2.")

    print()

    if choice == "1":
        print("  [MODE] Controller")
        print()
        controller_loop()
    else:
        print("  [MODE] Voice")
        print()
        voice_control_loop()


if __name__ == "__main__":
    main()