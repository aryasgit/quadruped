"""
controller.py — ROBOT CONTROLLER WITH AVOIDANCE
=================================================
Main robot controller integrating Xbox gamepad input with
perception-driven obstacle avoidance.

This file is a modified version of main_controller.py. All gait math,
imports, servo pipeline, and trick calls are IDENTICAL to the original.
The only additions are:
  - Reads PerceptionFrame from a queue (non-blocking)
  - ProximityState state machine modifies gait behavior
  - Auto-avoidance: stop → read bearing → turn away → resume

Uses ONLY turn_left() and turn_right() for avoidance.
Never calls left() or right() — physically broken.

Queue contract:
  - Reads from: perception_queue (PerceptionFrame)
  - Writes to:  robot_state_queue (RobotState)
"""

import time
import math
import sys
import queue
import threading

# ── Layer 6 — Gait ──────────────────────────────────────────────
from gait.generator import _leg_trajectory

# ── Layer 3 — IK ────────────────────────────────────────────────
from ik.solver import solve_all_legs

# ── Layer 2 — Joint space ───────────────────────────────────────
from joints.conventions import apply_joint_conventions
from joints.space import normalize_all

# ── Hardware ────────────────────────────────────────────────────
from hardware.pca9685 import init_pca, set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS

# ── Tricks ──────────────────────────────────────────────────────
from stance.tricks import (
    stand, shake, bow, wiggle, pushups,
    bheek, high_five, sit, stretch,
    tilt_dance, combo,
)

# ── State contracts ─────────────────────────────────────────────
from state import (
    BootMode, ProximityState, ControlMode,
    RobotState, IMUSnapshot, StepState,
    PerceptionFrame, classify_distance,
)


# ─────────────────────────────────────────────────────────────────
# GAIT CONFIG (identical to main_controller.py)
# ─────────────────────────────────────────────────────────────────

FREQ        = 1.5
DT          = 0.02   # 50 Hz

STEP_LENGTH = 0.06
STEP_HEIGHT = 0.030
DUTY        = 0.80

LATERAL_STEP_LENGTH = 0.04
LATERAL_STEP_HEIGHT = 0.020
LATERAL_FREQ        = 1.5

TURN_STEP_LENGTH = 0.04
TURN_STEP_HEIGHT = 0.025
TURN_FREQ        = 1.5

STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18


# ─────────────────────────────────────────────────────────────────
# DIRECTION CONFIG
# ─────────────────────────────────────────────────────────────────

KEY_MAP = {
    'w': "forward",
    's': "backward",
    'a': "left",
    'd': "right",
    'q': "turn_left",
    'e': "turn_right",
    'c': "height",
    # Tricks
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

TRICK_MAP = {
    "shake":      shake,
    "bow":        bow,
    "wiggle":     wiggle,
    "pushups":    pushups,
    "bheek":      bheek,
    "high_five":  high_five,
    "sit":        sit,
    "stretch":    stretch,
    "tilt_dance": tilt_dance,
    "combo":      combo,
}


# ─────────────────────────────────────────────────────────────────
# HEIGHT MODES
# ─────────────────────────────────────────────────────────────────

HEIGHT_MODES = [
    ("HIGH",   -0.19),
    ("NORMAL", -0.18),
    ("LOW",    -0.15),
    ("CROUCH", -0.12),
]
HEIGHT_TRANSITION = 0.3


# ─────────────────────────────────────────────────────────────────
# COXA PRELOAD
# ─────────────────────────────────────────────────────────────────

COXA_DELTA_BIAS = {
    "FL": +1.5,
    "FR": +1.5,
    "RL": +1.5,
    "RR": +1.5,
}


# ─────────────────────────────────────────────────────────────────
# AUTO-GENERATED SERVO CHANNEL MAP
# ─────────────────────────────────────────────────────────────────

CHANNELS = {}
for _leg in ("FL", "FR", "RL", "RR"):
    CHANNELS[f"{_leg}_COXA"]  = COXA[_leg]
    CHANNELS[f"{_leg}_THIGH"] = THIGHS[f"T{_leg}"]
    CHANNELS[f"{_leg}_WRIST"] = WRISTS[f"W{_leg}"]


# ─────────────────────────────────────────────────────────────────
# DIAGONAL GROUPS (trot gait)
# ─────────────────────────────────────────────────────────────────

DIAG_A = ("FL", "RR")
DIAG_B = ("FR", "RL")


# ─────────────────────────────────────────────────────────────────
# STAND POSE
# ─────────────────────────────────────────────────────────────────

STAND_FEET = {
    "FL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
    "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
    "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
}


# ─────────────────────────────────────────────────────────────────
# AVOIDANCE TUNING
# ─────────────────────────────────────────────────────────────────

AVOIDANCE_COOLDOWN   = 1.0    # seconds after avoidance before re-trigger
SPEED_SCALE_SLOW     = 0.5    # frequency multiplier for SLOW/CAUTION


# ─────────────────────────────────────────────────────────────────
# HELPER: phase wrapping
# ─────────────────────────────────────────────────────────────────

def wrap_phase(p: float) -> float:
    return p - math.floor(p)


# ─────────────────────────────────────────────────────────────────
# HELPER: lateral trajectory
# (defined inline in main_controller.py — NOT in gait/generator.py)
# ─────────────────────────────────────────────────────────────────

def _lateral_trajectory(phase: float, step_length: float,
                        step_height: float, duty: float):
    """
    Lateral (Y-axis) trajectory generator.
    Same math as _leg_trajectory but for sideways movement.
    Returns: (dy, dz)
    """
    if phase < duty:
        s = phase / duty
        dy = +step_length / 2 - s * step_length
        dz = 0.0
        return dy, dz

    s = (phase - duty) / (1.0 - duty)
    dy = -step_length / 2 + s * step_length
    dz = step_height * math.sin(math.pi * s)
    return dy, dz


# ─────────────────────────────────────────────────────────────────
# FOOT TARGET GENERATORS
# (all defined inline in main_controller.py)
# ─────────────────────────────────────────────────────────────────

def compute_feet_forward_backward(phase: float, direction: str,
                                   stance_z: float = STANCE_Z) -> dict:
    x_mult = 1 if direction == "forward" else -1
    feet = {}

    for leg in ("FL", "FR", "RL", "RR"):
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        dx, dz = _leg_trajectory(leg_phase, STEP_LENGTH, STEP_HEIGHT, DUTY)
        dx = dx * x_mult
        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X + dx, base_y, stance_z + dz)

    return feet


def compute_feet_lateral(phase: float, direction: str,
                          stance_z: float = STANCE_Z) -> dict:
    feet = {}

    for leg in ("FL", "FR", "RL", "RR"):
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)

        if direction == "left":
            leg_phase = 1.0 - leg_phase

        dy, dz = _lateral_trajectory(
            leg_phase, LATERAL_STEP_LENGTH, LATERAL_STEP_HEIGHT, DUTY)

        y_mult = +1 if leg in ("FR", "RR") else -1
        dy = dy * y_mult

        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X, base_y + dy, stance_z + dz)

    return feet


def compute_feet_turn(phase: float, direction: str,
                       stance_z: float = STANCE_Z) -> dict:
    turn_mult = 1 if direction == "turn_left" else -1
    feet = {}

    for leg in ("FL", "FR", "RL", "RR"):
        leg_phase = phase if leg in DIAG_A else wrap_phase(phase + 0.5)
        dx, dz = _leg_trajectory(
            leg_phase, TURN_STEP_LENGTH, TURN_STEP_HEIGHT, DUTY)

        if leg in ("FL", "RL"):
            dx = -dx * turn_mult
        else:
            dx = +dx * turn_mult

        base_y = STANCE_Y if leg in ("FL", "RL") else -STANCE_Y
        feet[leg] = (STANCE_X + dx, base_y, stance_z + dz)

    return feet


def compute_feet_directional(phase: float, direction: str,
                              stance_z: float = STANCE_Z) -> dict:
    """Unified interface — routes to correct trajectory generator."""
    if direction in FORWARD_BACKWARD:
        return compute_feet_forward_backward(phase, direction, stance_z)
    elif direction in LEFT_RIGHT:
        return compute_feet_lateral(phase, direction, stance_z)
    elif direction in TURN:
        return compute_feet_turn(phase, direction, stance_z)
    else:
        return {
            "FL": (STANCE_X,  STANCE_Y, stance_z),
            "FR": (STANCE_X, -STANCE_Y, stance_z),
            "RL": (STANCE_X,  STANCE_Y, stance_z),
            "RR": (STANCE_X, -STANCE_Y, stance_z),
        }


# ─────────────────────────────────────────────────────────────────
# SERVO PIPELINE HELPERS
# (identical to main_controller.py)
# ─────────────────────────────────────────────────────────────────

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
    """Execute full IK → servo pipeline. Returns True on success."""
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


# ─────────────────────────────────────────────────────────────────
# EXECUTE SINGLE CYCLE
# Time-based phase, identical to main_controller.py
# ─────────────────────────────────────────────────────────────────

def execute_single_cycle(direction: str, stance_z: float = STANCE_Z,
                          freq_override: float = None,
                          perception_poll_fn=None):
    """
    Execute gait cycles in the given direction.
    Forward/Backward: 1 cycle. Lateral: 2 cycles. Turn: 1 cycle.
    Returns when complete.

    freq_override: if set, overrides the default frequency (used by
                   SLOW/CAUTION state to reduce speed).
    perception_poll_fn: called each loop iteration if provided, so
                        the controller can check for STOP mid-cycle.
                        Returns True if we should abort.
    """
    if direction in LEFT_RIGHT:
        freq = freq_override if freq_override else LATERAL_FREQ
        num_cycles = 2
        label = f"{direction.upper()} (lateral x2)"
    elif direction in TURN:
        freq = freq_override if freq_override else TURN_FREQ
        num_cycles = 1
        label = f"{direction.upper().replace('_', ' ')}"
    else:
        freq = freq_override if freq_override else FREQ
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

        # Compute phase (0→1 per cycle, wraps for multiple cycles)
        phase = (elapsed / single_cycle_duration) % 1.0

        # === PERCEPTION CHECK: abort forward if STOP zone entered ===
        if perception_poll_fn and direction in FORWARD_BACKWARD:
            if perception_poll_fn():
                print(f"\r  Executing: {label} [MID-CYCLE STOP]")
                stand_feet = {
                    "FL": (STANCE_X,  STANCE_Y, stance_z),
                    "FR": (STANCE_X, -STANCE_Y, stance_z),
                    "RL": (STANCE_X,  STANCE_Y, stance_z),
                    "RR": (STANCE_X, -STANCE_Y, stance_z),
                }
                execute_step(stand_feet)
                return

        # Compute and execute foot targets
        feet = compute_feet_directional(phase, direction, stance_z)
        execute_step(feet)

        # Progress bar
        progress = int((elapsed / total_duration) * 10)
        print(f"\r  Executing: {label} [{'=' * progress}{' ' * (10 - progress)}]",
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


# ─────────────────────────────────────────────────────────────────
# HEIGHT TRANSITION
# ─────────────────────────────────────────────────────────────────

def transition_height(from_z: float, to_z: float):
    """Smoothly interpolate between stance heights."""
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


# ─────────────────────────────────────────────────────────────────
# GAMEPAD INPUT
# ─────────────────────────────────────────────────────────────────

def get_key_blocking(controller, poll_fn=None):
    """
    Blocking gamepad read — identical mapping to main_controller.py.
    poll_fn is called every 10ms while waiting (for perception/IMU).
    """
    import pygame

    while True:
        pygame.event.pump()

        # D-PAD
        hat = controller.get_hat(0)
        if hat == (0, 1):   return 'w'
        if hat == (0, -1):  return 's'
        if hat == (-1, 0):  return 'q'
        if hat == (1, 0):   return 'e'

        # TRIGGERS
        lt = controller.get_axis(2)
        rt = controller.get_axis(5)
        if lt > 0.7: return '3'   # Wiggle
        if rt > 0.7: return '2'   # Stretch

        # BUTTONS
        if controller.get_button(4): return '1'   # Shake (LB)
        if controller.get_button(5): return '5'   # Bheek (RB)
        if controller.get_button(1): return '4'   # Push-ups (B)
        if controller.get_button(0): return '9'   # Tilt dance (A)
        if controller.get_button(3): return 'c'   # Height cycle (Y)
        if controller.get_button(2): return 'x'   # Quit (X)
        if controller.get_button(6): return '7'   # Sit (M1)
        if controller.get_button(7): return '6'   # High-five (M2)

        # Poll perception/IMU while idle
        if poll_fn:
            poll_fn()

        time.sleep(0.01)


# ═════════════════════════════════════════════════════════════════
# CONTROLLER CLASS — wraps everything above with perception
# ═════════════════════════════════════════════════════════════════

class RobotController:
    """
    Integrates Xbox gamepad with perception-driven avoidance.
    Runs in main thread (blocking gamepad reads).
    """

    def __init__(self, boot_mode: BootMode,
                 perception_queue: queue.Queue = None,
                 robot_state_queue: queue.Queue = None):
        self.boot_mode = boot_mode
        self.perc_queue = perception_queue
        self.state_queue = robot_state_queue

        # Proximity state — updated from perception queue
        self.proximity = ProximityState.SAFE
        self.control_mode = ControlMode.MANUAL
        self.nearest_distance = None
        self.nearest_bearing = None
        self.last_avoidance_time = 0.0

        # IMU
        self.imu_snapshot = IMUSnapshot()
        self._imu_filter = None
        self._imu_ready = False

        # Height mode
        self.height_index = 1  # NORMAL
        self.current_z = HEIGHT_MODES[1][1]

    # ── Hardware init ────────────────────────────────────────────

    def init_hardware(self):
        """Initialize PCA9685 + IMU."""
        try:
            init_pca()
            print("[CTRL] PCA9685 initialized")
        except Exception as e:
            raise OSError(
                f"Robot not responding on I2C. Check connection and power. ({e})")

        try:
            from hardware.imu import init_mpu, calibrate, IMUFilter
            init_mpu()
            print("[CTRL] Calibrating IMU — keep robot FLAT and STILL...")
            calib = calibrate(samples=200)
            self._imu_filter = IMUFilter(calib)
            self._imu_ready = True
            print("[CTRL] IMU ready")
        except Exception as e:
            print(f"[CTRL] IMU init failed (non-fatal): {e}")
            self._imu_ready = False

    # ── Perception polling ───────────────────────────────────────

    def _poll_perception(self):
        """Non-blocking drain of perception queue. Keeps latest."""
        if self.perc_queue is None:
            return

        latest = None
        while True:
            try:
                latest = self.perc_queue.get_nowait()
            except queue.Empty:
                break

        if latest is not None:
            self.proximity = latest.proximity_state
            self.nearest_distance = latest.nearest_distance_m
            self.nearest_bearing = latest.nearest_bearing_deg

    def _is_stop_zone(self) -> bool:
        """Check if currently in STOP zone. Used as mid-cycle abort."""
        self._poll_perception()
        return self.proximity == ProximityState.STOP

    # ── IMU polling ──────────────────────────────────────────────

    def _poll_imu(self):
        if not self._imu_ready:
            return
        try:
            r, p, rr, pr = self._imu_filter.update()
            self.imu_snapshot = IMUSnapshot(
                roll_deg=r, pitch_deg=p,
                roll_rate=rr, pitch_rate=pr,
                timestamp=time.time(),
            )
        except Exception:
            pass

    # ── Idle poll (called while waiting for gamepad input) ───────

    def _idle_poll(self):
        """Called every 10ms while waiting for gamepad button press."""
        self._poll_perception()
        self._poll_imu()
        self._publish_state("waiting")

    # ── State publishing ─────────────────────────────────────────

    def _publish_state(self, last_command: str = "idle"):
        if self.state_queue is None:
            return

        rs = RobotState(
            timestamp=time.time(),
            proximity=self.proximity,
            control_mode=self.control_mode,
            imu=self.imu_snapshot,
            step=StepState(),
            height_mode=HEIGHT_MODES[self.height_index][0],
            stance_z=self.current_z,
            last_command=last_command,
        )

        try:
            self.state_queue.put_nowait(rs)
        except queue.Full:
            try:
                self.state_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.state_queue.put_nowait(rs)
            except queue.Full:
                pass

    # ── Effective frequency ──────────────────────────────────────

    def _effective_freq(self, base_freq: float) -> float:
        if self.proximity in (ProximityState.SLOW, ProximityState.CAUTION):
            return base_freq * SPEED_SCALE_SLOW
        return base_freq

    # ── Avoidance maneuver ───────────────────────────────────────

    def _should_avoid(self, direction: str) -> bool:
        if direction != "forward":
            return False
        if self.proximity not in (ProximityState.STOP, ProximityState.SLOW):
            return False
        if time.monotonic() - self.last_avoidance_time < AVOIDANCE_COOLDOWN:
            return False
        if self.nearest_bearing is None:
            return False
        return True

    def _execute_avoidance(self):
        """
        Auto obstacle avoidance:
          1. Stop in place
          2. Read bearing from radar
          3. Turn away (positive bearing = right → turn_left)
          4. Resume
        Uses ONLY turn_left / turn_right. Never left / right.
        """
        self.control_mode = ControlMode.AUTO_AVOID
        self._publish_state("auto_avoid")
        print(f"[AVOID] Obstacle at {self.nearest_distance:.2f}m, "
              f"bearing {self.nearest_bearing:+.1f}°")

        # 1. Full stop
        execute_step(STAND_FEET)
        time.sleep(0.2)

        # 2. Determine turn direction
        bearing = self.nearest_bearing if self.nearest_bearing else 0.0
        if bearing >= 0:
            turn_dir = "turn_left"
            print(f"[AVOID] Obstacle right ({bearing:+.1f}°) → turning LEFT")
        else:
            turn_dir = "turn_right"
            print(f"[AVOID] Obstacle left ({bearing:+.1f}°) → turning RIGHT")

        # 3. Execute turn — more cycles for closer obstacles
        dist = self.nearest_distance if self.nearest_distance else 0.5
        cycles = max(1, min(3, int(0.8 / max(dist, 0.2))))

        for _ in range(cycles):
            self._poll_perception()
            execute_single_cycle(turn_dir, self.current_z)

        self.last_avoidance_time = time.monotonic()
        self.control_mode = ControlMode.MANUAL
        self._publish_state("avoidance_done")
        print("[AVOID] Complete — resuming manual control")

    # ── Main run loop ────────────────────────────────────────────

    def run(self):
        """
        Main control loop. Blocks on gamepad input.
        Perception data affects behavior between inputs and mid-cycle.
        """
        import pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("[CTRL] ERROR: No gamepad detected. Plug in Xbox controller.")
            return

        controller = pygame.joystick.Joystick(0)
        controller.init()
        print(f"[CTRL] Gamepad: {controller.get_name()}")

        # Print controls
        print("=" * 50)
        print("  QUADRUPED CONTROLLER — PERCEPTION INTEGRATED")
        print("=" * 50)
        print()
        print("  D-Pad: Move | Y: Height | X: Quit")
        print("  LB: Shake | RB: Bheek | B: Push-ups | A: Tilt")
        print("  LT: Wiggle | RT: Bow | M1: Sit | M2: High-five")
        print()
        if self.boot_mode in (BootMode.CAMERA_ONLY, BootMode.FULL):
            print("  [PERCEPTION ACTIVE] Obstacle avoidance enabled")
        else:
            print("  [NO CAMERA] Manual control only")
        print()
        print("Height modes:")
        for name, z in HEIGHT_MODES:
            marker = " <-- default" if name == "NORMAL" else ""
            print(f"  {name:8s}: Z={z:.2f}m{marker}")
        print()
        print("-" * 50)

        # Stand pose
        print("[CTRL] Moving to stand pose...")
        execute_step(STAND_FEET)
        time.sleep(0.5)
        print("[CTRL] Ready!")
        print("-" * 50)

        try:
            while True:
                mode_name = HEIGHT_MODES[self.height_index][0]
                zone_str = self.proximity.name

                print()
                print(f"[{zone_str}] Waiting for command [{mode_name}]: ",
                      end="", flush=True)

                key = get_key_blocking(controller, poll_fn=self._idle_poll)
                print(key.upper())

                # ── QUIT ──
                if key == 'x':
                    print("[CTRL] Returning to stand and exiting...")
                    stand()
                    break

                if key not in KEY_MAP:
                    print(f"  Unknown key '{key}'")
                    continue

                action = KEY_MAP[key]

                # ── HEIGHT CYCLE ──
                if action == "height":
                    old_z = self.current_z
                    self.height_index = (self.height_index + 1) % len(HEIGHT_MODES)
                    new_name, new_z = HEIGHT_MODES[self.height_index]
                    print(f"  Height → {new_name} (Z={new_z})")
                    transition_height(old_z, new_z)
                    self.current_z = new_z
                    self._publish_state(f"height_{new_name}")
                    continue

                # ── TRICKS ──
                if action in TRICK_MAP:
                    print(f"  → {action}")
                    self._publish_state(f"trick_{action}")
                    try:
                        TRICK_MAP[action]()
                    except Exception as e:
                        print(f"  [WARN] Trick error: {e}")
                    stand()
                    continue

                # ── MOVEMENT ──
                direction = action
                self._poll_perception()

                # STOP zone: block forward, trigger avoidance
                if direction == "forward" and self.proximity == ProximityState.STOP:
                    print(f"  !! BLOCKED — obstacle at "
                          f"{self.nearest_distance:.2f}m")
                    if self._should_avoid(direction):
                        self._execute_avoidance()
                    else:
                        execute_step(STAND_FEET)
                    continue

                # SLOW/CAUTION + forward: auto-avoid or slow down
                if direction == "forward" and self.proximity in (
                        ProximityState.SLOW, ProximityState.CAUTION):
                    if self._should_avoid(direction):
                        self._execute_avoidance()
                        continue
                    print(f"  → {direction} (SLOW — "
                          f"{self.nearest_distance:.2f}m)")
                    self._publish_state(f"slow_{direction}")
                    execute_single_cycle(
                        direction, self.current_z,
                        freq_override=self._effective_freq(FREQ),
                        perception_poll_fn=self._is_stop_zone)
                    continue

                # SAFE: normal execution
                print(f"  → {direction}")
                self._publish_state(direction)
                execute_single_cycle(
                    direction, self.current_z,
                    perception_poll_fn=self._is_stop_zone
                        if direction in FORWARD_BACKWARD else None)

        except KeyboardInterrupt:
            print("\n[CTRL] Caught Ctrl+C")

        finally:
            print("[CTRL] Returning to stand pose...")
            try:
                stand()
            except Exception:
                pass
            self._publish_state("shutdown")
            pygame.quit()
            print("[CTRL] Controller stopped.")


# ═════════════════════════════════════════════════════════════════
# HARDWARE CHECK
# ═════════════════════════════════════════════════════════════════

class RemoteIOError(Exception):
    """Raised when robot I2C communication fails."""
    pass


def check_robot_available():
    """Probe I2C bus for PCA9685. Raises RemoteIOError if not found."""
    try:
        from hardware.i2c_bus import get_i2c_bus
        from hardware.absolute_truths import PCA_ADDR
        bus = get_i2c_bus()
        bus.read_byte(PCA_ADDR)
        print(f"[HW CHECK] PCA9685 found at 0x{PCA_ADDR:02X}")
        return True
    except Exception as e:
        raise RemoteIOError(
            f"Robot not responding on I2C. Check connection and power. ({e})")