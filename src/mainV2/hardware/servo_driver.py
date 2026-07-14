# hardware/servo_driver.py
"""
Layer 1.2 — SERVO DRIVER (PCA9685)
===================================

Complete servo control driver for 12-servo quadruped robot.

Provides:
    - Angle-based servo control (caller never sees pulse math)
    - Leg+joint addressing (caller never sees channel numbers)
    - Automatic inversion handling for right-side servos
    - Mechanical limit enforcement on every command
    - Smooth speed-controlled motion (non-blocking)
    - Preset poses: stand, perpendicular
    - Per-servo enable/disable (limp mode)
    - Emergency disable_all()

Architecture:
    - Caller commands angles in degrees via leg ("FR") + joint ("thigh")
    - Driver clamps to mechanical limits from truths.py
    - Driver handles inversion (right-side servos mirror left-side)
    - Driver converts angle → PCA9685 pulse ticks
    - Smooth motion engine: call update() in main loop
    - No threading, no blocking sleeps

Integration:
    from hardware.servo_driver import create_servo_driver
    servo = create_servo_driver()
    servo.stand(speed=200)
    while servo.moving():
        servo.update()
        time.sleep(0.02)

NON-RESPONSIBILITIES:
    - No inverse kinematics
    - No gait sequencing
    - No balance / PID control
    - No IMU reading

All hardware constants come from truths.py — nothing is hardcoded here.
"""

import time
import math

from hardware.truths import (
    get_bus,
    ADDR,
    PCA9685,
    CHANNELS,
    LIMITS,
    PERPENDICULAR,
    STAND,
    LEGS,
    JOINTS,
)


# ╔══════════════════════════════════════════════════════════════╗
# ║  LOW-LEVEL PCA9685 HELPERS                                   ║
# ╚══════════════════════════════════════════════════════════════╝

def _read_byte(bus, addr, reg):
    """Single byte read."""
    return bus.read_byte_data(addr, reg)


def _write_byte(bus, addr, reg, value):
    """Single byte write."""
    bus.write_byte_data(addr, reg, value)


# ╔══════════════════════════════════════════════════════════════╗
# ║  PER-SERVO INTERNAL STATE                                    ║
# ╚══════════════════════════════════════════════════════════════╝

class _ServoState:
    """
    Tracks the complete state of one physical servo.

    Created once per servo at driver construction.
    Updated by set_angle() and update().
    """

    __slots__ = (
        "leg",              # "FR", "FL", "RR", "RL"
        "joint",            # "coxa", "thigh", "wrist"
        "channel",          # PCA9685 channel number (0-15)
        "low",              # mechanical minimum angle (degrees)
        "high",             # mechanical maximum angle (degrees)
        "inverted",         # True if increasing angle = decreasing physical angle
        "current_angle",    # where the servo IS right now (degrees)
        "target_angle",     # where it's GOING (degrees)
        "speed",            # degrees per second (0 = instant)
        "enabled",          # True if PWM output is active
    )

    def __init__(self, leg, joint, channel, low, high, inverted):
        self.leg = leg
        self.joint = joint
        self.channel = channel
        self.low = low
        self.high = high
        self.inverted = inverted
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.speed = 0.0
        self.enabled = False

    @property
    def at_target(self):
        """True if current angle has reached target angle."""
        return abs(self.current_angle - self.target_angle) < 0.01

    def clamp(self, angle):
        """Clamp angle to mechanical limits."""
        return max(self.low, min(self.high, angle))

    def __repr__(self):
        inv = "INV" if self.inverted else "   "
        ena = "ON " if self.enabled else "OFF"
        return (
            f"{self.leg}.{self.joint:6s} ch={self.channel:2d} "
            f"[{self.low:3.0f}..{self.high:3.0f}] {inv} "
            f"cur={self.current_angle:6.1f} tgt={self.target_angle:6.1f} "
            f"spd={self.speed:5.0f} {ena}"
        )


# ╔══════════════════════════════════════════════════════════════╗
# ║  SERVO DRIVER — MAIN CLASS                                   ║
# ╚══════════════════════════════════════════════════════════════╝

class ServoDriver:
    """
    Complete servo control driver for 12-servo quadruped.

    Usage:
        servo = ServoDriver()
        servo.enable_all()
        servo.stand(speed=200)
        while servo.moving():
            servo.update()
            time.sleep(0.02)
        servo.disable_all()

    Or use the factory:
        servo = create_servo_driver()
    """

    def __init__(self):
        """
        Construct the driver and initialize PCA9685.

        All servos start DISABLED (no PWM output, servos limp).
        Call enable_all() or enable(leg, joint) before commanding.
        """
        self._bus = get_bus()
        self._last_time = None

        # ── Build servo state map ─────────────────────────────
        # Key: (leg, joint) tuple
        # Value: _ServoState object
        self._servos = {}

        for leg in LEGS:
            for joint in JOINTS:
                channel = CHANNELS[leg][joint]
                limits = LIMITS[leg][joint]
                low = limits["low"]
                high = limits["high"]
                inverted = limits["inverted"]

                state = _ServoState(
                    leg=leg,
                    joint=joint,
                    channel=channel,
                    low=low,
                    high=high,
                    inverted=inverted,
                )

                # Initialize current/target to perpendicular
                perp = PERPENDICULAR[leg][joint]
                state.current_angle = perp
                state.target_angle = perp

                self._servos[(leg, joint)] = state

        # ── Initialize PCA9685 ────────────────────────────────
        self._init_pca9685()

    # ──────────────────────────────────────────────────────────
    # PCA9685 initialization
    # ──────────────────────────────────────────────────────────

    def _init_pca9685(self):
        """
        Initialize PCA9685 for 50 Hz servo PWM.

        Sequence:
            1. Enter sleep mode (required to change prescaler)
            2. Set prescaler for 50 Hz
            3. Wake from sleep
            4. Wait for oscillator
            5. Restart PWM generation
        """
        addr = ADDR.PCA9685

        # Read current MODE1
        mode1 = _read_byte(self._bus, addr, PCA9685.MODE1)

        # Enter sleep mode (bit 4 = SLEEP)
        _write_byte(self._bus, addr, PCA9685.MODE1, (mode1 | 0x10) & 0xFF)
        time.sleep(0.005)

        # Calculate and set prescaler for target frequency
        # PCA9685 formula: prescale = round(25MHz / (4096 × freq)) - 1
        prescale = round(25_000_000 / (4096 * PCA9685.FREQ_HZ)) - 1
        _write_byte(self._bus, addr, PCA9685.PRESCALE, prescale)

        # Wake from sleep (clear SLEEP bit)
        _write_byte(self._bus, addr, PCA9685.MODE1, mode1 & ~0x10 & 0xFF)
        time.sleep(0.005)  # wait for oscillator

        # Restart (set RESTART bit 7)
        _write_byte(self._bus, addr, PCA9685.MODE1, mode1 | 0x80)
        time.sleep(0.005)

        # Verify prescaler
        actual_prescale = _read_byte(self._bus, addr, PCA9685.PRESCALE)
        actual_freq = 25_000_000 / (4096 * (actual_prescale + 1))

        print(
            f"[SERVO] PCA9685 initialized "
            f"(prescale={actual_prescale}, freq={actual_freq:.1f}Hz)"
        )

    # ──────────────────────────────────────────────────────────
    # Pulse math
    # ──────────────────────────────────────────────────────────

    def _angle_to_pulse(self, state, angle):
        """
        Convert a clamped angle to PCA9685 pulse ticks.

        Args:
            state:  _ServoState for this servo
            angle:  angle in degrees (already clamped to limits)

        Returns:
            Integer pulse tick count [PULSE_MIN, PULSE_MAX]

        Logic:
            1. Normalize angle to [0.0, 1.0] within joint's range
            2. If inverted (right-side), flip the fraction
            3. Map fraction to [PULSE_MIN, PULSE_MAX]
        """
        angle_range = state.high - state.low
        if angle_range <= 0:
            return PCA9685.PULSE_MIN

        fraction = (angle - state.low) / angle_range

        if state.inverted:
            fraction = 1.0 - fraction

        pulse = PCA9685.PULSE_MIN + fraction * (PCA9685.PULSE_MAX - PCA9685.PULSE_MIN)
        pulse = int(round(pulse))

        # Final safety clamp
        pulse = max(PCA9685.PULSE_MIN, min(PCA9685.PULSE_MAX, pulse))

        return pulse

    def _write_pulse(self, channel, pulse):
        """
        Write pulse ticks to a PCA9685 channel.

        Each channel has 4 registers: ON_L, ON_H, OFF_L, OFF_H.
        We set ON time to 0 and OFF time to the pulse width.
        """
        addr = ADDR.PCA9685
        reg_base = PCA9685.LED0_ON_L + 4 * channel

        on = 0
        off = pulse

        _write_byte(self._bus, addr, reg_base + 0, on & 0xFF)
        _write_byte(self._bus, addr, reg_base + 1, (on >> 8) & 0xFF)
        _write_byte(self._bus, addr, reg_base + 2, off & 0xFF)
        _write_byte(self._bus, addr, reg_base + 3, (off >> 8) & 0xFF)

    def _write_off(self, channel):
        """
        Turn off PWM output on a channel (servo goes limp).

        Setting bit 4 of the ON_H register (full ON) and clearing
        the OFF register disables the channel output entirely.
        Actually, setting bit 4 of OFF_H (full OFF) is the correct
        way to disable output per the PCA9685 datasheet.
        """
        addr = ADDR.PCA9685
        reg_base = PCA9685.LED0_ON_L + 4 * channel

        _write_byte(self._bus, addr, reg_base + 0, 0x00)
        _write_byte(self._bus, addr, reg_base + 1, 0x00)
        _write_byte(self._bus, addr, reg_base + 2, 0x00)
        _write_byte(self._bus, addr, reg_base + 3, 0x10)  # bit 4 = full OFF

    def _commit_servo(self, state):
        """
        Write the current angle of a servo to hardware.
        Only writes if the servo is enabled.
        """
        if state.enabled:
            pulse = self._angle_to_pulse(state, state.current_angle)
            self._write_pulse(state.channel, pulse)
        else:
            self._write_off(state.channel)

    # ──────────────────────────────────────────────────────────
    # State lookup
    # ──────────────────────────────────────────────────────────

    def _get_state(self, leg, joint):
        """
        Get the _ServoState for a specific servo.
        Raises KeyError with helpful message if not found.
        """
        key = (leg, joint)
        if key not in self._servos:
            raise KeyError(
                f"Unknown servo '{leg}.{joint}'. "
                f"Valid legs: {LEGS}, valid joints: {JOINTS}"
            )
        return self._servos[key]

    # ──────────────────────────────────────────────────────────
    # Single servo commands
    # ──────────────────────────────────────────────────────────

    def set_angle(self, leg, joint, angle, speed=0):
        """
        Command a single servo to a target angle.

        Args:
            leg:    "FR", "FL", "RR", or "RL"
            joint:  "coxa", "thigh", or "wrist"
            angle:  target angle in degrees (clamped to limits)
            speed:  degrees per second (0 = instant move)

        If speed=0, the servo moves immediately to the target.
        If speed>0, the servo moves smoothly — call update() in your loop.
        """
        state = self._get_state(leg, joint)
        angle = state.clamp(float(angle))

        state.target_angle = angle
        state.speed = float(max(0, speed))

        if state.speed == 0:
            # Instant move
            state.current_angle = angle
            self._commit_servo(state)

    def get_angle(self, leg, joint):
        """
        Get the current tracked angle of a servo.

        Returns the software-tracked angle in degrees.
        This is where the servo IS, not where it's GOING.
        """
        state = self._get_state(leg, joint)
        return state.current_angle

    def get_target(self, leg, joint):
        """
        Get the target angle of a servo.

        Returns the angle the servo is moving toward.
        """
        state = self._get_state(leg, joint)
        return state.target_angle

    # ──────────────────────────────────────────────────────────
    # Multi-servo commands
    # ──────────────────────────────────────────────────────────

    def set_leg(self, leg, coxa, thigh, wrist, speed=0):
        """
        Command all 3 joints of a single leg.

        Args:
            leg:    "FR", "FL", "RR", or "RL"
            coxa:   coxa angle in degrees
            thigh:  thigh angle in degrees
            wrist:  wrist angle in degrees
            speed:  degrees per second (0 = instant)
        """
        self.set_angle(leg, "coxa", coxa, speed=speed)
        self.set_angle(leg, "thigh", thigh, speed=speed)
        self.set_angle(leg, "wrist", wrist, speed=speed)

    def set_all(self, pose, speed=0):
        """
        Command all 12 servos from a pose dictionary.

        Args:
            pose:   dict matching truths.py format:
                    {"FR": {"coxa": 47, "thigh": 98, "wrist": 122}, ...}
            speed:  degrees per second (0 = instant)

        The pose dict can be partial — only specified legs/joints
        will be commanded. Unspecified servos keep their current target.
        """
        for leg in pose:
            if leg not in LEGS:
                raise KeyError(f"Unknown leg '{leg}'. Valid: {LEGS}")
            joints = pose[leg]
            for joint in joints:
                if joint not in JOINTS:
                    raise KeyError(f"Unknown joint '{joint}'. Valid: {JOINTS}")
                self.set_angle(leg, joint, joints[joint], speed=speed)

    # ──────────────────────────────────────────────────────────
    # Preset poses
    # ──────────────────────────────────────────────────────────

    def stand(self, speed=0):
        """
        Move all servos to the measured standing pose.

        Args:
            speed: degrees per second (0 = instant)

        The stand pose angles come from truths.STAND, which were
        determined through physical testing and tuning.
        """
        self.set_all(STAND, speed=speed)

    def perpendicular(self, speed=0):
        """
        Move all servos to perpendicular pose.

        Args:
            speed: degrees per second (0 = instant)

        The perpendicular pose has all limb segments at 90° to the body.
        Used during assembly for accurate servo mounting.
        """
        self.set_all(PERPENDICULAR, speed=speed)

    # ──────────────────────────────────────────────────────────
    # Smooth motion engine
    # ──────────────────────────────────────────────────────────

    def update(self):
        """
        Advance all servos toward their targets.

        Must be called at a regular rate (20-100 Hz recommended).
        Measures real dt internally — no fixed timestep assumption.

        For servos with speed=0 (instant), this does nothing
        because they already reached their target in set_angle().

        Returns:
            True if any servo is still moving toward its target.
            False if all servos have reached their targets.
        """
        now = time.monotonic()
        if self._last_time is None:
            dt = 0.02
        else:
            dt = now - self._last_time
        self._last_time = now

        # Clamp dt to prevent huge jumps
        dt = max(0.001, min(0.1, dt))

        any_moving = False

        for state in self._servos.values():
            if not state.enabled:
                continue

            if state.at_target:
                continue

            if state.speed <= 0:
                # Should already be at target (instant move)
                continue

            any_moving = True

            # Compute step size for this frame
            step = state.speed * dt
            diff = state.target_angle - state.current_angle

            if abs(diff) <= step:
                # Close enough — snap to target
                state.current_angle = state.target_angle
            elif diff > 0:
                state.current_angle += step
            else:
                state.current_angle -= step

            # Clamp (safety)
            state.current_angle = state.clamp(state.current_angle)

            # Write to hardware
            self._commit_servo(state)

        return any_moving

    def moving(self):
        """
        Check if any servo is still moving toward its target.

        Returns True if at least one enabled servo hasn't reached
        its target angle yet.
        """
        for state in self._servos.values():
            if state.enabled and not state.at_target:
                return True
        return False

    # ──────────────────────────────────────────────────────────
    # Enable / disable
    # ──────────────────────────────────────────────────────────

    def enable(self, leg=None, joint=None):
        """
        Enable PWM output for specific or all servos.

        Args:
            leg:    Specific leg or None for all legs
            joint:  Specific joint or None for all joints

        When a servo is enabled, it drives to its current angle.
        """
        for state in self._iter_servos(leg, joint):
            state.enabled = True
            self._commit_servo(state)

    def disable(self, leg=None, joint=None):
        """
        Disable PWM output for specific or all servos.

        Args:
            leg:    Specific leg or None for all legs
            joint:  Specific joint or None for all joints

        Disabled servos go limp (no holding torque).
        """
        for state in self._iter_servos(leg, joint):
            state.enabled = False
            self._write_off(state.channel)

    def enable_all(self):
        """Enable all 12 servos."""
        self.enable()

    def disable_all(self):
        """
        Emergency stop — all servos go limp immediately.

        Disables PWM output on all channels. Servos lose holding
        torque and the robot will collapse under gravity.
        """
        for state in self._servos.values():
            state.enabled = False
            self._write_off(state.channel)
        print("[SERVO] All servos disabled")

    def _iter_servos(self, leg=None, joint=None):
        """
        Iterate over servos matching the filter.

        If leg is None, matches all legs.
        If joint is None, matches all joints.
        """
        for (l, j), state in self._servos.items():
            if leg is not None and l != leg:
                continue
            if joint is not None and j != joint:
                continue
            yield state

    # ──────────────────────────────────────────────────────────
    # Properties (read-only access to state)
    # ──────────────────────────────────────────────────────────

    @property
    def angles(self):
        """
        Current angles of all servos.

        Returns dict: {"FR": {"coxa": 47.0, "thigh": 98.0, ...}, ...}
        """
        result = {}
        for leg in LEGS:
            result[leg] = {}
            for joint in JOINTS:
                result[leg][joint] = self._servos[(leg, joint)].current_angle
        return result

    @property
    def targets(self):
        """
        Target angles of all servos.

        Returns dict: {"FR": {"coxa": 47.0, "thigh": 120.0, ...}, ...}
        """
        result = {}
        for leg in LEGS:
            result[leg] = {}
            for joint in JOINTS:
                result[leg][joint] = self._servos[(leg, joint)].target_angle
        return result

    @property
    def enabled_map(self):
        """
        Enable state of all servos.

        Returns dict: {"FR": {"coxa": True, ...}, ...}
        """
        result = {}
        for leg in LEGS:
            result[leg] = {}
            for joint in JOINTS:
                result[leg][joint] = self._servos[(leg, joint)].enabled
        return result


# ╔══════════════════════════════════════════════════════════════╗
# ║  FACTORY FUNCTION                                            ║
# ╚══════════════════════════════════════════════════════════════╝

def create_servo_driver():
    """
    Create a fully initialized ServoDriver.

    Returns:
        ServoDriver instance, ready to use.
        All servos start DISABLED (call enable_all() to activate).
    """
    driver = ServoDriver()
    return driver


# ╔══════════════════════════════════════════════════════════════╗
# ║  SMOKE TEST                                                  ║
# ╚══════════════════════════════════════════════════════════════╝

if __name__ == "__main__":
    import sys

    print("=" * 70)
    print("  SERVO DRIVER — SMOKE TEST")
    print("=" * 70)

    servo = create_servo_driver()

    # ──────────────────────────────────────────────────────────
    # [1] Servo state table
    # ──────────────────────────────────────────────────────────
    print("\n[1] Servo State Table (initial)")
    print("-" * 70)
    print(
        f"  {'Servo':<12s} {'Ch':>3s} {'Limits':>12s} {'Inv':>4s} "
        f"{'Perp':>5s} {'Stand':>6s} {'Pulse@Perp':>11s} {'Pulse@Stand':>12s}"
    )
    print("-" * 70)

    for leg in LEGS:
        for joint in JOINTS:
            state = servo._get_state(leg, joint)
            perp = PERPENDICULAR[leg][joint]
            stand_angle = STAND[leg][joint]

            pulse_perp = servo._angle_to_pulse(state, perp)
            pulse_stand = servo._angle_to_pulse(state, stand_angle)

            inv = "INV" if state.inverted else ""
            print(
                f"  {leg}.{joint:<6s}  {state.channel:3d} "
                f"  [{state.low:3.0f}..{state.high:3.0f}]  {inv:>3s} "
                f" {perp:5.0f} {stand_angle:6.0f} "
                f"  {pulse_perp:5d}       {pulse_stand:5d}"
            )

    # ──────────────────────────────────────────────────────────
    # [2] Pulse range verification
    # ──────────────────────────────────────────────────────────
    print(f"\n[2] Pulse Range Verification")
    print("-" * 70)

    all_ok = True
    for leg in LEGS:
        for joint in JOINTS:
            state = servo._get_state(leg, joint)

            pulse_low = servo._angle_to_pulse(state, state.low)
            pulse_high = servo._angle_to_pulse(state, state.high)

            lo = min(pulse_low, pulse_high)
            hi = max(pulse_low, pulse_high)

            if lo < PCA9685.PULSE_MIN or hi > PCA9685.PULSE_MAX:
                print(f"  FAIL — {leg}.{joint}: pulse [{lo}..{hi}] outside [{PCA9685.PULSE_MIN}..{PCA9685.PULSE_MAX}]")
                all_ok = False

    if all_ok:
        print(f"  PASS — all pulses within [{PCA9685.PULSE_MIN}..{PCA9685.PULSE_MAX}]")

    # ──────────────────────────────────────────────────────────
    # [3] Symmetry check
    # ──────────────────────────────────────────────────────────
    print(f"\n[3] Left/Right Symmetry Check")
    print("-" * 70)

    pairs = [("FR", "FL"), ("RR", "RL")]
    for right, left in pairs:
        for joint in JOINTS:
            r_state = servo._get_state(right, joint)
            l_state = servo._get_state(left, joint)

            r_perp = PERPENDICULAR[right][joint]
            l_perp = PERPENDICULAR[left][joint]

            r_pulse = servo._angle_to_pulse(r_state, r_perp)
            l_pulse = servo._angle_to_pulse(l_state, l_perp)

            diff = abs(r_pulse - l_pulse)
            ok = "OK" if diff < 50 else "CHECK"
            print(
                f"  {right}.{joint:<6s} perp={r_perp:3.0f}° pulse={r_pulse:3d}   "
                f"{left}.{joint:<6s} perp={l_perp:3.0f}° pulse={l_pulse:3d}   "
                f"diff={diff:3d} {ok}"
            )

    # ──────────────────────────────────────────────────────────
    # [4] Stand pose (optional — moves real servos)
    # ──────────────────────────────────────────────────────────
    if "--stand" in sys.argv:
        print(f"\n[4] Stand Pose — MOVING SERVOS")
        print("-" * 70)
        print("  Enabling all servos...")

        servo.enable_all()
        time.sleep(0.1)

        # Check if smooth stand requested
        if "--smooth" in sys.argv:
            speed = 150.0  # degrees per second
            for arg in sys.argv:
                if arg.startswith("--speed="):
                    try:
                        speed = float(arg.split("=")[1])
                    except ValueError:
                        pass

            print(f"  Moving to stand pose (smooth, {speed:.0f}°/s)...")
            servo.stand(speed=speed)

            step_count = 0
            while servo.moving():
                servo.update()
                step_count += 1
                if step_count % 10 == 0:
                    # Print progress
                    angles = servo.angles
                    fr_thigh = angles["FR"]["thigh"]
                    fl_thigh = angles["FL"]["thigh"]
                    print(
                        f"    step {step_count:4d}  "
                        f"FR.thigh={fr_thigh:6.1f}  "
                        f"FL.thigh={fl_thigh:6.1f}"
                    )
                time.sleep(0.02)

            print(f"  Stand reached in {step_count} steps")

        else:
            print("  Moving to stand pose (instant)...")
            servo.stand()

        # Print final angles
        print("\n  Final angles:")
        angles = servo.angles
        for leg in LEGS:
            a = angles[leg]
            s = STAND[leg]
            print(
                f"    {leg}:  coxa={a['coxa']:6.1f} (target {s['coxa']:3.0f})  "
                f"thigh={a['thigh']:6.1f} (target {s['thigh']:3.0f})  "
                f"wrist={a['wrist']:6.1f} (target {s['wrist']:3.0f})"
            )

        print("\n  Press Enter to disable all servos...")
        try:
            input()
        except KeyboardInterrupt:
            pass

        servo.disable_all()

    elif "--perp" in sys.argv:
        print(f"\n[4] Perpendicular Pose — MOVING SERVOS")
        print("-" * 70)
        print("  Enabling all servos...")

        servo.enable_all()
        time.sleep(0.1)

        print("  Moving to perpendicular pose (instant)...")
        servo.perpendicular()

        print("\n  Final angles:")
        angles = servo.angles
        for leg in LEGS:
            a = angles[leg]
            p = PERPENDICULAR[leg]
            print(
                f"    {leg}:  coxa={a['coxa']:6.1f} (target {p['coxa']:3.0f})  "
                f"thigh={a['thigh']:6.1f} (target {p['thigh']:3.0f})  "
                f"wrist={a['wrist']:6.1f} (target {p['wrist']:3.0f})"
            )

        print("\n  Press Enter to disable all servos...")
        try:
            input()
        except KeyboardInterrupt:
            pass

        servo.disable_all()

    elif "--sweep" in sys.argv:
        print(f"\n[4] Sweep Test — MOVING SINGLE SERVO")
        print("-" * 70)

        # Default: FR thigh
        sweep_leg = "FR"
        sweep_joint = "thigh"
        sweep_speed = 100.0

        for arg in sys.argv:
            if arg.startswith("--leg="):
                sweep_leg = arg.split("=")[1].upper()
            elif arg.startswith("--joint="):
                sweep_joint = arg.split("=")[1].lower()
            elif arg.startswith("--speed="):
                try:
                    sweep_speed = float(arg.split("=")[1])
                except ValueError:
                    pass

        state = servo._get_state(sweep_leg, sweep_joint)
        print(
            f"  Sweeping {sweep_leg}.{sweep_joint} "
            f"[{state.low:.0f}°..{state.high:.0f}°] "
            f"at {sweep_speed:.0f}°/s"
        )
        print("  Press Ctrl+C to stop\n")

        servo.enable(sweep_leg, sweep_joint)
        time.sleep(0.1)

        # Move to low limit first
        servo.set_angle(sweep_leg, sweep_joint, state.low)
        time.sleep(0.5)

        # Sweep up
        print("  Sweeping UP...")
        servo.set_angle(sweep_leg, sweep_joint, state.high, speed=sweep_speed)
        try:
            while servo.moving():
                servo.update()
                angle = servo.get_angle(sweep_leg, sweep_joint)
                pulse = servo._angle_to_pulse(state, angle)
                print(f"    angle={angle:6.1f}°  pulse={pulse:4d}", end="\r")
                time.sleep(0.02)
            print(f"\n  Reached {state.high:.0f}°")

            time.sleep(0.5)

            # Sweep down
            print("  Sweeping DOWN...")
            servo.set_angle(sweep_leg, sweep_joint, state.low, speed=sweep_speed)
            while servo.moving():
                servo.update()
                angle = servo.get_angle(sweep_leg, sweep_joint)
                pulse = servo._angle_to_pulse(state, angle)
                print(f"    angle={angle:6.1f}°  pulse={pulse:4d}", end="\r")
                time.sleep(0.02)
            print(f"\n  Reached {state.low:.0f}°")

        except KeyboardInterrupt:
            print("\n  Interrupted")

        servo.disable(sweep_leg, sweep_joint)
        print(f"  {sweep_leg}.{sweep_joint} disabled")

    else:
        print(f"\n[4] Physical Tests (skipped — use flags to enable)")
        print("  --stand              Move to stand pose (instant)")
        print("  --stand --smooth     Move to stand pose (smooth)")
        print("  --perp               Move to perpendicular pose")
        print("  --sweep              Sweep single servo through range")
        print("  --speed=N            Set speed in °/s (default: 150)")
        print("  --leg=FR             Set sweep leg (default: FR)")
        print("  --joint=thigh        Set sweep joint (default: thigh)")

    # ──────────────────────────────────────────────────────────
    # [5] Summary
    # ──────────────────────────────────────────────────────────
    print()
    print("=" * 70)
    print("  SMOKE TEST COMPLETE")
    print("=" * 70)