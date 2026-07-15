"""
L5 — CONTROL LOOP (the ONE loop)
================================

Wires command -> gait -> IK -> conventions -> servos, reads IMU state, emits
telemetry. Owns timing (monotonic, fixed-rate), phase accumulation, mode state,
and safe-pose-on-error. Exactly one loop body — no gamepad/keyboard/web copies.

Set hardware=False for a dry run (no I2C): validates timing/phase/gait/IK with
no robot attached.
"""

import time
from dataclasses import replace

from config import gait_params as GP
from config.robot_spec import LEGS, stance_foot, apply_com_offset, apply_body_tilt
from control.command import Command, clean, gait_freq, height_z
from gait.gait import foot_targets as trot_foot_targets
from gait.crawl import foot_targets as crawl_foot_targets
from kinematics.ik import solve_all
from hardware.conventions import joints_to_servo


def stand_feet(z):
    return {leg: stance_foot(leg, z) for leg in LEGS}


class WalkController:
    def __init__(self, hardware=True, telemetry=None, imu_enabled=True, gait="trot",
                 trim_roll=None, trim_pitch=None, strafe_trim=None):
        self.hardware = hardware
        self.telemetry = telemetry
        self.imu_enabled = imu_enabled and hardware
        self.gait = gait                       # "trot" | "crawl"
        self.trim_roll = GP.TRIM_ROLL if trim_roll is None else trim_roll
        self.trim_pitch = GP.TRIM_PITCH if trim_pitch is None else trim_pitch
        self.strafe_trim = GP.STRAFE_TRIM if strafe_trim is None else strafe_trim
        self.phase = 0.0
        self._pca = None
        self._imu = None

    # -- lifecycle --
    def start(self, calibrate=True):
        if self.hardware:
            from hardware import pca9685
            self._pca = pca9685
            self._pca.init()
        if self.imu_enabled:
            from estimation.imu_state import IMUEstimator
            self._imu = IMUEstimator()
            if calibrate:
                self._imu.calibrate()

    # -- one tick: pure-ish compute + optional hardware write --
    def step(self, cmd: Command, dt: float):
        cmd = clean(cmd)
        z = height_z(cmd)

        if cmd.is_moving() and self.strafe_trim:
            s = max(-1.0, min(1.0, cmd.strafe + self.strafe_trim))
            cmd = replace(cmd, strafe=s)

        if cmd.is_moving():
            freq = gait_freq(cmd, self.gait)
            self.phase = (self.phase + freq * dt) % 1.0
            if self.gait == "crawl":
                feet = crawl_foot_targets(self.phase, cmd, None, z)
            else:
                feet = trot_foot_targets(self.phase, cmd, GP.DEFAULT_GAIT, z)
        else:
            feet = stand_feet(z)   # phase frozen for continuity

        feet = apply_com_offset(feet)   # center the real (off-center) CoM
        feet = apply_body_tilt(feet, self.trim_pitch, self.trim_roll)  # constant lean trim
        deltas, reachable = solve_all(feet)
        servo = joints_to_servo(deltas)

        note = None if reachable else "unreachable-clamped"
        if self.hardware and self._pca is not None:
            try:
                self._pca.apply_pose(servo)
            except OSError as e:
                note = f"bus-fault:{e}"   # guarded — loop never dies on I2C

        state = None
        if self._imu is not None:
            try:
                state = self._imu.update()
            except OSError:
                pass
        if self.telemetry is not None:
            self.telemetry.log_tick(self.phase, cmd, state, dt, note)
        return feet, servo, state

    # -- fixed-rate run driven by a command source --
    def run(self, command_source, duration=None, dt=GP.LOOP_DT):
        t_end = None if duration is None else time.monotonic() + duration
        while True:
            t0 = time.monotonic()
            if t_end is not None and t0 >= t_end:
                break
            cmd = command_source()
            if cmd is None:
                break
            self.step(cmd, dt)
            # maintain fixed rate
            sleep = dt - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

    def settle_to_stand(self, height="NORMAL"):
        self.step(Command(mode="IDLE", height=height), GP.LOOP_DT)
