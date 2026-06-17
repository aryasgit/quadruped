#!/usr/bin/env python3
"""
BARQ v1 — HARDWARE runner. The sim's twin: same trajectory frames, same
50 Hz cadence, real PCA9685 instead of PyBullet (D-013).

    # robot ON THE STAND, hand-placed near the crouch pose, servo rail on:
    ~/barq_v1/venv/bin/python stack/runtime/run_robot.py --scenario stand
    ... --scenario pose_sweep | weight_shift | walk [--cycles 2]
    ... --dry-run        # full pipeline against the simulated bus, no hardware

Every run: engage at crouch (staggered) -> ramp to stand -> scenario ->
ramp back to crouch -> ALL OFF. Ctrl-C at any moment = ALL OFF.

Requires stack/config/servo_calibration.yaml (calibration GUI, all 12
servos with 2+ recorded points). See docs/06_CALIBRATION_PROTOCOL.md.
"""

import argparse
import datetime
import signal
import sys
import time
from pathlib import Path

STACK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STACK_ROOT))

from barq1 import gait, trajectories as tj
from barq1.imu import IMU
from barq1.kinematics import body_ik, stance_feet_world
from barq1.pca9685 import PCA9685
from barq1.servo_map import CalibrationError, ServoMap
from runtime.robot_io import RobotIO, Telemetry

ARTIFACTS = Path.home() / "barq_v1" / "artifacts"


def crouch_angles():
    return body_ik(stance_feet_world(tj.CROUCH_H), body_xyz=(0, 0, tj.CROUCH_H))


def stand_angles():
    return body_ik(stance_feet_world(tj.STAND_H), body_xyz=(0, 0, tj.STAND_H))


def scenario_frames(name, cycles):
    if name == "stand":
        return tj.hold(stance_feet_world(tj.STAND_H), (0, 0, tj.STAND_H), (0, 0, 0), 3.0)
    if name == "pose_sweep":
        return tj.pose_sweep()
    if name == "weight_shift":
        return tj.weight_shift_lift()
    if name == "walk":
        return gait.crawl(cycles)
    raise ValueError(name)


def run(frames, io, telem, imu):
    period = tj.DT
    overruns = 0
    next_t = time.monotonic()
    for i, (feet, xyz, rpy) in enumerate(frames):
        angles = io.apply(body_ik(feet, body_xyz=xyz, body_rpy=rpy))
        rp = gy = None
        if imu is not None:
            rp, gy = imu.update()
        telem.record(i, angles, rp, gy, overruns)
        next_t += period
        lag = next_t - time.monotonic()
        if lag > 0:
            time.sleep(lag)
        else:
            overruns += 1
            next_t = time.monotonic()
    if overruns:
        print(f"[run] {overruns} frame overruns (loop slower than 50 Hz)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="stand",
                    choices=["stand", "pose_sweep", "weight_shift", "walk"])
    ap.add_argument("--cycles", type=int, default=2, help="walk cycles")
    ap.add_argument("--dry-run", action="store_true",
                    help="simulated I2C bus — full pipeline, no hardware")
    ap.add_argument("--calib", default=None, help="calibration yaml path")
    ap.add_argument("--no-imu", action="store_true")
    ap.add_argument("--max-speed", type=float, default=240.0, help="deg/s slew cap")
    ap.add_argument("--yes", action="store_true", help="skip the stand confirm")
    ap.add_argument("--teleop", action="store_true",
                    help="PS4 controller drives the robot (pose + TRIANGLE walk)")
    ap.add_argument("--hold", action="store_true",
                    help="engage directly at STAND and hold it (no crouch ramp, "
                         "no scenario) until Ctrl-C — safe first-bring-up mode")
    args = ap.parse_args()

    try:
        smap = ServoMap.from_yaml(args.calib) if args.calib else ServoMap.from_yaml()
    except CalibrationError as e:
        print(f"[run_robot] {e}")
        return 1

    near = "STAND" if args.hold else "crouch"
    if not (args.dry_run or args.yes):
        ans = input(f"Robot ON THE STAND, legs free, hand-placed near {near}? [y/N] ")
        if ans.strip().lower() != "y":
            print("aborted.")
            return 1

    pca = PCA9685(sim=True) if args.dry_run else PCA9685()
    io = RobotIO(pca=pca, smap=smap, max_speed_dps=args.max_speed)

    imu = None
    if not args.no_imu:
        try:
            imu = IMU(sim=args.dry_run)
            if not args.dry_run:
                print("[imu] calibrating — keep the robot still…")
                imu.calibrate()
        except Exception as e:
            print(f"[imu] unavailable ({e}) — continuing without")
            imu = None

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    telem = Telemetry(ARTIFACTS / f"run-{args.scenario}-{stamp}.jsonl")

    signal.signal(signal.SIGINT, lambda *_: (_panic(io, telem)))

    try:
        if args.hold:
            # Safe first-bring-up: engage DIRECTLY at the verified stand pose
            # (never command a non-stand pose, so unverified slope signs can't
            # bite) and hold it until Ctrl-C.
            _hold_stand(io, telem, imu)
        else:
            print("[run] engaging crouch (staggered)…")
            io.engage(crouch_angles())
            time.sleep(1.0)
            print("[run] ramping to stand…")
            run(tj.stance_ramp(tj.CROUCH_H, tj.STAND_H, 3.0), io, telem, imu)
            if args.teleop:
                _teleop(args, io, telem, imu)
            else:
                print(f"[run] scenario: {args.scenario}")
                run(scenario_frames(args.scenario, args.cycles), io, telem, imu)
            print("[run] ramping down…")
            run(tj.stance_ramp(tj.STAND_H, tj.CROUCH_H, 3.0), io, telem, imu)
    finally:
        io.all_off()
        telem.close()
        print("[run] all outputs OFF.")
    return 0


def _hold_stand(io, telem, imu):
    """Engage at the verified stand pose and hold it at 50 Hz until Ctrl-C."""
    from barq1.command import GaitCommand
    from barq1.controller import Controller
    from barq1.kinematics import body_ik

    ctrl = Controller(dt=tj.DT, start_state="stand")
    print("[run] engaging STAND (staggered, slew-limited)…")
    io.engage(stand_angles())
    time.sleep(1.0)
    print("[run] holding STAND. Ctrl-C (or kill -INT <pid>) -> all outputs OFF.")
    nxt = time.monotonic()
    i = 0
    while True:
        feet, xyz, rpy = ctrl.step(GaitCommand(state="stand"))
        angles = io.apply(body_ik(feet, body_xyz=xyz, body_rpy=rpy))
        rp = gy = None
        if imu is not None:
            rp, gy = imu.update()
        telem.record(i, angles, rp, gy)
        i += 1
        nxt += tj.DT
        lag = nxt - time.monotonic()
        if lag > 0:
            time.sleep(lag)
        else:
            nxt = time.monotonic()


def _teleop(args, io, telem, imu):
    """Controller-FSM teleop on hardware; SQUARE = ESTOP (all-off)."""
    from barq1.controller import Controller
    from barq1.kinematics import body_ik
    from teleop.drive import teleop_loop
    from teleop.ps4 import PS4

    pad = PS4()
    ctrl = Controller(dt=tj.DT, start_state="stand")
    state = {"next": time.monotonic(), "i": 0}

    def emit(cmd):
        feet, xyz, rpy = ctrl.step(cmd)
        angles = io.apply(body_ik(feet, body_xyz=xyz, body_rpy=rpy))
        rp = gy = None
        if imu is not None:
            rp, gy = imu.update()
        telem.record(state["i"], angles, rp, gy)
        state["i"] += 1

    def step():
        state["next"] += tj.DT
        lag = state["next"] - time.monotonic()
        if lag > 0:
            time.sleep(lag)
        else:
            state["next"] = time.monotonic()

    teleop_loop(pad, emit, step, estop=lambda: _panic(io, telem))


def _panic(io, telem):
    io.all_off()
    telem.close()
    print("\n[ESTOP] all outputs OFF.")
    sys.exit(130)


if __name__ == "__main__":
    sys.exit(main())
