"""
L6 — HEADLESS CLI COMMAND SOURCE
================================

Drive the robot with scripted commands (no phone needed) so gait behaviour is
repeatable and telemetry-logged. Essential for isolating the "random/delayed"
walk bug: same command every run.

Examples (run from src/main):
    python -m interface.cli --dry --fwd 1 --dur 5          # offline, no hardware
    python -m interface.cli --stand --dur 3                # hold stand
    python -m interface.cli --fwd 1 --dur 5 --run fwd5     # real robot, logged
    python -m interface.cli --turn 1 --dur 4 --run turnL   # turn left

Safety: defaults to WALK only when a motion axis is given; otherwise stands.
"""

import argparse

from control.command import Command
from control.loop import WalkController
from telemetry.logger import TelemetryLogger


def main():
    p = argparse.ArgumentParser(description="Headless quadruped command source")
    p.add_argument("--fwd", type=float, default=0.0, help="-1..1 (+forward)")
    p.add_argument("--strafe", type=float, default=0.0, help="-1..1 (+right)")
    p.add_argument("--turn", type=float, default=0.0, help="-1..1 (+left/CCW)")
    p.add_argument("--height", default="NORMAL", choices=["HIGH", "NORMAL", "LOW", "CROUCH"])
    p.add_argument("--gait", default="trot", choices=["trot", "crawl"])
    p.add_argument("--trim-roll", type=float, default=None, help="constant body roll trim (m); default from config")
    p.add_argument("--trim-pitch", type=float, default=None, help="constant body pitch trim (m); default from config")
    p.add_argument("--strafe-trim", type=float, default=None, help="constant lateral bias to cancel crab (+right)")
    p.add_argument("--dur", type=float, default=5.0, help="seconds")
    p.add_argument("--stand", action="store_true", help="hold stand pose (no gait)")
    p.add_argument("--dry", action="store_true", help="no hardware (offline test)")
    p.add_argument("--no-imu", action="store_true", help="disable IMU read")
    p.add_argument("--no-calib", action="store_true", help="skip IMU calibration")
    p.add_argument("--run", default="cli", help="telemetry run name")
    args = p.parse_args()

    mode = "IDLE" if args.stand else "WALK"
    cmd = Command(
        fwd=0.0 if args.stand else args.fwd,
        strafe=0.0 if args.stand else args.strafe,
        turn=0.0 if args.stand else args.turn,
        height=args.height,
        mode=mode,
    )

    tele = TelemetryLogger(run_name=args.run)
    ctrl = WalkController(
        hardware=not args.dry,
        telemetry=tele,
        imu_enabled=not args.no_imu,
        gait=args.gait,
        trim_roll=args.trim_roll,
        trim_pitch=args.trim_pitch,
        strafe_trim=args.strafe_trim,
    )
    print(f"[cli] mode={mode} cmd=(fwd={cmd.fwd}, strafe={cmd.strafe}, turn={cmd.turn}) "
          f"height={args.height} dur={args.dur}s hardware={not args.dry}")
    print(f"[cli] telemetry -> {tele.path}")
    try:
        ctrl.start(calibrate=(not args.dry and not args.no_calib and not args.no_imu))
        ctrl.run(lambda: cmd, duration=args.dur)
    finally:
        # return to a safe stand, then close
        try:
            ctrl.step(Command(mode="IDLE", height=args.height), 0.02)
        except Exception:
            pass
        tele.close()
        print(f"[cli] done. log: {tele.path}")


if __name__ == "__main__":
    main()
