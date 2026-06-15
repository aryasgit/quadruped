#!/usr/bin/env python3
"""
BARQ v1 simulation runner (D-008).

    ~/barq_v1/venv/bin/python stack/sim/run_sim.py                 # all, headless, fast
    ~/barq_v1/venv/bin/python stack/sim/run_sim.py --scenario pose_sweep
    DISPLAY=:0 ... run_sim.py --gui --loop          # ONE window, cycles forever
    DISPLAY=:0 ... run_sim.py --gui --scenario settle   # one pass, holds 4 s

GUI mode paces at realtime automatically (--fast to disable) and keeps the
window briefly after each scenario; --loop keeps a single window open and
cycles the scenarios until you close the window or Ctrl-C.

Headless (default) runs at full speed and writes per-scenario CSV time
series + PPM snapshots to ~/barq_v1/artifacts/sim-<stamp>/ (outside the
repo, per docs policy).
"""

import argparse
import datetime
import sys
from pathlib import Path

STACK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STACK_ROOT))

import pybullet as p

from sim.scenarios import SCENARIOS
from sim.world import SimRobot, save_snapshot


ARTIFACTS = Path.home() / "barq_v1" / "artifacts"


def _print_metrics(name, metrics):
    print(f"\n=== {name} ===")
    for k, v in metrics.items():
        print(f"  {k:28} {v:.2f}" if isinstance(v, float) else f"  {k:28} {v}")


def run_once(names, args):
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = ARTIFACTS / f"sim-{stamp}"
    if not args.no_artifacts:
        outdir.mkdir(parents=True, exist_ok=True)

    for name in names:
        robot = SimRobot(gui=args.gui, realtime=False if args.fast else None,
                         fidelity=not args.ideal)
        try:
            metrics, rec = SCENARIOS[name](robot)
            _print_metrics(name, metrics)
            if not args.no_artifacts:
                rec.save_csv(outdir / f"{name}.csv")
                save_snapshot(str(outdir / f"{name}.ppm"))
            if args.gui:
                robot.step(4.0)  # hold the final pose so the eye can catch up
        finally:
            p.disconnect(robot.client)

    if not args.no_artifacts:
        print(f"\nartifacts -> {outdir}")


def run_loop(names, args):
    """One persistent window; cycle scenarios until the window is closed."""
    robot = SimRobot(gui=args.gui, realtime=False if args.fast else None,
                     fidelity=not args.ideal)
    lap = 0
    try:
        while True:
            lap += 1
            print(f"\n--- demo lap {lap} (close the window or Ctrl-C to stop) ---")
            for name in names:
                robot.reset()
                metrics, _ = SCENARIOS[name](robot)
                _print_metrics(name, metrics)
                robot.step(2.0)  # hold between scenarios
    except KeyboardInterrupt:
        print("\nstopped.")
    except (p.error, TypeError):
        # closing the GUI window makes API calls raise p.error or return
        # None mid-scenario (-> TypeError downstream); both mean "user's done"
        print("\nwindow closed — bye.")
    finally:
        try:
            p.disconnect(robot.client)
        except p.error:
            pass


def run_teleop(args):
    """PS4 controller drives the simulated robot (pose + walk bursts)."""
    from barq1.kinematics import body_ik, to_urdf_joints
    from sim.scenarios import _spawn_standing
    from teleop.drive import teleop_loop
    from teleop.ps4 import PS4, NoController

    try:
        pad = PS4()
    except NoController as e:
        print(f"[teleop] {e}")
        return

    robot = SimRobot(gui=True, fidelity=not args.ideal)
    _spawn_standing(robot)

    def command(feet, xyz, rpy):
        robot.command(to_urdf_joints(body_ik(feet, body_xyz=xyz, body_rpy=rpy)))

    def reset():
        print("[teleop] reset")
        robot.reset()
        _spawn_standing(robot)

    try:
        teleop_loop(pad, command, lambda: robot.step(1 / 50), estop=reset,
                    walk_cycles=args.cycles)
    except p.error:
        print("window closed — bye.")
    finally:
        try:
            p.disconnect(robot.client)
        except p.error:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="all", choices=["all", *SCENARIOS])
    ap.add_argument("--gui", action="store_true", help="PyBullet GUI (use via VNC)")
    ap.add_argument("--loop", action="store_true",
                    help="single window, cycle scenarios until closed (implies --gui)")
    ap.add_argument("--teleop", action="store_true",
                    help="PS4 controller drives the sim (implies --gui)")
    ap.add_argument("--cycles", type=int, default=2,
                    help="walk cycles (teleop TRIANGLE / walk scenario)")
    ap.add_argument("--fast", action="store_true",
                    help="disable realtime pacing in GUI mode")
    ap.add_argument("--realtime", action="store_true",
                    help=argparse.SUPPRESS)  # legacy no-op: GUI is realtime by default
    ap.add_argument("--ideal", action="store_true",
                    help="disable the hardware actuation boundary (no tick "
                         "quantization / transport delay) — ideal float path")
    ap.add_argument("--no-artifacts", action="store_true")
    args = ap.parse_args()

    if args.loop or args.teleop:
        args.gui = True
        args.no_artifacts = True

    if args.teleop:
        run_teleop(args)
        return
    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    if args.loop:
        run_loop(names, args)
    else:
        run_once(names, args)


if __name__ == "__main__":
    main()
