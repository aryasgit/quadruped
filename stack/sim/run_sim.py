#!/usr/bin/env python3
"""
BARQ v1 simulation runner (D-008).

    ~/barq_v1/venv/bin/python stack/sim/run_sim.py                 # all scenarios
    ~/barq_v1/venv/bin/python stack/sim/run_sim.py --scenario pose_sweep
    DISPLAY=:0 ... run_sim.py --gui --realtime --scenario settle   # watch via VNC

Headless by default (DIRECT + software renderer). Writes per-scenario CSV
time series + PPM snapshots to ~/barq_v1/artifacts/sim-<stamp>/ (outside
the repo, per docs policy).
"""

import argparse
import datetime
import sys
from pathlib import Path

STACK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STACK_ROOT))

from sim.scenarios import SCENARIOS
from sim.world import SimRobot, save_snapshot

ARTIFACTS = Path.home() / "barq_v1" / "artifacts"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="all", choices=["all", *SCENARIOS])
    ap.add_argument("--gui", action="store_true", help="PyBullet GUI (use via VNC)")
    ap.add_argument("--realtime", action="store_true")
    ap.add_argument("--no-artifacts", action="store_true")
    args = ap.parse_args()

    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = ARTIFACTS / f"sim-{stamp}"
    if not args.no_artifacts:
        outdir.mkdir(parents=True, exist_ok=True)

    results = {}
    for name in names:
        robot = SimRobot(gui=args.gui)
        try:
            metrics, rec = SCENARIOS[name](robot)
            results[name] = metrics
            if not args.no_artifacts:
                rec.save_csv(outdir / f"{name}.csv")
                save_snapshot(str(outdir / f"{name}.ppm"))
        finally:
            import pybullet as p
            p.disconnect(robot.client)

        print(f"\n=== {name} ===")
        for k, v in metrics.items():
            print(f"  {k:28} {v:.2f}" if isinstance(v, float) else f"  {k:28} {v}")

    if not args.no_artifacts:
        print(f"\nartifacts -> {outdir}")
    return results


if __name__ == "__main__":
    main()
