"""
DIAGNOSTIC — static leg-lift test
=================================

From a stable stand, raise ONE leg at a time by a fixed height and HOLD it, so
we can observe (with eyes + IMU) whether:
  * the swing foot actually clears the ground (lift works on the ground), and
  * the robot stays level or tips — and WHICH WAY (reveals the real CoM).

Optionally applies a body CoM lean toward the opposite corner before lifting
(the crawl's core primitive, in isolation) so we can tune the lean per leg.

Everything moves via smooth interpolation through the verified pipeline
(foot targets -> IK -> conventions -> servos). Statically slow and safe.

Run from src/main:
    python -m interface.diag                       # lift each leg, no lean
    python -m interface.diag --shift-y 0.03 --shift-x 0.02   # with CoM lean
    python -m interface.diag --legs FL --lift 0.05 --hold 4  # one leg, longer
"""

import argparse
import time

from config.robot_spec import LEGS, stance_foot, apply_com_offset
from config import gait_params as GP
from kinematics.ik import solve_all
from hardware.conventions import joints_to_servo

# opposite-corner lean direction (x_sign, y_sign) per lifting leg
_OPP = {"FL": (-1, -1), "FR": (-1, +1), "RL": (+1, -1), "RR": (+1, +1)}

DT = 0.02


def _write(pca, feet):
    deltas, _ = solve_all(feet)
    try:
        pca.apply_pose(joints_to_servo(deltas))
    except OSError as e:
        print(f"\n[diag] bus blip (tolerated): {e}")


def _interp(pca, imu, tele, a, b, dur):
    n = max(1, int(dur / DT))
    for i in range(n + 1):
        t = i / n
        feet = {leg: tuple(a[leg][k] + (b[leg][k] - a[leg][k]) * t for k in range(3))
                for leg in a}
        _write(pca, feet)
        state = None
        if imu is not None:
            try:
                state = imu.update()
            except OSError:
                pass
        if tele is not None and state is not None:
            tele.log({"roll": round(state.roll, 2), "pitch": round(state.pitch, 2),
                      "yaw_rate": round(state.yaw_rate, 2)})
        time.sleep(DT)
    return b


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--legs", nargs="+", default=list(LEGS),
                   choices=list(LEGS), help="which legs to test, in order")
    p.add_argument("--lift", type=float, default=0.04, help="foot lift height (m)")
    p.add_argument("--shift-x", type=float, default=0.0, help="CoM fore-aft lean (m)")
    p.add_argument("--shift-y", type=float, default=0.0, help="CoM lateral lean (m)")
    p.add_argument("--hold", type=float, default=3.0, help="hold each lift (s)")
    p.add_argument("--height", default="NORMAL")
    p.add_argument("--no-imu", action="store_true")
    args = p.parse_args()

    z = GP.HEIGHT_MODES.get(args.height, GP.HEIGHT_MODES["NORMAL"])
    from hardware import pca9685
    pca9685.init()
    imu = None
    if not args.no_imu:
        from estimation.imu_state import IMUEstimator
        imu = IMUEstimator()
        print("[diag] calibrating IMU — hold still ...")
        imu.calibrate()
    from telemetry.logger import TelemetryLogger
    tele = TelemetryLogger(run_name="diag")

    def stand(shift=(0.0, 0.0)):
        sx, sy = shift
        base = apply_com_offset({leg: stance_foot(leg, z) for leg in LEGS})
        return {leg: (base[leg][0] - sx, base[leg][1] - sy, base[leg][2]) for leg in LEGS}

    print(f"[diag] lift={args.lift*100:.0f}cm  shift=({args.shift_x*100:.0f},{args.shift_y*100:.0f})cm  "
          f"hold={args.hold}s  legs={args.legs}")
    cur = stand()
    _write(pca9685, cur)
    time.sleep(1.0)

    try:
        for leg in args.legs:
            ox, oy = _OPP[leg]
            lean = (ox * args.shift_x, oy * args.shift_y)
            print(f"[diag] --> {leg}: lean {('none' if not (args.shift_x or args.shift_y) else lean)}, then lift")
            # 1) lean onto the support triangle
            leaned = stand(lean)
            cur = _interp(pca9685, imu, tele, cur, leaned, 0.6)
            time.sleep(0.4)
            # 2) lift this leg's foot
            up = {l: (x, y, z + (args.lift if l == leg else 0.0))
                  for l, (x, y, _z) in leaned.items()}
            # keep leaned x/y, only change z of the lifted leg
            up = dict(leaned)
            lx, ly, lz = leaned[leg]
            up[leg] = (lx, ly, lz + args.lift)
            cur = _interp(pca9685, imu, tele, cur, up, 0.5)
            print(f"[diag]     holding {leg} up {args.hold}s — watch: foot off ground? robot tip?")
            hold_frames = int(args.hold / DT)
            for _ in range(hold_frames):
                _write(pca9685, up)
                if imu is not None:
                    try:
                        s = imu.update()
                        tele.log({"leg": leg, "roll": round(s.roll, 2), "pitch": round(s.pitch, 2)})
                    except OSError:
                        pass
                time.sleep(DT)
            # 3) lower + un-lean back to stand
            cur = _interp(pca9685, imu, tele, cur, stand(), 0.6)
            time.sleep(0.5)
    finally:
        _interp(pca9685, imu, tele, cur, stand(), 0.5)
        tele.close()
        print(f"[diag] done. log: {tele.path}")


if __name__ == "__main__":
    main()
