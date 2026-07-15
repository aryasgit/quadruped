"""
IMU-CLOSED-LOOP CoM CALIBRATION
===============================

Finds the body CoM offset by measuring, with the IMU, how the robot tilts when
each leg is lifted, then iteratively shifting the neutral body position until
lifting ANY leg keeps the body level (fore-aft and lateral balanced).

Method (coordinate descent on IMU feedback):
  For a candidate foot offset (ox, oy):
    * baseline tilt with all 4 feet down,
    * lift each leg ~3.5 cm, hold, record tilt DELTA vs baseline,
  then:
    fore_aft_err = mean_pitch(rear legs)  + mean_pitch(front legs)   (0 = balanced)
    lateral_err  = mean_roll(left legs)   + mean_roll(right legs)     (0 = balanced)
    ox -= kx * fore_aft_err     # feet back  -> body fwd  -> CoM fwd
    oy += ky * lateral_err      # feet left  -> body right -> CoM right
  Adaptive step: if total tilt cost rises, undo and halve the gains.

ROBOT MUST BE FREE-STANDING ON A LEVEL-ISH FLOOR (not held), so lifting a leg
actually shifts weight and tilts the body.

Run from src/main:
    python -m interface.calibrate_com
    python -m interface.calibrate_com --iters 8 --lift 0.035
"""

import argparse
import time

from config.robot_spec import LEGS, stance_foot, apply_com_offset
from config import gait_params as GP
from kinematics.ik import solve_all
from hardware.conventions import joints_to_servo

DT = 0.02
FRONT = ("FL", "FR"); REAR = ("RL", "RR")
LEFT = ("FL", "RL");  RIGHT = ("FR", "RR")


def _write(pca, feet):
    try:
        pca.apply_pose(joints_to_servo(solve_all(feet)[0]))
    except OSError as e:
        print(f"\n[cal] bus blip (tolerated): {e}")


def _interp(pca, a, b, dur):
    n = max(1, int(dur / DT))
    for i in range(n + 1):
        t = i / n
        _write(pca, {l: tuple(a[l][k] + (b[l][k] - a[l][k]) * t for k in range(3)) for l in a})
        time.sleep(DT)
    return b


def _avg_tilt(imu, secs):
    rs, ps, n = 0.0, 0.0, 0
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        try:
            s = imu.update(); rs += s.roll; ps += s.pitch; n += 1
        except OSError:
            pass
        time.sleep(DT)
    n = max(n, 1)
    return rs / n, ps / n


def _stand(z, ox, oy):
    return apply_com_offset({l: stance_foot(l, z) for l in LEGS}, ox, oy)


def measure(pca, imu, z, ox, oy, lift, hold):
    """Return {leg: (droll, dpitch)} tilt deltas vs baseline for this offset."""
    base = _stand(z, ox, oy)
    _interp(pca, base, base, 0.0)
    _write(pca, base); time.sleep(0.4)
    broll, bpitch = _avg_tilt(imu, 0.3)
    out = {}
    cur = base
    for leg in LEGS:
        up = dict(base)
        x, y, zz = base[leg]
        up[leg] = (x, y, zz + lift)
        cur = _interp(pca, cur, up, 0.35)
        r, p = _avg_tilt(imu, hold)
        out[leg] = (r - broll, p - bpitch)
        cur = _interp(pca, cur, base, 0.35)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=7)
    ap.add_argument("--lift", type=float, default=0.035)
    ap.add_argument("--hold", type=float, default=0.5)
    ap.add_argument("--height", default="NORMAL")
    ap.add_argument("--kx", type=float, default=0.0015)
    ap.add_argument("--ky", type=float, default=0.0015)
    ap.add_argument("--tol", type=float, default=1.5, help="deg, convergence")
    ap.add_argument("--cap", type=float, default=0.045, help="max |offset| m")
    ap.add_argument("--fore-aft-only", action="store_true", default=True,
                    help="only shift fore-aft (thigh/wrist); never move coxas (lateral)")
    ap.add_argument("--allow-lateral", dest="fore_aft_only", action="store_false",
                    help="also shift lateral (moves coxas — widens stance)")
    args = ap.parse_args()

    z = GP.HEIGHT_MODES.get(args.height, GP.HEIGHT_MODES["NORMAL"])
    from hardware import pca9685
    pca9685.init()
    from estimation.imu_state import IMUEstimator
    imu = IMUEstimator()
    print("[cal] robot must be FREE-STANDING on a level floor. Calibrating IMU — hold still ...")
    imu.calibrate()

    ox, oy = 0.0, 0.0
    kx, ky = args.kx, args.ky
    if args.fore_aft_only:
        ky = 0.0   # never move coxas; lateral stays 0
        print("[cal] FORE-AFT ONLY: coxas stay at stand; shifting via thigh/wrist.")
    prev_cost = None
    prev = (0.0, 0.0)
    print(f"\n[cal] starting.  lift={args.lift*100:.1f}cm hold={args.hold}s tol={args.tol}deg\n")

    for it in range(args.iters):
        tilt = measure(pca9685, imu, z, ox, oy, args.lift, args.hold)
        fa = (tilt["RL"][1] + tilt["RR"][1]) / 2 + (tilt["FL"][1] + tilt["FR"][1]) / 2
        lat = (tilt["FL"][0] + tilt["RL"][0]) / 2 + (tilt["FR"][0] + tilt["RR"][0]) / 2
        cost = sum((r * r + p * p) ** 0.5 for r, p in tilt.values())

        print(f"--- iter {it}  offset=({ox*100:+.2f},{oy*100:+.2f})cm ---")
        for leg in LEGS:
            print(f"    {leg}: droll={tilt[leg][0]:+6.2f}  dpitch={tilt[leg][1]:+6.2f}")
        print(f"    fore_aft_err={fa:+.2f}  lateral_err={lat:+.2f}  cost={cost:.2f}")

        converged = abs(fa) < args.tol and (args.fore_aft_only or abs(lat) < args.tol)
        if converged:
            print("\n[cal] CONVERGED — fore-aft balanced within tolerance.")
            if args.fore_aft_only:
                print(f"    (lateral_err={lat:+.2f} left as-is — needs coxa to fix)")
            break

        # adaptive: if worse than last, revert and shrink gains
        if prev_cost is not None and cost > prev_cost * 1.15:
            ox, oy = prev
            kx *= 0.5; ky *= 0.5
            print(f"    (cost rose -> revert, shrink gains kx={kx:.4f} ky={ky:.4f})")
            continue

        prev, prev_cost = (ox, oy), cost
        ox = max(-args.cap, min(args.cap, ox - kx * fa))
        oy = max(-args.cap, min(args.cap, oy + ky * lat))

    # settle at final offset
    _interp(pca9685, _stand(z, ox, oy), _stand(z, ox, oy), 0.0)
    _write(pca9685, _stand(z, ox, oy)); time.sleep(0.3)
    print("\n" + "=" * 52)
    print(f"[cal] RESULT  COM_OFFSET_X = {ox:+.4f}   COM_OFFSET_Y = {oy:+.4f}   (meters)")
    print(f"[cal] i.e. ({ox*100:+.2f} cm, {oy*100:+.2f} cm) foot offset")
    print("[cal] Set these in config/robot_spec.py to bake in.")
    print("=" * 52)


if __name__ == "__main__":
    main()
