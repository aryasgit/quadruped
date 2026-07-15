"""
PER-LEG TILT CALIBRATION (IMU closed loop, coxa-free)
=====================================================

For each leg, find the body tilt (created by asymmetric leg heights — thigh/wrist
only, no coxa) that lets that leg lift with NO further body tilt, i.e. the weight
is already shifted off it. Method per leg:

    apply candidate (pitch, roll) body tilt  ->  lift the leg  ->  measure the
    ADDITIONAL tilt the lift causes  ->  add counter-tilt  ->  repeat until the
    lift induces ~0 tilt.

Output: the 4 per-leg (pitch, roll) tilts to apply BEFORE lifting each leg during
the crawl. Robot must be FREE-STANDING on a level-ish floor.

Run from src/main:
    python -m interface.calibrate_leg_tilt
    python -m interface.calibrate_leg_tilt --legs RL --lift 0.04
"""

import argparse
import time

from config.robot_spec import LEGS, stance_foot, apply_body_tilt
from config import gait_params as GP
from kinematics.ik import solve_all
from hardware.conventions import joints_to_servo

DT = 0.02


def _write(pca, feet):
    try:
        pca.apply_pose(joints_to_servo(solve_all(feet)[0]))
    except OSError as e:
        print(f"\n[tilt] bus blip (tolerated): {e}")


def _interp(pca, a, b, dur):
    n = max(1, int(dur / DT))
    for i in range(n + 1):
        t = i / n
        _write(pca, {l: tuple(a[l][k] + (b[l][k] - a[l][k]) * t for k in range(3)) for l in a})
        time.sleep(DT)
    return b


def _avg_tilt(imu, secs, skip=0.0):
    """Average roll/pitch over `secs`, discarding the first `skip` s (settling)."""
    t0 = time.monotonic()
    r = p = 0.0; n = 0
    while time.monotonic() - t0 < secs:
        try:
            s = imu.update()
            if time.monotonic() - t0 >= skip:
                r += s.roll; p += s.pitch; n += 1
        except OSError:
            pass
        time.sleep(DT)
    n = max(n, 1)
    return r / n, p / n


def _measure_once(pca, imu, z, leg, pitch, roll, lift, hold):
    base = _base(z, pitch, roll)
    _write(pca, base); time.sleep(0.6)
    br, bp = _avg_tilt(imu, 0.6, skip=0.2)
    up = dict(base)
    x, y, zz = base[leg]
    up[leg] = (x, y, zz + lift)
    _interp(pca, base, up, 0.35)
    r, p = _avg_tilt(imu, hold, skip=0.4)   # skip the lift transient
    _interp(pca, up, base, 0.35)
    return r - br, p - bp


def _base(z, pitch, roll):
    feet = {l: stance_foot(l, z) for l in LEGS}
    return apply_body_tilt(feet, pitch, roll)


def measure(pca, imu, z, leg, pitch, roll, lift, hold, reps=2):
    """Median-of-reps induced (droll, dpitch) — robust to per-lift noise."""
    rs, ps = [], []
    for _ in range(reps):
        dr, dp = _measure_once(pca, imu, z, leg, pitch, roll, lift, hold)
        rs.append(dr); ps.append(dp)
    rs.sort(); ps.sort()
    m = len(rs) // 2
    return rs[m], ps[m]


def _parabolic_min(pts):
    """Given [(x, err)], return the interpolated x minimizing err (fallback argmin)."""
    best = min(pts, key=lambda t: t[1])
    i = pts.index(best)
    if 0 < i < len(pts) - 1:
        x0, y0 = pts[i - 1]; x1, y1 = pts[i]; x2, y2 = pts[i + 1]
        denom = (y0 - 2 * y1 + y2)
        if abs(denom) > 1e-9:
            xv = x1 + 0.5 * (y0 - y2) / denom * (x1 - x0)
            lo, hi = min(x0, x2), max(x0, x2)
            return max(lo, min(hi, xv))
    return best[0]


def _sweep(pca, imu, z, leg, axis, fixed, vals, args):
    pts = []
    for v in vals:
        pitch = v if axis == "pitch" else fixed
        roll = v if axis == "roll" else fixed
        dr, dp = _measure_once(pca, imu, z, leg, pitch, roll, args.lift, args.hold)
        err = abs(dp) if axis == "pitch" else abs(dr)
        pts.append((v, err))
        print(f"    {axis}={v*100:+4.1f}cm -> droll={dr:+5.2f} dpitch={dp:+5.2f}  |{axis}-err|={err:4.2f}")
    star = _parabolic_min(pts)
    print(f"    -> best {axis} = {star*100:+.2f}cm")
    return star


def calibrate_leg(pca, imu, z, leg, args):
    print(f"\n=== leg {leg} ===")
    vals = [-0.030, -0.015, 0.0, 0.015, 0.030]
    print("  [roll sweep @ pitch=0]")
    rc = _sweep(pca, imu, z, leg, "roll", 0.0, vals, args)
    print(f"  [pitch sweep @ roll={rc*100:+.1f}cm]")
    pc = _sweep(pca, imu, z, leg, "pitch", rc, vals, args)
    # verify at the chosen point
    dr, dp = measure(pca, imu, z, leg, pc, rc, args.lift, args.hold)
    print(f"  {leg} RESULT pitch={pc*100:+.2f}cm roll={rc*100:+.2f}cm  ->  residual droll={dr:+.2f} dpitch={dp:+.2f}")
    return pc, rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--legs", nargs="+", default=list(LEGS), choices=list(LEGS))
    ap.add_argument("--lift", type=float, default=0.04)
    ap.add_argument("--hold", type=float, default=1.0)
    ap.add_argument("--iters", type=int, default=8)
    ap.add_argument("--gain", type=float, default=0.0022, help="m per deg")
    ap.add_argument("--tol", type=float, default=2.0, help="deg")
    ap.add_argument("--cap", type=float, default=0.030, help="max |tilt| m")
    ap.add_argument("--height", default="NORMAL")
    args = ap.parse_args()

    z = GP.HEIGHT_MODES.get(args.height, GP.HEIGHT_MODES["NORMAL"])
    from hardware import pca9685
    pca9685.init()
    from estimation.imu_state import IMUEstimator
    imu = IMUEstimator()
    print("[tilt] robot FREE-STANDING on a level floor. Calibrating IMU — hold still ...")
    imu.calibrate()

    _write(pca9685, _base(z, 0, 0)); time.sleep(0.6)
    results = {}
    for leg in args.legs:
        results[leg] = calibrate_leg(pca9685, imu, z, leg, args)
        _interp(pca9685, _base(z, *results[leg][::-1]) if False else _base(z, 0, 0), _base(z, 0, 0), 0.4)

    print("\n" + "=" * 56)
    print("[tilt] PER-LEG BODY TILT (pitch, roll) in meters — apply before lifting each leg:")
    print("LEG_TILT = {")
    for leg in args.legs:
        pc, rc = results[leg]
        print(f'    "{leg}": ({pc:+.4f}, {rc:+.4f}),   # pitch {pc*100:+.1f}cm, roll {rc*100:+.1f}cm')
    print("}")
    print("=" * 56)


if __name__ == "__main__":
    main()
