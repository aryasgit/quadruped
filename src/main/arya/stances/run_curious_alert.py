#!/usr/bin/env python3
"""
arya/stances/run_curious_alert.py
===================================
Diagnostic runner for the CuriousAlert stance.

Run from src/main:
    python -m arya.stances.run_curious_alert               # full sequence
    python -m arya.stances.run_curious_alert --phase rise  # alert rise only
    python -m arya.stances.run_curious_alert --phase scan  # scan only
    python -m arya.stances.run_curious_alert --phase bob   # sniff bob only
    python -m arya.stances.run_curious_alert --phase lean  # curiosity lean only
    python -m arya.stances.run_curious_alert --loop 3      # repeat 3 times
    python -m arya.stances.run_curious_alert --lean-side right
    python -m arya.stances.run_curious_alert --scan-amp 0.015 --phase scan
"""

import argparse
import sys
import os
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from hardware.pca9685 import init_pca
from arya.stances.curious_alert import (
    CuriousAlert,
    stand,
    _neutral_feet,
    _send,
    _transition,
)

PHASES = ("rise", "scan", "bob", "lean", "all")


def _go_alert(stance: CuriousAlert) -> dict:
    """Transition to alert base and return the alert feet dict."""
    alert = stance._alert_base()
    _transition(_neutral_feet(), alert, duration=stance.ALERT_DURATION)
    time.sleep(0.20)
    return alert


def _from_alert(alert: dict, stance: CuriousAlert) -> None:
    """Return from alert base to neutral."""
    _transition(alert, _neutral_feet(), duration=stance.RECOVER_DURATION)


def run_phase(stance: CuriousAlert, phase: str) -> None:
    if phase == "rise":
        print("[PHASE] Alert Rise")
        alert = stance._phase_alert_rise()
        time.sleep(1.0)
        _from_alert(alert, stance)

    elif phase == "scan":
        print("[PHASE] Scan")
        alert = _go_alert(stance)
        stance._phase_scan(alert)
        _from_alert(alert, stance)

    elif phase == "bob":
        print("[PHASE] Sniff Bob")
        alert = _go_alert(stance)
        stance._phase_sniff_bob(alert)
        _from_alert(alert, stance)

    elif phase == "lean":
        print("[PHASE] Curiosity Lean")
        alert = _go_alert(stance)
        stance._phase_lean(alert)
        _from_alert(alert, stance)

    elif phase == "all":
        print("[PHASE] Full CuriousAlert sequence")
        stance.execute()

    else:
        print(f"[ERROR] Unknown phase '{phase}'. Choose from: {PHASES}")
        sys.exit(1)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Diagnostic runner for CuriousAlert stance",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Phases:
  rise    Alert rise + front splay only, then return to neutral
  scan    Sinusoidal head-scan only (alert → scan → neutral)
  bob     Sniff bob only            (alert → bob  → neutral)
  lean    Curiosity lean only       (alert → lean → neutral)
  all     Full 5-phase sequence (default)

Tuning examples:
  --scan-amp 0.015      reduce scan Y sweep (safer on slippery floor)
  --bob-n 8             more sniff bobs
  --lean-side right     tilt to the right instead
  --loop 3              run the selected phase 3 times
        """,
    )
    ap.add_argument(
        "--phase",
        default="all",
        choices=PHASES,
        help="Run only a specific phase (default: all)",
    )
    ap.add_argument(
        "--loop",
        type=int,
        default=1,
        metavar="N",
        help="Repeat the selected phase N times (default: 1)",
    )
    ap.add_argument(
        "--lean-side",
        default="left",
        choices=("left", "right"),
        dest="lean_side",
        help="Side for the curiosity lean (default: left)",
    )
    ap.add_argument(
        "--scan-amp",
        type=float,
        default=None,
        metavar="M",
        dest="scan_amp",
        help="Override scan Y amplitude in meters (default: 0.020)",
    )
    ap.add_argument(
        "--bob-n",
        type=int,
        default=None,
        metavar="N",
        dest="bob_n",
        help="Override number of sniff bob cycles (default: 5)",
    )
    ap.add_argument(
        "--scan-cycles",
        type=int,
        default=None,
        metavar="N",
        dest="scan_cycles",
        help="Override number of scan sweeps (default: 2)",
    )
    ap.add_argument(
        "--verbose",
        action="store_true",
        help="Print timing info for each iteration",
    )
    args = ap.parse_args()

    # ── hardware init ─────────────────────────────────────────────
    print("Initializing PCA9685 ... ", end="", flush=True)
    init_pca()
    print("OK")

    # ── configure stance instance ─────────────────────────────────
    stance = CuriousAlert()
    stance.LEAN_SIDE = args.lean_side

    overrides = []
    if args.scan_amp is not None:
        stance.SCAN_Y_AMP = args.scan_amp
        overrides.append(f"scan_amp={args.scan_amp:.4f} m")
    if args.bob_n is not None:
        stance.BOB_N_CYCLES = args.bob_n
        overrides.append(f"bob_n={args.bob_n}")
    if args.scan_cycles is not None:
        stance.SCAN_N_CYCLES = args.scan_cycles
        overrides.append(f"scan_cycles={args.scan_cycles}")
    if overrides:
        print(f"[CONFIG] overrides: {', '.join(overrides)}")

    estimated = {
        "rise":  stance.ALERT_DURATION + 1.0 + stance.RECOVER_DURATION,
        "scan":  stance.ALERT_DURATION + stance.SCAN_N_CYCLES * stance.SCAN_PERIOD + stance.RECOVER_DURATION + 0.5,
        "bob":   stance.ALERT_DURATION + stance.BOB_N_CYCLES * stance.BOB_PERIOD + stance.RECOVER_DURATION + 0.5,
        "lean":  stance.ALERT_DURATION + 0.9 + stance.LEAN_HOLD + 0.7 + stance.RECOVER_DURATION,
        "all":   (stance.ALERT_DURATION + 0.2
                  + stance.SCAN_N_CYCLES * stance.SCAN_PERIOD + 0.45
                  + stance.BOB_N_CYCLES * stance.BOB_PERIOD + 0.4
                  + 0.9 + stance.LEAN_HOLD + 0.7 + 0.1
                  + stance.RECOVER_DURATION),
    }
    print(f"[INFO] phase='{args.phase}'  loops={args.loop}  "
          f"~{estimated[args.phase]:.1f}s per iteration")

    # ── go to stand then run ──────────────────────────────────────
    print("Going to stand ...")
    stand()
    time.sleep(0.5)

    try:
        for iteration in range(args.loop):
            if args.loop > 1:
                print(f"\n--- Iteration {iteration + 1}/{args.loop} ---")

            t0 = time.monotonic()
            run_phase(stance, args.phase)
            elapsed = time.monotonic() - t0

            if args.verbose or args.loop > 1:
                print(f"[TIMING] completed in {elapsed:.2f}s")

            if iteration < args.loop - 1:
                print("Returning to stand between loops ...")
                stand()
                time.sleep(0.8)

    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Returning to neutral ...")
        _send(_neutral_feet())
        sys.exit(0)

    print("\nReturning to stand ...")
    stand()
    print("Done.")


if __name__ == "__main__":
    main()
