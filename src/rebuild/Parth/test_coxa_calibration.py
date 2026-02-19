"""
Parth/test_coxa_calibration.py — COXA CALIBRATION VERIFIER
===========================================================

Tests whether the coxa "stand" values in absolute_truths.py actually
produce perpendicular (straight-out) leg positions.

Four tests:
  1. VISUAL  — Sends each leg to its coxa stand angle one at a time.
               You physically check: is the leg pointing straight out?
  2. SWEEP   — Sweeps each coxa ±15° around stand so you can see
               where "straight out" actually is.
  3. IK ZERO — Sends IK delta = 0 through the full pipeline and
               prints what physical angle each coxa gets.
  4. RAW     — Just prints all calibration values for inspection.

Run:
    cd src/rebuild
    python -m Parth.test_coxa_calibration
    python -m Parth.test_coxa_calibration --test 2   # sweep only
"""

import time

from hardware.pca9685 import set_servo_angle, init_pca
from hardware.absolute_truths import (
    COXA, THIGHS, WRISTS,
    COXA_STAND, COXA_MECH,
    THIGH_STAND, WRIST_STAND,
)
from layer3.leg_ik import solve_all_legs
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all


LEGS = ["FL", "FR", "RL", "RR"]

COXA_CH  = {leg: COXA[leg] for leg in LEGS}
THIGH_CH = {"FL": THIGHS["TFL"], "FR": THIGHS["TFR"],
            "RL": THIGHS["TRL"], "RR": THIGHS["TRR"]}
WRIST_CH = {"FL": WRISTS["WFL"], "FR": WRISTS["WFR"],
            "RL": WRISTS["WRL"], "RR": WRISTS["WRR"]}

# Extract perp values from COXA_MECH dicts
COXA_PERP = {leg: COXA_MECH[leg]["perp"] for leg in LEGS}


def send_stand_all():
    """Send every servo to its stand position."""
    for leg in LEGS:
        set_servo_angle(COXA_CH[leg],  COXA_STAND[leg])
        set_servo_angle(THIGH_CH[leg], THIGH_STAND["T" + leg])
        set_servo_angle(WRIST_CH[leg], WRIST_STAND["W" + leg])


# ──────────────────────────────────────────────
#  TEST 1: VISUAL
# ──────────────────────────────────────────────

def test_visual():
    """Send each coxa to COXA_STAND one at a time. You confirm visually."""
    print("\n" + "=" * 60)
    print("TEST 1: VISUAL COXA CHECK")
    print("=" * 60)
    print("Each leg will go to its COXA_STAND angle.")
    print("Check: does the leg point straight out from the body?\n")

    send_stand_all()
    time.sleep(1)

    for leg in LEGS:
        stand = COXA_STAND[leg]
        perp  = COXA_PERP[leg]
        print(f"\n  {leg}:  COXA_STAND = {stand}°,  COXA_PERP = {perp}°")
        print(f"        Channel = {COXA_CH[leg]}")

        if stand != perp:
            print(f"        ⚠️  STAND ≠ PERP  (diff = {stand - perp}°)")
        else:
            print(f"        ✓  STAND == PERP")

        set_servo_angle(COXA_CH[leg], stand)
        input(f"        → Is {leg} pointing straight out? [Enter to continue] ")

    print("\n  Done. All legs at stand.\n")


# ──────────────────────────────────────────────
#  TEST 2: SWEEP
# ──────────────────────────────────────────────

def test_sweep():
    """Sweep each coxa ±15° around stand. You find the true perpendicular."""
    print("\n" + "=" * 60)
    print("TEST 2: COXA SWEEP (find true perpendicular)")
    print("=" * 60)
    print("Each coxa will sweep from STAND-15° to STAND+15°.")
    print("Watch the leg and note which angle looks straight.\n")

    send_stand_all()
    time.sleep(1)

    for leg in LEGS:
        stand = COXA_STAND[leg]
        lo = max(stand - 15, 0)
        hi = min(stand + 15, 180)
        print(f"\n  {leg}:  COXA_STAND = {stand}°")
        print(f"  Sweeping {lo}° → {hi}° in 1° steps")
        print(f"  Press Ctrl+C to stop at current angle\n")

        try:
            for angle in range(lo, hi + 1):
                set_servo_angle(COXA_CH[leg], angle)
                input(f"    {leg} coxa = {angle}°  "
                      f"(delta from stand = {angle - stand:+d}°)   "
                      f"[Enter for next]")

            ans = input(
                f"  What angle looked straight for {leg}? "
                f"(Enter to keep {stand}): "
            ).strip()

            if ans:
                correct = int(ans)
                diff = correct - stand
                if diff != 0:
                    print(f"  ⚠️  {leg} COXA_STAND should be {correct}° "
                          f"(current {stand}°, off by {diff}°)")
                    print(f"       → Update COXA_STAND[\"{leg}\"] in "
                          f"hardware/absolute_truths.py")
                else:
                    print(f"  ✓  {leg} COXA_STAND is correct")
            else:
                print(f"  ✓  {leg} kept at {stand}°")

            # reset to stand
            set_servo_angle(COXA_CH[leg], stand)

        except KeyboardInterrupt:
            print(f"\n  Stopped. Resetting {leg} to stand.")
            set_servo_angle(COXA_CH[leg], stand)

    send_stand_all()
    print("\n  Done. All legs back to stand.\n")


# ──────────────────────────────────────────────
#  TEST 3: IK ZERO PIPELINE
# ──────────────────────────────────────────────

def test_ik_zero():
    """Send IK delta=0 through the full pipeline and check symmetry."""
    print("\n" + "=" * 60)
    print("TEST 3: IK ZERO-DELTA PIPELINE CHECK")
    print("=" * 60)
    print("Sending neutral stand pose through:")
    print("  solve_all_legs → apply_joint_conventions → normalize_all\n")

    stance_y = 0.080
    stance_z = -0.17
    feet = {
        "FL": (0.0,  stance_y, stance_z),
        "FR": (0.0, -stance_y, stance_z),
        "RL": (0.0,  stance_y, stance_z),
        "RR": (0.0, -stance_y, stance_z),
    }

    deltas = solve_all_legs(feet)
    print("  IK deltas (degrees):")
    for joint, val in sorted(deltas.items()):
        print(f"    {joint:12s} = {val:+7.2f}°")

    conv = apply_joint_conventions(deltas)
    print("\n  After joint_conventions:")
    for joint, val in sorted(conv.items()):
        print(f"    {joint:12s} = {val:+7.2f}°")

    phys = normalize_all(conv)
    print("\n  Final physical angles (sent to servo):")
    for joint, val in sorted(phys.items()):
        print(f"    {joint:12s} = {val:6.2f}°")

    # symmetry check
    print("\n  Symmetry check (left vs right — should be < 3°):")
    pairs = [
        ("FL_COXA",  "FR_COXA"),
        ("RL_COXA",  "RR_COXA"),
        ("FL_THIGH", "FR_THIGH"),
        ("RL_THIGH", "RR_THIGH"),
        ("FL_WRIST", "FR_WRIST"),
        ("RL_WRIST", "RR_WRIST"),
    ]
    any_bad = False
    for left, right in pairs:
        diff = abs(phys[left] - phys[right])
        marker = "✓" if diff < 3.0 else "⚠️"
        if diff >= 3.0:
            any_bad = True
        print(f"    |{left} - {right}| = {diff:.2f}°  {marker}")

    if any_bad:
        print("\n  ⚠️  ASYMMETRY DETECTED")
        print("     Check COXA_STAND values in hardware/absolute_truths.py")
        print("     Run --test 2 (sweep) to find the correct values")
    else:
        print("\n  ✓  All pairs within 3° — calibration looks good")

    # optionally send to servos
    ans = input("\n  Send these angles to servos? [y/N]: ").strip().lower()
    if ans == "y":
        all_channels = {}
        for leg in LEGS:
            all_channels[f"{leg}_COXA"]  = COXA_CH[leg]
            all_channels[f"{leg}_THIGH"] = THIGH_CH[leg]
            all_channels[f"{leg}_WRIST"] = WRIST_CH[leg]

        for joint, angle in phys.items():
            if joint in all_channels:
                set_servo_angle(all_channels[joint], angle)
        print("  Sent. Robot should be standing neutrally.")
        print("  Check: are all legs perpendicular and symmetric?")
        input("  [Enter to finish] ")


# ──────────────────────────────────────────────
#  TEST 4: RAW VALUES
# ──────────────────────────────────────────────

def test_coxa_values():
    """Print all calibration values for inspection."""
    print("\n" + "=" * 60)
    print("TEST 4: RAW CALIBRATION VALUES")
    print("=" * 60)

    print("\n  COXA channels and stand angles:")
    for leg in LEGS:
        print(f"    {leg}:  ch={COXA[leg]:2d}  "
              f"STAND={COXA_STAND[leg]:3d}°  "
              f"PERP={COXA_PERP[leg]:3d}°  "
              f"diff={COXA_STAND[leg] - COXA_PERP[leg]:+d}°")

    print(f"\n  Left  rear vs right rear COXA_STAND: "
          f"RL={COXA_STAND['RL']}° vs RR={COXA_STAND['RR']}°  "
          f"Δ={COXA_STAND['RL'] - COXA_STAND['RR']}°")
    print(f"  Left front vs right front COXA_STAND: "
          f"FL={COXA_STAND['FL']}° vs FR={COXA_STAND['FR']}°  "
          f"Δ={COXA_STAND['FL'] - COXA_STAND['FR']}°")

    if abs(COXA_STAND["RL"] - COXA_STAND["RR"]) > 3:
        print(f"\n  ⚠️  RL-RR coxa difference is "
              f"{COXA_STAND['RL'] - COXA_STAND['RR']}° — "
              f"this may cause yaw drift during trot!")


# ──────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Coxa calibration verifier")
    ap.add_argument("--test", type=int, choices=[1, 2, 3, 4], default=0,
                    help="Run specific test (1=visual, 2=sweep, 3=IK, 4=raw)")
    args = ap.parse_args()

    init_pca()

    if args.test == 0:
        test_coxa_values()    # 4 — just prints
        test_ik_zero()        # 3 — pipeline check
        test_visual()         # 1 — one at a time
        test_sweep()          # 2 — find correct angle
    elif args.test == 1:
        test_visual()
    elif args.test == 2:
        test_sweep()
    elif args.test == 3:
        test_ik_zero()
    elif args.test == 4:
        test_coxa_values()
