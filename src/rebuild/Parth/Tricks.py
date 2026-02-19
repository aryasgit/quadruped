"""
Parth/Tricks.py — SHOW-OFF TRICKS FOR SPOTMICRO
================================================

A collection of crowd-pleasing moves using the proven servo pipeline:
    solve_all_legs → apply_joint_conventions → normalize_all

Tricks:
    1. shake       — Offer paw for a handshake
    2. bow         — Play bow (front down, rear up)
    3. wiggle      — Butt wiggle (excited dog)
    4. pushups     — Squat up and down
    5. stand_up    — Stand on rear legs and pan left-right
    6. sit         — Sit like a dog (rear down, front up)
    7. stretch     — Morning stretch (front down, rear legs extended)
    8. tilt_dance  — Roll side to side rhythmically
    9. combo       — Chain multiple tricks together

Run:
    cd src/rebuild
    python -m Parth.Tricks                  # interactive menu
    python -m Parth.Tricks --trick shake    # single trick
    python -m Parth.Tricks --demo           # run all tricks in sequence
"""

import time
import math
import argparse
from typing import Dict, Tuple

# ── proven pipeline ─────────────────────────────
from layer3.leg_ik import solve_all_legs, _STAND_Y
from layer2.joint_conventions import apply_joint_conventions
from layer2.joint_space import normalize_all

# ── hardware ────────────────────────────────────
from hardware.pca9685 import set_servo_angle, init_pca
from hardware.absolute_truths import COXA, THIGHS, WRISTS


# ================================================================
#  SERVO CHANNEL MAP
# ================================================================
SERVO_MAP = {
    "FL_COXA": COXA["FL"],  "FL_THIGH": THIGHS["TFL"],  "FL_WRIST": WRISTS["WFL"],
    "FR_COXA": COXA["FR"],  "FR_THIGH": THIGHS["TFR"],  "FR_WRIST": WRISTS["WFR"],
    "RL_COXA": COXA["RL"],  "RL_THIGH": THIGHS["TRL"],  "RL_WRIST": WRISTS["WRL"],
    "RR_COXA": COXA["RR"],  "RR_THIGH": THIGHS["TRR"],  "RR_WRIST": WRISTS["WRR"],
}

# ================================================================
#  GEOMETRY CONSTANTS
# ================================================================
STANCE_Y = 0.080       # lateral foot offset (m)
STANCE_Z = -0.170      # standing height (m)
DT = 0.025             # 40 Hz update rate


# ================================================================
#  HELPERS
# ================================================================
def _neutral_feet() -> Dict[str, Tuple[float, float, float]]:
    """Return neutral standing foot positions."""
    return {
        "FL": (0.0,  STANCE_Y, STANCE_Z),
        "FR": (0.0, -STANCE_Y, STANCE_Z),
        "RL": (0.0,  STANCE_Y, STANCE_Z),
        "RR": (0.0, -STANCE_Y, STANCE_Z),
    }


def _send(feet: Dict[str, Tuple[float, float, float]]):
    """Push foot targets through the full pipeline to servos."""
    deltas = solve_all_legs(feet)
    conv = apply_joint_conventions(deltas)
    phys = normalize_all(conv)
    for joint, angle in phys.items():
        ch = SERVO_MAP.get(joint)
        if ch is not None:
            set_servo_angle(ch, angle)


def _lerp_feet(a: Dict, b: Dict, t: float) -> Dict:
    """Linearly interpolate between two foot-position dicts."""
    t = max(0.0, min(1.0, t))
    result = {}
    for leg in a:
        ax, ay, az = a[leg]
        bx, by, bz = b[leg]
        result[leg] = (
            ax + (bx - ax) * t,
            ay + (by - ay) * t,
            az + (bz - az) * t,
        )
    return result


def _smooth(t: float) -> float:
    """Smooth step (ease in/out)."""
    return t * t * (3.0 - 2.0 * t)


def _transition(start_feet: Dict, end_feet: Dict, duration: float = 0.5):
    """Smoothly interpolate from start to end pose."""
    steps = max(1, int(duration / DT))
    for i in range(steps + 1):
        t = _smooth(i / steps)
        _send(_lerp_feet(start_feet, end_feet, t))
        time.sleep(DT)


def stand():
    """Go to neutral stand."""
    _send(_neutral_feet())
    time.sleep(0.3)


# ================================================================
#  TRICK 1: SHAKE (offer paw)
# ================================================================
def shake():
    """Offer front-right paw for a handshake."""
    print("🤝 Shake!")
    neutral = _neutral_feet()

    # step 1: move FR paw forward (along body)
    forward = dict(neutral)
    forward["FR"] = (0.06, -STANCE_Y, STANCE_Z)
    _transition(neutral, forward, duration=0.4)

    # step 2: lift paw up from forward position
    offer = dict(forward)
    offer["FR"] = (0.06, -STANCE_Y + 0.01, STANCE_Z + 0.06)
    _transition(forward, offer, duration=0.5)

    # hold for handshake
    print("   (waiting for handshake...)")
    time.sleep(2.0)

    # small up-down shake motion
    for _ in range(2):
        up = dict(offer)
        up["FR"] = (0.06, -STANCE_Y + 0.01, STANCE_Z + 0.075)
        _transition(offer, up, duration=0.2)
        _transition(up, offer, duration=0.2)

    _transition(offer, neutral, duration=0.8)


# ================================================================
#  TRICK 3: BOW (play bow)
# ================================================================
def bow():
    """Play bow — front legs down, rear stays up."""
    print("🙇 Bow!")
    neutral = _neutral_feet()

    bowed = dict(neutral)
    # front legs retract (body drops at front) — FL gets less drop and less forward shift
    bowed["FL"] = (0.015,  STANCE_Y, STANCE_Z + 0.033)
    bowed["FR"] = (0.03, -STANCE_Y, STANCE_Z + 0.04)
    # rear legs extend slightly (body stays up at rear)
    bowed["RL"] = (-0.01,  STANCE_Y, STANCE_Z - 0.015)
    bowed["RR"] = (-0.01, -STANCE_Y, STANCE_Z - 0.015)

    _transition(neutral, bowed, duration=0.8)
    time.sleep(1.0)
    _transition(bowed, neutral, duration=0.8)


# ================================================================
#  TRICK 4: WIGGLE (butt wiggle)
# ================================================================
def wiggle(cycles: int = 4):
    """Excited butt wiggle — rear sways left and right."""
    print("🍑 Wiggle!")
    neutral = _neutral_feet()

    twist_b = neutral  # initialize for first iteration
    for c in range(cycles):
        # rear twists left (RL forward, RR back)
        twist_a = dict(neutral)
        twist_a["RL"] = ( 0.015,  STANCE_Y, STANCE_Z)
        twist_a["RR"] = (-0.015, -STANCE_Y, STANCE_Z)
        src = neutral if c == 0 else twist_b
        _transition(src, twist_a, duration=0.12)

        # rear twists right (RL back, RR forward)
        twist_b = dict(neutral)
        twist_b["RL"] = (-0.015,  STANCE_Y, STANCE_Z)
        twist_b["RR"] = ( 0.015, -STANCE_Y, STANCE_Z)
        _transition(twist_a, twist_b, duration=0.12)

    _transition(twist_b, neutral, duration=0.3)


# ================================================================
#  TRICK 5: PUSHUPS
# ================================================================
def pushups(reps: int = 3):
    """Squat up and down like pushups."""
    print("💪 Pushups!")
    neutral = _neutral_feet()

    high = neutral  # initialize for first iteration
    for i in range(reps):
        # go low
        low = {
            "FL": (0.0,  STANCE_Y, STANCE_Z - 0.035),
            "FR": (0.0, -STANCE_Y, STANCE_Z - 0.035),
            "RL": (0.0,  STANCE_Y, STANCE_Z - 0.035),
            "RR": (0.0, -STANCE_Y, STANCE_Z - 0.035),
        }
        src = neutral if i == 0 else high
        _transition(src, low, duration=0.4)

        # go high
        high = {
            "FL": (0.0,  STANCE_Y, STANCE_Z + 0.025),
            "FR": (0.0, -STANCE_Y, STANCE_Z + 0.025),
            "RL": (0.0,  STANCE_Y, STANCE_Z + 0.025),
            "RR": (0.0, -STANCE_Y, STANCE_Z + 0.025),
        }
        _transition(low, high, duration=0.4)

    _transition(high, neutral, duration=0.5)


# ================================================================
#  TRICK 6: LOOK AROUND
# ================================================================
def stand_up():
    """Pan the front coxas left and right (simulates head turn)."""
    print("👀 Look around!")
    neutral = _neutral_feet()

    # look left — shift both front feet right (body turns left)
    look_l = dict(neutral)
    look_l["FL"] = (-0.06,  STANCE_Y, STANCE_Z)
    look_l["FR"] = (-0.06, -STANCE_Y, STANCE_Z)
    look_l["RL"] = (0.03,  STANCE_Y, STANCE_Z)
    look_l["RR"] = (0.03, -STANCE_Y, STANCE_Z)

    _transition(neutral, look_l, duration=0.6)
    time.sleep(0.5)

    # look right
    look_r = dict(neutral)
    look_r["FL"] = (0.06,  STANCE_Y, STANCE_Z)
    look_r["FR"] = (0.06, -STANCE_Y, STANCE_Z)
    look_r["RL"] = (-0.03,  STANCE_Y, STANCE_Z)
    look_r["RR"] = (-0.03, -STANCE_Y, STANCE_Z)

    _transition(look_l, look_r, duration=0.8)
    time.sleep(0.5)

    _transition(look_r, neutral, duration=0.6)

    # Recovery: rear knees are on the ground after standing on hind legs.
    # Stage 1 — half-crouch: retract all legs so knees clear the ground
    #           (less torque needed from this geometry)
    half = {
        "FL": (0.0,  STANCE_Y, STANCE_Z + 0.04),
        "FR": (0.0, -STANCE_Y, STANCE_Z + 0.04),
        "RL": (0.0,  STANCE_Y, STANCE_Z + 0.04),
        "RR": (0.0, -STANCE_Y, STANCE_Z + 0.04),
    }
    _send(half)
    time.sleep(0.5)

    # Stage 2 — smoothly lower to full stance
    _transition(half, neutral, duration=0.8)


# ================================================================
#  TRICK 7: SIT
# ================================================================
def sit():
    """Sit like a dog — rear crouches, front stays up."""
    print("🐕 Sit!")
    neutral = _neutral_feet()

    sitting = dict(neutral)
    # rear legs retract (body drops at rear)
    sitting["RL"] = (-0.02,  STANCE_Y, STANCE_Z + 0.05)
    sitting["RR"] = (-0.02, -STANCE_Y, STANCE_Z + 0.05)
    # front legs extend slightly (body stays up at front)
    sitting["FL"] = (0.01,  STANCE_Y, STANCE_Z - 0.015)
    sitting["FR"] = (0.01, -STANCE_Y, STANCE_Z - 0.015)

    _transition(neutral, sitting, duration=1.0)
    time.sleep(2.0)
    _transition(sitting, neutral, duration=1.0)


# ================================================================
#  TRICK 8: STRETCH
# ================================================================
def stretch():
    """Morning stretch — front goes down, rear extends back."""
    print("🐕‍🦺 Stretch!")
    neutral = _neutral_feet()

    stretched = dict(neutral)
    # front drops and pushes forward
    stretched["FL"] = (0.04,  STANCE_Y, STANCE_Z - 0.04)
    stretched["FR"] = (0.04, -STANCE_Y, STANCE_Z - 0.04)
    # rear pushes back and lifts slightly
    stretched["RL"] = (-0.03,  STANCE_Y, STANCE_Z + 0.01)
    stretched["RR"] = (-0.03, -STANCE_Y, STANCE_Z + 0.01)

    _transition(neutral, stretched, duration=1.0)
    time.sleep(1.5)

    # optional: deeper stretch
    deeper = dict(stretched)
    deeper["FL"] = (0.05,  STANCE_Y, STANCE_Z - 0.05)
    deeper["FR"] = (0.05, -STANCE_Y, STANCE_Z - 0.05)
    _transition(stretched, deeper, duration=0.5)
    time.sleep(1.0)

    _transition(deeper, neutral, duration=1.0)


# ================================================================
#  TRICK 9: TILT DANCE
# ================================================================
def tilt_dance(cycles: int = 4):
    """Roll side to side rhythmically."""
    print("💃 Tilt dance!")
    neutral = _neutral_feet()

    tilt_r = neutral  # initialize for first iteration
    for i in range(cycles):
        # tilt left — left legs shorter, right legs taller
        tilt_l = dict(neutral)
        tilt_l["FL"] = (0.0,  STANCE_Y, STANCE_Z + 0.012)
        tilt_l["RL"] = (0.0,  STANCE_Y, STANCE_Z + 0.012)
        tilt_l["FR"] = (0.0, -STANCE_Y, STANCE_Z - 0.012)
        tilt_l["RR"] = (0.0, -STANCE_Y, STANCE_Z - 0.012)

        src = neutral if i == 0 else tilt_r
        _transition(src, tilt_l, duration=0.3)

        # tilt right
        tilt_r = dict(neutral)
        tilt_r["FL"] = (0.0,  STANCE_Y, STANCE_Z - 0.012)
        tilt_r["RL"] = (0.0,  STANCE_Y, STANCE_Z - 0.012)
        tilt_r["FR"] = (0.0, -STANCE_Y, STANCE_Z + 0.012)
        tilt_r["RR"] = (0.0, -STANCE_Y, STANCE_Z + 0.012)

        _transition(tilt_l, tilt_r, duration=0.3)

    _transition(tilt_r, neutral, duration=0.5)


# ================================================================
#  TRICK 10: COMBO
# ================================================================
def combo():
    """Chain multiple tricks into a show routine."""
    print("\n🎬 COMBO ROUTINE — START!\n")

    stand()
    time.sleep(0.5)

    stand_up()
    time.sleep(0.5)

    bow()
    time.sleep(0.5)

    wiggle()
    time.sleep(0.5)

    pushups(reps=2)
    time.sleep(0.5)

    stretch()
    time.sleep(0.5)

    tilt_dance(cycles=3)
    time.sleep(0.5)

    sit()
    time.sleep(0.5)

    shake()
    time.sleep(0.5)

    stand()
    print("\n🎬 COMBO ROUTINE — DONE!\n")


# ================================================================
#  TRICK REGISTRY
# ================================================================
TRICKS = {
    "shake":       (shake,       "Offer paw for handshake"),
    "bow":         (bow,         "Play bow"),
    "wiggle":      (wiggle,      "Excited butt wiggle"),
    "pushups":     (pushups,     "Squat up and down"),
    "stand_up":    (stand_up,    "Stand on rear legs and pan"),
    "sit":         (sit,         "Sit like a dog"),
    "stretch":     (stretch,     "Morning stretch"),
    "tilt_dance":  (tilt_dance,  "Roll side to side"),
    "combo":       (combo,       "Chain all tricks together"),
}


# ================================================================
#  INTERACTIVE MENU
# ================================================================
def interactive():
    """Interactive trick selector."""
    print("\n" + "=" * 50)
    print("  🐕 SPOTMICRO TRICK MENU 🐕")
    print("=" * 50)

    tricks_list = list(TRICKS.keys())
    while True:
        print()
        for i, name in enumerate(tricks_list, 1):
            _, desc = TRICKS[name]
            print(f"  {i:2d}. {name:15s} — {desc}")
        print(f"  {len(tricks_list)+1:2d}. {'stand':15s} — Return to neutral")
        print(f"   0. {'quit':15s} — Exit")
        print()

        try:
            choice = input("  Pick a trick (number or name): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nBye! 🐾")
            break

        if choice in ('0', 'quit', 'q'):
            print("Bye! 🐾")
            break

        # handle number input
        if choice.isdigit():
            idx = int(choice)
            if idx == len(tricks_list) + 1:
                stand()
                continue
            if 1 <= idx <= len(tricks_list):
                choice = tricks_list[idx - 1]
            else:
                print("  ❌ Invalid number")
                continue

        if choice == 'stand':
            stand()
            continue

        if choice in TRICKS:
            func, _ = TRICKS[choice]
            try:
                func()
            except KeyboardInterrupt:
                print("\n  Interrupted — returning to stand")
                stand()
        else:
            print(f"  ❌ Unknown trick: {choice}")


# ================================================================
#  MAIN
# ================================================================
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="SpotMicro Tricks")
    ap.add_argument("--trick", type=str, default=None,
                    help="Run a single trick by name")
    ap.add_argument("--demo", action="store_true",
                    help="Run all tricks in sequence")
    ap.add_argument("--list", action="store_true",
                    help="List available tricks")
    args = ap.parse_args()

    init_pca()

    if args.list:
        print("\nAvailable tricks:")
        for name, (_, desc) in TRICKS.items():
            print(f"  {name:15s} — {desc}")
        print()

    elif args.demo:
        stand()
        combo()

    elif args.trick:
        if args.trick in TRICKS:
            stand()
            func, _ = TRICKS[args.trick]
            func()
            stand()
        else:
            print(f"Unknown trick: {args.trick}")
            print(f"Available: {', '.join(TRICKS.keys())}")

    else:
        stand()
        interactive()
        stand()
