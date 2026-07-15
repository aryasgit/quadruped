"""
stance/tricks.py — SHOW-OFF TRICKS 
================================================

A collection of crowd-pleasing moves using the proven servo pipeline:
    solve_all_legs → apply_joint_conventions → normalize_all

Tricks:
    1. shake       — Offer paw for a handshake
    2. bow         — Play bow (front down, rear up)
    3. wiggle      — Butt wiggle (excited dog)
    4. pushups     — Squat up and down
    5. bheek      — Stand on rear legs and pan left-right
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
import threading
from typing import Dict, Tuple

# BUG FIX (trick-thread race): a cooperative abort flag the trick pipeline honors.
# The old build gated execute_step with a _trick_stop event, but tricks write
# through _send (below), NOT execute_step, so the gate never actually stopped a
# runaway trick thread. This flag makes _send a no-op once abort is requested, so
# a timed-out trick stops touching the servos and the main thread owns the bus.
_abort = threading.Event()

def request_abort():
    _abort.set()

def clear_abort():
    _abort.clear()

def aborted():
    return _abort.is_set()

# ── proven pipeline ─────────────────────────────
from ik.solver import solve_all_legs, _STAND_Y, _STAND_Z
from joints.conventions import apply_joint_conventions
from joints.space import normalize_all

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
# Use the SAME stand reference as the IK solver to ensure deltas are zero at neutral
STANCE_Y = _STAND_Y    # lateral foot offset (m) — from leg_ik.py
STANCE_Z = _STAND_Z    # standing height (m) — from leg_ik.py
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
    """
    Push foot targets through the full pipeline to servos.

    BUG FIX: the whole pipeline is now guarded. Previously an I2C error (or a
    bad IK target) raised straight out of a trick, unwinding through _transition
    and killing the control loop / leaving the robot frozen mid-pose. Now a
    failed frame is skipped and the trick (and main loop) survive. Transient bus
    NAKs are already absorbed by the locked, retrying i2c_bus.
    """
    if _abort.is_set():
        return   # trick was aborted (e.g. timed out) — stop writing servos
    try:
        deltas = solve_all_legs(feet)
        conv = apply_joint_conventions(deltas)
        phys = normalize_all(conv)
        for joint, angle in phys.items():
            ch = SERVO_MAP.get(joint)
            if ch is not None:
                set_servo_angle(ch, angle)
    except Exception as e:
        print(f"[TRICK] frame skipped (pipeline error): {e}")


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

    # step 1: SHIFT WEIGHT LEFT before lifting right paw
    #         - Move FL outward (wider stance on support side)
    #         - Move body slightly left by shifting FR inward
    #         - Rear legs shift left to move CoM over left tripod
    weight_left = dict(neutral)
    weight_left["FL"] = (0.02,  STANCE_Y + 0.015, STANCE_Z - 0.01)  # FL out & down (support)
    weight_left["FR"] = (0.0,  -STANCE_Y + 0.02, STANCE_Z)          # FR inward (prep to lift)
    weight_left["RL"] = (0.0,   STANCE_Y + 0.01, STANCE_Z)          # RL slightly out
    weight_left["RR"] = (0.0,  -STANCE_Y + 0.01, STANCE_Z)          # RR slightly in
    _transition(neutral, weight_left, duration=0.5)
    time.sleep(0.2)

    # step 2: lift FR paw up and farther forward (more thigh-forward reach)
    offer = dict(weight_left)
    offer["FR"] = (0.085, -STANCE_Y + 0.02, STANCE_Z + 0.07)
    _transition(weight_left, offer, duration=0.65)

    # Minimal hold before starting handshake motion.
    print("   (waiting for handshake...)")
    time.sleep(0.20)

    # Slow up-down shake driven only by wrist axis (keep FR x/y fixed; no coxa sweep).
    for _ in range(3):
        up = dict(offer)
        up["FR"] = (offer["FR"][0], offer["FR"][1], STANCE_Z + 0.115)
        _transition(offer, up, duration=0.34)
        down = dict(offer)
        down["FR"] = (offer["FR"][0], offer["FR"][1], STANCE_Z + 0.050)
        _transition(up, down, duration=0.34)
        _transition(down, offer, duration=0.30)

    # step 3: lower paw back to weight-shifted position first
    _transition(offer, weight_left, duration=0.5)
    time.sleep(0.1)

    # step 4: return to neutral
    _transition(weight_left, neutral, duration=0.5)


# ================================================================
#  TRICK 3: BOW (play bow)
# ================================================================
def bow():
    """Play bow — front legs down, rear stays up."""
    print("🙇 Bow!")
    neutral = _neutral_feet()

    bowed = dict(neutral)
    # Deeper front bow with stronger front drop and rear extension.
    bowed["FL"] = (0.020,  STANCE_Y, STANCE_Z + 0.060)
    bowed["FR"] = (0.020, -STANCE_Y, STANCE_Z + 0.060)
    bowed["RL"] = (-0.015,  STANCE_Y, STANCE_Z - 0.022)
    bowed["RR"] = (-0.015, -STANCE_Y, STANCE_Z - 0.022)

    _transition(neutral, bowed, duration=0.9)

    # Slow "nodding" motion while bowed: tiny pitch oscillation to look natural.
    nod_down = dict(bowed)
    nod_down["FL"] = (0.020,  STANCE_Y, STANCE_Z + 0.068)
    nod_down["FR"] = (0.020, -STANCE_Y, STANCE_Z + 0.068)
    nod_down["RL"] = (-0.015,  STANCE_Y, STANCE_Z - 0.026)
    nod_down["RR"] = (-0.015, -STANCE_Y, STANCE_Z - 0.026)

    nod_up = dict(bowed)
    nod_up["FL"] = (0.020,  STANCE_Y, STANCE_Z + 0.053)
    nod_up["FR"] = (0.020, -STANCE_Y, STANCE_Z + 0.053)
    nod_up["RL"] = (-0.015,  STANCE_Y, STANCE_Z - 0.018)
    nod_up["RR"] = (-0.015, -STANCE_Y, STANCE_Z - 0.018)

    for _ in range(2):
        _transition(bowed, nod_down, duration=0.65)
        _transition(nod_down, nod_up, duration=0.75)
        _transition(nod_up, bowed, duration=0.65)

    _transition(bowed, neutral, duration=0.9)


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
def bheek():
    """Pan the front coxas left and right (simulates head turn)."""
    print("👀 Look around!")
    neutral = _neutral_feet()

    # look left — shift both front feet back MORE AGGRESSIVELY to tip backward
    look_l = dict(neutral)
    look_l["FL"] = (-0.085,  STANCE_Y, STANCE_Z)   # was -0.06, now -0.085
    look_l["FR"] = (-0.085, -STANCE_Y, STANCE_Z)   # was -0.06, now -0.085
    look_l["RL"] = (0.04,  STANCE_Y, STANCE_Z + 0.020)  # extra rear wrist bend for stable sit-back
    look_l["RR"] = (0.04, -STANCE_Y, STANCE_Z + 0.020)  # extra rear wrist bend for stable sit-back

    _transition(neutral, look_l, duration=0.4)    # was 0.6, now 0.4 (faster for momentum)
    time.sleep(1.5)  # wait for robot to settle on rear legs

    # ── Front paws pump up and down together while in the air ──
    # Push forward (X) and outward (Y) so legs extend out from body
    for _ in range(3):
        # both front paws forward and out
        paws_up = dict(look_l)
        paws_up["FL"] = (0.06,  STANCE_Y + 0.03, STANCE_Z + 0.04)
        paws_up["FR"] = (0.06, -STANCE_Y - 0.03, STANCE_Z + 0.04)
        _transition(look_l, paws_up, duration=0.15)

        # both front paws back toward body
        paws_dn = dict(look_l)
        paws_dn["FL"] = (look_l["FL"][0],  STANCE_Y, STANCE_Z)
        paws_dn["FR"] = (look_l["FR"][0], -STANCE_Y, STANCE_Z)
        _transition(paws_up, paws_dn, duration=0.15)

    # return to look_l base before panning
    _transition(paws_dn, look_l, duration=0.2)
    time.sleep(0.3)

    # look right
    look_r = dict(neutral)
    look_r["FL"] = (0.06,  STANCE_Y, STANCE_Z)
    look_r["FR"] = (0.06, -STANCE_Y, STANCE_Z)
    look_r["RL"] = (-0.03,  STANCE_Y, STANCE_Z)
    look_r["RR"] = (-0.03, -STANCE_Y, STANCE_Z)

    _transition(look_l, look_r, duration=0.8)
    time.sleep(0.5)

    _transition(look_r, neutral, duration=0.6)

    # ── Recovery: robot is reared up on 2 hind legs ──
    # Step 1: gently bring front legs down to a crouched position
    #         (all legs retracted so body is low — less torque needed)
    crouch = {
        "FL": (0.0,  STANCE_Y, STANCE_Z + 0.05),
        "FR": (0.0, -STANCE_Y, STANCE_Z + 0.05),
        "RL": (0.0,  STANCE_Y, STANCE_Z + 0.05),
        "RR": (0.0, -STANCE_Y, STANCE_Z + 0.05),
    }
    _send(crouch)
    time.sleep(0.6)

    # Step 2: push up to a half-stance (intermediate height)
    half = {
        "FL": (0.0,  STANCE_Y, STANCE_Z + 0.025),
        "FR": (0.0, -STANCE_Y, STANCE_Z + 0.025),
        "RL": (0.0,  STANCE_Y, STANCE_Z + 0.025),
        "RR": (0.0, -STANCE_Y, STANCE_Z + 0.025),
    }
    _transition(crouch, half, duration=0.5)
    time.sleep(0.3)

    # Step 3: smoothly rise to full stance
    _transition(half, neutral, duration=0.6)


# ================================================================
#  TRICK 6b: HIGH FIVE (rear up + handshake)
# ================================================================
def high_five():
    """Rear up on hind legs and offer right paw for a high-five."""
    print("✋ High Five!")
    neutral = _neutral_feet()

    # step 1: tip backward onto hind legs (same as bheek)
    reared = dict(neutral)
    reared["FL"] = (-0.085,  STANCE_Y, STANCE_Z)
    reared["FR"] = (-0.085, -STANCE_Y, STANCE_Z)
    reared["RL"] = (0.04,  STANCE_Y, STANCE_Z + 0.020)
    reared["RR"] = (0.04, -STANCE_Y, STANCE_Z + 0.020)

    _transition(neutral, reared, duration=0.4)
    time.sleep(1.0)  # settle on rear legs

    # step 2: extend FR paw forward slowly for high-five
    #         keep FL tucked for balance
    offer = dict(reared)
    offer["FL"] = (-0.04,  STANCE_Y, STANCE_Z - 0.02)  # FL tucked closer to body
    offer["FR"] = (0.06, -STANCE_Y - 0.02, STANCE_Z + 0.03)  # FR extended out

    _transition(reared, offer, duration=0.8)  # slow extension

    # hold for high-five
    print("   (waiting for high-five...)")
    time.sleep(2.5)

    # step 3: wave motion — larger amplitude for visibility
    for _ in range(3):
        wave_up = dict(offer)
        wave_up["FR"] = (0.09, -STANCE_Y - 0.03, STANCE_Z + 0.07)  # extend out & up
        _transition(offer, wave_up, duration=0.18)
        wave_down = dict(offer)
        wave_down["FR"] = (0.03, -STANCE_Y - 0.01, STANCE_Z)  # pull back & down
        _transition(wave_up, wave_down, duration=0.18)
        _transition(wave_down, offer, duration=0.18)

    # step 4: retract paw back to reared position
    _transition(offer, reared, duration=0.5)
    time.sleep(0.3)

    # step 5: recovery pounce (same as bheek)
    crouch = {
        "FL": (0.0,  STANCE_Y, STANCE_Z + 0.05),
        "FR": (0.0, -STANCE_Y, STANCE_Z + 0.05),
        "RL": (0.0,  STANCE_Y, STANCE_Z + 0.05),
        "RR": (0.0, -STANCE_Y, STANCE_Z + 0.05),
    }
    _send(crouch)
    time.sleep(0.6)

    half = {
        "FL": (0.0,  STANCE_Y, STANCE_Z + 0.025),
        "FR": (0.0, -STANCE_Y, STANCE_Z + 0.025),
        "RL": (0.0,  STANCE_Y, STANCE_Z + 0.025),
        "RR": (0.0, -STANCE_Y, STANCE_Z + 0.025),
    }
    _transition(crouch, half, duration=0.5)
    time.sleep(0.3)

    _transition(half, neutral, duration=0.6)


# ================================================================
#  TRICK 7: SIT
# ================================================================
def sit():
    """Sit like a dog — rear crouches, front stays up."""
    print("🐕 Sit!")
    neutral = _neutral_feet()

    sitting = dict(neutral)
    # Rear thighs should not travel too far back, but rear wrists fold inward more.
    sitting["RL"] = (-0.042,  STANCE_Y, STANCE_Z + 0.092)
    sitting["RR"] = (-0.042, -STANCE_Y, STANCE_Z + 0.092)
    # Front thighs/wrists move back more to keep the sit balanced and compact.
    sitting["FL"] = (-0.010,  STANCE_Y, STANCE_Z - 0.010)
    sitting["FR"] = (-0.010, -STANCE_Y, STANCE_Z - 0.010)

    _transition(neutral, sitting, duration=1.0)
    time.sleep(2.0)
    _transition(sitting, neutral, duration=1.0)


# ================================================================
#  TRICK 8: STRETCH
# ================================================================
def stretch():
    """Front-wrist handstand with synchronized rear-leg air motion and smooth landing."""
    print("🤸 Handstand!")
    neutral = _neutral_feet()

    # Step 1: whole-body tilt forward until front wrists are fully closed.
    tilt_front = dict(neutral)
    tilt_front["FL"] = (0.030,  STANCE_Y, STANCE_Z + 0.125)
    tilt_front["FR"] = (0.030, -STANCE_Y, STANCE_Z + 0.125)
    tilt_front["RL"] = (0.015,  STANCE_Y, STANCE_Z + 0.035)
    tilt_front["RR"] = (0.015, -STANCE_Y, STANCE_Z + 0.035)
    _transition(neutral, tilt_front, duration=1.05)

    # Step 2: compress the rear legs to load energy before the launch.
    rear_compress = dict(tilt_front)
    rear_compress["RL"] = (0.035,  STANCE_Y, STANCE_Z + 0.020)
    rear_compress["RR"] = (0.035, -STANCE_Y, STANCE_Z + 0.020)
    _transition(tilt_front, rear_compress, duration=0.40)

    # Step 3: explosive rear-leg push (rear wrists back fast) while front thighs drive forward.
    handstand = dict(rear_compress)
    handstand["FL"] = (0.080,  STANCE_Y, STANCE_Z + 0.060)
    handstand["FR"] = (0.080, -STANCE_Y, STANCE_Z + 0.060)
    handstand["RL"] = (-0.100,  STANCE_Y, STANCE_Z + 0.132)
    handstand["RR"] = (-0.100, -STANCE_Y, STANCE_Z + 0.132)
    _transition(rear_compress, handstand, duration=0.12)

    # Step 4: handstand demo — move rear legs slowly and in sync while in the air.
    rear_high = dict(handstand)
    rear_high["RL"] = (-0.098,  STANCE_Y, STANCE_Z + 0.146)
    rear_high["RR"] = (-0.098, -STANCE_Y, STANCE_Z + 0.146)

    rear_low = dict(handstand)
    rear_low["RL"] = (-0.080,  STANCE_Y, STANCE_Z + 0.118)
    rear_low["RR"] = (-0.080, -STANCE_Y, STANCE_Z + 0.118)

    hold = dict(handstand)
    for _ in range(3):
        _transition(hold, rear_high, duration=0.55)
        _transition(rear_high, rear_low, duration=0.60)
        hold = rear_low
    _transition(hold, handstand, duration=0.45)

    # Step 5: recover by bringing front thighs back first, then land and stand.
    front_back = dict(handstand)
    front_back["FL"] = (-0.040,  STANCE_Y, STANCE_Z + 0.065)
    front_back["FR"] = (-0.040, -STANCE_Y, STANCE_Z + 0.065)
    front_back["RL"] = (-0.072,  STANCE_Y, STANCE_Z + 0.108)
    front_back["RR"] = (-0.072, -STANCE_Y, STANCE_Z + 0.108)
    _transition(handstand, front_back, duration=0.60)

    landing = dict(neutral)
    landing["FL"] = (-0.015,  STANCE_Y, STANCE_Z + 0.040)
    landing["FR"] = (-0.015, -STANCE_Y, STANCE_Z + 0.040)
    landing["RL"] = (-0.025,  STANCE_Y, STANCE_Z + 0.045)
    landing["RR"] = (-0.025, -STANCE_Y, STANCE_Z + 0.045)
    _transition(front_back, landing, duration=0.65)
    _transition(landing, neutral, duration=0.85)


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

    bheek()
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
    "bheek":      (bheek,       "Stand on rear legs and pan"),
    "high_five":   (high_five,   "Rear up and offer paw for high-five"),
    "sit":         (sit,         "Sit like a dog"),
    "stretch":     (stretch,     "Front-wrist handstand with smooth landing"),
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
