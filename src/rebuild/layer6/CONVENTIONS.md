"""
============================================================
CRITICAL STACK CONVENTIONS — READ BEFORE MODIFYING ANY CODE
============================================================

This project follows a STRICT separation of concerns:

    Layer 6 (Gait Generator)
        → Generates IDEAL, GEOMETRIC foot trajectories
        → Outputs HIP-LOCAL (x, y, z) targets ONLY
        → NO knowledge of servos, mirroring, or hardware quirks

    Layer 3 (IK)
        → Converts foot positions into MATHEMATICAL joint angles
        → Assumes IDEAL SpotMicro-style symmetric geometry

    Layer 2.5 (Joint Conventions)  <<< 🔒 LOCKED TRUTH 🔒
        → Converts IK angles into PHYSICAL servo deltas
        → This is the ONLY place where real-world asymmetry exists

------------------------------------------------------------
COORDINATE FRAME (USED EVERYWHERE ABOVE LAYER 2.5)
------------------------------------------------------------

HIP-LOCAL coordinates (right-handed frame):

    x : forward  (+)
    y : left     (+)
    z : up       (+)

Ground is NEGATIVE z.

STANCE reference:
    STANCE_X = 0.0
    STANCE_Y = ±0.07   (left positive, right negative)
    STANCE_Z = -0.18

Layer 6 MUST ALWAYS output values in this frame.
NO mirroring, no sign hacks, no per-leg tricks here.

------------------------------------------------------------
WHY RIGHT LEGS LOOK "WRONG" WITHOUT CONVENTIONS
------------------------------------------------------------

Physically, the robot has:

    • Mirrored servos on the right side
    • Identical IK math for all legs (by design)
    • DIFFERENT physical rotation directions on right wrists

Result:
    ✔ Thighs move correctly on both sides
    ❌ Right wrists EXTEND when they should CONTRACT

THIS IS NOT A GAIT PROBLEM.
THIS IS NOT AN IK PROBLEM.
THIS IS A HARDWARE MIRRORING PROBLEM.

------------------------------------------------------------
THE CANONICAL FIX (DO NOT DUPLICATE ELSEWHERE)
------------------------------------------------------------

The ONLY correct solution is in Layer 2.5:

    joint_conventions.py

Specifically:

    - Coxa:
        FL, RL  → inverted
        FR, RR  → normal

    - Thigh:
        SAME sign for all legs

    - Wrist:
        SAME base sign for all legs
        PLUS:
            RIGHT LEG WRISTS (FR, RR) are INVERTED
            to compensate for physical servo mirroring

This guarantees:
    • Perfectly symmetric arcs in Cartesian space
    • Identical thigh motion left/right
    • Correct knee flexion direction on real hardware

------------------------------------------------------------
IMPORTANT RULES (DO NOT VIOLATE)
------------------------------------------------------------

❌ DO NOT:
    - Flip x or z for right legs in Layer 6
    - Add "visual mirror" hacks in gait code
    - Apply per-leg wrist fixes in IK
    - Re-invert wrists anywhere else

✅ DO:
    - Generate clean, symmetric trajectories in Layer 6
    - Let IK remain pure geometry
    - Let Layer 2.5 absorb ALL hardware asymmetry

------------------------------------------------------------
WHY THIS WORKS (AND STAYS WORKING)
------------------------------------------------------------

This architecture matches:
    ✔ Original SpotMicro math model
    ✔ Simulation sliders behaving perfectly
    ✔ Physical servo reality
    ✔ Future extensions (balance, FSM, compliance)

If something breaks:
    • Gait looks wrong → check Layer 6
    • Leg geometry looks wrong → check Layer 3
    • One side does the opposite → check Layer 2.5

------------------------------------------------------------
TL;DR — ABSOLUTE LAW
------------------------------------------------------------

Layer 6 does NOT know servos exist.
Layer 3 does NOT know hardware exists.
Layer 2.5 is the ONLY place reality leaks in.

If this file is copied elsewhere:
    COPY THIS COMMENT WITH IT.

============================================================
"""
