import math

STANCE_X = 0.0
STANCE_Y = 0.07
STANCE_Z = -0.18


# -------------------------------------------------
# 1️⃣ Play Bow
# -------------------------------------------------

def play_bow(t, depth=0.04):
    """
    Front legs go down, rear stays.
    Looks dramatic but safe.
    """
    phase = 0.5 - 0.5 * math.cos(2 * math.pi * t)

    bow = depth * phase

    return {
        "FL": (STANCE_X,  STANCE_Y, STANCE_Z + bow),
        "FR": (STANCE_X, -STANCE_Y, STANCE_Z + bow),
        "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
        "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
    }


# -------------------------------------------------
# 2️⃣ Happy Bounce
# -------------------------------------------------

def happy_bounce(t, amp=0.025):
    """
    All legs compress slightly.
    Looks playful.
    """
    bounce = amp * math.sin(2 * math.pi * t)

    return {
        "FL": (STANCE_X,  STANCE_Y, STANCE_Z + bounce),
        "FR": (STANCE_X, -STANCE_Y, STANCE_Z + bounce),
        "RL": (STANCE_X,  STANCE_Y, STANCE_Z + bounce),
        "RR": (STANCE_X, -STANCE_Y, STANCE_Z + bounce),
    }


# -------------------------------------------------
# 3️⃣ Body Sway (more visible)
# -------------------------------------------------

def body_sway(t, amp=0.025):
    """
    Shifts body sideways by offsetting all feet.
    """
    shift = amp * math.sin(2 * math.pi * t)

    return {
        "FL": (STANCE_X,  STANCE_Y + shift, STANCE_Z),
        "FR": (STANCE_X, -STANCE_Y + shift, STANCE_Z),
        "RL": (STANCE_X,  STANCE_Y + shift, STANCE_Z),
        "RR": (STANCE_X, -STANCE_Y + shift, STANCE_Z),
    }


# -------------------------------------------------
# 4️⃣ Real Paw Lift (Wave Base)
# -------------------------------------------------

def paw_lift(t, lift=0.05, reach=0.03):
    """
    Lift front left leg forward slightly.
    This looks like a real wave base.
    """
    phase = 0.5 - 0.5 * math.cos(2 * math.pi * t)

    z = lift * phase
    x = reach * phase

    return {
        "FL": (STANCE_X + x, STANCE_Y, STANCE_Z + z),
        "FR": (STANCE_X, -STANCE_Y, STANCE_Z),
        "RL": (STANCE_X,  STANCE_Y, STANCE_Z),
        "RR": (STANCE_X, -STANCE_Y, STANCE_Z),
    }
