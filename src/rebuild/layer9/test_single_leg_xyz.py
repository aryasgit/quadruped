import time
import math

from layer4.single_leg_xyz_controller import SingleLegXYZController


# ==============================
# CONFIG
# ==============================
LEG = "FL"

AXIS = "x"        # "x", "y", or "z"
AMPLITUDE = 0.04  # meters (±)
CENTER = {
    "x": 0.20,
    "y": 0.00,
    "z": -0.18,
}

PERIOD = 3.0      # seconds (one full oscillation)
STEPS = 120       # resolution


def main():
    ctrl = SingleLegXYZController(LEG)

    print(f"Oscillating leg {LEG} along {AXIS}-axis")

    for i in range(STEPS + 1):
        phase = 2 * math.pi * i / STEPS
        delta = AMPLITUDE * math.sin(phase)

        x = CENTER["x"]
        y = CENTER["y"]
        z = CENTER["z"]

        if AXIS == "x":
            x += delta
        elif AXIS == "y":
            y += delta
        elif AXIS == "z":
            z += delta
        else:
            raise ValueError("AXIS must be x, y, or z")

        ctrl.move_foot_xyz(x, y, z)
        time.sleep(PERIOD / STEPS)

    print("Oscillation complete")


if __name__ == "__main__":
    main()
