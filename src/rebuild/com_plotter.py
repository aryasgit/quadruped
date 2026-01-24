# com_plotter.py
#
# Live COM visualizer
# READ-ONLY diagnostics
# No control, no hardware, no state mutation

import time
import math
import multiprocessing as mp

import matplotlib.pyplot as plt
import matplotlib.animation as animation


# ===============================
# FOOT GEOMETRY (must match state.py)
# ===============================
FOOT_POSITIONS = {
    "FR": ( 0.18, -0.12),
    "FL": ( 0.18,  0.12),
    "RR": (-0.18, -0.12),
    "RL": (-0.18,  0.12),
}


# ===============================
# MAIN PLOTTER PROCESS
# ===============================
def com_plotter_main(state_q: mp.Queue):
    """
    Runs in a separate process.
    Consumes RobotState dicts and visualizes COM.
    """

    plt.style.use("dark_background")

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_title("COM Projection (Body Frame)")

    # Plot limits (meters)
    LIM = 0.30
    ax.set_xlim(-LIM, LIM)
    ax.set_ylim(-LIM, LIM)
    ax.set_aspect("equal")

    ax.set_xlabel("X (forward)")
    ax.set_ylabel("Y (left)")

    # Draw origin
    ax.scatter(0, 0, c="white", s=10, alpha=0.5)

    # Draw feet
    foot_x = [p[0] for p in FOOT_POSITIONS.values()]
    foot_y = [p[1] for p in FOOT_POSITIONS.values()]
    ax.scatter(foot_x, foot_y, c="cyan", s=80, label="Feet")

    # Draw support polygon
    poly_x = foot_x + [foot_x[0]]
    poly_y = foot_y + [foot_y[0]]
    ax.plot(poly_x, poly_y, "--", color="cyan", alpha=0.4)

    # COM marker
    com_dot, = ax.plot([], [], "ro", markersize=8, label="COM")

    # COM trail
    trail_x, trail_y = [], []
    trail_plot, = ax.plot([], [], "r-", alpha=0.4)

    # Stability bar
    stab_ax = fig.add_axes([0.15, 0.02, 0.7, 0.04])
    stab_ax.set_xlim(0, 1)
    stab_ax.set_ylim(0, 1)
    stab_ax.axis("off")

    stab_bar = stab_ax.barh([0.5], [1.0], height=0.6, color="green")[0]
    stab_text = stab_ax.text(
        0.5, 0.5, "STABILITY",
        ha="center", va="center", color="black", fontsize=10, weight="bold"
    )

    last_update = time.time()

    def update(_):
        nonlocal last_update

        # Drain queue, keep latest
        state = None
        while not state_q.empty():
            state = state_q.get_nowait()

        if state is None:
            return com_dot, trail_plot, stab_bar

        com_x = state["com_x"]
        com_y = state["com_y"]
        stability = state["stability"]

        # Update COM dot
        com_dot.set_data([com_x], [com_y])

        # Update trail
        trail_x.append(com_x)
        trail_y.append(com_y)
        if len(trail_x) > 100:
            trail_x.pop(0)
            trail_y.pop(0)
        trail_plot.set_data(trail_x, trail_y)

        # Update stability bar
        stab_bar.set_width(stability)
        if stability > 0.66:
            stab_bar.set_color("green")
        elif stability > 0.33:
            stab_bar.set_color("orange")
        else:
            stab_bar.set_color("red")

        stab_text.set_text(f"STABILITY {stability:.2f}")

        return com_dot, trail_plot, stab_bar

    ani = animation.FuncAnimation(
        fig, update, interval=40, blit=False
    )

    plt.legend(loc="upper right")
    plt.show()
