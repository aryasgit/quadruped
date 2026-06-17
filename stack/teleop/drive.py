"""
Teleop mapping (D-012/D-017) — robot-agnostic.
==============================================

Maps a PS4 pad to a GaitCommand and feeds it to a Controller via the
caller's `emit(cmd)` callback (which runs controller.step -> body_ik ->
sim or hardware). One mapping for sim and hardware.

Controls
  CROSS (X)   stand        TRIANGLE  walk        CIRCLE  idle
  SQUARE      ESTOP (hardware all-off / sim reset)    OPTIONS  quit
  in WALK:  left stick = move (fwd/back + strafe),  right stick X = turn
  in STAND: right stick = lean (roll/pitch),        left stick X = yaw
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from barq1.command import GaitCommand
from barq1.velocity_gait import GaitConfig

ROLL_AMP = 0.12
PITCH_AMP = 0.10
YAW_AMP = 0.06          # planted-foot yaw is limited (D-015) — keep small


def stick_to_velocity(lx, ly, cfg):
    """Left stick -> 4-quadrant proportional body velocity.

    The dominant axis selects the quadrant (front/back vs left/right), and
    that axis's magnitude sets the speed — a small push = slow, full push =
    max. Returns (vx, vy) in m/s; the other axis is zeroed so motion stays
    cardinal (no diagonal blend). Sticks are already deadzoned + normalized
    to [-1, 1] with stick-up = +ly, stick-left = -lx.
    """
    if abs(ly) >= abs(lx):
        return ly * cfg.max_vx, 0.0        # front (+) / back (-)
    return 0.0, -lx * cfg.max_vy           # left (+) / right (-)


def teleop_loop(pad, emit, step, estop=None, status=lambda m: print(m)):
    c = GaitConfig()
    cmd = GaitCommand(state="stand")
    status("[teleop] STAND. CROSS=stand TRIANGLE=walk CIRCLE=idle SQUARE=estop "
           "OPTIONS=quit | walk: L-stick move, R-stick X turn | stand: R-stick lean")
    while True:
        axes, btns, edges = pad.poll()

        if "options" in edges:
            status("[teleop] quit")
            return
        if "square" in edges and estop:
            # sim estop resets and keeps driving; hardware estop all-offs and
            # exits the process itself — so DON'T return here (only OPTIONS or
            # closing the window quits the session).
            status("[teleop] ESTOP")
            estop()
        if "triangle" in edges:
            cmd.state = "walk"; status("[teleop] WALK")
        if "x" in edges:
            cmd.state = "stand"; status("[teleop] STAND")
        if "circle" in edges:
            cmd.state = "idle"; status("[teleop] IDLE")

        if cmd.state == "walk":
            cmd.vx, cmd.vy = stick_to_velocity(axes["lx"], axes["ly"], c)  # L-stick 4-quadrant
            cmd.wz = -axes["rx"] * c.max_wz         # R-stick X = turn (left = +)
            cmd.roll = cmd.pitch = cmd.yaw = 0.0
        else:
            cmd.roll = axes["rx"] * ROLL_AMP
            cmd.pitch = axes["ry"] * PITCH_AMP
            cmd.yaw = -axes["lx"] * YAW_AMP
            cmd.vx = cmd.vy = cmd.wz = 0.0

        emit(cmd)
        step()
