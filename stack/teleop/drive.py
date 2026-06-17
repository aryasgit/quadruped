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
            estop()
            return
        if "triangle" in edges:
            cmd.state = "walk"; status("[teleop] WALK")
        if "x" in edges:
            cmd.state = "stand"; status("[teleop] STAND")
        if "circle" in edges:
            cmd.state = "idle"; status("[teleop] IDLE")

        if cmd.state == "walk":
            cmd.vx = axes["ly"] * c.max_vx          # stick up = forward
            cmd.vy = -axes["lx"] * c.max_vy         # stick left = +left
            cmd.wz = -axes["rx"] * c.max_wz         # stick left = turn left
            cmd.roll = cmd.pitch = cmd.yaw = 0.0
        else:
            cmd.roll = axes["rx"] * ROLL_AMP
            cmd.pitch = axes["ry"] * PITCH_AMP
            cmd.yaw = -axes["lx"] * YAW_AMP
            cmd.vx = cmd.vy = cmd.wz = 0.0

        emit(cmd)
        step()
