"""
Teleop mapping + loop (D-012) — robot-agnostic.
===============================================

One mapping for sim and hardware; the caller supplies callbacks:

  command(feet, xyz, rpy)  -> send one 50 Hz frame to sim or hardware
  step()                   -> advance time (sim physics step / wall sleep)
  estop()                  -> square button (hardware: ALL OFF; sim: reset)

Controls
  right stick      roll (x) / pitch (y)
  left stick x     yaw
  left stick y     body height (+/- 2.5 cm)
  TRIANGLE         walk: run `cycles` crawl cycles, then back to pose mode
  CIRCLE           recenter pose
  SQUARE           ESTOP (hardware) / reset pose (sim)
  OPTIONS          quit
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from barq1 import gait
from barq1.kinematics import stance_feet_world
from barq1.trajectories import STAND_H

ROLL_AMP = 0.15
PITCH_AMP = 0.12
YAW_AMP = 0.15
H_AMP = 0.025
SMOOTH = 0.18          # low-pass on stick -> pose (per 50 Hz frame)


def teleop_loop(pad, command, step, estop=None, walk_cycles=2,
                status=lambda msg: print(msg)):
    feet = stance_feet_world(STAND_H)
    pose = [0.0, 0.0, 0.0, STAND_H]      # roll, pitch, yaw, height
    status("[teleop] pose mode — sticks: pose · TRIANGLE: walk · "
           "CIRCLE: recenter · SQUARE: estop/reset · OPTIONS: quit")
    while True:
        axes, btns, edges = pad.poll()

        if "options" in edges:
            status("[teleop] quit")
            return
        if "square" in edges and estop:
            estop()
            return
        if "circle" in edges:
            pose = [0.0, 0.0, 0.0, STAND_H]
        if "triangle" in edges:
            status(f"[teleop] walking {walk_cycles} cycles…")
            for f, xyz, rpy in gait.crawl(walk_cycles):
                command(f, xyz, rpy)
                step()
            # crawl ends recentered over the (advanced) feet at the same
            # joint angles as neutral stance -> safe to re-zero the frame
            feet = stance_feet_world(STAND_H)
            pose = [0.0, 0.0, 0.0, STAND_H]
            status("[teleop] pose mode")
            continue

        target = (axes["rx"] * ROLL_AMP, axes["ry"] * PITCH_AMP,
                  axes["lx"] * YAW_AMP, STAND_H + axes["ly"] * H_AMP)
        for i in range(4):
            pose[i] += SMOOTH * (target[i] - pose[i])

        command(feet, (0.0, 0.0, pose[3]), (pose[0], pose[1], pose[2]))
        step()
