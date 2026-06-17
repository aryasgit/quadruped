"""
Sim scenarios (D-008) — each returns a metrics dict and logs a time series.

All motion comes from barq1.trajectories / barq1.gait (D-013): the same
frame streams the hardware runtime consumes. The sim executor commands a
frame, steps physics DT, and records ground-truth metrics (D-009: metrics
only, never control).
"""

import math
import sys
from pathlib import Path

STACK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STACK_ROOT))

from barq1 import gait, trajectories as tj
from barq1.command import GaitCommand
from barq1.kinematics import body_ik, stance_feet_world, to_urdf_joints
from barq1.velocity_gait import VelocityGait

STAND_H = tj.STAND_H
CROUCH_H = tj.CROUCH_H


class Recorder:
    COLS = ("t", "x", "y", "h", "roll", "pitch", "yaw",
            "roll_cmd", "pitch_cmd", "yaw_cmd", "margin", "contacts")

    def __init__(self):
        self.rows = []

    def sample(self, robot, cmd_rpy=(float("nan"),) * 3):
        (x, y, z), (r, pch, yw) = robot.body_state()
        m = robot.support_margin()
        c = sum(robot.foot_contacts().values())
        self.rows.append((robot.t, x, y, z, r, pch, yw, *cmd_rpy,
                          m if m is not None else float("nan"), c))

    def col(self, name):
        i = self.COLS.index(name)
        return [row[i] for row in self.rows]

    def save_csv(self, path):
        with open(path, "w") as f:
            f.write(",".join(self.COLS) + "\n")
            for row in self.rows:
                f.write(",".join(f"{v:.6f}" if isinstance(v, float) else str(v)
                                 for v in row) + "\n")


def run_trajectory(robot, frames, rec, sample_every=2):
    """Drive the robot with a frame stream; sample metrics every k frames."""
    for i, (feet, xyz, rpy) in enumerate(frames):
        robot.command(to_urdf_joints(body_ik(feet, body_xyz=xyz, body_rpy=rpy)))
        robot.step(tj.DT)
        if i % sample_every == 0:
            rec.sample(robot, cmd_rpy=rpy)


def _spawn_standing(robot, h=STAND_H):
    """Teleport joints to the stance solution, settle briefly."""
    feet = stance_feet_world(h)
    targets = to_urdf_joints(body_ik(feet, body_xyz=(0, 0, h)))
    robot.teleport_joints(targets)
    robot.command(targets)
    robot.step(1.0)
    return feet


def _max_tilt(rec):
    return max(max(abs(v) for v in rec.col("roll")),
               max(abs(v) for v in rec.col("pitch")))


def settle(robot):
    """Drop at stance pose; does it land standing, level, on 4 feet?"""
    rec = Recorder()
    feet = stance_feet_world(STAND_H)
    targets = to_urdf_joints(body_ik(feet, body_xyz=(0, 0, STAND_H)))
    robot.teleport_joints(targets)
    robot.command(targets)
    for _ in range(120):
        robot.step(1 / 60)
        rec.sample(robot)
    (x, y, h), (r, p_, yw) = robot.body_state()
    return {
        "final_height_m": h,
        "height_err_mm": (h - STAND_H) * 1000,
        "final_roll_deg": math.degrees(r),
        "final_pitch_deg": math.degrees(p_),
        "contacts": sum(robot.foot_contacts().values()),
        "support_margin_mm": (robot.support_margin() or 0) * 1000,
    }, rec


def stand_up(robot):
    """Crouch -> stand ramp over 2 s; level throughout?"""
    rec = Recorder()
    _spawn_standing(robot, CROUCH_H)
    run_trajectory(robot, tj.chain(
        tj.stance_ramp(CROUCH_H, STAND_H, 2.0),
        tj.hold(stance_feet_world(STAND_H), (0, 0, STAND_H), (0, 0, 0), 0.5),
    ), rec)
    (x, y, h), _ = robot.body_state()
    return {
        "final_height_m": h,
        "height_err_mm": (h - STAND_H) * 1000,
        "max_tilt_deg": math.degrees(_max_tilt(rec)),
        "contacts": sum(robot.foot_contacts().values()),
        "support_margin_mm": (robot.support_margin() or 0) * 1000,
    }, rec


def pose_sweep(robot):
    """Sinusoidal roll, pitch, yaw, height with feet planted — open-loop
    body-pose tracking, the core posture-control primitive."""
    rec = Recorder()
    _spawn_standing(robot)
    run_trajectory(robot, tj.pose_sweep(), rec)
    rms = {}
    for axis in ("roll", "pitch", "yaw"):
        pairs = [(a, c) for a, c in zip(rec.col(axis), rec.col(axis + "_cmd"))
                 if not math.isnan(c)]
        rms[axis] = math.sqrt(sum((a - c) ** 2 for a, c in pairs) / len(pairs))
    margins = [m for m in rec.col("margin") if not math.isnan(m)]
    return {
        "rms_roll_track_deg": math.degrees(rms["roll"]),
        "rms_pitch_track_deg": math.degrees(rms["pitch"]),
        "rms_yaw_track_deg": math.degrees(rms["yaw"]),
        "min_support_margin_mm": min(margins) * 1000,
        "contacts": sum(robot.foot_contacts().values()),
    }, rec


def weight_shift(robot):
    """The static-walk primitive: tripod shift, lift FL 4 cm, hold, return."""
    rec = Recorder()
    _spawn_standing(robot)
    run_trajectory(robot, tj.weight_shift_lift(), rec)
    (x, y, h), (r, p_, yw) = robot.body_state()
    hold_margins = [row[Recorder.COLS.index("margin")] for row in rec.rows
                    if row[Recorder.COLS.index("contacts")] == 3
                    and not math.isnan(row[Recorder.COLS.index("margin")])]
    return {
        "survived": h > 0.10 and abs(math.degrees(r)) < 10 and abs(math.degrees(p_)) < 10,
        "final_height_m": h,
        "min_margin_3leg_mm": (min(hold_margins) * 1000) if hold_margins else None,
        "max_tilt_deg": math.degrees(_max_tilt(rec)),
        "contacts_final": sum(robot.foot_contacts().values()),
    }, rec


def walk(robot, cycles=3, params=gait.CrawlParams()):
    """THE milestone: crawl forward `cycles` cycles, open-loop."""
    rec = Recorder()
    _spawn_standing(robot, params.stand_height)
    (x0, y0, _), (_, _, yaw0) = robot.body_state()
    run_trajectory(robot, gait.crawl(cycles, params), rec)
    (x1, y1, h), (r, p_, yaw1) = robot.body_state()
    expected = gait.expected_advance(cycles, params)
    margins = [m for m in rec.col("margin") if not math.isnan(m)]
    heights = rec.col("h")
    duration = cycles * gait.cycle_time(params)
    fell = min(heights) < 0.10 or math.degrees(_max_tilt(rec)) > 15
    return {
        "fell": fell,
        "distance_x_mm": (x1 - x0) * 1000,
        "expected_x_mm": expected * 1000,
        "efficiency_pct": 100 * (x1 - x0) / expected,
        "drift_y_mm": (y1 - y0) * 1000,
        "yaw_drift_deg": math.degrees(yaw1 - yaw0),
        "min_support_margin_mm": min(margins) * 1000,
        "max_tilt_deg": math.degrees(_max_tilt(rec)),
        "avg_speed_mm_s": (x1 - x0) * 1000 / duration,
        "contacts_final": sum(robot.foot_contacts().values()),
    }, rec


def _velocity_run(robot, cmd, secs, cfg=None):
    """Drive the velocity gait (D-016) at a fixed command for `secs`."""
    g = VelocityGait(cfg)
    h = g.cfg.stand_height
    targets = to_urdf_joints(body_ik(stance_feet_world(h), body_xyz=(0, 0, h)))
    robot.teleport_joints(targets)
    robot.command(targets)
    robot.step(0.5)
    (x0, y0, _), (_, _, yaw0) = robot.body_state()
    g.reset()
    rec = Recorder()

    def frames():
        for _ in range(int(round(secs / g.cfg.dt))):
            yield g.step(cmd)

    run_trajectory(robot, frames(), rec)
    (x1, y1, z1), (_, _, yaw1) = robot.body_state()
    margins = sorted(m for m in rec.col("margin") if not math.isnan(m))
    return {
        "x0": x0, "y0": y0, "yaw0": yaw0, "x1": x1, "y1": y1, "z1": z1,
        "yaw1": yaw1, "margins": margins, "rec": rec,
        "max_tilt_deg": math.degrees(_max_tilt(rec)),
        "fell": z1 < 0.10 or math.degrees(_max_tilt(rec)) > 15,
    }


def _vel_metrics(d):
    m = d["margins"]
    return {
        "fell": d["fell"],
        "distance_x_mm": (d["x1"] - d["x0"]) * 1000,
        "drift_y_mm": (d["y1"] - d["y0"]) * 1000,
        "yaw_deg": math.degrees(d["yaw1"] - d["yaw0"]),
        "median_margin_mm": m[len(m) // 2] * 1000,
        "p10_margin_mm": m[len(m) // 10] * 1000,
        "neg_margin_pct": 100.0 * sum(1 for v in m if v < 0) / len(m),
        "max_tilt_deg": d["max_tilt_deg"],
    }


def vel_forward(robot):
    """Velocity gait: walk forward at the safe-envelope max for 8 s."""
    d = _velocity_run(robot, GaitCommand(vx=0.024), 8.0)
    return _vel_metrics(d), d["rec"]


def vel_turn(robot):
    """Velocity gait: turn left in place at the safe-envelope max for 8 s."""
    d = _velocity_run(robot, GaitCommand(wz=0.10), 8.0)
    return _vel_metrics(d), d["rec"]


def vel_strafe(robot):
    """Velocity gait: strafe left at the safe-envelope max for 8 s."""
    d = _velocity_run(robot, GaitCommand(vy=0.022), 8.0)
    return _vel_metrics(d), d["rec"]


SCENARIOS = {
    "settle": settle,
    "stand_up": stand_up,
    "pose_sweep": pose_sweep,
    "weight_shift": weight_shift,
    "walk": walk,
    "vel_forward": vel_forward,
    "vel_turn": vel_turn,
    "vel_strafe": vel_strafe,
}
