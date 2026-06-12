"""
Sim scenarios (D-008) — each returns a metrics dict and logs a time series.

All scenarios drive the robot exactly as hardware will be driven: joint
position commands computed by barq1.kinematics, nothing else. Ground-truth
probes are recorded as metrics (D-009).
"""

import math
import sys
from pathlib import Path

STACK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STACK_ROOT))

from barq1.kinematics import LEGS, body_ik, stance_feet_world, to_urdf_joints

STAND_H = 0.155          # spotMicro default stand height
CROUCH_H = 0.125


class Recorder:
    COLS = ("t", "h", "roll", "pitch", "yaw", "margin", "contacts")

    def __init__(self):
        self.rows = []

    def sample(self, robot):
        (x, y, z), (r, pch, yw) = robot.body_state()
        m = robot.support_margin()
        c = sum(robot.foot_contacts().values())
        self.rows.append((robot.t, z, r, pch, yw, m if m is not None else float("nan"), c))

    def col(self, name):
        i = self.COLS.index(name)
        return [row[i] for row in self.rows]

    def save_csv(self, path):
        with open(path, "w") as f:
            f.write(",".join(self.COLS) + "\n")
            for row in self.rows:
                f.write(",".join(f"{v:.6f}" if isinstance(v, float) else str(v)
                                 for v in row) + "\n")


def _drive(robot, rec, feet, xyz, rpy, seconds, sample_dt=1 / 60):
    """Command one body pose and run physics, sampling metrics."""
    robot.command(to_urdf_joints(body_ik(feet, body_xyz=xyz, body_rpy=rpy)))
    steps = max(1, int(round(seconds / sample_dt)))
    for _ in range(steps):
        robot.step(sample_dt)
        rec.sample(robot)


def _spawn_standing(robot, h=STAND_H):
    """Teleport joints to the stance solution, settle briefly."""
    feet = stance_feet_world(h)
    targets = to_urdf_joints(body_ik(feet, body_xyz=(0, 0, h)))
    robot.teleport_joints(targets)
    robot.command(targets)
    robot.step(1.0)
    return feet


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
    n = 60
    for i in range(n + 1):
        h = CROUCH_H + (STAND_H - CROUCH_H) * i / n
        _drive(robot, rec, stance_feet_world(h), (0, 0, h), (0, 0, 0), 2.0 / n)
    robot.step(0.5)
    rec.sample(robot)
    (x, y, h), (r, p_, yw) = robot.body_state()
    max_tilt = max(max(abs(v) for v in rec.col("roll")),
                   max(abs(v) for v in rec.col("pitch")))
    return {
        "final_height_m": h,
        "height_err_mm": (h - STAND_H) * 1000,
        "max_tilt_deg": math.degrees(max_tilt),
        "contacts": sum(robot.foot_contacts().values()),
        "support_margin_mm": (robot.support_margin() or 0) * 1000,
    }, rec


def pose_sweep(robot):
    """Sinusoidal roll, pitch, yaw, height with feet planted — body-pose IK
    tracking, the core posture-control primitive."""
    rec = Recorder()
    feet = _spawn_standing(robot)
    cmd_log = {"roll": [], "pitch": [], "yaw": []}
    axes = [("roll", 0.15, 0), ("pitch", 0.12, 1), ("yaw", 0.15, 2)]
    for name, amp, idx in axes:
        for i in range(120):
            ang = amp * math.sin(2 * math.pi * i / 120)
            rpy = [0.0, 0.0, 0.0]
            rpy[idx] = ang
            _drive(robot, rec, feet, (0, 0, STAND_H), tuple(rpy), 1 / 30)
            for k in cmd_log:
                cmd_log[k].append(ang if k == name else 0.0)
    # height bob
    for i in range(120):
        h = STAND_H + 0.02 * math.sin(2 * math.pi * i / 120)
        _drive(robot, rec, feet, (0, 0, h), (0, 0, 0), 1 / 30)
        for k in cmd_log:
            cmd_log[k].append(0.0)

    n = len(cmd_log["roll"])
    achieved = {k: rec.col(k)[-n:] for k in ("roll", "pitch", "yaw")}
    rms = {}
    for k in ("roll", "pitch", "yaw"):
        errs = [a - c for a, c in zip(achieved[k], cmd_log[k])]
        rms[k] = math.sqrt(sum(e * e for e in errs) / len(errs))
    margins = [m for m in rec.col("margin") if not math.isnan(m)]
    slip = _toe_slip(feet, robot)
    return {
        "rms_roll_track_deg": math.degrees(rms["roll"]),
        "rms_pitch_track_deg": math.degrees(rms["pitch"]),
        "rms_yaw_track_deg": math.degrees(rms["yaw"]),
        "min_support_margin_mm": min(margins) * 1000,
        "max_toe_slip_mm": slip,
        "contacts": sum(robot.foot_contacts().values()),
    }, rec


def weight_shift_lift(robot):
    """The static-walk primitive: shift the body over the RR-FR-RL support
    triangle, lift FL 40 mm, hold, set down, recenter. Open-loop."""
    rec = Recorder()
    feet = dict(_spawn_standing(robot))
    shift = (-0.030, -0.035)   # toward the diagonal of the support triangle
    # 1) shift weight
    for i in range(1, 31):
        xyz = (shift[0] * i / 30, shift[1] * i / 30, STAND_H)
        _drive(robot, rec, feet, xyz, (0, 0, 0), 1 / 30)
    # 2) lift FL
    fl0 = feet["FL"]
    for i in range(1, 31):
        feet["FL"] = (fl0[0], fl0[1], 0.040 * i / 30)
        _drive(robot, rec, feet, (shift[0], shift[1], STAND_H), (0, 0, 0), 1 / 30)
    margins_lift = [m for m in rec.rows[-30:] if not math.isnan(m[5])]
    # 3) hold
    _drive(robot, rec, feet, (shift[0], shift[1], STAND_H), (0, 0, 0), 1.0)
    # 4) down + recenter
    for i in range(29, -1, -1):
        feet["FL"] = (fl0[0], fl0[1], 0.040 * i / 30)
        _drive(robot, rec, feet, (shift[0], shift[1], STAND_H), (0, 0, 0), 1 / 30)
    for i in range(29, -1, -1):
        xyz = (shift[0] * i / 30, shift[1] * i / 30, STAND_H)
        _drive(robot, rec, feet, xyz, (0, 0, 0), 1 / 30)
    robot.step(0.5)
    rec.sample(robot)

    (x, y, h), (r, p_, yw) = robot.body_state()
    hold_margins = [row[5] for row in rec.rows if row[6] == 3 and not math.isnan(row[5])]
    return {
        "survived": h > 0.10 and abs(math.degrees(r)) < 10 and abs(math.degrees(p_)) < 10,
        "final_height_m": h,
        "min_margin_3leg_mm": (min(hold_margins) * 1000) if hold_margins else None,
        "max_tilt_deg": math.degrees(max(max(abs(v) for v in rec.col("roll")),
                                         max(abs(v) for v in rec.col("pitch")))),
        "contacts_final": sum(robot.foot_contacts().values()),
    }, rec


def _toe_slip(feet_cmd, robot):
    """Max planted-toe drift from its commanded spot (mm)."""
    worst = 0.0
    toes = robot.toe_positions()
    for leg in LEGS:
        cx, cy, cz = feet_cmd[leg]
        if cz > 0.001:
            continue
        tx, ty, _ = toes[leg]
        worst = max(worst, math.hypot(tx - cx, ty - cy) * 1000)
    return worst


SCENARIOS = {
    "settle": settle,
    "stand_up": stand_up,
    "pose_sweep": pose_sweep,
    "weight_shift": weight_shift_lift,
}
