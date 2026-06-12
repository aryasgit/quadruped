"""
PyBullet world + robot wrapper (D-008).
=======================================

Owns: xacro expansion, world setup, joint indexing, the open-loop command
interface, and ground-truth probes (body state, contacts, COM, support
margin). Ground truth is for METRICS ONLY — controllers must not consume
it (D-009).
"""

import math
import sys
import time
from pathlib import Path

STACK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STACK_ROOT))

import pybullet as p
import xacro

from barq1.kinematics import URDF_LEG_NAME, LEGS
from sim.servo_model import command_position

XACRO_PATH = STACK_ROOT / "urdf" / "barq_v1.urdf.xacro"
URDF_PATH = STACK_ROOT / "urdf" / "barq_v1.generated.urdf"  # gitignored

TIME_STEP = 1.0 / 240.0


def _strip_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
        else:
            _strip_comments(child)


def _expand_urdf():
    """xacro -> plain URDF, written next to stl/ so relative meshes resolve.

    Comments are stripped: Bullet's bundled XML parser trips over the
    non-ASCII characters in the xacro header; pretty-printing keeps the
    generated file diffable/debuggable."""
    doc = xacro.process_file(str(XACRO_PATH))
    _strip_comments(doc)
    pretty = doc.toprettyxml(indent="  ")
    URDF_PATH.write_text("\n".join(l for l in pretty.splitlines() if l.strip()))
    return URDF_PATH


class SimRobot:
    """The BARQ v1 robot in a PyBullet world, driven open-loop."""

    def __init__(self, gui=False, start_height=0.18):
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(TIME_STEP)
        p.setPhysicsEngineParameter(numSolverIterations=60)

        plane = p.createCollisionShape(p.GEOM_PLANE)
        self.ground = p.createMultiBody(0, plane)
        p.changeDynamics(self.ground, -1, lateralFriction=1.0)

        self.robot = p.loadURDF(
            str(_expand_urdf()),
            basePosition=(0, 0, start_height),
            flags=p.URDF_USE_INERTIA_FROM_FILE,
        )

        # Joint / link maps
        self.joint_index = {}
        self.toe_link = {}
        for i in range(p.getNumJoints(self.robot)):
            info = p.getJointInfo(self.robot, i)
            name = info[1].decode()
            if info[2] == p.JOINT_REVOLUTE:
                self.joint_index[name] = i
            if name.endswith("_toe"):
                leg = {v: k for k, v in URDF_LEG_NAME.items()}[name[:-4]]
                self.toe_link[leg] = i
                p.changeDynamics(self.robot, i, lateralFriction=1.1,
                                 spinningFriction=0.05, rollingFriction=0.01)

        assert len(self.joint_index) == 12, sorted(self.joint_index)
        assert len(self.toe_link) == 4

        self.t = 0.0

    # -- commanding (the only channel hardware has) --------------------------

    def command(self, urdf_joint_targets):
        """{urdf_joint_name: angle_rad} — like writing PWM, nothing returned."""
        for name, target in urdf_joint_targets.items():
            command_position(p, self.robot, self.joint_index[name], target)

    def teleport_joints(self, urdf_joint_targets):
        """Set joint states instantly (spawn/reset only — not physical)."""
        for name, target in urdf_joint_targets.items():
            p.resetJointState(self.robot, self.joint_index[name], target)

    def step(self, seconds, realtime=False):
        n = max(1, int(round(seconds / TIME_STEP)))
        for _ in range(n):
            p.stepSimulation()
            self.t += TIME_STEP
            if realtime:
                time.sleep(TIME_STEP)

    # -- ground-truth probes (metrics only, never control: D-009) ------------

    def body_state(self):
        pos, quat = p.getBasePositionAndOrientation(self.robot)
        rpy = p.getEulerFromQuaternion(quat)
        return pos, rpy

    def foot_contacts(self):
        """{leg: bool in contact with ground}."""
        out = {}
        for leg, link in self.toe_link.items():
            pts = p.getContactPoints(bodyA=self.robot, bodyB=self.ground,
                                     linkIndexA=link)
            out[leg] = len(pts) > 0
        return out

    def toe_positions(self):
        """{leg: world xyz of toe link origin}."""
        out = {}
        for leg, link in self.toe_link.items():
            out[leg] = p.getLinkState(self.robot, link)[0]
        return out

    def com(self):
        """Whole-robot center of mass (world). PyBullet's base position and
        getLinkState()[0] are already COM positions, so this is a plain
        mass-weighted average."""
        base_m = p.getDynamicsInfo(self.robot, -1)[0]
        base_com = p.getBasePositionAndOrientation(self.robot)[0]
        mass = base_m
        acc = [base_m * c for c in base_com]
        for i in range(p.getNumJoints(self.robot)):
            m = p.getDynamicsInfo(self.robot, i)[0]
            if m <= 0:
                continue
            com_i = p.getLinkState(self.robot, i)[0]
            for k in range(3):
                acc[k] += m * com_i[k]
            mass += m
        return tuple(a / mass for a in acc), mass

    def support_margin(self):
        """Min distance (m) from the COM ground-projection to the support
        polygon boundary. Positive = inside (stable), negative = outside.
        None if fewer than 3 feet are in contact."""
        contacts = self.foot_contacts()
        pts = [self.toe_positions()[leg][:2] for leg in LEGS if contacts[leg]]
        if len(pts) < 3:
            return None
        (cx, cy, _), _ = self.com()
        return _polygon_margin(pts, (cx, cy))


def _polygon_margin(pts, q):
    """Signed distance from q to the convex polygon through pts (2D)."""
    cx = sum(x for x, _ in pts) / len(pts)
    cy = sum(y for _, y in pts) / len(pts)
    ordered = sorted(pts, key=lambda v: math.atan2(v[1] - cy, v[0] - cx))
    inside = True
    dmin = float("inf")
    n = len(ordered)
    for i in range(n):
        ax, ay = ordered[i]
        bx, by = ordered[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        # cross > 0 -> q left of edge (ccw polygon)
        cross = ex * (q[1] - ay) - ey * (q[0] - ax)
        if cross < 0:
            inside = False
        # point-segment distance
        L2 = ex * ex + ey * ey
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((q[0] - ax) * ex + (q[1] - ay) * ey) / L2))
        px, py = ax + t * ex, ay + t * ey
        dmin = min(dmin, math.hypot(q[0] - px, q[1] - py))
    return dmin if inside else -dmin


def save_snapshot(path, yaw=50, pitch=-25, distance=0.65, target=(0, 0, 0.10),
                  width=640, height=480):
    """Software-rendered camera image -> binary PPM (no GPU/GL needed)."""
    view = p.computeViewMatrixFromYawPitchRoll(target, distance, yaw, pitch, 0, 2)
    proj = p.computeProjectionMatrixFOV(60, width / height, 0.01, 5.0)
    _, _, rgba, _, _ = p.getCameraImage(width, height, view, proj,
                                        renderer=p.ER_TINY_RENDERER)
    import numpy as np
    img = np.reshape(np.asarray(rgba, dtype=np.uint8), (height, width, 4))
    with open(path, "wb") as f:
        f.write(f"P6 {width} {height} 255\n".encode())
        f.write(img[:, :, :3].tobytes())
