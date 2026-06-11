#!/usr/bin/env python3
"""
Expand stack/urdf/barq_v1.urdf.xacro and verify the kinematic frame
against barq1.geometry. Run after any URDF edit:

    ~/barq_v1/venv/bin/python stack/tools/validate_urdf.py
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

STACK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STACK_ROOT))

import xacro

from barq1 import geometry

XACRO_PATH = STACK_ROOT / "urdf" / "barq_v1.urdf.xacro"
EPS = 1e-9
FAILS = []


def check(label, actual, expected):
    ok = abs(actual - expected) < EPS
    print(f"  {'ok' if ok else 'FAIL':4} {label:48} {actual:.5f} (want {expected:.5f})")
    if not ok:
        FAILS.append(label)


def main():
    doc = xacro.process_file(str(XACRO_PATH))
    robot = ET.fromstring(doc.toxml())

    joints = {j.get("name"): j for j in robot.findall("joint")}
    links = {l.get("name"): l for l in robot.findall("link")}

    def origin(jname):
        return [float(v) for v in joints[jname].find("origin").get("xyz").split()]

    print(f"[urdf] {XACRO_PATH.name}: {len(links)} links, {len(joints)} joints")

    fl = origin("front_left_shoulder")
    rl = origin("rear_left_shoulder")
    fr = origin("front_right_shoulder")
    check("coxa shaft span (front-rear)", fl[0] - rl[0], geometry.BODY_LENGTH)
    check("hip shaft span (left-right)", fl[1] - fr[1], geometry.BODY_WIDTH)
    check("hip link offset (coxa->thigh)", abs(origin("front_left_leg")[1]), geometry.HIP_LINK)
    check("upper leg (thigh->knee)", abs(origin("front_left_foot")[2]), geometry.UPPER_LEG)
    check("lower leg (knee->toe)", abs(origin("front_left_toe")[2]), geometry.LOWER_LEG)

    for pos, sx, sy in (("front_left", 1, 1), ("front_right", 1, -1),
                        ("rear_left", -1, 1), ("rear_right", -1, -1)):
        o = origin(f"{pos}_shoulder")
        check(f"{pos}_shoulder at ({sx:+d}shiftx, {sy:+d}shifty)",
              o[0] * sx + o[1] * sy,
              geometry.BODY_LENGTH / 2 + geometry.BODY_WIDTH / 2)

    body_box = links["base_link"].find("collision/geometry/box").get("size").split()
    check("base_link collision box length", float(body_box[0]), 0.1615)
    check("base_link collision box width", float(body_box[1]), geometry.SHELL_WIDTH)
    check("base_link collision box height", float(body_box[2]), geometry.SHELL_HEIGHT)

    missing = []
    for mesh in robot.iter("mesh"):
        rel = mesh.get("filename")
        if not (XACRO_PATH.parent / rel).exists():
            missing.append(rel)
    print(f"  {'ok' if not missing else 'FAIL':4} all referenced meshes exist"
          + (f" — missing: {sorted(set(missing))}" if missing else
             f" ({len(list(robot.iter('mesh')))} refs)"))
    if missing:
        FAILS.append("meshes")

    if FAILS:
        print(f"[urdf] {len(FAILS)} check(s) FAILED")
        sys.exit(1)
    print("[urdf] all checks passed")


if __name__ == "__main__":
    main()
