#ik/util.py
from math import atan2, pi, radians, cos, sin
import numpy as np


def point_to_rad(p1, p2):
    """
    Angle from the +p1 axis toward the +p2 axis, mapped to [0, 2pi).
    Equivalent to atan2(p2, p1) % (2*pi).
    (0, 0) is geometrically undefined — returns 0 by atan2 convention.
    """
    return atan2(p2, p1) % (2 * pi)


def RotMatrix3D(rotation=[0,0,0], is_radians=True):
    """
    Extrinsic ZYX rotation matrix (yaw × pitch × roll).
    rotation: [roll, pitch, yaw]
    """
    roll, pitch, yaw = rotation

    if not is_radians:
        roll  = radians(roll)
        pitch = radians(pitch)
        yaw   = radians(yaw)

    rotX = np.matrix([[1,          0,           0],
                      [0, cos(roll), -sin(roll)],
                      [0, sin(roll),  cos(roll)]])
    rotY = np.matrix([[ cos(pitch), 0, sin(pitch)],
                      [          0, 1,          0],
                      [-sin(pitch), 0, cos(pitch)]])
    rotZ = np.matrix([[cos(yaw), -sin(yaw), 0],
                      [sin(yaw),  cos(yaw), 0],
                      [       0,         0, 1]])

    return rotZ * rotY * rotX
