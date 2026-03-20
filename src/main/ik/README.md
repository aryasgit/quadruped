# ik/ — Layer 3: Inverse Kinematics

> Pure geometry. No hardware. No balance. Converts hip-local foot positions into joint angles.

---

## Files

| File | Responsibility |
|---|---|
| `kinematics.py` | Core 3D IK engine — SpotMicro exact geometric solver |
| `solver.py` | Public IK interface, stand reference, multi-leg solve |
| `util.py` | Rotation matrix (ZYX extrinsic), `point_to_rad` |

---

## Coordinate System

All foot targets are expressed in **hip-local coordinates** — origin at the coxa joint (J1) of that leg.

```
     X (forward)
     │
     │
     └──── Y (left)
    /
   Z (up)
```

- Positive X = forward
- Positive Y = left (outward for left legs, inward for right legs)
- Positive Z = up (negative Z = below hip = standing)
- Standing foot is at approximately `(0.0, ±0.07, -0.18)` in hip-local coords

---

## kinematics.py — Core Engine

3-link planar IK with a hip abductor joint, adapted from the SpotMicro geometry.

### Joint Names

| Code | Physical Joint | DoF |
|---|---|---|
| J1 (theta_1) | Coxa — hip abductor | Rotation about X-axis (lateral splay) |
| J2 (theta_2) | Thigh — hip flexor | Rotation in sagittal plane |
| J3 (theta_3) | Wrist — knee | Rotation in sagittal plane |

### Link Lengths

```python
link_1 = 0.01605   # coxa (hip abductor arm)
link_2 = 0.10832   # thigh
link_3 = 0.13476   # shin (wrist)
```

### `leg_IK(xyz, legID, rot, is_radians, center_offset)`

Full leg solve including body rotation offset:

```python
from ik.kinematics import kinematics

ik = kinematics()
t1, t2, t3, j1, j2, j3, j4 = ik.leg_IK(
    xyz=[0.0, 0.07, -0.18],
    legID=0,          # 0=FL, 1=RL, 2=FR, 3=RR
    is_radians=True
)
```

Returns `[theta_1, theta_2, theta_3, j1_coords, j2_coords, j3_coords, j4_coords]`.

Joint coordinates `j1`–`j4` are in leg-local space, useful for visualisation.

### `angle_corrector(angles, is_right)`

Applies per-robot mechanical offsets to map from IK math angles to the convention expected by the servo mount. Applied internally inside `leg_IK_calc`.

**Do not call directly.** All corrections are in `kinematics.py` and `joints/conventions.py`.

The corrections encode:
- `theta_1`: subtract π (right) or wrap (left) to map to servo zero
- `theta_2`: subtract 1.5π then add ±45° offset for thigh mount angle
- `theta_3`: negate and add 45° for wrist mount angle

---

## solver.py — Public Interface

Wraps `kinematics.py` and exposes a clean API to the rest of the stack.

### Stand Reference

At module load, `solver.py` computes the IK solution for the nominal stand pose and stores it as `_STAND_REF`. All `solve_leg_ik` outputs are **deltas from stand** — zero means "same as standing".

```python
_STAND_Z = -0.18
_STAND_Y =  0.07
_STAND_X =  0.0
```

### `solve_leg_ik(x, y, z, leg) → (coxa_delta, thigh_delta, wrist_delta)`

Solve one leg. Returns degrees. Zero = stand pose.

```python
from ik.solver import solve_leg_ik

dc, dt, dw = solve_leg_ik(x=0.0, y=0.07, z=-0.18, leg="FL")
# dc, dt, dw ≈ 0.0, 0.0, 0.0  (standing pose)
```

### `solve_all_legs(foot_targets) → dict`

Solve all four legs at once.

```python
from ik.solver import solve_all_legs

foot_targets = {
    "FL": (0.0,  0.07, -0.18),
    "FR": (0.0, -0.07, -0.18),
    "RL": (0.0,  0.07, -0.18),
    "RR": (0.0, -0.07, -0.18),
}

deltas = solve_all_legs(foot_targets)
# Returns:
# {
#   "FL_COXA": 0.0, "FL_THIGH": 0.0, "FL_WRIST": 0.0,
#   "FR_COXA": 0.0, "FR_THIGH": 0.0, "FR_WRIST": 0.0,
#   ...
# }
```

---

## util.py

### `RotMatrix3D(rotation, is_radians) → np.matrix`

Extrinsic ZYX rotation matrix (yaw × pitch × roll).

```python
from ik.util import RotMatrix3D
import numpy as np

R = RotMatrix3D([0, 0, 0.1], is_radians=True)   # 0.1 rad yaw
```

### `point_to_rad(p1, p2) → float`

Angle from +p1 axis toward +p2 axis, mapped to `[0, 2π)`. Equivalent to `atan2(p2, p1) % (2π)`.

---

## Leg ID Mapping

| ID | Leg |
|---|---|
| 0 | FL (Front Left) |
| 1 | RL (Rear Left) |
| 2 | FR (Front Right) |
| 3 | RR (Rear Right) |

**Note:** The ID ordering matches the `kinematics.py` leg_origins matrix. The `solver.py` `LEG_ID` dict maps string names to these IDs.

---

## Workspace Limits

The IK clamps automatically if the target is unreachable (beyond `link_2 + link_3`). A warning is printed. For normal walking, foot targets should stay within:

- Z: `-0.05` to `-0.22` (meters from hip)
- Y: `0.04` to `0.12` (lateral, for left legs)
- X: `-0.08` to `+0.08` (fore/aft)

Going outside these ranges risks mechanical binding even if IK succeeds numerically.