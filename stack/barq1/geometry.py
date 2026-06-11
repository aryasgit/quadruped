"""
Layer 0 — ROBOT GEOMETRY (kinematic wireframe)
==============================================

Joint-axis-to-joint-axis dimensions of BARQ v1, in meters. This is what
the IK layer must use — shell dimensions are cosmetic.

Decision D6 (2026-06-11, stack/docs/RESEARCH_LOG.md): BARQ v1 is a
spotMicro stretched between the coxa shafts to house the Jetson; legs and
shoulder modules are stock spotMicro prints. spotMicro values are trusted
for everything stock; the axle span is the measured BARQ value (CAD, rear
coxa shaft centre to front coxa shaft centre, confirmed by Aryaman).

Constants only — no logic, no imports.
"""

# Longitudinal coxa-shaft span. BARQ measured (spotMicro: 0.186).
BODY_LENGTH = 0.2075

# Lateral hip-shaft span. Matches spotMicro exactly (measured 78 = 78).
BODY_WIDTH = 0.078

# Coxa axis -> thigh axis lateral offset (stock spotMicro 'shift').
HIP_LINK = 0.055

# Thigh axis -> knee axis (stock spotMicro print).
UPPER_LEG = 0.1075

# Knee axis -> toe reference point (stock spotMicro print).
LOWER_LEG = 0.130

# Shell envelope (clearance/display only; never used by IK).
SHELL_LENGTH = 0.3424   # rendered cover-to-cover extent of the URDF
SHELL_WIDTH = 0.110
SHELL_HEIGHT = 0.070
