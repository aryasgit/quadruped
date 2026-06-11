"""BARQ v1 — from-scratch control stack (revival, 2026).

Layered like the legacy stack that worked, rebuilt clean:

  Layer 0   barq1.truths    measured electrical/mechanical facts (constants only)
  Layer 0.5 barq1.servos    derived per-servo table (pure, no I/O)
  Layer 1   barq1.pca9685   PCA9685 driver over smbus2 (ticks canonical)

Reference research: mike4192/spotMicro (URDF, kinematics, calibration method).
"""

__version__ = "0.1.0"
