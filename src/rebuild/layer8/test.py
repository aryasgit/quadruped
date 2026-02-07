"""
TEST — Layer 8 Gait Phase Supervisor
===================================

Verifies:
- Correct STANCE / SWING classification
- Swing legs are frozen
- Stance legs receive posture-modified targets
"""

import time
import math

from layer8.gait_phase import GaitPhaseSupervisor
from layer6.gait_generator import generate_foot_targets


# ---------------- CONFIG ----------------

FREQ = 0.5          # Hz
DT   = 0.1          # seconds

PHASE_OFFSET = {
    "FL": 0.00,
    "FR": 0.50,
    "RL": 0.50,
    "RR": 0.00,
}

DUTY = 0.60


# ---------------- INIT ----------------

supervisor = GaitPhaseSupervisor(
    phase_offset=PHASE_OFFSET,
    duty_factor=DUTY,
)


# Fake posture: exaggerate Z to make effect obvious
def fake_posture(nominal):
    out = {}
    for leg, (x, y, z) in nominal.items():
        out[leg] = (x, y, z + 0.03)
    return out


print("\n[TEST] Layer 8 gait phase supervision\n")

t = 0.0
try:
    while True:
        nominal = generate_foot_targets(
            t=t,
            frequency=FREQ,
        )

        posture = fake_posture(nominal)

        final, states = supervisor.step(
            t=t,
            frequency=FREQ,
            nominal_targets=nominal,
            posture_targets=posture,
        )

        print(f"\nTime {t:5.2f}s")
        for leg in ("FL", "FR", "RL", "RR"):
            nz = nominal[leg][2]
            fz = final[leg][2]
            print(
                f"{leg} | {states[leg]:6s} | "
                f"z_nom={nz:+.3f} → z_out={fz:+.3f}"
            )

        t += DT
        time.sleep(DT)

except KeyboardInterrupt:
    print("\n[TEST] Stopped.")
