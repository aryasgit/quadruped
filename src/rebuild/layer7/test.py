"""
TEST — Layer 7 Leg FSM
=====================

This test:
- Runs the leg FSM
- Prints which leg is in SWING
- Verifies clean sequencing (one leg at a time)

NO IK
NO hardware
"""

import time
from layer7.leg_fsm import LegFSM, LegState


DT = 0.05  # 20 Hz
SWING_TIME = 0.4  # seconds per leg

fsm = LegFSM(swing_time=SWING_TIME)

print("[TEST] Leg FSM running — CTRL+C to stop\n")

try:
    while True:
        states = fsm.update(DT)

        swing_legs = [leg for leg, s in states.items() if s == LegState.SWING]
        stance_legs = [leg for leg, s in states.items() if s == LegState.STANCE]

        print(
            f"SWING: {swing_legs} | STANCE: {stance_legs}"
        )

        time.sleep(DT)

except KeyboardInterrupt:
    print("\n[TEST] FSM test stopped cleanly.")
