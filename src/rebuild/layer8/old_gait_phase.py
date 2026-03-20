import time
import math


class GaitPhase:
    def __init__(self, swing_duration=0.9, lift_height=0.015):
        self.swing_duration = swing_duration
        self.lift_height = lift_height
        self._t0 = None
        self.phase = 0.0

    # ---- EXPLICIT START API ----
    def start(self):
        self._t0 = time.time()
        self.phase = 0.0

    def reset(self):
        self._t0 = None
        self.phase = 0.0

    def update(self):
        if self._t0 is None:
            return 0.0, False

        dt = time.time() - self._t0
        self.phase = min(dt / self.swing_duration, 1.0)
        done = self.phase >= 1.0
        return self.phase, done

    def lift(self):
        return self.lift_height * math.sin(math.pi * self.phase)
