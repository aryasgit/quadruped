import time
import math

class BraceReflex:
    def _init_(
        self,
        accel_threshold=3000,
        brace_duration=0.25,
        min_scale=0.35,
        max_bias_deg=15.0
    ):
        self.accel_threshold = accel_threshold
        self.brace_duration = brace_duration
        self.min_scale = min_scale
        self.max_bias_deg = max_bias_deg

        self._last_a = None
        self._brace_until = 0.0

        self.roll_bias = 0.0
        self.pitch_bias = 0.0

    def update(self, ax, ay, az):
        now = time.time()

        if self._last_a is None:
            self._last_a = (ax, ay, az)
            return False, 1.0, 0.0, 0.0

        dax = ax - self._last_a[0]
        day = ay - self._last_a[1]
        daz = az - self._last_a[2]
        self._last_a = (ax, ay, az)

        delta = math.sqrt(dax*dax + day*day + daz*daz)

        if delta > self.accel_threshold:
            self._brace_until = now + self.brace_duration

            mag = max(delta, 1.0)
            self.pitch_bias = -(dax / mag) * self.max_bias_deg
            self.roll_bias  = -(day / mag) * self.max_bias_deg

        brace_active = now < self._brace_until
        scale = self.min_scale if brace_active else 1.0

        if not brace_active:
            self.roll_bias = 0.0
            self.pitch_bias = 0.0

        return brace_active, scale, self.roll_bias, self.pitch_bias
