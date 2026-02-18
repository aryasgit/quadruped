# layer7/demo_main.py

import time
from layer7.demo import execute, bow, bounce, tail_wag, handshake, sit

MODE_DURATION = 6.0

print("Layer7 Demo Engine")
print("Modes: BOW → BOUNCE → TAIL → SHAKE → SIT")
print("CTRL+C to stop")

t0 = time.time()

try:
    while True:
        t = time.time() - t0
        phase = int(t // MODE_DURATION) % 5
        local_t = t % MODE_DURATION

        if phase == 0:
            feet = bow(local_t)
            execute(feet)

        elif phase == 1:
            feet = bounce(local_t)
            execute(feet)

        elif phase == 2:
            tail_wag(local_t)

        elif phase == 3:
            feet = handshake(local_t)
            execute(feet)

        elif phase == 4:
            progress = min(1.0, local_t / MODE_DURATION)
            feet = sit(progress)
            execute(feet)

        time.sleep(0.02)

except KeyboardInterrupt:
    print("Demo stopped.")
