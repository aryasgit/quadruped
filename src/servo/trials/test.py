from pca9685 import PCA9685

# ==============================
# CONFIG
# ==============================
BUS = 7
ADDR = 0x40
FREQ = 50

PWM_MIN = 80
PWM_MAX = 561

ANGLE_MIN = 0
ANGLE_MAX = 270

NUM_SERVOS = 12

# ==============================
# INIT
# ==============================
pca = PCA9685(bus=BUS, addr=ADDR, freq=FREQ)

def angle_to_pwm(angle):
    """
    Map 0–270 degrees to PWM_MIN–PWM_MAX
    """
    angle = max(ANGLE_MIN, min(ANGLE_MAX, angle))
    pwm = PWM_MIN + (angle / ANGLE_MAX) * (PWM_MAX - PWM_MIN)
    return int(pwm)

print("=== Manual Servo Test (270° Servos) ===")
print("Enter servo channel (0–11) and angle (0–270)")
print("Type 'q' to quit\n")

# ==============================
# LOOP
# ==============================
while True:
    ch = input("Servo channel (0-11): ")

    if ch.lower() == 'q':
        print("Exiting.")
        break

    if not ch.isdigit():
        print("Invalid channel.\n")
        continue

    ch = int(ch)
    if ch < 0 or ch >= NUM_SERVOS:
        print("Channel out of range.\n")
        continue

    ang = input("Angle (0-270): ")

    if ang.lower() == 'q':
        print("Exiting.")
        break

    try:
        ang = float(ang)
    except ValueError:
        print("Invalid angle.\n")
        continue

    pwm = angle_to_pwm(ang)
    pca.set_pwm(ch, 0, pwm)

    print(f"→ Servo {ch} set to {ang}° (PWM={pwm})\n")

