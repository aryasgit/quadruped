# quick_test.py (temporary)
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import WRISTS

set_servo_angle(WRISTS["WFL"], 78)
