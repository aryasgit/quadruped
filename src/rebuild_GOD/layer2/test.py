# quick_test.py (temporary)
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import WRISTS, THIGHS, COXA

set_servo_angle(WRISTS["WFL"], 78)
set_servo_angle(WRISTS["WFR"], 122)
set_servo_angle(WRISTS["WRL"], 78)
set_servo_angle(WRISTS["WRR"], 122)
set_servo_angle(THIGHS["TFL"], 170)
set_servo_angle(THIGHS["TFR"], 100)
set_servo_angle(THIGHS["TRL"], 170)
set_servo_angle(THIGHS["TRR"], 100)
set_servo_angle(COXA["FL"], 45)
set_servo_angle(COXA["FR"], 45)
set_servo_angle(COXA["RL"], 45)
set_servo_angle(COXA["RR"], 45)
