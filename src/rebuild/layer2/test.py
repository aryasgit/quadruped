#layer2/test.py
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import WRISTS, THIGHS, COXA

set_servo_angle(WRISTS["WFL"], 80)
set_servo_angle(WRISTS["WFR"], 123)
set_servo_angle(WRISTS["WRL"], 76)
set_servo_angle(WRISTS["WRR"], 125)
set_servo_angle(THIGHS["TFL"], 166)
set_servo_angle(THIGHS["TFR"], 99)
set_servo_angle(THIGHS["TRL"], 174)
set_servo_angle(THIGHS["TRR"], 101)
set_servo_angle(COXA["FL"], 43)
set_servo_angle(COXA["FR"], 46)
set_servo_angle(COXA["RL"], 45)
set_servo_angle(COXA["RR"], 46)
