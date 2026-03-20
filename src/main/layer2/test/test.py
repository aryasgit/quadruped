#layer2/test.py
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import (
    WRISTS, THIGHS, COXA,
    WRIST_STAND, THIGH_STAND, COXA_STAND,
)

set_servo_angle(WRISTS["WFL"], WRIST_STAND["WFL"])
set_servo_angle(WRISTS["WFR"], WRIST_STAND["WFR"])
set_servo_angle(WRISTS["WRL"], WRIST_STAND["WRL"])
set_servo_angle(WRISTS["WRR"], WRIST_STAND["WRR"])
set_servo_angle(THIGHS["TFL"], THIGH_STAND["TFL"])
set_servo_angle(THIGHS["TFR"], THIGH_STAND["TFR"])
set_servo_angle(THIGHS["TRL"], THIGH_STAND["TRL"])
set_servo_angle(THIGHS["TRR"], THIGH_STAND["TRR"])
set_servo_angle(COXA["FL"], COXA_STAND["FL"])
set_servo_angle(COXA["FR"], COXA_STAND["FR"])
set_servo_angle(COXA["RL"], COXA_STAND["RL"])
set_servo_angle(COXA["RR"], COXA_STAND["RR"])
