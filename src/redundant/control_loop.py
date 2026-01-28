# control_loop.py
#
# Minimal glue layer:
# COMCommand → leg dz → actuators
# NO estimation
# NO balance logic

import time
from smbus2 import SMBus

from absolute_truths import BUS
from imu import init_mpu, calibrate_imu, IMUFilter, lock_stand_reference
from state import StateEstimator
from balance_controller import BalanceController
from com_mapper import com_to_leg_dz
from actuators import init_pca, go_stand, apply_leg_dz


def main():
    bus = SMBus(BUS)

    # ---------- INIT ----------
    init_pca()
    init_mpu(bus)

    calib = calibrate_imu(bus)
    imu = IMUFilter(calib)
    lock_stand_reference(imu, bus)

    go_stand()

    estimator = StateEstimator(imu)
    bc = BalanceController()

    print("[CTRL] COM-based balance active\n")

    # ---------- MAIN LOOP ----------
    while True:
        imu.update(bus)
        state = estimator.update()

        # 1) Balance → COM
        com_cmd = bc.compute(state)

        # 2) COM → leg dz
        dz = com_to_leg_dz(com_cmd)

        # 3) Apply to hardware
        apply_leg_dz(dz)

        # 4) Debug
        print(
            f"dx={com_cmd.dx:+.3f} dy={com_cmd.dy:+.3f} | "
            f"dz={{{', '.join(f'{k}:{v:+.1f}' for k,v in dz.items())}}}"
        )

        time.sleep(0.05)  # 20 Hz


if __name__ == "__main__":
    main()
