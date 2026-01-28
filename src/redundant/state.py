# state.py
#
# Authoritative robot state estimator
# NO control, NO actuation, NO decisions

import math
from dataclasses import dataclass
from balance_controller import BalanceController
from com_mapper import com_to_leg_xyz
from actuators import apply_leg_xyz
from imu import IMUFilter


# ==================================================
# FOOT GEOMETRY (BODY FRAME, METERS)
# ==================================================
# +X forward, +Y left, Z up

FOOT_POSITIONS = {
    "FL": (-0.12, -0.08),
    "FR": (-0.12, +0.08),
    "RL": (+0.11, -0.08),
    "RR": (+0.11, +0.08),
}

BODY_HEIGHT = 0.24  # meters (measured)

@dataclass
class RobotState:
    roll: float
    pitch: float

    com_x: float
    com_y: float

    load: dict          # per-foot load share
    stability: float    # 0–1

class StateEstimator:
    def __init__(self, imu: IMUFilter):
        self.imu = imu

    def compute_com_projection(self):
        roll  = math.radians(self.imu.roll)
        pitch = math.radians(self.imu.pitch)

        # Physical COM shift on ground plane
        com_x = BODY_HEIGHT * math.tan(pitch)
        com_y = -BODY_HEIGHT * math.tan(roll)

        return com_x, com_y


    def compute_load_distribution(self, com_x, com_y):
        inv_dist = {}
        total = 0.0

        for leg, (fx, fy) in FOOT_POSITIONS.items():
            d = math.hypot(com_x - fx, com_y - fy)
            w = 1.0 / max(d, 1e-3)
            inv_dist[leg] = w
            total += w

        load = {k: v / total for k, v in inv_dist.items()}
        return load

    def compute_stability(self, com_x, com_y):
        max_extent = max(
            math.hypot(fx, fy) for fx, fy in FOOT_POSITIONS.values()
        )
        dist = math.hypot(com_x, com_y)

        stability = 1.0 - dist / max_extent
        return max(0.0, min(1.0, stability))

    def update(self) -> RobotState:
        com_x, com_y = self.compute_com_projection()
        load = self.compute_load_distribution(com_x, com_y)
        stability = self.compute_stability(com_x, com_y)

        return RobotState(
            roll=self.imu.roll,
            pitch=self.imu.pitch,
            com_x=com_x,
            com_y=com_y,
            load=load,
            stability=stability,
        )

# ===============================
# CLI / DIAGNOSTIC ENTRY
# ===============================
if __name__ == "__main__":
    import multiprocessing as mp
    from smbus2 import SMBus
    from absolute_truths import BUS
    from imu import (
        init_mpu,
        calibrate_imu,
        IMUFilter,
        lock_stand_reference,
    )

    bus = SMBus(BUS)

    init_mpu(bus)
    calib = calibrate_imu(bus)
    imu = IMUFilter(calib)
    lock_stand_reference(imu, bus)

    estimator = StateEstimator(imu)
    
    # Balance controller (stateless externally; retains internal smoothing)
    bc = BalanceController()

    # OPTIONAL: spawn com_plotter process if you want live GUI (requires com_plotter to be usable)
    try:
        import multiprocessing as mp
        from com_plotter import com_plotter_main
        state_q = mp.Queue(maxsize=50)
        plot_proc = mp.Process(target=com_plotter_main, args=(state_q,), daemon=True)
        plot_proc.start()
        plot_enabled = True
    except Exception:
        plot_enabled = False


    print("[STATE] estimator live\n")
    
    while True:
        roll, pitch, _, _ = imu.update(bus)

        state = estimator.update()

        # ---- BALANCE (COM SPACE ONLY) ----
        com_cmd = bc.compute(state)
        # ---- COM → LEG XYZ ----
        leg_cmd = com_to_leg_xyz(com_cmd.dx, com_cmd.dy)
        # ---- RATE LIMIT (SAFETY / VISIBILITY) ----
        MAX_DZ = 0.04   # meters
        MAX_DY = 0.05

        for v in leg_cmd.values():
            v["dz"] = max(-MAX_DZ, min(MAX_DZ, v["dz"]))
            v["dy"] = max(-MAX_DY, min(MAX_DY, v["dy"]))

        # ---- ACTUATE (ALL DOFs) ----
        apply_leg_xyz(leg_cmd)



        # ---- DEBUG / DIAGNOSTIC PRINT ----
        print(
            f"COM cmd dx={com_cmd.dx:+.3f} "
            f"dy={com_cmd.dy:+.3f} | "
            f"state COM ({state.com_x:+.3f}, {state.com_y:+.3f}) "
            f"stab={state.stability:.2f}"
        )

        # ---- PLOTTER FEED (READ-ONLY) ----
        if plot_enabled:
            try:
                state_q.put_nowait({
                    "com_x": state.com_x,
                    "com_y": state.com_y,
                    "stability": state.stability,
                    "cmd_dx": com_cmd.dx,
                    "cmd_dy": com_cmd.dy,
                })
            except Exception:
                pass

        time.sleep(0.1)



