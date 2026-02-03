# controller.py
import time

from layer5.posture_controller_gait_only import posture_step
from layer7.leg_fsm import LegFSM
from layer8.gait_phase import GaitPhase
from layer9.swing_trajectory import SwingTrajectory
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import WRISTS, THIGHS, COXA

SERVO_CHANNELS = {
    "FL_COXA": COXA["FL"], "FL_THIGH": THIGHS["TFL"], "FL_WRIST": WRISTS["WFL"],
    "FR_COXA": COXA["FR"], "FR_THIGH": THIGHS["TFR"], "FR_WRIST": WRISTS["WFR"],
    "RR_COXA": COXA["RR"], "RR_THIGH": THIGHS["TRR"], "RR_WRIST": WRISTS["WRR"],
    "RL_COXA": COXA["RL"], "RL_THIGH": THIGHS["TRL"], "RL_WRIST": WRISTS["WRL"],
}

# ------------------------
# INIT
# ------------------------
leg_fsm = LegFSM()
gait = GaitPhase(swing_duration=0.6)
swing = SwingTrajectory(step_length=0.2, step_height=0.140)

foot_contact = {l: True for l in ["FL", "FR", "RL", "RR"]}

# Nominal stance (LOCKED)
STANCE = {
    "FL": ( 0.20,  0.07, -0.18),
    "FR": ( 0.20, -0.07, -0.18),
    "RL": (-0.20,  0.07, -0.18),
    "RR": (-0.20, -0.07, -0.18),
}

# ------------------------
# MAIN LOOP
# ------------------------
while True:

    # ---- Gait timing ----
    if leg_fsm.active_leg and gait._t0 is None:
        gait.start()

    phase, swing_done = gait.update()
    leg_fsm.update(
        allow_leg_lift=True,
        foot_contact=foot_contact,
        swing_done=swing_done,
        load_done=True,
    )


    # ---- Foot targets (Layer 9) ----
    foot_targets = dict(STANCE)

    if leg_fsm.active_leg:
        leg = leg_fsm.active_leg
        dx, dz = swing.sample(phase)

        x, y, z = STANCE[leg]
        foot_targets[leg] = (x + dx, y, z + dz)
        print("SWING", leg, "dx dz =", dx, dz)


        if dz > 0.01:
            foot_contact[leg] = False

    # --- DIAGNOSTIC PRINTS (temporary) ---
    # Print foot_targets in meters
    for leg in ("FL","FR","RL","RR"):
        x,y,z = foot_targets[leg]
        print(f"FT {leg} x={x:.3f} y={y:.3f} z={z:.3f}", end=" | ")
    print()

    # Call IK directly per-leg (if your solve_all_legs wrapper exists, print its output)
    from layer3.leg_ik import solve_all_legs  # ok, already used in posture_step alt
    raw_deltas = solve_all_legs(foot_targets)
    print("RAW IK DELTAS:", {k: round(v,4) for k,v in raw_deltas.items()})

    # If you have apply_joint_conventions now
    from layer2.joint_conventions import apply_joint_conventions
    conv = apply_joint_conventions(raw_deltas)
    print("AFTER_CONV:", {k: round(v,4) for k,v in conv.items()})

    # After any mirroring logic (if present in alt layer5)
    # and after normalize_all (physical map)
    from layer2.joint_space import normalize_all
    phys = normalize_all(conv)
    print("PHYSICAL ANGLES (deg):", {k: round(v,3) for k,v in phys.items()})

    # Print coxa lock if applied
    if "FL_COXA" in phys:
        print("COXA (deg):", phys["FL_COXA"], phys["FR_COXA"], phys["RL_COXA"], phys["RR_COXA"])
    # --- end diagnostics ---

    # ---- Posture + IK (Layer 5) ----
    physical = posture_step(foot_targets)

    # ---- Actuation ----
    for joint, angle in physical.items():
        set_servo_angle(SERVO_CHANNELS[joint], angle)

    # ---- FSM completion ----
    if swing_done and leg_fsm.active_leg:
        foot_contact[leg_fsm.active_leg] = True
        gait.reset()

    time.sleep(0.02)
