from layer7.leg_fsm import LegFSM

fsm = LegFSM()

foot_contact = {
    "FL": True,
    "FR": True,
    "RL": True,
    "RR": True,
}

for step in range(6):
    # simulate contact break AFTER unload
    if fsm.active_leg:
        foot_contact[fsm.active_leg] = False

    fsm.update(
        allow_leg_lift=True,
        foot_contact=foot_contact,
        swing_done=True,
        load_done=True,
    )

    print(f"Step {step}: active={fsm.active_leg}, states={fsm.state}")

    # restore contact once leg finishes
    for k in foot_contact:
        foot_contact[k] = True
