"""
CONTROLLER INTEGRATION PATCH
=============================
Paste these blocks into controller.py in the places marked.
Search for each FIND: comment to locate the insertion point.
"""

# ─────────────────────────────────────────────────────────────────
# BLOCK A — Imports (add after existing imports)
# ─────────────────────────────────────────────────────────────────
from arya.perception import (
    OakDCamera, SORTTracker,
    WorldModel, BehaviourFSM, AvoidancePlanner,
    Telemetry, telemetry_push,
    apply_gait_command,
)

# ─────────────────────────────────────────────────────────────────
# BLOCK B — Initialisation (add inside main(), before the while loop)
# ─────────────────────────────────────────────────────────────────
cam      = OakDCamera(verbose=False)
tracker  = SORTTracker(iou_threshold=0.25, max_misses=5, min_hits=2)
world_m  = WorldModel(dt=DT)
fsm      = BehaviourFSM()
planner  = AvoidancePlanner()
tel      = Telemetry(port=5000)

cam.start()
tel.start()
print("[PERC] Perception pipeline started")

# ─────────────────────────────────────────────────────────────────
# BLOCK C — Perception tick (add at TOP of the main while loop,
#           before get_key_blocking)
# ─────────────────────────────────────────────────────────────────
frame, raw_dets = cam.get()
tracks          = tracker.update(raw_dets)
world_state     = world_m.update(tracks)
fsm_out         = fsm.update(world_state)
gait_cmd        = planner.plan(fsm_out, world_state)

telemetry_push(
    frame      = frame,
    detections = raw_dets,
    tracks     = tracks,
    world      = world_state,
    fsm        = fsm_out,
    cmd        = gait_cmd,
)

# ─────────────────────────────────────────────────────────────────
# BLOCK D — Gait command injection (FIND: execute_single_cycle call)
#
# BEFORE:
#     execute_single_cycle(command, current_z)
#
# AFTER:
# ─────────────────────────────────────────────────────────────────
effective_dir, eff_step, eff_height, eff_freq = apply_gait_command(
    cmd               = gait_cmd,
    operator_direction = command,
    step_length        = STEP_LENGTH,
    step_height        = STEP_HEIGHT,
    freq               = FREQ,
)

if effective_dir is not None:
    # Temporarily override globals for this cycle
    _orig_step, _orig_h, _orig_freq = STEP_LENGTH, STEP_HEIGHT, FREQ
    STEP_LENGTH  = eff_step
    STEP_HEIGHT  = eff_height
    FREQ         = eff_freq
    execute_single_cycle(effective_dir, current_z)
    STEP_LENGTH, STEP_HEIGHT, FREQ = _orig_step, _orig_h, _orig_freq
else:
    # Planner says STOP — hold stance
    execute_step(STAND_FEET)

# ─────────────────────────────────────────────────────────────────
# BLOCK E — Cleanup (add in finally block alongside existing shutdown)
# ─────────────────────────────────────────────────────────────────
cam.stop()
print("[PERC] Camera stopped")