"""
main.py — QUADRUPED BOOT ENTRY POINT
======================================
Entry point for the full BARQ quadruped system.

Sequence:
  1. CLI boot prompt — user selects hardware config (1/2/3)
  2. Hardware detection — probe OAK-D and I2C before any threads
  3. Start perception thread (if camera connected)
  4. Start dashboard server (always)
  5. Start aggregator thread (bridges queues → dashboard)
  6. Start controller main loop (if robot connected, blocks main thread)
  7. If camera-only: spin and serve dashboard until Ctrl+C

Thread map:
  Main thread    → controller (blocking gamepad reads) OR idle spin
  perception     → daemon thread (DepthAI pipeline)
  dashboard      → daemon thread (Flask server)
  aggregator     → daemon thread (merges perception + robot state → dashboard)

Queue map:
  perception_queue  : PerceptionFrame  (perception → aggregator/controller)
  robot_state_queue : RobotState       (controller → aggregator)
"""

import sys
import time
import signal
import queue
import threading

from state import BootMode, DashboardData, ProximityState, ControlMode, IMUSnapshot, StepState


# ── Custom exceptions ────────────────────────────────────────────

class PeripheralError(Exception):
    pass

class RemoteIOError(Exception):
    pass


# ── Boot prompt ──────────────────────────────────────────────────

def boot_prompt() -> BootMode:
    """Display hardware selection prompt. Returns BootMode."""
    print()
    print("=" * 50)
    print("  QUADRUPED BOOT SEQUENCE — HARDWARE CONFIG")
    print("=" * 50)
    print()
    print("  What hardware is connected?")
    print()
    print("    [1] Camera only (OAK-D Pro)")
    print("    [2] Robot only (no camera)")
    print("    [3] Both camera and robot")
    print()

    while True:
        try:
            choice = input("  Enter choice (1/2/3): ").strip()
            if choice == '1':
                return BootMode.CAMERA_ONLY
            elif choice == '2':
                return BootMode.ROBOT_ONLY
            elif choice == '3':
                return BootMode.FULL
            else:
                print("  Invalid. Enter 1, 2, or 3.")
        except (EOFError, KeyboardInterrupt):
            print("\n  Aborted.")
            sys.exit(0)


# ── Hardware detection ───────────────────────────────────────────

def detect_hardware(mode: BootMode):
    """
    Probe connected hardware BEFORE any threads start.
    Raises PeripheralError or RemoteIOError with clean messages.
    """
    print()
    print("-" * 50)
    print("  HARDWARE DETECTION")
    print("-" * 50)

    if mode in (BootMode.CAMERA_ONLY, BootMode.FULL):
        print("[HW] Probing OAK-D Pro...")
        try:
            from perception import check_oakd_available
            check_oakd_available()
        except Exception as e:
            msg = f"PeripheralError: OAK-D Pro not detected. Check USB connection.\n  Detail: {e}"
            if mode == BootMode.FULL:
                raise PeripheralError(msg)
            else:
                raise PeripheralError(msg)

    if mode in (BootMode.ROBOT_ONLY, BootMode.FULL):
        print("[HW] Probing robot I2C bus...")
        try:
            from controller import check_robot_available
            check_robot_available()
        except Exception as e:
            msg = f"RemoteIOError: Robot not responding on I2C. Check connection and power.\n  Detail: {e}"
            if mode == BootMode.FULL:
                raise RemoteIOError(msg)
            else:
                raise RemoteIOError(msg)

    print("[HW] All hardware OK")
    print("-" * 50)
    print()


# ── Aggregator thread ────────────────────────────────────────────

def aggregator_thread(boot_mode: BootMode,
                      perception_queue: queue.Queue,
                      robot_state_queue: queue.Queue,
                      stop_event: threading.Event):
    """
    Merges perception and robot state into a single DashboardData
    object and pushes it to the dashboard module.

    Runs at ~30 Hz. Reads latest from both queues (non-blocking),
    combines into DashboardData, calls dashboard.update_dashboard().
    """
    from dashboard import update_dashboard

    start_time = time.time()

    # Latest cached values
    latest_perc = None
    latest_robot = None

    while not stop_event.is_set():
        # Drain perception queue — keep latest
        while True:
            try:
                latest_perc = perception_queue.get_nowait()
            except queue.Empty:
                break

        # Drain robot state queue — keep latest
        while True:
            try:
                latest_robot = robot_state_queue.get_nowait()
            except queue.Empty:
                break

        # Build dashboard data
        dd = DashboardData(
            timestamp=time.time(),
            boot_mode=boot_mode,
            uptime_s=time.time() - start_time,
        )

        if latest_perc:
            dd.rgb_jpeg = latest_perc.rgb_jpeg
            dd.depth_jpeg = latest_perc.depth_jpeg
            dd.detections = latest_perc.detections
            dd.nearest_distance_m = latest_perc.nearest_distance_m
            dd.nearest_bearing_deg = latest_perc.nearest_bearing_deg
            dd.proximity_state = latest_perc.proximity_state
            dd.perception_fps = latest_perc.fps

        if latest_robot:
            dd.control_mode = latest_robot.control_mode
            dd.imu = latest_robot.imu
            dd.step = latest_robot.step
            dd.height_mode = latest_robot.height_mode
            dd.last_command = latest_robot.last_command
            # Robot's proximity assessment may override if more recent
            if latest_robot.proximity != ProximityState.SAFE or latest_perc is None:
                dd.proximity_state = latest_robot.proximity

        update_dashboard(dd)
        time.sleep(0.033)  # ~30 Hz


# ── Main ─────────────────────────────────────────────────────────

def main():
    # Step 1: Boot prompt
    mode = boot_prompt()
    print(f"\n  Selected: {mode.name}\n")

    # Step 2: Hardware detection (before any threads)
    try:
        detect_hardware(mode)
    except (PeripheralError, RemoteIOError) as e:
        print()
        print("!" * 50)
        print(f"  {e}")
        print("!" * 50)
        sys.exit(1)

    # Step 3: Create queues
    perception_queue = queue.Queue(maxsize=2)
    robot_state_queue = queue.Queue(maxsize=4)
    stop_event = threading.Event()

    # Graceful shutdown
    def _shutdown(sig, frame):
        print("\n[MAIN] Shutting down...")
        stop_event.set()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # Step 4: Start dashboard (always — even in robot-only mode)
    from dashboard import start_dashboard
    dash_thread = start_dashboard(port=5000)
    print("[MAIN] Dashboard started")

    # Step 5: Start aggregator thread
    agg = threading.Thread(
        target=aggregator_thread,
        args=(mode, perception_queue, robot_state_queue, stop_event),
        daemon=True,
        name="aggregator",
    )
    agg.start()
    print("[MAIN] Aggregator started")

    # Step 6: Start perception thread (if camera connected)
    perc_thread_obj = None
    if mode in (BootMode.CAMERA_ONLY, BootMode.FULL):
        from perception import PerceptionThread
        perc_thread_obj = PerceptionThread(
            output_queue=perception_queue,
            nn_confidence=0.4,
            ir_flood=200,
            ir_dot=400,
        )
        perc_thread_obj.start()
        print("[MAIN] Perception thread started")
        # Give OAK-D time to initialize
        time.sleep(3.0)

    # Step 7: Start controller (if robot connected)
    if mode in (BootMode.ROBOT_ONLY, BootMode.FULL):
        from controller import RobotController

        ctrl = RobotController(
            boot_mode=mode,
            perception_queue=perception_queue if mode == BootMode.FULL else None,
            robot_state_queue=robot_state_queue,
        )

        print("[MAIN] Initializing robot hardware...")
        ctrl.init_hardware()

        print("[MAIN] Starting controller (main thread)...")
        print()
        try:
            ctrl.run()  # Blocks on gamepad input
        except Exception as e:
            print(f"[MAIN] Controller error: {e}")
        finally:
            stop_event.set()

    else:
        # Camera-only mode — spin and serve dashboard
        print()
        print("=" * 50)
        print("  CAMERA-ONLY MODE")
        print("  Dashboard running at http://0.0.0.0:5000")
        print("  Press Ctrl+C to exit")
        print("=" * 50)
        print()

        try:
            while not stop_event.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass

    # Cleanup
    print("[MAIN] Cleaning up...")
    stop_event.set()

    if perc_thread_obj:
        perc_thread_obj.stop()
        print("[MAIN] Perception stopped")

    print("[MAIN] Done.")


if __name__ == "__main__":
    main()