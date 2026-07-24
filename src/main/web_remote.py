import json
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from threading import Lock

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn


ANALOG_DEADZONE = 0.15
STALE_TIMEOUT = 0.40
TILT_STALE_TIMEOUT = 0.50   # phone gyro tilt decays to 0 if updates stop


def apply_deadzone(value: float, deadzone: float = ANALOG_DEADZONE) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


@dataclass
class RemoteState:
    fwd: float = 0.0
    strafe: float = 0.0
    turn: float = 0.0
    last_update: float = 0.0
    connected: bool = False
    commands: deque = field(default_factory=deque)
    # Phone/tablet gyro tilt, each in [-1, 1].
    #   tilt_roll  : + = lean right   (side to side)
    #   tilt_pitch : + = nose down    (front to back)
    tilt_roll: float = 0.0
    tilt_pitch: float = 0.0
    tilt_update: float = 0.0


class WebControlHub:
    """
    Thread-safe browser input hub.

    Browser sends:
      - axis packets continuously
      - command packets on button taps
    Main loop reads:
      - current analog axes
      - queued commands
    """

    def __init__(self, deadzone: float = ANALOG_DEADZONE):
        self.deadzone = deadzone
        self.scheme = "http"   # set to "https" by start() when a TLS cert exists
        self._lock = Lock()
        self._state = RemoteState()

    def set_axes(self, fwd: float, strafe: float, turn: float):
        with self._lock:
            self._state.fwd = float(fwd)
            self._state.strafe = float(strafe)
            self._state.turn = float(turn)
            self._state.last_update = time.time()

    def push_command(self, cmd: str):
        if not cmd:
            return
        with self._lock:
            self._state.commands.append(cmd)
            self._state.last_update = time.time()

    def poll_command(self):
        with self._lock:
            if self._state.commands:
                return self._state.commands.popleft()
            return None

    def flush_commands(self):
        with self._lock:
            self._state.commands.clear()

    def set_tilt(self, roll: float, pitch: float):
        with self._lock:
            self._state.tilt_roll = max(-1.0, min(1.0, float(roll)))
            self._state.tilt_pitch = max(-1.0, min(1.0, float(pitch)))
            self._state.tilt_update = time.time()

    def get_tilt(self):
        """Latest phone gyro (roll, pitch) in [-1, 1]; (0, 0) if stale/disabled."""
        with self._lock:
            if time.time() - self._state.tilt_update > TILT_STALE_TIMEOUT:
                return 0.0, 0.0
            return self._state.tilt_roll, self._state.tilt_pitch

    def set_connected(self, connected: bool):
        with self._lock:
            self._state.connected = connected
            self._state.last_update = time.time()

    def is_connected(self) -> bool:
        with self._lock:
            return self._state.connected

    def read_sticks(self):
        """
        Returns deadzoned analog values.
        If input is stale, returns zeros.
        """
        with self._lock:
            if time.time() - self._state.last_update > STALE_TIMEOUT:
                return 0.0, 0.0, 0.0

            fwd = apply_deadzone(self._state.fwd, self.deadzone)
            strafe = apply_deadzone(self._state.strafe, self.deadzone)
            turn = apply_deadzone(self._state.turn, self.deadzone)
            return fwd, strafe, turn

    def stick_magnitude(self) -> float:
        fwd, strafe, turn = self.read_sticks()
        return min(1.0, (fwd * fwd + strafe * strafe + turn * turn) ** 0.5)

    def has_activity(self) -> bool:
        return self.stick_magnitude() > 0.3 or self.has_pending_command()

    def has_pending_command(self) -> bool:
        with self._lock:
            return bool(self._state.commands)

    def wait_for_idle(self, timeout_s: float = 1.5):
        t0 = time.time()
        while True:
            if self.stick_magnitude() <= 0.3 and not self.has_pending_command():
                return
            if time.time() - t0 > timeout_s:
                return
            time.sleep(0.02)

    def start(self, host: str = "0.0.0.0", port: int = 8000):
        app = self._build_app()

        # Serve over HTTPS if a self-signed cert is present. Mobile browsers only
        # expose device-orientation (gyro) sensors on a secure context, so the
        # phone-tilt feature needs TLS. Falls back to plain HTTP if no cert.
        cert_dir = Path(__file__).with_name("certs")
        certfile = cert_dir / "cert.pem"
        keyfile = cert_dir / "key.pem"
        use_tls = certfile.exists() and keyfile.exists()
        self.scheme = "https" if use_tls else "http"

        cfg = dict(app=app, host=host, port=port, log_level="warning", access_log=False)
        if use_tls:
            cfg["ssl_certfile"] = str(certfile)
            cfg["ssl_keyfile"] = str(keyfile)
        config = uvicorn.Config(**cfg)
        server = uvicorn.Server(config)

        # BUG FIX: surface server startup failures. The old code ran server.run
        # in a daemon thread and slept 0.5s; if the port was in use or bind
        # failed, the thread died silently and main() printed the URL as if up.
        def _run():
            try:
                server.run()
            except Exception as e:
                print(f"[WEB][ERROR] Server thread crashed: {e}")

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()

        # Wait for a real readiness signal instead of a blind sleep.
        for _ in range(50):
            if getattr(server, "started", False):
                break
            time.sleep(0.1)
        if not getattr(server, "started", False):
            print(f"[WEB][WARN] Server not confirmed listening on {host}:{port} "
                  f"(port already in use? bind failed?)")
        return thread

    def _build_app(self):
        app = FastAPI()
        html_path = Path(__file__).with_name("index.html")

        @app.get("/")
        async def index():
            if html_path.exists():
                return HTMLResponse(html_path.read_text(encoding="utf-8"))
            return HTMLResponse("<h1>Missing index.html</h1>")

        @app.get("/health")
        async def health():
            return {"ok": True, "connected": self.is_connected()}

        @app.websocket("/ws")
        async def ws_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.set_connected(True)

            try:
                while True:
                    msg = await websocket.receive_text()
                    try:
                        data = json.loads(msg)
                    except json.JSONDecodeError:
                        continue

                    msg_type = data.get("type")

                    if msg_type == "axis":
                        self.set_axes(
                            data.get("fwd", 0.0),
                            data.get("strafe", 0.0),
                            data.get("turn", 0.0),
                        )
                    elif msg_type == "command":
                        self.push_command(str(data.get("cmd", "")).lower())
                    elif msg_type == "tilt":
                        self.set_tilt(data.get("roll", 0.0), data.get("pitch", 0.0))
                    elif msg_type == "ping":
                        self.set_connected(True)

            except WebSocketDisconnect:
                pass
            finally:
                self.set_connected(False)

        return app