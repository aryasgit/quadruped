"""
PS4 (DualShock 4) controller input via evdev (D-012).
=====================================================

Works over USB cable or Bluetooth (pairing steps in docs/HANDOFF.md).
Non-blocking: poll() drains pending events and returns the current state.

Axes are normalized to [-1, 1] with a deadzone; y axes are flipped so
"stick up" is positive.

If /dev/input is not readable:  sudo usermod -aG input $USER  + re-login.
"""

import select

try:
    from evdev import InputDevice, ecodes, list_devices
except ImportError:  # pragma: no cover
    InputDevice = None

_NAMES = ("wireless controller", "dualshock", "dualsense", "sony")

_AXES = {
    0: "lx", 1: "ly",        # left stick
    3: "rx", 4: "ry",        # right stick
    2: "l2", 5: "r2",        # triggers (0..255)
}
_BUTTONS = {
    304: "x", 305: "circle", 307: "triangle", 308: "square",
    310: "l1", 311: "r1", 314: "share", 315: "options", 316: "ps",
}


class NoController(RuntimeError):
    pass


def find_controller():
    if InputDevice is None:
        raise NoController("evdev not installed (pip install evdev)")
    try:
        paths = list_devices()
    except PermissionError as e:
        raise NoController(f"{e} — add yourself to the input group: "
                           "sudo usermod -aG input $USER, then re-login") from e
    for path in paths:
        try:
            dev = InputDevice(path)
        except (PermissionError, OSError):
            continue
        if any(k in dev.name.lower() for k in _NAMES):
            caps = dev.capabilities()
            if ecodes.EV_ABS in caps:      # the gamepad node, not its touchpad
                return dev
    raise NoController(
        "no PS4 controller found. USB: plug it in. Bluetooth: pair it "
        "(see HANDOFF). If it's connected, check 'input' group membership.")


class PS4:
    def __init__(self, dev=None, deadzone=0.08):
        self.dev = dev or find_controller()
        self.deadzone = deadzone
        self.axes = {k: 0.0 for k in ("lx", "ly", "rx", "ry", "l2", "r2")}
        self.buttons = {k: False for k in _BUTTONS.values()}
        self.pressed = set()       # rising edges since last poll
        print(f"[ps4] using {self.dev.name} @ {self.dev.path}")

    def _norm(self, code, value):
        if code in (2, 5):                       # triggers 0..255 -> 0..1
            return value / 255.0
        v = (value - 128) / 128.0                # sticks 0..255, center 128
        if abs(v) < self.deadzone:
            return 0.0
        if code in (1, 4):                       # stick up = positive
            v = -v
        return max(-1.0, min(1.0, v))

    def poll(self):
        """Drain pending events; returns (axes, buttons, rising_edges)."""
        self.pressed = set()
        while True:
            r, _, _ = select.select([self.dev.fd], [], [], 0)
            if not r:
                break
            for ev in self.dev.read():
                if ev.type == ecodes.EV_ABS and ev.code in _AXES:
                    self.axes[_AXES[ev.code]] = self._norm(ev.code, ev.value)
                elif ev.type == ecodes.EV_KEY and ev.code in _BUTTONS:
                    name = _BUTTONS[ev.code]
                    if ev.value == 1 and not self.buttons[name]:
                        self.pressed.add(name)
                    self.buttons[name] = bool(ev.value)
        return self.axes, self.buttons, self.pressed


if __name__ == "__main__":
    import time
    pad = PS4()
    print("move sticks / press buttons (Ctrl-C to quit)")
    while True:
        axes, btns, edges = pad.poll()
        live = {k: round(v, 2) for k, v in axes.items() if abs(v) > 0}
        if live or edges:
            print(live, sorted(edges) or "")
        time.sleep(0.05)
