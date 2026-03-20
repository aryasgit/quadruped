"""
tools/servo_gui.py
==================
Servo calibration / debug GUI for the quadruped.

Uses the hardware abstraction stack:
  Layer 0  — hardware/absolute_truths.py  (channel maps, stand angles)
  Layer 1.1 — hardware/i2c_bus.py         (shared SMBus)
  Layer 1.2 — hardware/pca9685.py         (angle → PWM, channel write)

NON-RESPONSIBILITIES:
  - No IK, no gait, no balance logic
  - This is a diagnostic tool only
"""

import tkinter as tk

from hardware.pca9685 import init_pca, set_servo_angle
from hardware.absolute_truths import (
    COXA,
    THIGHS,
    WRISTS,
    COXA_STAND,
    THIGH_STAND,
    WRIST_STAND,
)

# ------------------------------------------------------------------
# Build a flat  channel → (label, stand_angle)  lookup from Layer 0
# ------------------------------------------------------------------

def _build_channel_table() -> dict[int, tuple[str, float]]:
    """
    Merge the three joint group dicts into one table keyed by PCA channel.
    Returns {channel: (human_label, stand_angle_deg)}
    """
    table: dict[int, tuple[str, float]] = {}

    for name, ch in COXA.items():
        table[ch] = (f"COXA {name}  [CH {ch}]", COXA_STAND[name])

    for name, ch in THIGHS.items():
        table[ch] = (f"THIGH {name} [CH {ch}]", THIGH_STAND[name])

    for name, ch in WRISTS.items():
        table[ch] = (f"WRIST {name} [CH {ch}]", WRIST_STAND[name])

    return table


CHANNEL_TABLE = _build_channel_table()

# Sorted channel list so the GUI rows are ordered 0–11
CHANNELS = sorted(CHANNEL_TABLE)


# ------------------------------------------------------------------
# GUI callbacks
# ------------------------------------------------------------------

def _on_slider_change(channel: int, value: str) -> None:
    """Called by tkinter Scale on every change."""
    set_servo_angle(channel, float(value))


def _move_to_standing() -> None:
    """Reset all sliders and servos to the stand pose from Layer 0."""
    for ch in CHANNELS:
        _, stand_angle = CHANNEL_TABLE[ch]
        sliders[ch].set(stand_angle)
        # slider callback fires automatically, but be explicit for safety
        set_servo_angle(ch, stand_angle)


# ------------------------------------------------------------------
# Startup
# ------------------------------------------------------------------

init_pca()

# Safe start: drive servos to stand pose before the GUI exists.
# Can't touch sliders yet — they haven't been created.
for _ch, (_, _angle) in CHANNEL_TABLE.items():
    set_servo_angle(_ch, _angle)

# ------------------------------------------------------------------
# GUI layout
# ------------------------------------------------------------------

root = tk.Tk()
root.title("Quadruped Servo Control — diagnostic")

sliders: dict[int, tk.Scale] = {}

for row_idx, ch in enumerate(CHANNELS):
    label_text, stand_angle = CHANNEL_TABLE[ch]

    frame = tk.Frame(root, padx=8, pady=4)
    frame.grid(row=row_idx // 2, column=row_idx % 2, sticky="w")

    tk.Label(frame, text=label_text, anchor="w", width=22).pack(anchor="w")

    slider = tk.Scale(
        frame,
        from_=0,
        to=270,
        orient=tk.HORIZONTAL,
        length=320,
        resolution=1,
        command=lambda val, c=ch: _on_slider_change(c, val),
    )
    slider.set(stand_angle)
    slider.pack()

    sliders[ch] = slider

# Sync all sliders to stand pose now that the dict is fully populated
_move_to_standing()

# Stand button
btn_row = (len(CHANNELS) + 1) // 2
tk.Button(
    root,
    text="Move to Standing Position",
    command=_move_to_standing,
    padx=12,
    pady=6,
).grid(row=btn_row, column=0, columnspan=2, pady=12)

root.mainloop()