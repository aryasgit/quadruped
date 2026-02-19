"""
Parth/TricksGUI.py — DARK CYBER GUI FOR SPOTMICRO TRICKS
=========================================================

A tkinter GUI matching the pose_v3.py dark aesthetic.
Runs tricks from Tricks2.py in background threads so the GUI stays responsive.

Run:
    cd src/rebuild
    python -m Parth.TricksGUI
"""

import time
import threading
import tkinter as tk
from tkinter import ttk

# ── import tricks engine ────────────────────────
from hardware.pca9685 import init_pca
from Parth.Tricks2 import (
    stand, shake, bow, wiggle, pushups, bheek,
    sit, stretch, tilt_dance, combo, TRICKS,
)


# ================================================================
#  COLOUR PALETTE  (from pose_v3.py)
# ================================================================
CYBER_BG       = "#0b0b0b"
CYBER_PANEL    = "#151515"
CYBER_BAR_BG   = "#222222"
CYBER_BTN      = "#222222"
CYBER_BTN_HI   = "#333333"
CYBER_PINK     = "#ff2ec4"
CYBER_PINK_DIM = "#c61aa3"
CYBER_GREEN    = "#00ff88"
CYBER_RED      = "#ff4444"
CYBER_TEXT     = "#e6e6e6"
CYBER_MUTED    = "#9a9a9a"
CYBER_HEADER   = "#111111"

# ================================================================
#  FONTS
# ================================================================
FONT_BIG  = ("Segoe UI Semibold", 22)
FONT_HDR  = ("Segoe UI Semibold", 13)
FONT_MAIN = ("Segoe UI", 11)
FONT_MONO = ("Consolas", 10)
FONT_SM   = ("Consolas", 9)


# ================================================================
#  STATE
# ================================================================
trick_running = False        # True while a trick thread is active
current_trick = ""           # name of trick in progress
trick_start_time = 0.0


# ================================================================
#  TRICK RUNNER (background thread)
# ================================================================
def _run_trick(name: str, func, status_var, btn_map):
    """Execute a trick in a background thread and update GUI state."""
    global trick_running, current_trick, trick_start_time

    trick_running = True
    current_trick = name
    trick_start_time = time.time()
    status_var.set(f"▶  {name.upper()} running ...")

    # disable all buttons
    for b in btn_map.values():
        b.config(state="disabled")

    try:
        func()
    except Exception as e:
        status_var.set(f"⚠  {name}: {e}")
    finally:
        trick_running = False
        current_trick = ""
        elapsed = time.time() - trick_start_time
        status_var.set(f"✔  {name} done  ({elapsed:.1f}s)")

        # re-enable buttons
        for b in btn_map.values():
            b.config(state="normal")


def launch_trick(name, func, status_var, btn_map):
    """Kick off a trick if none is running."""
    if trick_running:
        status_var.set("⏳  Wait — trick in progress")
        return
    t = threading.Thread(
        target=_run_trick,
        args=(name, func, status_var, btn_map),
        daemon=True,
    )
    t.start()


# ================================================================
#  BUILD GUI
# ================================================================
def build_gui():
    root = tk.Tk()
    root.title("SPOTMICRO // TRICKS")
    root.geometry("520x750")
    root.configure(bg=CYBER_BG)
    root.resizable(False, True)

    # ────────── Header ──────────
    header = tk.Frame(root, bg=CYBER_HEADER)
    header.pack(fill="x")

    tk.Label(
        header,
        text="SPOTMICRO // TRICKS",
        font=FONT_BIG,
        fg="#ffffff",
        bg=CYBER_HEADER,
        pady=6,
    ).pack(anchor="w", padx=14)

    tk.Label(
        header,
        text="shake • bow • wiggle • pushups • bheek • sit • stretch • tilt • combo",
        font=FONT_SM,
        fg=CYBER_MUTED,
        bg=CYBER_HEADER,
    ).pack(anchor="w", padx=16, pady=(0, 8))

    # ────────── Scrollable body ──────────
    body_container = tk.Frame(root, bg=CYBER_BG)
    body_container.pack(fill="both", expand=True, padx=8, pady=4)

    canvas = tk.Canvas(body_container, bg=CYBER_PANEL, highlightthickness=0)
    canvas.pack(side="left", fill="both", expand=True)

    scrollbar = tk.Scrollbar(body_container, orient="vertical", command=canvas.yview)
    scrollbar.pack(side="right", fill="y")
    canvas.configure(yscrollcommand=scrollbar.set)

    body = tk.Frame(canvas, bg=CYBER_PANEL)
    canvas.create_window((0, 0), window=body, anchor="nw")

    def _on_configure(event):
        canvas.configure(scrollregion=canvas.bbox("all"))
        # make inner frame fill canvas width
        canvas.itemconfig(canvas.find_all()[0], width=canvas.winfo_width())

    body.bind("<Configure>", _on_configure)
    canvas.bind("<Configure>", lambda e: canvas.itemconfig(
        canvas.find_all()[0], width=e.width
    ))

    # mouse wheel
    def _on_mousewheel(event):
        canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    canvas.bind_all("<MouseWheel>", _on_mousewheel)
    # Linux scroll
    canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-3, "units"))
    canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(3, "units"))

    # ────────── Status bar ──────────
    status_var = tk.StringVar(value="Ready — pick a trick")
    status_bar = tk.Label(
        root,
        textvariable=status_var,
        font=FONT_MONO,
        fg=CYBER_TEXT,
        bg=CYBER_BG,
        anchor="w",
        pady=6,
    )
    status_bar.pack(fill="x", padx=12, side="bottom")

    # ────────── Button storage ──────────
    btn_map = {}

    # ────────── Helper: section label ──────────
    def section(text, parent=body, top_pad=14):
        tk.Label(
            parent,
            text=text,
            font=FONT_HDR,
            fg="#ffffff",
            bg=CYBER_PANEL,
        ).pack(anchor="w", padx=12, pady=(top_pad, 4))

    # ────────── Helper: trick button ──────────
    def trick_btn(name, func, label, bg_color=CYBER_BTN,
                  fg_color=CYBER_TEXT, font=FONT_MAIN, parent=body):
        b = tk.Button(
            parent,
            text=label,
            command=lambda: launch_trick(name, func, status_var, btn_map),
            bg=bg_color,
            fg=fg_color,
            activebackground=CYBER_BTN_HI,
            activeforeground="#ffffff",
            relief="flat",
            font=font,
            cursor="hand2",
        )
        b.pack(fill="x", padx=12, pady=3)
        btn_map[name] = b
        return b

    # ================================================================
    #  STAND (always available)
    # ================================================================
    section("RESET", top_pad=10)

    trick_btn(
        "stand", stand,
        "🏠  STAND  (neutral)",
        bg_color="#1a3320", fg_color=CYBER_GREEN, font=FONT_HDR,
    )

    # ================================================================
    #  INDIVIDUAL TRICKS
    # ================================================================
    section("TRICKS")

    trick_btn("shake",      shake,                "🤝  Shake")
    trick_btn("bow",        bow,                  "🙇  Bow")
    trick_btn("wiggle",     wiggle,               "🍑  Wiggle")
    trick_btn("pushups",    pushups,              "💪  Pushups")
    trick_btn("bheek",      bheek,                "🐕  Bheek  (stand on rear)")
    trick_btn("sit",        sit,                  "🐕  Sit")
    trick_btn("stretch",    stretch,              "🐕‍🦺  Stretch")
    trick_btn("tilt_dance", tilt_dance,           "💃  Tilt Dance")

    # ================================================================
    #  COMBO / DEMO
    # ================================================================
    section("SHOW")

    trick_btn(
        "combo", combo,
        "🎬  COMBO  (all tricks)",
        bg_color=CYBER_PINK, fg_color="#000000", font=FONT_HDR,
    )

    # ================================================================
    #  LIVE INDICATOR (updates while trick runs)
    # ================================================================
    section("STATUS", top_pad=16)

    indicator_frame = tk.Frame(body, bg=CYBER_PANEL)
    indicator_frame.pack(fill="x", padx=12, pady=(0, 6))

    trick_name_var = tk.StringVar(value="IDLE")
    elapsed_var = tk.StringVar(value="0.0s")

    tk.Label(
        indicator_frame,
        text="TRICK :",
        font=FONT_MONO,
        fg=CYBER_MUTED,
        bg=CYBER_PANEL,
    ).pack(side="left")

    tk.Label(
        indicator_frame,
        textvariable=trick_name_var,
        font=FONT_HDR,
        fg=CYBER_PINK,
        bg=CYBER_PANEL,
    ).pack(side="left", padx=(6, 20))

    tk.Label(
        indicator_frame,
        text="TIME :",
        font=FONT_MONO,
        fg=CYBER_MUTED,
        bg=CYBER_PANEL,
    ).pack(side="left")

    tk.Label(
        indicator_frame,
        textvariable=elapsed_var,
        font=FONT_MONO,
        fg=CYBER_TEXT,
        bg=CYBER_PANEL,
    ).pack(side="left", padx=4)

    # progress bar
    bar_bg = tk.Frame(body, bg=CYBER_BAR_BG, height=8)
    bar_bg.pack(fill="x", padx=12, pady=(0, 12))

    bar_fg = tk.Frame(bar_bg, bg=CYBER_PINK, width=0, height=8)
    bar_fg.pack(side="left")

    # ────────── Periodic update ──────────
    pulse_phase = [0]

    def update():
        if trick_running:
            trick_name_var.set(current_trick.upper())
            elapsed = time.time() - trick_start_time
            elapsed_var.set(f"{elapsed:.1f}s")

            # pulsing bar animation
            pulse_phase[0] = (pulse_phase[0] + 1) % 40
            w = int(200 * (0.5 + 0.5 * (pulse_phase[0] / 40)))
            bar_fg.config(width=w, bg=CYBER_PINK)
        else:
            trick_name_var.set("IDLE")
            elapsed_var.set("—")
            bar_fg.config(width=0)

        root.after(100, update)

    update()
    root.mainloop()


# ================================================================
#  MAIN
# ================================================================
if __name__ == "__main__":
    init_pca()
    stand()
    build_gui()
    # return to stand on exit
    stand()
