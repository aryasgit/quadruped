"""
tools/imu_sweep_checker.py — Live IMU Sweep Coverage Checker
=============================================================

Run this BEFORE recording your stability session to make sure
you've swept through the full roll/pitch range the robot will
encounter.

How it works
------------
Reads the live IMU at 50 Hz.  The roll × pitch space is divided
into 5-degree bins.  Each bin needs THRESHOLD samples (default 25,
= 0.5s of holding still at that angle) before it counts as swept.

A tkinter window shows the 2D grid in real time:
  - Dark grey  : not yet visited
  - Dim green  : visited but not enough samples yet
  - Bright green : complete (>= THRESHOLD samples)
  - Purple     : current IMU reading right now

Usage
-----
python tools/imu_sweep_checker.py

Optional flags:
  --threshold N   samples needed per bin (default 25)
  --range N       ± N degrees shown on each axis (default 30)
  --step N        bin size in degrees (default 5)
  --no-imu        demo mode with keyboard arrows (no hardware needed)

Controls (keyboard)
-------------------
  R          reset all counts
  S          save coverage snapshot as PNG
  Q / Esc    quit

Dependencies
------------
  tkinter   (stdlib)
  PIL       (only for --save snapshot, optional)
  IMUFilter from hardware.imu (your robot stack)
"""

import argparse
import math
import sys
import os
import time
import threading
import tkinter as tk
import tkinter.font as tkfont

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# ─── palette ───────────────────────────────────────────────────────────
BG          = '#0e0e0e'
GRID_BG     = '#1a1a1a'
CELL_EMPTY  = '#1e1e1e'
CELL_BORDER = '#2e2e2e'
CELL_PART   = '#2a5c44'    # partial: dim green
CELL_FULL   = '#4dbc8e'    # complete: bright green
CELL_ACTIVE = '#8a7ff0'    # current position: purple
CELL_ACTIVE_BORDER = '#c5bfff'
TEXT_MAIN   = '#e8e5de'
TEXT_DIM    = '#5a5a5a'
TEXT_ACCENT = '#c8ff00'
ACCENT_DIM  = '#4a5c00'


class SweepChecker:
    def __init__(self, threshold=25, angle_range=30, step=5, demo_mode=False):
        self.threshold   = threshold
        self.angle_range = angle_range
        self.step        = step
        self.demo_mode   = demo_mode

        # Build bin edges: e.g. [-30,-25,...,30] for range=30, step=5
        self.bins = list(range(-angle_range, angle_range + step, step))
        self.n    = len(self.bins)

        # counts[roll_idx][pitch_idx]
        self.counts = [[0]*self.n for _ in range(self.n)]

        self.cur_roll  = 0.0
        self.cur_pitch = 0.0
        self.running   = True

        self._build_ui()

        if not demo_mode:
            self._imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
            self._imu_thread.start()
        else:
            print("[DEMO] No IMU — use arrow keys to move the cursor.")

        self.root.mainloop()

    # ──────────────────────────────────────────────────────────────────
    # UI BUILD
    # ──────────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("IMU Sweep Checker — BARQ")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)

        CELL = 30         # px per grid cell
        PAD  = 48         # axis label padding
        INFO = 130        # right info panel width
        MARGIN = 16

        grid_w = self.n * CELL
        grid_h = self.n * CELL
        win_w  = PAD + grid_w + INFO + MARGIN * 3
        win_h  = MARGIN + 28 + MARGIN + PAD + grid_h + PAD + MARGIN

        self.root.geometry(f'{win_w}x{win_h}')

        # ── Title bar ─────────────────────────────────────────────────
        tk.Label(self.root, text="IMU SWEEP COVERAGE",
                 font=('Courier', 9), fg=TEXT_ACCENT, bg=BG,
                 anchor='w').place(x=MARGIN, y=MARGIN)

        self.pct_var = tk.StringVar(value='0%')
        tk.Label(self.root, textvariable=self.pct_var,
                 font=('Courier', 20, 'bold'), fg=TEXT_MAIN, bg=BG
                 ).place(x=win_w - INFO - MARGIN, y=MARGIN - 4)

        # ── Canvas for grid ───────────────────────────────────────────
        self.canvas = tk.Canvas(
            self.root,
            width  = PAD + grid_w + MARGIN,
            height = PAD + grid_h + PAD,
            bg=BG, highlightthickness=0
        )
        self.canvas.place(x=0, y=MARGIN + 28 + MARGIN)

        ox = PAD    # grid origin x on canvas
        oy = PAD    # grid origin y on canvas

        # Draw axis labels
        for i, v in enumerate(self.bins):
            label = ('+' if v > 0 else '') + str(v)
            # Pitch axis (top) — columns
            self.canvas.create_text(
                ox + i * CELL + CELL//2, oy - 8,
                text=label, font=('Courier', 7), fill=TEXT_DIM, anchor='s'
            )
            # Roll axis (left) — rows
            self.canvas.create_text(
                ox - 6, oy + i * CELL + CELL//2,
                text=label, font=('Courier', 7), fill=TEXT_DIM, anchor='e'
            )

        # Axis titles
        self.canvas.create_text(
            ox + grid_w//2, 12,
            text='PITCH (deg)', font=('Courier', 8), fill=TEXT_DIM, anchor='center'
        )
        self.canvas.create_text(
            14, oy + grid_h//2,
            text='ROLL (deg)', font=('Courier', 8), fill=TEXT_DIM, anchor='center',
            angle=90
        )

        # Draw zero lines
        zero_idx = self.bins.index(0) if 0 in self.bins else None
        if zero_idx is not None:
            zx = ox + zero_idx * CELL
            zy = oy + zero_idx * CELL
            self.canvas.create_line(zx, oy, zx, oy+grid_h, fill='#333333', width=1)
            self.canvas.create_line(ox, zy, ox+grid_w, zy, fill='#333333', width=1)

        # Draw cells and store rect IDs
        self.rects = [[None]*self.n for _ in range(self.n)]
        for ri in range(self.n):
            for ci in range(self.n):
                x1 = ox + ci * CELL + 1
                y1 = oy + ri * CELL + 1
                x2 = x1 + CELL - 2
                y2 = y1 + CELL - 2
                rid = self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=CELL_EMPTY, outline=CELL_BORDER, width=1
                )
                self.rects[ri][ci] = rid

        self._ox   = ox
        self._oy   = oy
        self._CELL = CELL

        # ── Right info panel ──────────────────────────────────────────
        px = PAD + grid_w + MARGIN * 2
        py = MARGIN + 28 + MARGIN + PAD

        tk.Label(self.root, text="CURRENT",
                 font=('Courier', 7), fg=TEXT_DIM, bg=BG).place(x=px, y=py)
        self.roll_var  = tk.StringVar(value='R:  +0.0°')
        self.pitch_var = tk.StringVar(value='P:  +0.0°')
        tk.Label(self.root, textvariable=self.roll_var,
                 font=('Courier', 10), fg=TEXT_MAIN, bg=BG).place(x=px, y=py+14)
        tk.Label(self.root, textvariable=self.pitch_var,
                 font=('Courier', 10), fg=TEXT_MAIN, bg=BG).place(x=px, y=py+30)

        sep_y = py + 50
        tk.Frame(self.root, bg='#2a2a2a', height=1,
                 width=INFO-8).place(x=px, y=sep_y)

        tk.Label(self.root, text="COVERAGE",
                 font=('Courier', 7), fg=TEXT_DIM, bg=BG).place(x=px, y=sep_y+8)
        self.done_var    = tk.StringVar(value='Complete:  0')
        self.partial_var = tk.StringVar(value='Partial:   0')
        self.empty_var   = tk.StringVar(value='Empty:     '+str(self.n*self.n))
        tk.Label(self.root, textvariable=self.done_var,
                 font=('Courier', 9), fg='#4dbc8e', bg=BG).place(x=px, y=sep_y+22)
        tk.Label(self.root, textvariable=self.partial_var,
                 font=('Courier', 9), fg=CELL_PART, bg=BG).place(x=px, y=sep_y+38)
        tk.Label(self.root, textvariable=self.empty_var,
                 font=('Courier', 9), fg=TEXT_DIM, bg=BG).place(x=px, y=sep_y+54)

        sep2_y = sep_y + 78
        tk.Frame(self.root, bg='#2a2a2a', height=1,
                 width=INFO-8).place(x=px, y=sep2_y)

        # Legend
        tk.Label(self.root, text="LEGEND",
                 font=('Courier', 7), fg=TEXT_DIM, bg=BG).place(x=px, y=sep2_y+8)
        def leg(y, color, label):
            tk.Frame(self.root, bg=color, width=10, height=10).place(x=px, y=sep2_y+y)
            tk.Label(self.root, text=label, font=('Courier', 8),
                     fg=TEXT_DIM, bg=BG).place(x=px+14, y=sep2_y+y-1)
        leg(22, CELL_ACTIVE,  'Current pos')
        leg(36, CELL_FULL,    'Complete')
        leg(50, CELL_PART,    'Partial')
        leg(64, CELL_EMPTY,   'Not visited')

        # Controls hint
        hint_y = sep2_y + 84
        tk.Frame(self.root, bg='#2a2a2a', height=1,
                 width=INFO-8).place(x=px, y=hint_y)
        hints = ["R  — reset", "S  — save PNG", "Q  — quit"]
        if self.demo_mode:
            hints = ["Arrows — move"] + hints
        for i, h in enumerate(hints):
            tk.Label(self.root, text=h, font=('Courier', 8),
                     fg=TEXT_DIM, bg=BG).place(x=px, y=hint_y+8+i*14)

        # ── Key bindings ───────────────────────────────────────────────
        self.root.bind('<Key-r>', lambda e: self._reset())
        self.root.bind('<Key-R>', lambda e: self._reset())
        self.root.bind('<Key-s>', lambda e: self._save_png())
        self.root.bind('<Key-S>', lambda e: self._save_png())
        self.root.bind('<Key-q>', lambda e: self._quit())
        self.root.bind('<Escape>', lambda e: self._quit())

        if self.demo_mode:
            self.root.bind('<Left>',  lambda e: self._demo_move(0, -self.step))
            self.root.bind('<Right>', lambda e: self._demo_move(0, +self.step))
            self.root.bind('<Up>',    lambda e: self._demo_move(-self.step, 0))
            self.root.bind('<Down>',  lambda e: self._demo_move(+self.step, 0))

        # ── Periodic UI refresh ────────────────────────────────────────
        self._schedule_refresh()

    # ──────────────────────────────────────────────────────────────────
    # IMU THREAD
    # ──────────────────────────────────────────────────────────────────

    def _imu_loop(self):
        try:
            from hardware.imu import init_mpu, calibrate, IMUFilter
        except ImportError as e:
            print(f"[ERROR] Cannot import IMU: {e}")
            print("[INFO]  Run with --no-imu for demo mode.")
            self.demo_mode = True
            return

        print("[IMU] Initialising MPU6050...")
        init_mpu()
        print("[IMU] Calibrating (200 samples) — keep robot flat and still...")
        calib = calibrate(samples=200)
        imu   = IMUFilter(calib)
        print("[IMU] Ready — start tilting the robot through angles.")

        DT = 0.02
        while self.running:
            t0 = time.time()
            try:
                imu.update()
                self.cur_roll  = imu.roll
                self.cur_pitch = imu.pitch
                ri, ci = self._bin_indices(self.cur_roll, self.cur_pitch)
                if ri is not None:
                    self.counts[ri][ci] = min(
                        self.counts[ri][ci] + 1,
                        self.threshold * 4
                    )
            except Exception as ex:
                print(f"[WARN] IMU read error: {ex}")
            time.sleep(max(0, DT - (time.time() - t0)))

    # ──────────────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────────────

    def _bin_indices(self, roll, pitch):
        """Return (row, col) bin indices for given roll/pitch, or (None, None)."""
        half = self.step / 2
        for i, b in enumerate(self.bins):
            if b - half <= roll < b + half:
                ri = i
                break
        else:
            return None, None
        for j, b in enumerate(self.bins):
            if b - half <= pitch < b + half:
                ci = j
                break
        else:
            return None, None
        return ri, ci

    def _cell_color(self, ri, ci):
        cur_ri, cur_ci = self._bin_indices(self.cur_roll, self.cur_pitch)
        if ri == cur_ri and ci == cur_ci:
            return CELL_ACTIVE, CELL_ACTIVE_BORDER
        v = self.counts[ri][ci]
        if v == 0:
            return CELL_EMPTY, CELL_BORDER
        if v < self.threshold:
            # interpolate dim→bright green
            t = v / self.threshold
            return CELL_PART, CELL_BORDER
        return CELL_FULL, CELL_BORDER

    def _refresh(self):
        if not self.running:
            return
        done = partial = 0
        total = self.n * self.n

        for ri in range(self.n):
            for ci in range(self.n):
                fill, outline = self._cell_color(ri, ci)
                rid = self.rects[ri][ci]
                self.canvas.itemconfig(rid, fill=fill, outline=outline)
                v = self.counts[ri][ci]
                if v >= self.threshold:
                    done += 1
                elif v > 0:
                    partial += 1

        pct = int(done / total * 100)
        self.pct_var.set(f'{pct}%')
        self.done_var.set(f'Complete:  {done}')
        self.partial_var.set(f'Partial:   {partial}')
        self.empty_var.set(f'Empty:     {total - done - partial}')

        rs = f'R:  {self.cur_roll:+.1f}°'
        ps = f'P:  {self.cur_pitch:+.1f}°'
        self.roll_var.set(rs)
        self.pitch_var.set(ps)

        self._schedule_refresh()

    def _schedule_refresh(self):
        self.root.after(40, self._refresh)   # ~25 fps UI update

    def _reset(self):
        self.counts = [[0]*self.n for _ in range(self.n)]
        print("[RESET] Coverage cleared.")

    def _demo_move(self, droll, dpitch):
        self.cur_roll  = max(-self.angle_range, min(self.angle_range, self.cur_roll  + droll))
        self.cur_pitch = max(-self.angle_range, min(self.angle_range, self.cur_pitch + dpitch))
        ri, ci = self._bin_indices(self.cur_roll, self.cur_pitch)
        if ri is not None:
            self.counts[ri][ci] = min(self.counts[ri][ci] + self.threshold, self.threshold * 4)

    def _save_png(self):
        try:
            from PIL import ImageGrab
            x = self.root.winfo_rootx()
            y = self.root.winfo_rooty()
            w = self.root.winfo_width()
            h = self.root.winfo_height()
            img = ImageGrab.grab(bbox=(x, y, x+w, y+h))
            path = f"sweep_{int(time.time())}.png"
            img.save(path)
            print(f"[SAVE] Screenshot saved → {path}")
        except ImportError:
            print("[SAVE] Install Pillow to enable PNG export: pip install Pillow")
        except Exception as e:
            print(f"[SAVE] Failed: {e}")

    def _quit(self):
        self.running = False
        self.root.destroy()


# ──────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='IMU sweep coverage checker')
    ap.add_argument('--threshold', type=int,   default=25,
                    help='Samples needed per bin to count as swept (default: 25 = 0.5s)')
    ap.add_argument('--range',     type=int,   default=30,
                    help='Angle range ±N degrees per axis (default: 30)')
    ap.add_argument('--step',      type=int,   default=5,
                    help='Bin size in degrees (default: 5)')
    ap.add_argument('--no-imu',    action='store_true',
                    help='Demo mode: use arrow keys instead of live IMU')
    args = ap.parse_args()

    print("=" * 50)
    print("  IMU SWEEP CHECKER")
    print(f"  Range: ±{args.range}°  Step: {args.step}°  Threshold: {args.threshold} samples")
    print("=" * 50)

    SweepChecker(
        threshold   = args.threshold,
        angle_range = args.range,
        step        = args.step,
        demo_mode   = args.no_imu,
    )


if __name__ == '__main__':
    main()