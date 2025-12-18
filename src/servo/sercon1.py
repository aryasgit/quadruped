import time
import json
import threading
import tkinter as tk
from tkinter import ttk
from smbus2 import SMBus

# ===============================
# PCA9685 CONFIG  (UNCHANGED)
# ===============================
BUS = 7
ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE

PULSE_MIN = 80
PULSE_MAX = 561

bus = SMBus(BUS)

# ===============================
# SERVO MAPPING (UNCHANGED)
# ===============================
SERVO_MAP = {
    0: 38, 1: 37, 2: 102, 3: 154,
    4: 142, 5: 61, 6: 37, 7: 46,
    8: 85, 9: 176, 10: 137, 11: 61
}

SERVO_NAMES = [
    "RRS", "RLS", "RRM", "RLM", "RRF", "RLF",
    "FRS", "FLS", "FRM", "FLM", "FRF", "FLF"
]

# ===============================
# PCA FUNCTIONS (UNCHANGED)
# ===============================
def init_pca():
    bus.write_byte_data(ADDR, MODE1, 0x00)
    time.sleep(0.01)
    prescale = int(25000000 / (4096 * 50) - 1)
    bus.write_byte_data(ADDR, MODE1, 0x10)
    bus.write_byte_data(ADDR, PRESCALE, prescale)
    bus.write_byte_data(ADDR, MODE1, 0x00)
    time.sleep(0.005)
    bus.write_byte_data(ADDR, MODE1, 0x80)

def set_pulse(channel, pulse):
    base = 0x06 + 4 * channel
    bus.write_byte_data(ADDR, base + 0, 0x00)
    bus.write_byte_data(ADDR, base + 1, 0x00)
    bus.write_byte_data(ADDR, base + 2, pulse & 0xFF)
    bus.write_byte_data(ADDR, base + 3, (pulse >> 8) & 0x0F)

def angle_to_pulse(angle):
    return int(PULSE_MIN + (angle / 270.0) * (PULSE_MAX - PULSE_MIN))

def send_standing_pulses():
    for ch, angle in SERVO_MAP.items():
        set_pulse(ch, angle_to_pulse(angle))

# ===============================
# GUI CALLBACKS (UNCHANGED)
# ===============================
def slider_callback(channel, val):
    set_pulse(channel, angle_to_pulse(float(val)))

# ===============================
# PROGRAM START (CRITICAL ORDER)
# ===============================
init_pca()
send_standing_pulses()

# ===============================
# GUI SETUP
# ===============================
root = tk.Tk()
root.title("Quadruped Servo Control & Motion Sequencer")

sliders = {}

servo_frame = tk.LabelFrame(root, text="Servo Control")
servo_frame.grid(row=0, column=0, padx=10, pady=10)

for i in range(12):
    frame = tk.Frame(servo_frame)
    frame.grid(row=i, column=0, sticky="w")

    tk.Label(frame, text=f"CH {i} ({SERVO_NAMES[i]})", width=12).pack(side="left")
    s = tk.Scale(
        frame, from_=0, to=270, orient=tk.HORIZONTAL, length=260,
        command=lambda v, ch=i: slider_callback(ch, v)
    )
    s.pack(side="left")
    sliders[i] = s

# sync sliders
for ch, ang in SERVO_MAP.items():
    sliders[ch].set(ang)

# ===============================
# POSE + SEQUENCE SYSTEM
# ===============================
POSE_FILE = "poses.json"
poses = {}
sequence = []
stop_flag = False

def read_current_pose():
    return {ch: sliders[ch].get() for ch in sliders}

def apply_pose(pose):
    for ch, ang in pose.items():
        sliders[ch].set(ang)

def save_poses():
    with open(POSE_FILE, "w") as f:
        json.dump(poses, f, indent=2)

def load_poses():
    global poses
    try:
        with open(POSE_FILE, "r") as f:
            raw = json.load(f)
            poses = {k: {int(c): v for c, v in v.items()} for k, v in raw.items()}
    except FileNotFoundError:
        poses = {}

def interpolate_pose(start, end, duration, steps=25):
    global stop_flag
    for i in range(steps):
        if stop_flag:
            return
        alpha = (i + 1) / steps
        pose = {ch: start[ch] + alpha * (end[ch] - start[ch]) for ch in start}
        apply_pose(pose)
        time.sleep(duration / steps)

def play_sequence(loop=False):
    def runner():
        global stop_flag
        stop_flag = False
        while not stop_flag:
            for step in sequence:
                if stop_flag:
                    return
                start = read_current_pose()
                interpolate_pose(start, poses[step["pose"]], step["time"])
            if not loop:
                break
    threading.Thread(target=runner, daemon=True).start()

def stop_motion():
    global stop_flag
    stop_flag = True

# ===============================
# POSE MANAGER GUI
# ===============================
pose_frame = tk.LabelFrame(root, text="Pose Manager")
pose_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")

pose_name = tk.Entry(pose_frame)
pose_name.pack(fill="x")

pose_list = tk.Listbox(pose_frame, height=6)
pose_list.pack(fill="both")

def refresh_pose_list():
    pose_list.delete(0, tk.END)
    for p in poses:
        pose_list.insert(tk.END, p)

def save_pose():
    name = pose_name.get()
    if name:
        poses[name] = read_current_pose()
        save_poses()
        refresh_pose_list()

def load_pose():
    sel = pose_list.curselection()
    if sel:
        apply_pose(poses[pose_list.get(sel)])

def delete_pose():
    sel = pose_list.curselection()
    if sel:
        poses.pop(pose_list.get(sel))
        save_poses()
        refresh_pose_list()

tk.Button(pose_frame, text="Save", command=save_pose).pack(fill="x")
tk.Button(pose_frame, text="Load", command=load_pose).pack(fill="x")
tk.Button(pose_frame, text="Delete", command=delete_pose).pack(fill="x")

# ===============================
# SEQUENCE BUILDER GUI
# ===============================
seq_frame = tk.LabelFrame(root, text="Sequence Builder")
seq_frame.grid(row=1, column=1, padx=10, pady=10, sticky="n")

pose_select = ttk.Combobox(seq_frame, values=list(poses.keys()))
pose_select.pack(fill="x")

time_entry = tk.Entry(seq_frame)
time_entry.insert(0, "0.5")
time_entry.pack(fill="x")

seq_list = tk.Listbox(seq_frame, height=6)
seq_list.pack(fill="both")

def add_step():
    p = pose_select.get()
    t = float(time_entry.get())
    sequence.append({"pose": p, "time": t})
    seq_list.insert(tk.END, f"{p} ({t}s)")

tk.Button(seq_frame, text="Add Step", command=add_step).pack(fill="x")
tk.Button(seq_frame, text="Play", command=lambda: play_sequence(False)).pack(fill="x")
tk.Button(seq_frame, text="Loop", command=lambda: play_sequence(True)).pack(fill="x")
tk.Button(seq_frame, text="STOP", fg="red", command=stop_motion).pack(fill="x")

# ===============================
# START
# ===============================
load_poses()
refresh_pose_list()
root.mainloop()

