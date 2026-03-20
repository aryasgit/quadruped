import csv
from collections import defaultdict

INPUT = "run_log.csv"
OUTPUT = "twin_log_full.csv"

frames = defaultdict(dict)

GLOBAL_FIELDS = [
    "roll",
    "pitch",
    "posture_roll",
    "posture_pitch",
    "roll_effort",
    "offset_change",
    "posture_mode",
    "fsm",
]

def is_valid(val):
    return val is not None and val != ""

with open(INPUT) as f:
    reader = csv.DictReader(
        line for line in f
        if not line.startswith("#")
    )

    for r in reader:
        t = round(float(r["t"]), 3)
        joint = r["joint"]

        frames[t]["t"] = t

        # ---- Joint angles ----
        if is_valid(r.get("servo_angle")):
            frames[t][joint] = float(r["servo_angle"])

        # ---- Global signals ----
        for field in GLOBAL_FIELDS:
            if is_valid(r.get(field)):
                try:
                    frames[t][field] = float(r[field])
                except ValueError:
                    # non-numeric fields (fsm, posture_mode)
                    frames[t][field] = r[field]

# ---- Build header ----
fields = ["t"]
all_keys = set()

for f in frames.values():
    all_keys |= set(f.keys())

all_keys.discard("t")
fields += sorted(all_keys)

# ---- Write CSV ----
with open(OUTPUT, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fields)
    writer.writeheader()
    for t in sorted(frames):
        writer.writerow(frames[t])

print(f"[OK] Wrote {OUTPUT} with {len(frames)} frames")
