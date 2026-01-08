import csv
from collections import defaultdict

INPUT = "run_log.csv"
OUTPUT = "twin_log.csv"

# One frame = one timestamp
frames = defaultdict(dict)

with open(INPUT) as f:
    reader = csv.DictReader(
        line for line in f
        if not line.startswith("#")
    )
    for r in reader:
        t = round(float(r["t"]), 3)
        joint = r["joint"]
        angle = float(r["servo_angle"])
        contact = float(r["foot_contact"])

        frames[t]["t"] = t
        frames[t][joint] = angle

        # store contact only for feet
        if joint.startswith("FOOT_"):
            frames[t][joint + "_contact"] = contact

# Write clean frame-based CSV
fields = ["t"]

# Collect all joint names dynamically
joint_names = set()
for f in frames.values():
    joint_names |= set(f.keys())

joint_names.discard("t")
fields += sorted(joint_names)

with open(OUTPUT, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fields)
    writer.writeheader()
    for t in sorted(frames):
        writer.writerow(frames[t])

print(f"[OK] Wrote {OUTPUT} with {len(frames)} frames")
