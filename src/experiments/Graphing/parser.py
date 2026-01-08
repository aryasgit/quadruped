import csv
import json
import sys
from collections import defaultdict

LOG_FILE = "run_log.csv"
OUT_FILE = "parsed_log.json"

def parse_log(path):
    events = []
    current_event = None
    header = None

    with open(path, "r") as f:
        for line in f:
            line = line.strip()

            # ---------- COMMENT / META ----------
            if line.startswith("#"):
                if line.startswith("# EVENT_START"):
                    ts = float(line.split()[-1])
                    current_event = {
                        "event_start": ts,
                        "event_end": None,
                        "samples": []
                    }
                elif line.startswith("# EVENT_END"):
                    ts = float(line.split()[-1])
                    if current_event:
                        current_event["event_end"] = ts
                        events.append(current_event)
                        current_event = None
                continue

            # ---------- HEADER ----------
            if header is None:
                header = line.split(",")
                continue

            # ---------- DATA ROW ----------
            if current_event is not None:
                values = line.split(",")
                row = dict(zip(header, values))

                # convert numeric fields
                for k, v in row.items():
                    try:
                        row[k] = float(v)
                    except ValueError:
                        pass

                current_event["samples"].append(row)

    return events

def restructure(events):
    """
    Convert flat rows → time → joint → values
    """
    structured = []

    for ev in events:
        frames = defaultdict(dict)

        for r in ev["samples"]:
            t = r["t"]
            joint = r["joint"]

            frames[t][joint] = {
                "angle": r["servo_angle"],
                "delta": r["servo_delta"],
                "motion_cmd": r["motion_cmd"],
                "response": r["leg_response"],
                "contact": r["foot_contact"],
            }

        structured.append({
            "start": ev["event_start"],
            "end": ev["event_end"],
            "duration": ev["event_end"] - ev["event_start"],
            "frames": [
                {"t": t, "joints": joints}
                for t, joints in sorted(frames.items())
            ]
        })

    return structured

def summarize(events):
    summary = []

    for i, ev in enumerate(events):
        all_angles = defaultdict(list)
        contacts = defaultdict(list)

        for frame in ev["frames"]:
            for j, v in frame["joints"].items():
                all_angles[j].append(v["angle"])
                contacts[j].append(v["contact"])

        summary.append({
            "event": i,
            "duration": ev["duration"],
            "max_angles": {j: max(v) for j, v in all_angles.items()},
            "min_angles": {j: min(v) for j, v in all_angles.items()},
            "avg_contact": {j: sum(v)/len(v) for j, v in contacts.items()},
        })

    return summary

if __name__ == "__main__":
    raw_events = parse_log(LOG_FILE)
    # -------- FALLBACK: no explicit events --------
    if len(raw_events) == 0:
        print("[WARN] No EVENT markers found — treating full log as one event")

        raw_events = [{
            "event_start": 0.0,
            "event_end": None,
            "samples": []
        }]

        with open(LOG_FILE, "r") as f:
            header = None
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue

                if header is None:
                    header = line.split(",")
                    continue

                values = line.split(",")
                row = dict(zip(header, values))

                for k, v in row.items():
                    try:
                        row[k] = float(v)
                    except ValueError:
                        pass

                raw_events[0]["samples"].append(row)

        raw_events[0]["event_end"] = raw_events[0]["samples"][-1]["t"]

    structured = restructure(raw_events)
    summary = summarize(structured)

    out = {
        "num_events": len(structured),
        "events": structured,
        "summary": summary
    }

    with open(OUT_FILE, "w") as f:
        json.dump(out, f, indent=2)

    print(f"[OK] Parsed {len(structured)} event(s)")
    print(f"[OK] Output written to {OUT_FILE}")
