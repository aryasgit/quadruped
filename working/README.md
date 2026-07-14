# Working — Locked Web-Control Stack (Fallback)

This folder is a **frozen, self-contained snapshot** of the quadruped web-control
system that was verified running on the Jetson. It is the known-good fallback.

**Do not develop in this folder.** All further development happens elsewhere; this
copy stays untouched so there is always a working version to fall back to.

## What this is

The full phone/browser controller: FastAPI + WebSocket web app that drives the
robot (gaits, strafe, turn, height modes, tricks, brace, and IMU stability).

Entry point: `web_control.py` → serves the UI from `web_remote.py` + `index.html`.

## Hardware assumptions (see `hardware/absolute_truths.py`)

- I2C **bus 7**
- PCA9685 servo driver @ `0x40`
- MPU6050 IMU @ `0x68`

## Run

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python web_control.py
```

Then open the printed URL (e.g. `http://<jetson-ip>:8000`) on a phone that is on
the same Wi-Fi network as the Jetson.

## Files

- `web_control.py`   — main controller / entry point
- `web_remote.py`    — FastAPI + uvicorn + WebSocket hub (`WebControlHub`)
- `index.html`       — browser UI (served by `web_remote.py`)
- `hardware/`        — I2C bus, PCA9685, MPU6050 IMU, absolute truths
- `ik/`, `joints/`   — inverse kinematics + joint conventions/space
- `gait/`            — gait trajectory generator
- `stance/tricks.py` — trick routines
- `controllers/`     — brace + stability controllers

## Notes

- `hardware/pca9685.py` includes bounded retry on transient I2C `Errno 121`
  (Remote I/O) glitches, so occasional bus NAKs no longer crash tricks.
