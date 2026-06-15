# Phase 0 — Recovery: rebuild everything from bare metal

_Use when: new SD card / new Jetson / wiped venv / total loss. Skip to the
verification ladder if the environment merely looks suspect._

## 0.1 Machine identity

- Jetson Orin Nano, hostname `barq`, user `barq`, Ubuntu 22.04 aarch64,
  JetPack 6 (L4T R36.x). mDNS: `barq.local`. The Mac Mini is only an SSH
  client (`ssh barq@barq.local`); its Remote Login is off, so all file
  transfer is driven FROM the Mac (`~/.barq-channel.sh` there).
- I2C: 40-pin header pins 3/5 = **bus 7** (`/dev/i2c-7`). User must be in
  groups `i2c` (servo/IMU) and `input` (PS4). Check: `groups`.
- v2 lives in `~/barq_ws` (ROS 2 in Docker). NEVER mix the two projects.

## 0.2 Repo

```bash
# GitHub over SSH:443 (port 22 is blocked on this network)
cat >> ~/.ssh/config <<'EOF'
Host github.com
    HostName ssh.github.com
    Port 443
    User git
    IdentityFile ~/.ssh/id_ed25519
EOF
# key must be registered to the aryasgit GitHub account; else create one:
# ssh-keygen -t ed25519 && cat ~/.ssh/id_ed25519.pub  -> GitHub > SSH keys
mkdir -p ~/barq_v1/reference ~/barq_v1/artifacts
git clone git@github.com:aryasgit/quadruped.git ~/barq_v1/quadruped
cd ~/barq_v1/quadruped && git checkout Master
git config user.name "Aryaman Gupta"; git config user.email rayman3304@gmail.com
# read-only research reference:
git clone --depth 1 https://github.com/mike4192/spotMicro.git ~/barq_v1/reference/spotMicro
```

Fallback if SSH auth is gone: `git clone https://github.com/aryasgit/quadruped.git`
(read-only without a PAT; you can still work and push later once a key is
registered).

## 0.3 Python environment (the venv dance)

The host lacks `python3-venv` and sudo needs a password, so pip is
bootstrapped manually — this is the KNOWN-GOOD recipe:

```bash
python3 -m venv --without-pip ~/barq_v1/venv
curl -sS https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
~/barq_v1/venv/bin/python /tmp/get-pip.py
~/barq_v1/venv/bin/pip install -r ~/barq_v1/quadruped/stack/requirements.txt
```

Notes & fallbacks:
- **pybullet** has no aarch64 wheel: pip builds it from source (~5–15 min;
  needs `/usr/include/python3.10/Python.h` and gcc — both present on
  JetPack). If the build fails → (a) retry with
  `pip install --no-cache-dir pybullet`; (b) check disk space; (c) LAST
  resort: port `stack/sim/world.py` to MuJoCo (`pip install mujoco` has
  aarch64 wheels) — the only file touching pybullet APIs is `world.py`
  (~10 calls) + `save_snapshot`; scenarios/trajectories are engine-agnostic.
- If sudo IS available, `sudo apt install python3.10-venv python3-dev` makes
  this all standard.
- Optional insurance while you have internet:
  `~/barq_v1/venv/bin/pip download -r stack/requirements.txt -d ~/barq_v1/wheels`
  → later: `pip install --no-index --find-links ~/barq_v1/wheels -r …`.

## 0.4 Verification ladder (run in order; expected results are exact)

```bash
V=~/barq_v1/venv/bin/python; cd ~/barq_v1/quadruped
$V -m pytest stack/test -q                  # -> "15 passed"
$V stack/tools/validate_urdf.py             # -> "[urdf] all checks passed"
$V stack/sim/run_sim.py --no-artifacts      # all 5 scenarios; check against
                                            #    the baselines in 00_MASTER_PLAN
$V stack/runtime/run_robot.py --dry-run --scenario walk --cycles 1 \
    --calib /tmp/synth_calib.yaml           # see below for synth calib
```

Synthetic calibration for dry-runs (no hardware needed):

```bash
$V - <<'EOF'
import sys, yaml; sys.path.insert(0, "stack")
from barq1.servos import SERVOS
cal = {n: {"zero_ticks": (s.mech_lo+s.mech_hi)/2,
           "slope_ticks_per_deg": -1.59 if s.inverted else 1.59}
       for n, s in SERVOS.items()}
yaml.safe_dump({"servos": cal}, open("/tmp/synth_calib.yaml","w"))
EOF
```

Expected dry-run output: engage → ramp → walk → ramp → "all outputs OFF",
telemetry JSONL in `~/barq_v1/artifacts/`, 0 overruns.

## 0.5 Headless display + VNC (to SEE the sim)

X on :0 is forced via `/etc/X11/xorg.conf` (helper: `sudo ~/fix_display.sh`,
backup at `/etc/X11/xorg.conf.barq-backup`; comes up 1024×768). Then
`~/setup_vnc.sh` starts x11vnc on :5900 (password in `~/.vnc/passwd`).
Mac: Finder → ⌘K → `vnc://barq.local:5900`. Sim GUI:
`DISPLAY=:0 $V stack/sim/run_sim.py --loop`. Host screenshots of GL windows
come back black — judge through the VNC client only.

If the helpers are lost: any x11vnc invocation
`x11vnc -display :0 -auth /run/user/1000/gdm/Xauthority -rfbauth ~/.vnc/passwd -forever -shared -bg`
does the job; the xorg.conf trick is a ConnectedMonitor override on DP-0.

## GATE (phase 0 complete when)

- [ ] 15/15 tests, URDF validates, all 5 sim scenarios match baselines
- [ ] dry-run completes with 0 overruns
- [ ] you can watch the sim over VNC
- [ ] `git push` works from the repo
