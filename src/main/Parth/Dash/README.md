# BARQ Quadruped Robot Dashboard

Real-time parameter visualization dashboard for the BARQ quadruped robot.

## Features

- **Overview**: Link dimensions, body geometry, key metrics
- **Hardware**: I2C configuration, servo mapping, channel assignments  
- **Joints**: Joint limits, stand pose, range of motion visualization

## Installation

```bash
pip install -r requirements.txt
```

## Run

```bash
cd /home/a/quadruped/src/main/Parth/Dash
python3 robot_dashboard.py
```

Then open: http://127.0.0.1:8050

## Parameters Extracted from Code

All parameters are dynamically extracted from:
- `hardware/absolute_truths.py` - Hardware constants
- `ik/kinematics.py` - Link dimensions and body geometry
- `joints/space.py` - Joint ordering

No hardcoded values - everything is pulled from the live codebase.
