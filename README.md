# Quadruped Robotics Platform — Research & Development

## Overview

This repository documents the complete development of a multi-layered quadruped robot, designed for advanced locomotion, balance, and reflexive control. The project integrates hardware, software, and theoretical robotics principles, with a focus on modularity, extensibility, and robust real-world performance. It is the result of extensive research, experimentation, and iterative engineering by Aryaman Gupta and Krish Agarwal.

## Motivation & Research Goals

- Develop a platform for studying legged locomotion, balance, and reflexive behaviors
- Enable rapid prototyping of gait algorithms, sensor fusion, and control strategies
- Provide a modular stack for hardware abstraction, simulation, and real-time diagnostics
- Document obstacles, breakthroughs, and lessons learned for future researchers

## Layered Architecture

The codebase is organized into distinct layers, each responsible for a specific aspect of robot control:

- **Layer 1: Reflex & Balance** — Real-time IMU feedback, servo actuation, and low-level stabilization. Implements PD control, feedforward, and safety limits.
- **Layer 2: Joint Space & Conventions** — Abstracts joint kinematics, conventions, and mapping between physical and logical coordinates.
- **Layer 3: Kinematics & Leg IK** — Solves inverse kinematics for each leg, enabling precise foot placement and trajectory planning.
- **Layer 4: Pump & Axis Controllers** — Higher-level posture and axis control, including stand, lean, and axis-specific behaviors.
- **Layer 5: Posture & Gait Controllers** — Supervises overall posture, gait transitions, and multi-leg coordination.
- **Layer 6: Gait Generation** — Implements gait algorithms (crawl, trot, etc.), phase scheduling, and swing/stance classification.
- **Layer 7: Demo Engine & FSM** — Provides demo routines, finite state machines, and test harnesses for behaviors.
- **Layer 8: Gait Phase Supervisor** — Supervises gait phase transitions, ensuring correct stance/swing assignment.
- **Stance & Virtual Layers** — Specialized modules for calibration, virtual stick control, and advanced stance routines.

## Technical Details

- **Hardware Integration:**
  - Jetson platform for compute and peripheral control
  - PCA9685 PWM driver for servo actuation
  - IMU (MPU6050) for real-time orientation and acceleration feedback
  - Custom power distribution and sensor bus
- **Software Stack:**
  - Python 3.x, modular code structure
  - Real-time control loops (20 Hz typical)
  - GUI diagnostics (Tkinter-based)
  - CSV logging and system snapshot utilities
- **Control Algorithms:**
  - PD control with feedforward for roll/pitch stabilization
  - Complementary filtering for sensor fusion
  - Step primitives: UNLOADING → LIFTING → DONE → RECENTERING → CONTACT → IDLE
  - Gait sequencing: crawl, trot, and custom patterns
  - Slip/traction estimation and contact confidence
- **Safety & Diagnostics:**
  - Mechanical limits enforced in software
  - Real-time GUI for monitoring foot contact, slip, stability, and airborne confidence
  - System snapshot scripts for environment and dependency tracking

## Key Accomplishments

- Reliable IMU-based balance and reflex layer
- Modular gait generation and phase supervision
- GUI-assisted diagnostics for contact, slip, and stability
- Robust actuator bring-up and PWM sequencing
- Real-time unload and brace detection for safe leg transitions
- Legacy features: early posture FSM, demo routines, and hardware abstraction

## Version History & Legacy Insights

- **Legacy/Redundant:**
  - Early posture FSM and balance routines (see `Legacy/control/posture_fsm.py`)
  - Demo moves and motion primitives (see `Legacy/redundant/layer9/demo_moves.py`)
  - GUI and graphing experiments for pose and slip visualization
  - Obstacles: hardware jitter, sensor noise, mechanical linkage constraints
  - Solutions: improved filtering, mechanical safety limits, modular refactoring

## System Requirements

- Jetson platform (tested on Jetson Nano/Xavier)
- Python 3.x
- Required packages: numpy, matplotlib, smbus2, pandas, tkinter, etc.
- See `system_snapshot/pip_requirements.txt` and `apt_packages.txt` for full dependency list

## Usage & Demo

- See `main/controller.py` and `main/main_controller.py` for entry points
- Run GUI diagnostics via `src/Legacy/estimation/copy_pressure_sense.py` or `src/Legacy/experiments/lean_sit_kneel.py`
- Use provided VS Code tasks for remote execution and stopping robot scripts
- Refer to video links in the original README for milestone demonstrations

## Theoretical Background

- Implements principles from legged robotics, control theory, and sensor fusion
- Modular design enables rapid experimentation with new gaits and reflexes
- Step primitives and gait sequencing inspired by biological locomotion
- Contact and slip estimation based on real-time sensor feedback

## Acknowledgements

- Aryaman Gupta and Krish Agarwal — Lead developers and researchers
- All contributors to legacy and experimental modules

## License

© 2026 Aryaman Gupta and Krish Agarwal. All Rights Reserved.
This repository and its contents are proprietary and confidential. Unauthorized use is strictly prohibited.

---

For detailed technical notes, see code comments and system snapshot files. For research inquiries or collaboration, contact the authors directly.



