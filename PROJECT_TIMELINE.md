# Project Timeline & Feature Evolution

## 1. Genesis: System Bring-Up & Hardware Integration

### Novemeber 2025 — Initial Hardware Boot
- **Jetson platform initialization**: System boot sequence validated on Jetson Nano/Xavier, establishing the foundation for real-time robotics control.
- **Peripheral enumeration**: Early I²C device detection scripts confirmed sensor and actuator connectivity, forming the backbone for subsequent control layers.

### December 2025 — Actuator Control Pipeline
- **PWM servo bring-up**: Custom serial control scripts (`sercon_v1.py`, `sercon_v2.py`) enabled direct hardware actuation, overcoming power distribution and timing jitter obstacles.
- **Mechanical safety limits**: Early failures due to overextension led to the implementation of strict servo angle and pulse boundaries, codified in hardware abstraction layers.

## 2. Layered Control Architecture: Modular Expansion

### January 2026 — Layer 1–3: Reflexes, Kinematics, and Joint Space
- **Layer 1 reflexes**: Real-time balance and posture routines (`layer1_balance.py`, `balance_v2_pd_ff.py`) introduced PD control and feedforward compensation, addressing sensor noise and mechanical linkage constraints.
- **Kinematic solvers**: `leg_ik.py` and `kinematics.py` provided analytical and numerical solutions for leg positioning, enabling precise foot placement and stance control.
- **Joint conventions**: Standardized mapping (`joint_conventions.py`) resolved ambiguity in hardware-to-software translation, critical for multi-layer integration.

### Februrary 2026 — Layer 4–5: Posture & Balance
- **Posture FSM**: State machine (`posture_fsm.py`) formalized transitions between standing, leaning, sitting, and kneeling, inspired by biological locomotion.
- **Balance testers**: Experimental modules (`balance_tester.py`) quantified roll/pitch stability, leading to improved sensor fusion and filtering strategies.

## 3. Gait Generation & Advanced Locomotion

### February - March 2026 — Layer 6–8: Gait Synthesis & Phase Supervision
- **Gait generator**: `gait_generator.py` implemented phase-offset and duty-factor based gait synthesis, supporting crawl, trot, and custom patterns.
- **IMU integration**: `walk_controller_imu_integ.py` fused inertial data for real-time stabilization, mitigating slip and contact loss during dynamic motion.
- **Phase supervision**: `gait_phase.py` and `leg_fsm.py` enforced stance/swing classification, freezing swing legs and posture-modifying stance targets.

### December 2025 — Diagnostic & Visualization Tools
- **GUI diagnostics**: `copy_pressure_sense.py`, `lean_sit_kneel.py`, and graphing modules visualized foot contact, slip, and stability, enabling rapid debugging and parameter tuning.
- **CSV logging**: Systematic logging (`twin_log_full.csv`, `Posture.csv`) provided empirical data for performance analysis and research publication.

## 4. Legacy & Experimental Branches: Research Insights

### November 2025–January 2026 — Legacy Trials & Redundant Modules
- **Legacy posture FSM**: Early routines (`Legacy/control/posture_fsm.py`) explored state transitions and balance, informing later modular refactoring.
- **Demo moves & primitives**: `demo_moves.py` and graphing experiments tested motion primitives, revealing hardware jitter and sensor noise as key obstacles.
- **Solutions**: Improved filtering, mechanical safety limits, and modular code structure addressed these challenges, enabling robust reflex and gait layers.

## 5. Research Outcomes & Technical Reflections

- **Biologically inspired locomotion**: Step primitives and gait sequencing modeled after animal movement, validated through empirical trials and sensor feedback.
- **Sensor fusion**: Real-time IMU and contact sensor integration enabled slip detection, load transfer, and stability monitoring.
- **Modular experimentation**: Layered architecture facilitated rapid prototyping, allowing legacy and experimental modules to coexist and inform each other.
- **Obstacles**: Hardware jitter, sensor noise, and mechanical linkage constraints were systematically addressed through software and hardware co-design.

## 6. Current Status & Future Directions

- **Robust gait and reflex layers**: Proven pipelines for crawl, trot, and custom gaits, with real-time stabilization and contact estimation.
- **Research-grade logging and diagnostics**: Empirical data collection supports ongoing research and publication.
- **Open questions**: Further work on adaptive gait, terrain response, and advanced sensor fusion is ongoing.

---

For detailed technical notes, refer to code comments, log files, and system snapshots. For research collaboration, contact the authors.
