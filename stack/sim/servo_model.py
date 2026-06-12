"""
Open-loop servo model for simulation (D-008, D-009).
====================================================

The real interface is command-only: the Jetson writes a pulse width, the
DS3240MG's internal controller does the rest, and NOTHING comes back.
PyBullet's position controller with torque and velocity caps is a fair
stand-in for that internal loop; the caps are what make sim trajectories
transfer — an infinitely strong sim servo would "validate" moves the real
robot cannot do.

DS3240MG (Q-002 pending datasheet confirmation):
  stall torque   40 kg.cm = 3.92 N.m @ 7.4 V
  speed          ~0.15 s / 60 deg     -> ~7.0 rad/s no-load
Sim uses a derated continuous torque and slightly conservative velocity.
"""

STALL_TORQUE_NM = 3.92
MAX_TORQUE_NM = 3.0        # continuous derate
MAX_VELOCITY_RAD_S = 6.5
POSITION_GAIN = 0.30       # pybullet kp — stiff hobby-servo-ish tracking
VELOCITY_GAIN = 1.0


def command_position(p, body_id, joint_index, target_rad):
    """Command one joint exactly like the hardware: a position, nothing else."""
    p.setJointMotorControl2(
        bodyUniqueId=body_id,
        jointIndex=joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_rad,
        positionGain=POSITION_GAIN,
        velocityGain=VELOCITY_GAIN,
        force=MAX_TORQUE_NM,
        maxVelocity=MAX_VELOCITY_RAD_S,
    )
