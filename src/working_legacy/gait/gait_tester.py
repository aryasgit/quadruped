
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time
from generator import generate_foot_targets
from ik.solver import solve_all_legs
from joints.conventions import apply_joint_conventions
from joints.space import normalize_all
from hardware.pca9685 import set_servo_angle
from hardware.absolute_truths import COXA, THIGHS, WRISTS

def main():
	print("\nGait Generator Tester (per leg)")
	print("Enter which leg to move: FL, FR, RL, RR")
	leg = input("Leg: ").strip().upper()
	while leg not in ("FL", "FR", "RL", "RR"):
		print("Invalid leg. Please enter FL, FR, RL, or RR.")
		leg = input("Leg: ").strip().upper()

	freq = 0.5
	step = 0.10
	height = 0.045
	duty = 0.60
	dt = 0.05
	cycles = 2.0

	t = 0.0
	period = 1.0 / freq if freq > 0 else 1.0

	print(f"\nTesting gait generator for leg: {leg}")
	print(f"Frequency: {freq} Hz, Step: {step} m, Height: {height} m, Duty: {duty}")
	print(f"Time step: {dt} s. Press Ctrl+C to stop.\n")
	print("   t (s)   |    x (m)    y (m)    z (m)   |  coxa   thigh   wrist (deg)")
	print("---------------------------------------------------------------")

	# Stand pose for all legs
	STANCE_X = 0.0
	STANCE_Y = 0.07
	STANCE_Z = -0.18
	stand_feet = {
		"FL": (STANCE_X,  STANCE_Y, STANCE_Z),
		"FR": (STANCE_X, -STANCE_Y, STANCE_Z),
		"RL": (STANCE_X,  STANCE_Y, STANCE_Z),
		"RR": (STANCE_X, -STANCE_Y, STANCE_Z),
	}

	try:
		while True:
			# All legs at stand, except selected leg
			foot_targets = stand_feet.copy()
			# Update only the selected leg
			ft = generate_foot_targets(
				t,
				freq,
				step_length=step,
				step_height=height,
				duty=duty,
			)
			foot_targets[leg] = ft[leg]

			# Controller pipeline
			deltas = solve_all_legs(foot_targets)
			# Only move the selected leg, keep others at stand
			for l in foot_targets:
				if l != leg:
					deltas[f"{l}_COXA"] = 0.0
					deltas[f"{l}_THIGH"] = 0.0
					deltas[f"{l}_WRIST"] = 0.0
			deltas = apply_joint_conventions(deltas)
			physical = normalize_all(deltas)

			# Send only the selected leg's joints
			set_servo_angle(COXA[leg],   physical[f"{leg}_COXA"])
			set_servo_angle(THIGHS[f"T{leg}"], physical[f"{leg}_THIGH"])
			set_servo_angle(WRISTS[f"W{leg}"], physical[f"{leg}_WRIST"])

			x, y, z = foot_targets[leg]
			print(f"{t:8.3f} | {x:8.3f} {y:8.3f} {z:8.3f} | {physical[f'{leg}_COXA']:7.1f} {physical[f'{leg}_THIGH']:7.1f} {physical[f'{leg}_WRIST']:7.1f}")
			time.sleep(dt)
			t += dt
			if t >= period:
				t -= period  # Loop phase for continuous gait
	except KeyboardInterrupt:
		print("\nStopped by user.")

if __name__ == "__main__":
	main()
