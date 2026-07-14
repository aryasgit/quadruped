"""
Complete Robot Parameter Export
Extracts all parameters from codebase and exports to JSON
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import json
from math import degrees
from hardware.absolute_truths import *
from ik.kinematics import kinematics
from joints.space import JOINT_ORDER

def extract_all_parameters():
    """Extract every parameter from the robot codebase"""
    
    kin = kinematics()
    
    params = {
        # ====================
        # KINEMATIC PARAMETERS
        # ====================
        "kinematics": {
            "link_lengths_m": {
                "coxa_link1": kin.link_1,
                "thigh_link2": kin.link_2,
                "shin_wrist_link3": kin.link_3,
            },
            "link_lengths_mm": {
                "coxa_link1": round(kin.link_1 * 1000, 3),
                "thigh_link2": round(kin.link_2 * 1000, 3),
                "shin_wrist_link3": round(kin.link_3 * 1000, 3),
            },
            "body_dimensions_m": {
                "length": kin.length,
                "width": kin.width,
                "height": kin.hight,
            },
            "body_dimensions_mm": {
                "length": round(kin.length * 1000, 3),
                "width": round(kin.width * 1000, 3),
                "height": round(kin.hight * 1000, 3),
            },
            "phi_angle_rad": kin.phi,
            "phi_angle_deg": round(degrees(kin.phi), 2),
            "leg_origins_m": kin.leg_origins[:4].tolist(),
            "leg_origins_mm": (kin.leg_origins[:4] * 1000).tolist(),
        },
        
        # ====================
        # HARDWARE CONFIG
        # ====================
        "hardware": {
            "i2c": {
                "bus_number": BUS,
                "pca9685_address": f"0x{PCA_ADDR:02X}",
                "mpu6050_address": f"0x{MPU_ADDR:02X}",
            },
            "pca9685": {
                "mode1_register": f"0x{MODE1:02X}",
                "prescale_register": f"0x{PRESCALE:02X}",
                "pulse_min": PULSE_MIN,
                "pulse_max": PULSE_MAX,
                "pulse_range": PULSE_MAX - PULSE_MIN,
            },
            "mpu6050": {
                "power_mgmt_register": f"0x{PWR_MGMT_1:02X}",
                "accel_register": f"0x{ACCEL_XOUT_H:02X}",
                "gyro_register": f"0x{GYRO_XOUT_H:02X}",
            },
        },
        
        # ====================
        # SERVO MAPPING
        # ====================
        "servo_channels": {
            "wrists": {name: ch for name, ch in WRISTS.items()},
            "thighs": {name: ch for name, ch in THIGHS.items()},
            "coxa": {name: ch for name, ch in COXA.items()},
            "all_by_channel": {},
        },
        
        # ====================
        # JOINT LIMITS
        # ====================
        "joint_limits": {
            "wrists": {
                name: {
                    "min_deg": data["min"],
                    "max_deg": data["max"],
                    "perpendicular_deg": data["perp"],
                    "range_deg": abs(data["max"] - data["min"]),
                }
                for name, data in WRIST_MECH.items()
            },
            "thighs": {
                name: {
                    "min_deg": data["min"],
                    "max_deg": data["max"],
                    "perpendicular_deg": data["perp"],
                    "range_deg": abs(data["max"] - data["min"]),
                }
                for name, data in THIGH_MECH.items()
            },
            "coxa": {
                name: {
                    "min_deg": data["min"],
                    "max_deg": data["max"],
                    "perpendicular_deg": data["perp"],
                    "range_deg": abs(data["max"] - data["min"]),
                }
                for name, data in COXA_MECH.items()
            },
        },
        
        # ====================
        # STAND POSE
        # ====================
        "stand_pose": {
            "wrists": {name: angle for name, angle in WRIST_STAND.items()},
            "thighs": {name: angle for name, angle in THIGH_STAND.items()},
            "coxa": {name: angle for name, angle in COXA_STAND.items()},
        },
        
        # ====================
        # JOINT ORDER
        # ====================
        "joint_order": {
            "canonical_order": JOINT_ORDER,
            "leg_mapping": {
                "LF": ["FL_COXA", "FL_THIGH", "FL_WRIST"],
                "RF": ["FR_COXA", "FR_THIGH", "FR_WRIST"],
                "RR": ["RR_COXA", "RR_THIGH", "RR_WRIST"],
                "LR": ["RL_COXA", "RL_THIGH", "RL_WRIST"],
            },
        },
        
        # ====================
        # CAD MEASUREMENTS (from images)
        # ====================
        "cad_measurements_mm": {
            "shoulder_axis_width": 70.0,
            "shoulder_servo_spacing": 78.0,
            "mounting_plate_width": 97.0,
            "mounting_plate_length": 148.0,
            "chassis_front_width": 114.0,
            "chassis_total_length": 245.631,
            "chassis_height": 70.0,
            "upper_compartment_height": 49.513,
            "upper_compartment_width": 77.0,
            "upper_compartment_length": 40.0,
            "lower_compartment_depth": 26.93,
            "back_compartment_width": 100.0,
            "back_compartment_height": 63.43,
            "back_compartment_depth": 27.0,
            "straightened_leg_total": 113.92,
            "measured_shin_link": 134.76,
            "shoulder_to_body_center": 207.50,
        },
        
        # ====================
        # DERIVED PARAMETERS
        # ====================
        "derived": {
            "max_leg_reach_mm": round((kin.link_2 + kin.link_3) * 1000, 3),
            "min_leg_reach_mm": round(abs(kin.link_2 - kin.link_3) * 1000, 3),
            "total_servos": 12,
            "degrees_of_freedom": 12,
            "legs": 4,
            "dof_per_leg": 3,
        },
    }
    
    # Build all-by-channel mapping
    all_channels = {}
    for name, ch in WRISTS.items():
        all_channels[ch] = {"servo": name, "type": "Wrist", "joint": f"{name[1:]} Wrist"}
    for name, ch in THIGHS.items():
        all_channels[ch] = {"servo": name, "type": "Thigh", "joint": f"{name[1:]} Thigh"}
    for name, ch in COXA.items():
        all_channels[ch] = {"servo": name, "type": "Coxa", "joint": f"{name} Coxa"}
    params["servo_channels"]["all_by_channel"] = {str(k): v for k, v in sorted(all_channels.items())}
    
    return params


if __name__ == "__main__":
    print("=" * 70)
    print("BARQ QUADRUPED ROBOT - COMPLETE PARAMETER EXPORT")
    print("=" * 70)
    
    params = extract_all_parameters()
    
    # Save to JSON
    output_file = Path(__file__).parent / "robot_parameters.json"
    with open(output_file, 'w') as f:
        json.dump(params, f, indent=2)
    
    print(f"\n✓ Parameters exported to: {output_file}")
    print(f"✓ File size: {output_file.stat().st_size} bytes")
    
    # Print summary
    print("\n" + "=" * 70)
    print("PARAMETER SUMMARY")
    print("=" * 70)
    
    print("\n📐 KINEMATIC DIMENSIONS (mm)")
    print(f"  Coxa Link 1:  {params['kinematics']['link_lengths_mm']['coxa_link1']}")
    print(f"  Thigh Link 2: {params['kinematics']['link_lengths_mm']['thigh_link2']}")
    print(f"  Shin Link 3:  {params['kinematics']['link_lengths_mm']['shin_wrist_link3']}")
    print(f"  Body Length:  {params['kinematics']['body_dimensions_mm']['length']}")
    print(f"  Body Width:   {params['kinematics']['body_dimensions_mm']['width']}")
    
    print("\n🔧 HARDWARE CONFIG")
    print(f"  I2C Bus:      {params['hardware']['i2c']['bus_number']}")
    print(f"  PCA9685:      {params['hardware']['i2c']['pca9685_address']}")
    print(f"  MPU6050:      {params['hardware']['i2c']['mpu6050_address']}")
    print(f"  PWM Range:    {params['hardware']['pca9685']['pulse_min']} - {params['hardware']['pca9685']['pulse_max']}")
    
    print("\n🎮 SERVO CHANNELS")
    for ch, info in sorted(params['servo_channels']['all_by_channel'].items(), key=lambda x: int(x[0])):
        print(f"  Ch {ch:2s}: {info['servo']:4s} → {info['type']:5s} ({info['joint']})")
    
    print("\n📊 JOINT RANGES (degrees)")
    for joint, data in params['joint_limits']['coxa'].items():
        print(f"  {joint}: {data['min_deg']:3.0f}° to {data['max_deg']:3.0f}° (range: {data['range_deg']:3.0f}°)")
    
    print("\n🤖 STAND POSE (degrees)")
    for joint, angle in params['stand_pose']['coxa'].items():
        print(f"  {joint}: {angle}°")
    
    print("\n" + "=" * 70)
    print("✓ Export complete!")
    print("=" * 70)
