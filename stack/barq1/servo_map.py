"""
Layer 2.7 — SERVO MAP: IK joint angles (rad) -> calibrated PCA9685 ticks.
=========================================================================

Consumes the calibration file written by the GUI
(stack/config/servo_calibration.yaml). Per servo:

    ticks = zero_ticks + slope_ticks_per_deg * degrees(q)

`slope_ticks_per_deg` is SIGNED — mount direction lives in the slope, no
separate sign tables (the legacy stack's conventions.py is superseded).
Angles are in the URDF/IK convention; the physical reference poses that
define them are specified in docs/06_CALIBRATION_PROTOCOL.md.

Commands are clamped to each servo's measured mechanical window (truths).
"""

import math
from pathlib import Path

import yaml

from barq1.kinematics import LEGS
from barq1.servos import SERVOS

DEFAULT_CALIB = Path(__file__).resolve().parents[1] / "config" / "servo_calibration.yaml"

JOINTS = ("coxa", "thigh", "wrist")          # q1, q2, q3
SLOPE_SANE = (0.8, 2.4)                      # |ticks/deg|; nominal 1.59 for 270 deg


class CalibrationError(RuntimeError):
    pass


class ServoMap:
    def __init__(self, calib):
        """calib: {servo_name: {"zero_ticks": float, "slope_ticks_per_deg": float}}"""
        missing = [n for n in SERVOS if n not in calib
                   or "zero_ticks" not in calib[n]
                   or "slope_ticks_per_deg" not in calib[n]]
        if missing:
            raise CalibrationError(
                f"servos without a usable calibration (need zero + slope, i.e. "
                f"2 recorded points in the GUI): {sorted(missing)}")
        self.cal = {n: (float(calib[n]["zero_ticks"]),
                        float(calib[n]["slope_ticks_per_deg"])) for n in SERVOS}
        self.warnings = []
        for n, (zero, slope) in self.cal.items():
            spec = SERVOS[n]
            if not SLOPE_SANE[0] <= abs(slope) <= SLOPE_SANE[1]:
                self.warnings.append(f"{n}: |slope|={abs(slope):.3f} t/deg outside "
                                     f"{SLOPE_SANE} — re-measure?")
            if not spec.mech_lo <= zero <= spec.mech_hi:
                self.warnings.append(f"{n}: zero {zero:.0f} outside mech window "
                                     f"{spec.mech_lo}..{spec.mech_hi}")

    @classmethod
    def from_yaml(cls, path=DEFAULT_CALIB):
        path = Path(path)
        if not path.exists():
            raise CalibrationError(
                f"no calibration file at {path} — run the calibration GUI "
                f"(stack/tools/calibration_gui.py) and Save first")
        doc = yaml.safe_load(path.read_text()) or {}
        return cls(doc.get("servos") or {})

    # -- forward ------------------------------------------------------------

    def ticks_for(self, name, q_rad):
        """One servo: joint angle (rad) -> clamped ticks."""
        zero, slope = self.cal[name]
        spec = SERVOS[name]
        t = int(round(zero + slope * math.degrees(q_rad)))
        return max(spec.mech_lo, min(spec.mech_hi, t))

    def named_angles(self, leg_angles):
        """{leg: (q1,q2,q3)} -> {servo_name: q_rad}."""
        out = {}
        for leg, qs in leg_angles.items():
            for joint, q in zip(JOINTS, qs):
                out[f"{leg}_{joint}"] = q
        return out

    def to_ticks(self, leg_angles):
        """{leg: (q1,q2,q3)} -> {channel: ticks} ready for the PCA9685."""
        return {SERVOS[n].channel: self.ticks_for(n, q)
                for n, q in self.named_angles(leg_angles).items()}

    # -- inverse (display/telemetry) -----------------------------------------

    def angle_for(self, name, ticks):
        zero, slope = self.cal[name]
        return math.radians((ticks - zero) / slope)

    def report(self):
        lines = [f"{'servo':10} {'ch':>2} {'zero':>6} {'slope':>7}  mech window"]
        for n in SERVOS:
            zero, slope = self.cal[n]
            s = SERVOS[n]
            lines.append(f"{n:10} {s.channel:2} {zero:6.1f} {slope:7.3f}  "
                         f"{s.mech_lo}..{s.mech_hi}")
        lines += [f"WARNING: {w}" for w in self.warnings]
        return "\n".join(lines)


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    try:
        m = ServoMap.from_yaml()
        print(m.report())
    except CalibrationError as e:
        print(f"[servo_map] {e}")
        sys.exit(1)
