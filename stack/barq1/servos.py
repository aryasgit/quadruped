"""
Layer 0.5 — SERVO TABLE
=======================

Pure, derived view over the truths: one record per servo joining the
channel map, mechanical limits, perpendicular and stand references —
all converted to the tick domain the driver speaks.

No I/O, no state. If a number here looks wrong, fix barq1/truths.py
(by re-measuring), never this file.

Naming: BARQ legs are FL/FR/RL/RR; joints are coxa (link 1, hip),
thigh (link 2, upper leg), wrist (link 3, lower leg). The spotMicro
docs call legs LF/RF/LB/RB and joints 1/2/3 — sm_id carries that
cross-reference, e.g. FL_coxa == LF_1.
"""

from dataclasses import dataclass, asdict

from barq1 import truths
from barq1.pca9685 import legacy_deg_to_ticks

LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("coxa", "thigh", "wrist")

_SM_LEG = {"FL": "LF", "FR": "RF", "RL": "LB", "RR": "RB"}
_SM_LINK = {"coxa": 1, "thigh": 2, "wrist": 3}

_GROUPS = {
    # joint -> (channel map, mech map, stand map, legacy key prefix)
    "coxa":  (truths.COXA,   truths.COXA_MECH,  truths.COXA_STAND,  ""),
    "thigh": (truths.THIGHS, truths.THIGH_MECH, truths.THIGH_STAND, "T"),
    "wrist": (truths.WRISTS, truths.WRIST_MECH, truths.WRIST_STAND, "W"),
}


@dataclass(frozen=True)
class ServoSpec:
    name: str          # "FL_coxa"
    legacy_key: str    # truths key: "FL" / "TFL" / "WFL"
    sm_id: str         # spotMicro docs naming: "LF_1"
    channel: int       # PCA9685 channel
    mech_lo: int       # ticks — low end of measured mechanical window
    mech_hi: int       # ticks — high end of measured mechanical window
    inverted: bool     # legacy mech min > max: mirrored servo mount
    perp: int          # ticks at measured perpendicular reference
    stand: int         # ticks at measured stand pose

    def as_dict(self) -> dict:
        return asdict(self)


def _build() -> dict:
    table = {}
    for leg in LEGS:
        for joint in JOINTS:
            channels, mech_map, stand_map, prefix = _GROUPS[joint]
            key = prefix + leg
            mech = mech_map[key]
            lo, hi = sorted(
                (legacy_deg_to_ticks(mech["min"]), legacy_deg_to_ticks(mech["max"]))
            )
            name = f"{leg}_{joint}"
            table[name] = ServoSpec(
                name=name,
                legacy_key=key,
                sm_id=f"{_SM_LEG[leg]}_{_SM_LINK[joint]}",
                channel=channels[key],
                mech_lo=lo,
                mech_hi=hi,
                inverted=mech["min"] > mech["max"],
                perp=legacy_deg_to_ticks(mech["perp"]),
                stand=legacy_deg_to_ticks(stand_map[key]),
            )
    return table


SERVOS: dict = _build()                       # name -> ServoSpec
ORDER = [f"{leg}_{joint}" for leg in LEGS for joint in JOINTS]
BY_CHANNEL = {s.channel: s for s in SERVOS.values()}

assert len(SERVOS) == 12 and len(BY_CHANNEL) == 12, "channel map must be unique"


if __name__ == "__main__":
    from barq1.pca9685 import ticks_to_us
    print(f"{'servo':9} {'sm':5} ch  mech[ticks]   perp  stand   stand[us]")
    for name in ORDER:
        s = SERVOS[name]
        print(f"{s.name:9} {s.sm_id:5} {s.channel:2}  "
              f"{s.mech_lo:3}..{s.mech_hi:3} {'inv' if s.inverted else '   '}  "
              f"{s.perp:4}  {s.stand:4}   {ticks_to_us(s.stand):7.1f}")
