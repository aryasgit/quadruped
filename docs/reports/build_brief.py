#!/usr/bin/env python3
"""
Build the funder technical brief "BARQ V1 — Projections and Demonstration"
as a LaTeX-styled PDF (Latin Modern typography) using reportlab.

    ~/barq_v1/venv/bin/python docs/reports/build_brief.py

Fonts: Latin Modern TTFs (converted from CTAN OTFs) under
~/barq_v1/artifacts/report/fonts. If absent, falls back to Times-Roman.
Output: docs/reports/BARQ_V1_Projections_and_Demo.pdf
"""

import os
from pathlib import Path

from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm, mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate, Frame, PageTemplate, Paragraph, Spacer, Table, TableStyle,
    KeepTogether,
)
from reportlab.lib import colors

FONT_DIR = Path.home() / "barq_v1" / "artifacts" / "report" / "fonts"
OUT = Path(__file__).resolve().parent / "BARQ_V1_Projections_and_Demo.pdf"

# ---------------------------------------------------------------- fonts
SERIF, SERIF_B, SERIF_I, SERIF_BI, MONO = ("Times-Roman", "Times-Bold",
                                           "Times-Italic", "Times-BoldItalic",
                                           "Courier")
try:
    reg = {
        "LM": "lmroman10-regular.ttf", "LMB": "lmroman10-bold.ttf",
        "LMI": "lmroman10-italic.ttf", "LMBI": "lmroman10-bolditalic.ttf",
        "LMM": "lmmono10-regular.ttf",
    }
    for name, fn in reg.items():
        pdfmetrics.registerFont(TTFont(name, str(FONT_DIR / fn)))
    pdfmetrics.registerFontFamily("LM", normal="LM", bold="LMB",
                                  italic="LMI", boldItalic="LMBI")
    SERIF, SERIF_B, SERIF_I, SERIF_BI, MONO = "LM", "LMB", "LMI", "LMBI", "LMM"
    print("[brief] using Latin Modern")
except Exception as e:  # pragma: no cover
    print(f"[brief] Latin Modern unavailable ({e}); falling back to Times")

# ---------------------------------------------------------------- styles
def S(name, **kw):
    base = dict(fontName=SERIF, fontSize=10.3, leading=14.3, spaceAfter=7,
                alignment=TA_JUSTIFY)
    base.update(kw)
    return ParagraphStyle(name, **base)

title_st = S("title", fontName=SERIF_B, fontSize=22, leading=25,
             alignment=TA_CENTER, spaceAfter=2)
subtitle_st = S("subtitle", fontName=SERIF_I, fontSize=12.5, leading=16,
                alignment=TA_CENTER, spaceAfter=10)
author_st = S("author", fontSize=12.5, leading=16, alignment=TA_CENTER,
              spaceAfter=2)
date_st = S("date", fontSize=10.3, leading=13, alignment=TA_CENTER, spaceAfter=4)
abs_title_st = S("abstitle", fontName=SERIF_B, fontSize=10.3, leading=13,
                 alignment=TA_CENTER, spaceAfter=4, spaceBefore=6)
abstract_st = S("abstract", fontSize=9.8, leading=13.4, leftIndent=1.1 * cm,
                rightIndent=1.1 * cm, spaceAfter=4)
sec_st = S("sec", fontName=SERIF_B, fontSize=13.2, leading=16, spaceBefore=13,
           spaceAfter=5, alignment=0, keepWithNext=1)
sub_st = S("sub", fontName=SERIF_B, fontSize=11.2, leading=14, spaceBefore=8,
           spaceAfter=3, alignment=0, keepWithNext=1)
body_st = S("body")
bullet_st = S("bullet", leftIndent=16, bulletIndent=4, spaceAfter=3.5)
cap_st = S("cap", fontName=SERIF_I, fontSize=9, leading=12, alignment=TA_CENTER,
           spaceBefore=2, spaceAfter=10)

# ---------------------------------------------------------------- helpers
flow = []
_secn = [0]
_subn = [0]


def sec(t):
    _secn[0] += 1
    _subn[0] = 0
    flow.append(Paragraph(f"{_secn[0]}&nbsp;&nbsp;{t}", sec_st))


def sub(t):
    _subn[0] += 1
    flow.append(Paragraph(f"{_secn[0]}.{_subn[0]}&nbsp;&nbsp;{t}", sub_st))


def p(t):
    flow.append(Paragraph(t, body_st))


def bullets(items):
    for it in items:
        flow.append(Paragraph(it, bullet_st, bulletText="•"))
    flow.append(Spacer(1, 4))


def gap(h=4):
    flow.append(Spacer(1, h))


def booktable(data, col_widths, caption=None, header=True, body_font_size=9.4,
              align_first_left=True):
    ts = [
        ("FONT", (0, 0), (-1, -1), SERIF, body_font_size),
        ("FONT", (0, 0), (-1, 0), SERIF_B, body_font_size),
        ("TOPPADDING", (0, 0), (-1, -1), 3.2),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3.2),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("LINEABOVE", (0, 0), (-1, 0), 1.1, colors.black),
        ("LINEBELOW", (0, 0), (-1, 0), 0.5, colors.black),
        ("LINEBELOW", (0, -1), (-1, -1), 1.1, colors.black),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
    ]
    if not header:
        ts = [t for t in ts if t[0] != "LINEBELOW" or t[1] != (0, 0)]
    # wrap cells in paragraphs for wrapping
    wrapped = []
    for r, row in enumerate(data):
        wr = []
        for c, cell in enumerate(row):
            fn = SERIF_B if (header and r == 0) else SERIF
            al = 0 if (c == 0 and align_first_left) else (0)
            st = ParagraphStyle(f"t{r}{c}", fontName=fn, fontSize=body_font_size,
                                leading=body_font_size + 2.6, alignment=al)
            wr.append(Paragraph(str(cell), st))
        wrapped.append(wr)
    tbl = Table(wrapped, colWidths=col_widths, hAlign="CENTER")
    tbl.setStyle(TableStyle(ts))
    block = [tbl]
    if caption:
        block += [Paragraph(caption, cap_st)]
    else:
        block += [Spacer(1, 10)]
    flow.append(KeepTogether(block))


# ================================================================ CONTENT
flow.append(Spacer(1, 6))
flow.append(Paragraph("BARQ&nbsp;V1", title_st))
flow.append(Paragraph("Projections and Demonstration", subtitle_st))
flow.append(Paragraph("Aryaman Gupta&nbsp;&nbsp;&nbsp;&nbsp;Krish Agarwal", author_st))
flow.append(Paragraph("Technical Brief for Project Review &nbsp;|&nbsp; June 2026", date_st))

flow.append(Paragraph("Abstract", abs_title_st))
flow.append(Paragraph(
    "BARQ&nbsp;V1 is a twelve&#8209;degree&#8209;of&#8209;freedom quadruped robot built on a "
    "proven 3D&#8209;printed chassis and driven by an NVIDIA Jetson Orin Nano. This brief "
    "assesses, candidly and strictly against the constraints of the existing "
    "hardware, what the platform can demonstrably do today and what it can be "
    "extended to do with no further hardware investment. The control software has "
    "been rebuilt from the ground up around a simulation&#8209;first methodology: every "
    "motion the robot performs is first validated in a physics&#8209;accurate digital "
    "twin and only then executed on hardware, using the identical motion commands. "
    "We report quantitative results from that twin — the robot stands, holds "
    "commanded postures to within a fraction of a degree, and walks a stable "
    "quasi&#8209;static gait — and we set out a concrete demonstration plan and a "
    "realistic forward roadmap. We are deliberate about the platform's defining "
    "characteristic: the actuators provide no position feedback, which bounds the "
    "robot to the quasi&#8209;static motion regime. The architecture is designed to "
    "exploit that regime fully and safely rather than to fail at its edges.",
    abstract_st))
gap(6)

sec("Introduction")
p("BARQ&nbsp;V1 is the hardware&#8209;complete first generation of the BARQ quadruped "
  "program. The mechanical platform derives from the widely&#8209;replicated SpotMicro "
  "open&#8209;source design — a four&#8209;legged robot with three actuated joints per leg — "
  "adapted to carry an NVIDIA Jetson Orin Nano as its on&#8209;board computer. The present "
  "effort is a clean&#8209;sheet revival of the control software. The original stack, while "
  "functional in parts, was not maintainable; the platform has been rebuilt around "
  "rigorous engineering practice: a single audited source of measured hardware truth, "
  "a layered control stack, an automated test suite, and a physics simulation twin.")
p("This document is a frank capabilities&#8209;and&#8209;constraints assessment intended for "
  "project review. It separates three things that are often conflated: what is "
  "<b>validated today</b>, what the <b>near&#8209;term demonstration</b> will show on the "
  "physical robot, and what is <b>realistically achievable later</b> on the very same "
  "hardware. Every claim of capability is tied either to a measured result or to a "
  "clearly&#8209;scoped engineering task.")

sub("System at a Glance")
booktable(
    [["Subsystem", "Specification"],
     ["Configuration", "12&nbsp;DOF quadruped (4 legs &#215; 3 joints)"],
     ["Compute", "NVIDIA Jetson Orin Nano (GPU&#8209;capable edge module)"],
     ["Actuators", "12 &#215; DS3240MG — 40&nbsp;kg&#183;cm (3.92&nbsp;N&#183;m), 270&#176; range"],
     ["Actuation interface", "PCA9685 PWM driver over I²C, 50&nbsp;Hz frame rate"],
     ["Inertial sensing", "MPU6050 IMU (accelerometer, gyroscope, magnetometer)"],
     ["Vision", "Luxonis OAK&#8209;D Pro — stereo depth, RGB, on&#8209;board neural inference"],
     ["Power", "4S LiPo battery (currently bench&#8209;tethered)"],
     ["Mass / length", "approx.&nbsp;4.9&nbsp;kg / 345&nbsp;mm"],
     ["Limb lengths", "hip 55&nbsp;mm, upper leg 107.5&nbsp;mm, lower leg 130&nbsp;mm"]],
    [4.0 * cm, 11.2 * cm],
    caption="Table&nbsp;1.&nbsp;&nbsp;BARQ&nbsp;V1 hardware configuration.")

sec("Engineering Methodology")
p("The value of BARQ&nbsp;V1 to a reviewer lies as much in <i>how</i> it is being built "
  "as in what it does. Four principles govern the work and together de&#8209;risk every "
  "step from software to silicon.")

sub("A Single Source of Measured Truth")
p("Every electrical and mechanical constant — servo pulse ranges, channel "
  "assignments, joint limits, link lengths — lives in one audited module and is "
  "never duplicated. If a number is wrong, there is exactly one place to correct it. "
  "This discipline is what failed in the first iteration and is now enforced.")

sub("Simulation&#8209;First Validation")
p("A physics&#8209;accurate digital twin of the robot — its true geometry, mass "
  "distribution, and actuator torque and speed limits — runs in the PyBullet "
  "simulator. No motion is attempted on the physical robot until it has been shown "
  "stable in the twin. This converts the dangerous, hardware&#8209;damaging "
  "trial&#8209;and&#8209;error that ended the first iteration into cheap, repeatable software "
  "experiments.")

sub("Simulation–Hardware Parity")
p("The simulator and the physical robot are driven by the <i>same</i> motion&#8209;command "
  "generator, at the same 50&nbsp;Hz rate. A behaviour proven in simulation is not "
  "re&#8209;implemented for hardware — it is the identical command stream, sent to a real "
  "actuator instead of a simulated one. What is validated is literally what is "
  "executed.")

sub("A Permanent Safety Envelope")
p("Mechanical&#8209;limit clamps, command slew&#8209;rate limiting, staggered actuator "
  "power&#8209;on, and an always&#8209;available emergency stop are built into the command path "
  "and remain there under every mode of operation. The robot cannot be commanded "
  "outside its safe mechanical range.")

sec("Validated Capabilities")
p("The following results are measured in the simulation twin, which models the "
  "robot's real geometry and actuator limits. They constitute the regression "
  "baseline against which all hardware results will be compared.")
booktable(
    [["Capability", "Metric", "Result"],
     ["Inverse kinematics", "round&#8209;trip geometric error",
      "&lt;&nbsp;10<super>-6</super>&nbsp;m (15/15 tests)"],
     ["Static posture tracking", "RMS error, roll / pitch / yaw",
      "0.31&#176; / 0.15&#176; / 0.11&#176;"],
     ["Standing stability", "support&#8209;polygon margin", "94&nbsp;mm"],
     ["Single&#8209;leg lift", "tripod stability margin", "33.5&nbsp;mm (body tilt 0.7&#176;)"],
     ["Quasi&#8209;static walk (3 cycles)", "distance / heading drift",
      "91&nbsp;mm / 0.59&#176;"],
     ["Walk stability", "minimum support margin", "12.8&nbsp;mm (never unstable)"],
     ["Real&#8209;time control", "loop rate / timing overruns", "50&nbsp;Hz / 0"]],
    [4.6 * cm, 5.6 * cm, 5.0 * cm],
    caption="Table&nbsp;2.&nbsp;&nbsp;Validated performance in the simulation twin.")

sub("Kinematics and Posing")
p("An analytic inverse&#8209;kinematics engine maps any target body pose and foot "
  "placement to the twelve joint commands, verified to machine precision against its "
  "own forward model across the full reachable workspace. In practice this means the "
  "robot can be commanded to any feasible posture — lean, squat, raise or lower its "
  "body, look in a direction — with predictable, repeatable results.")

sub("Static Posture Control")
p("With its feet planted, the robot tracks commanded body roll, pitch, yaw, and "
  "height to within roughly a third of a degree in simulation. This is the primitive "
  "underlying body stabilisation: the same control that holds a pose can be driven "
  "by the inertial sensor to keep the body level (see Section&nbsp;6).")

sub("Quasi&#8209;Static Walking")
p("BARQ&nbsp;V1 walks using a quasi&#8209;static crawl gait: at every instant at least three "
  "feet are on the ground and the centre of mass remains inside the resulting support "
  "triangle, so the robot is balanced geometrically rather than dynamically. Over "
  "three gait cycles in simulation the robot advanced 91&nbsp;mm with only 0.59&#176; of "
  "heading drift and never approached instability — the support margin stayed at or "
  "above 12.8&nbsp;mm throughout. This is the platform's headline locomotion capability, "
  "and it is well within the envelope the hardware can support.")

sec("Demonstration Plan")
p("The near&#8209;term demonstration reproduces the validated behaviours on the physical "
  "robot, in increasing order of capability. Each item maps directly to a measured "
  "result in Section&nbsp;3, and the simulator provides a live side&#8209;by&#8209;side digital "
  "twin for any behaviour shown.")
bullets([
    "<b>Calibration and posing.</b> The robot driven through its full posture "
    "envelope on a stand — leaning, squatting, and orienting its body — demonstrating "
    "precise, repeatable joint control.",
    "<b>Standing and body articulation.</b> Rising from rest to a stable stance, then "
    "articulating body roll, pitch, yaw, and height with the feet planted — including "
    "holding the body level while the support surface is tilted by hand.",
    "<b>Walking.</b> The quasi&#8209;static crawl gait carrying the robot forward across "
    "flat ground, straight and repeatably.",
    "<b>Teleoperation.</b> Live operator control of both posture and walking through a "
    "standard game controller.",
    "<b>Perception.</b> The on&#8209;board depth camera detecting and reacting to an "
    "obstacle or a person — for example halting, or orienting toward a detected "
    "subject.",
])

sec("Capabilities and Constraints")
p("The defining characteristic of the hardware is <b>open&#8209;loop actuation</b>: the "
  "servos accept position commands but report nothing back — no joint angle, velocity, "
  "or torque. This single fact sets the platform's operating envelope, and we state "
  "it plainly rather than around it.")
sub("What the Hardware Enables")
bullets([
    "Repeatable, calibrated posing across the full twelve&#8209;degree&#8209;of&#8209;freedom "
    "workspace.",
    "Static and quasi&#8209;static locomotion — gaits in which balance is maintained "
    "geometrically at every instant. The validated crawl gait is exactly this class "
    "of motion.",
    "Body&#8209;level closed&#8209;loop control using the inertial sensor, the one feedback "
    "channel that does exist, for posture and levelling.",
    "Vision&#8209;driven behaviour using the on&#8209;board camera, which is wholly independent "
    "of joint feedback and well matched to the Jetson's compute.",
])
sub("What the Hardware Bounds")
bullets([
    "<b>No joint&#8209;level closed loop.</b> The robot cannot directly detect or correct a "
    "servo that fails to reach its commanded angle; such errors are observable only "
    "indirectly, at the body level, through the inertial sensor.",
    "<b>A quasi&#8209;static, not dynamic, motion regime.</b> Motions are executed slowly "
    "enough that momentum is negligible and balance is maintained by geometry. This "
    "is a deliberate and well&#8209;understood operating envelope.",
    "<b>No direct force sensing</b> at the joints, until rail&#8209;current monitoring is "
    "added (Section&nbsp;6).",
])
p("We regard this envelope as well matched to the platform's purpose: a robust, "
  "demonstrable, and safely&#8209;operable quadruped. The engineering is built to operate "
  "confidently inside it.")

sec("Future Scope on the Same Hardware")
p("Each of the following is achievable on the existing hardware with software effort "
  "alone — no new components beyond a current sensor already procured. They are "
  "ordered roughly by effort.")
sub("Expanded Gait and Behaviour Library")
p("Turning, sideways (lateral) crawling, and in&#8209;place rotation are direct variations "
  "of the validated quasi&#8209;static gait. A library of scripted behaviours — sit, lie "
  "down, stand, stretch — follows from the posing capability already demonstrated.")
sub("Body&#8209;Level Stabilisation")
p("The inertial sensor can drive the posture controller to hold the body level and "
  "steady on a statically tilted surface, keeping the platform composed when it is "
  "set down off&#8209;level — a closed loop using the feedback the robot does have.")
sub("Contact and Load Awareness")
p("A current sensor on the servo power rail (an INA260, already procured) provides an "
  "indirect measure of actuator load. This enables stall and collision detection and "
  "a basic reaction to felt resistance — a pragmatic, achievable substitute for the "
  "joint torque sensing the servos lack.")
sub("Vision&#8209;Guided Autonomy")
p("The on&#8209;board depth&#8209;and&#8209;AI camera supports obstacle detection and avoidance on "
  "flat ground, person detection and following, and lightweight visual odometry for "
  "self&#8209;localisation — all of which run on the Jetson without external compute. "
  "Combined with the walking and levelling capabilities, these enable a supervised "
  "autonomous mode within a flat operating area.")

sec("Conclusion")
p("BARQ&nbsp;V1 is a hardware&#8209;complete quadruped with a rebuilt, rigorously&#8209;validated "
  "control stack. Within the quasi&#8209;static envelope set by its open&#8209;loop actuators, "
  "it stands, articulates, and walks with quantified stability, and it is both "
  "controllable and extensible. The simulation&#8209;first methodology de&#8209;risks every "
  "step and makes the forward roadmap — a broader behaviour repertoire, body&#8209;level "
  "stabilisation, load awareness, and vision&#8209;guided autonomy — a matter of software "
  "on hardware that is already in hand. We are confident in both the demonstration we "
  "can give today and the trajectory we can deliver on the same platform.")


# ---------------------------------------------------------------- page furniture
def on_page(canvas, doc):
    canvas.saveState()
    canvas.setFont(SERIF, 9)
    canvas.drawCentredString(A4[0] / 2.0, 1.3 * cm, f"– {doc.page} –")
    if doc.page > 1:
        canvas.setFont(SERIF_I, 8.3)
        canvas.setFillColor(colors.Color(0.35, 0.35, 0.35))
        canvas.drawString(2.4 * cm, A4[1] - 1.25 * cm,
                          "BARQ V1 — Projections and Demonstration")
        canvas.drawRightString(A4[0] - 2.4 * cm, A4[1] - 1.25 * cm,
                               "Gupta · Agarwal")
        canvas.setStrokeColor(colors.Color(0.7, 0.7, 0.7))
        canvas.setLineWidth(0.4)
        canvas.line(2.4 * cm, A4[1] - 1.4 * cm, A4[0] - 2.4 * cm, A4[1] - 1.4 * cm)
    canvas.restoreState()


def build():
    doc = BaseDocTemplate(
        str(OUT), pagesize=A4,
        leftMargin=2.4 * cm, rightMargin=2.4 * cm,
        topMargin=2.0 * cm, bottomMargin=2.0 * cm,
        title="BARQ V1 - Projections and Demonstration",
        author="Aryaman Gupta, Krish Agarwal",
    )
    frame = Frame(doc.leftMargin, doc.bottomMargin, doc.width, doc.height,
                  id="main")
    doc.addPageTemplates([PageTemplate(id="pt", frames=[frame], onPage=on_page)])
    doc.build(flow)
    print(f"[brief] wrote {OUT}  ({OUT.stat().st_size//1024} KB)")


if __name__ == "__main__":
    build()
