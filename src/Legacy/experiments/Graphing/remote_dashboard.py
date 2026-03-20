import zmq
from collections import deque
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import sys

JETSON_IP = "192.168.1.34"   # ⚠️ change this
PORT = 5555

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect(f"tcp://{JETSON_IP}:{PORT}")
sock.setsockopt(zmq.SUBSCRIBE, b"")

app = QtWidgets.QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(title="REMOTE QUADRUPED TELEMETRY")
win.resize(1300, 800)
win.show()

PLOT_LEN = 400
buf = {k: deque(maxlen=PLOT_LEN) for k in [
    "t","roll","pitch","posture_roll","posture_pitch",
    "stability","roll_effort","offset_change",
    "fsm","posture","step","brace"
]}

# -------- plots --------
p1 = win.addPlot(title="Roll / Pitch")
p1.addLegend()
cr = p1.plot(pen="r", name="roll")
cp = p1.plot(pen="c", name="pitch")
win.nextRow()

p2 = win.addPlot(title="Posture Bias")
p2.addLegend()
cpr = p2.plot(pen="y", name="posture_roll")
cpp = p2.plot(pen="g", name="posture_pitch")
win.nextRow()

p3 = win.addPlot(title="FSM / Step / Brace")
cfsm = p3.plot(pen="r", name="fsm")
cstep = p3.plot(pen="g", name="step")
cbr = p3.plot(pen="m", name="brace")
win.nextRow()

p4 = win.addPlot(title="Effort")
p4.addLegend()
ce = p4.plot(pen="r", name="roll_effort")
co = p4.plot(pen="b", name="offset_change")

def update():
    while True:
        try:
            d = sock.recv_pyobj(flags=zmq.NOBLOCK)
            for k in buf:
                buf[k].append(d.get(k, 0))
        except zmq.Again:
            break

    if len(buf["t"]) < 5:
        return

    t0 = buf["t"][0]
    x = [ti - t0 for ti in buf["t"]]

    cr.setData(x, buf["roll"])
    cp.setData(x, buf["pitch"])
    cpr.setData(x, buf["posture_roll"])
    cpp.setData(x, buf["posture_pitch"])
    cfsm.setData(x, buf["fsm"])
    cstep.setData(x, buf["step"])
    cbr.setData(x, buf["brace"])
    ce.setData(x, buf["roll_effort"])
    co.setData(x, buf["offset_change"])

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

app.exec_()
