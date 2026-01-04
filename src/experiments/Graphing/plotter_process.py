import sys, time
import multiprocessing as mp
from collections import deque

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore


def plotter_main(q: mp.Queue):
    app = QtWidgets.QApplication(sys.argv)

    win = pg.GraphicsLayoutWidget(title="Quadruped Live Telemetry")
    win.resize(1200, 700)
    win.show()

    pg.setConfigOptions(antialias=False)

    PLOT_LEN = 300

    buf = {k: deque(maxlen=PLOT_LEN) for k in [
        "t",

        # orientation
        "roll", "pitch",
        "effective_roll", "effective_pitch",

        # posture
        "posture_roll", "posture_pitch",

        # effort
        "roll_effort", "offset_change",
        "avg_shoulder", "avg_foot", "avg_mid",

        # environment
        "stability", "foot_contact", "slip",

        # state
        "brace", "fsm", "posture", "step",
    ]}


    # -------- PLOTS --------
    # ========== ROW 1: ORIENTATION ==========
    p1 = win.addPlot(title="Orientation (deg)")
    p1.addLegend()
    c_roll  = p1.plot(pen="r", name="roll")
    c_pitch = p1.plot(pen="c", name="pitch")

    p1b = win.addPlot(title="Effective Orientation (deg)")
    p1b.addLegend()
    c_er = p1b.plot(pen="y", name="effective_roll")
    c_ep = p1b.plot(pen="g", name="effective_pitch")
    win.nextRow()

    # ========== ROW 2: POSTURE ==========
    p2 = win.addPlot(title="Posture Bias (deg)")
    p2.addLegend()
    c_pr = p2.plot(pen="w", name="posture_roll")
    c_pp = p2.plot(pen="m", name="posture_pitch")

    p2b = win.addPlot(title="FSM / Posture State")
    c_fsm = p2b.plot(pen="r", name="fsm")
    c_post = p2b.plot(pen="b", name="posture")
    c_step = p2b.plot(pen="g", name="step")
    win.nextRow()

    # ========== ROW 3: CONTROL EFFORT ==========
    p3 = win.addPlot(title="Control Effort")
    p3.addLegend()
    c_eff = p3.plot(pen="r", name="roll_effort")
    c_off = p3.plot(pen="b", name="offset_change")

    p3b = win.addPlot(title="Actuation Magnitude")
    p3b.addLegend()
    c_sh = p3b.plot(pen="y", name="avg_shoulder")
    c_ft = p3b.plot(pen="c", name="avg_foot")
    c_md = p3b.plot(pen="w", name="avg_mid")
    win.nextRow()

    # ========== ROW 4: ENVIRONMENT ==========
    p4 = win.addPlot(title="Stability / Brace")
    p4.addLegend()
    c_st = p4.plot(pen="g", name="stability")
    c_br = p4.plot(pen="m", name="brace")

    p4b = win.addPlot(title="Contact / Slip")
    p4b.addLegend()
    c_fc = p4b.plot(pen="w", name="foot_contact")
    c_sl = p4b.plot(pen="r", name="slip")


    def update():
        # Drain queue
        while not q.empty():
            d = q.get_nowait()
            for k in buf:
                buf[k].append(d[k])

        if len(buf["t"]) < 2:
            return
        if not buf["t"]:
            return
        t0 = buf["t"][0]
        x = [ti - t0 for ti in buf["t"]]


        # orientation
        c_roll.setData(x, buf["roll"])
        c_pitch.setData(x, buf["pitch"])
        c_er.setData(x, buf["effective_roll"])
        c_ep.setData(x, buf["effective_pitch"])

        # posture
        c_pr.setData(x, buf["posture_roll"])
        c_pp.setData(x, buf["posture_pitch"])

        # state
        c_fsm.setData(x, buf["fsm"])
        c_post.setData(x, buf["posture"])
        c_step.setData(x, buf["step"])

        # effort
        c_eff.setData(x, buf["roll_effort"])
        c_off.setData(x, buf["offset_change"])

        # actuation
        c_sh.setData(x, buf["avg_shoulder"])
        c_ft.setData(x, buf["avg_foot"])
        c_md.setData(x, buf["avg_mid"])

        # environment
        c_st.setData(x, buf["stability"])
        c_br.setData(x, buf["brace"])
        c_fc.setData(x, buf["foot_contact"])
        c_sl.setData(x, buf["slip"])


    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(100)   # 10 Hz UI refresh

    app.exec_()
