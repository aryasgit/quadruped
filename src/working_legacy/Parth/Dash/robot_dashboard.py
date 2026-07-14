"""
BARQ Quadruped Robot - Interactive Dashboard
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import dash
from dash import dcc, html, Input, Output
import plotly.graph_objects as go
import plotly.express as px
import numpy as np
import pandas as pd

from hardware.absolute_truths import *
from ik.kinematics import kinematics
from joints.space import JOINT_ORDER

class RobotParameters:
    def __init__(self):
        self.kin = kinematics()
        
        # Link dimensions (mm)
        self.link_dims = {
            "Coxa": self.kin.link_1 * 1000,
            "Thigh": self.kin.link_2 * 1000,
            "Shin": self.kin.link_3 * 1000,
        }
        
        # Body dimensions (mm)
        self.body_dims = {
            "Length": self.kin.length * 1000,
            "Width": self.kin.width * 1000,
            "Height": self.kin.hight * 1000,
        }
        
        # Hardware config
        self.i2c = {"Bus": BUS, "PCA": f"0x{PCA_ADDR:02X}", "IMU": f"0x{MPU_ADDR:02X}"}
        self.servo = {"Min": PULSE_MIN, "Max": PULSE_MAX, "Count": 12}
        
        # Channel mapping
        self.channels = {}
        for n, c in WRISTS.items():
            self.channels[n] = {"ch": c, "type": "Wrist"}
        for n, c in THIGHS.items():
            self.channels[n] = {"ch": c, "type": "Thigh"}
        for n, c in COXA.items():
            self.channels[n] = {"ch": c, "type": "Coxa"}
        
        # Joint limits
        self.limits = {}
        for j, d in {**WRIST_MECH, **THIGH_MECH, **COXA_MECH}.items():
            self.limits[j] = {"min": d["min"], "max": d["max"], "perp": d["perp"]}
        
        # Stand angles
        self.stand = {**WRIST_STAND, **THIGH_STAND, **COXA_STAND}

params = RobotParameters()

app = dash.Dash(__name__, title="BARQ Dashboard")

app.layout = html.Div([
    html.H1("BARQ QUADRUPED DASHBOARD", style={"textAlign": "center", "color": "#c8ff00"}),
    
    dcc.Tabs(id="tabs", value="overview", children=[
        dcc.Tab(label="OVERVIEW", value="overview"),
        dcc.Tab(label="HARDWARE", value="hardware"),
        dcc.Tab(label="JOINTS", value="joints"),
    ]),
    
    html.Div(id="content", style={"padding": "2rem"}),
], style={"backgroundColor": "#0a0a0a", "color": "#f0ede6", "fontFamily": "monospace"})

@app.callback(Output("content", "children"), Input("tabs", "value"))
def render(tab):
    if tab == "overview":
        df = pd.DataFrame({"Param": list(params.link_dims.keys()), "mm": list(params.link_dims.values())})
        fig = px.bar(df, x="Param", y="mm", title="Link Dimensions")
        fig.update_layout(plot_bgcolor="#0a0a0a", paper_bgcolor="#0a0a0a", font=dict(color="#f0ede6"))
        return dcc.Graph(figure=fig)
    
    elif tab == "hardware":
        data = [[k, v] for k, v in params.i2c.items()]
        return html.Table([html.Tr([html.Td(k), html.Td(v)]) for k, v in data])
    
    elif tab == "joints":
        df = pd.DataFrame([(k, v["min"], v["max"]) for k, v in params.limits.items()],
                          columns=["Joint", "Min", "Max"])
        fig = px.bar(df, x="Joint", y=["Min", "Max"], title="Joint Limits", barmode="group")
        fig.update_layout(plot_bgcolor="#0a0a0a", paper_bgcolor="#0a0a0a", font=dict(color="#f0ede6"))
        return dcc.Graph(figure=fig)

if __name__ == "__main__":
    print("BARQ Dashboard starting on http://127.0.0.1:8050")
    app.run_server(debug=True, host="0.0.0.0", port=8050)
