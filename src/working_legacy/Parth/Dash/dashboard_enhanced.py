"""
BARQ Quadruped Robot - Enhanced Interactive Dashboard
======================================================
Full-featured parameter visualization with live updates from codebase
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import dash
from dash import dcc, html, Input, Output, State, callback
import plotly.graph_objects as go
import plotly.express as px
import numpy as np
import pandas as pd
import json
from math import degrees, radians

# Import robot modules
from hardware.absolute_truths import *
from ik.kinematics import kinematics
from joints.space import JOINT_ORDER

# ===================================================================
# ENHANCED ROBOT PARAMETERS CLASS
# ===================================================================

class RobotParametersEnhanced:
    """Complete robot parameter extraction with visualization support"""
    
    def __init__(self):
        self.kin = kinematics()
        self.load_parameters()
    
    def load_parameters(self):
        """Load all robot parameters from codebase"""
        
        # Kinematic parameters
        self.links = {
            "Coxa (Link 1)": self.kin.link_1 * 1000,
            "Thigh (Link 2)": self.kin.link_2 * 1000,
            "Shin (Link 3)": self.kin.link_3 * 1000,
        }
        
        self.body = {
            "Length": self.kin.length * 1000,
            "Width": self.kin.width * 1000,
            "Height": self.kin.hight * 1000,
        }
        
        # Hardware configuration
        self.i2c = {
            "I2C Bus": BUS,
            "PCA9685": f"0x{PCA_ADDR:02X}",
            "MPU6050": f"0x{MPU_ADDR:02X}",
        }
        
        self.servo_spec = {
            "Model": "DS3240 MG",
            "Count": 12,
            "PWM Min": PULSE_MIN,
            "PWM Max": PULSE_MAX,
            "Range": PULSE_MAX - PULSE_MIN,
            "Travel": "270°",
        }
        
        # Build complete channel mapping
        self.channels = []
        for name, ch in sorted({**WRISTS, **THIGHS, **COXA}.items(), key=lambda x: x[1]):
            servo_type = "Wrist" if name in WRISTS else "Thigh" if name in THIGHS else "Coxa"
            self.channels.append({
                "Channel": ch,
                "Servo": name,
                "Type": servo_type,
                "Leg": name[1:] if len(name) > 2 else name,
            })
        
        # Joint limits with full details
        self.limits = []
        for joint, data in {**WRIST_MECH, **THIGH_MECH, **COXA_MECH}.items():
            jtype = "Wrist" if joint in WRIST_MECH else "Thigh" if joint in THIGH_MECH else "Coxa"
            self.limits.append({
                "Joint": joint,
                "Type": jtype,
                "Min": data["min"],
                "Max": data["max"],
                "Perp": data["perp"],
                "Range": abs(data["max"] - data["min"]),
            })
        
        # Stand pose angles
        self.stand = []
        for joint, angle in {**WRIST_STAND, **THIGH_STAND, **COXA_STAND}.items():
            jtype = "Wrist" if joint in WRIST_STAND else "Thigh" if joint in THIGH_STAND else "Coxa"
            self.stand.append({
                "Joint": joint,
                "Type": jtype,
                "Angle": angle,
            })
        
        # CAD measurements from images
        self.cad = {
            "Shoulder Width": 70.0,
            "Shoulder Spacing": 78.0,
            "Plate Width": 97.0,
            "Plate Length": 148.0,
            "Chassis Width": 114.0,
            "Chassis Length": 245.631,
            "Total Leg Length": 113.92,
        }
    
    def get_leg_skeleton(self, leg_id=0, xyz=[0, 0, -0.18]):
        """Get 3D coordinates of leg joints for visualization"""
        result = self.kin.leg_IK(xyz=xyz, legID=leg_id, rot=[0, 0, 0])
        joints = result[3:7]  # j1, j2, j3, j4
        
        # Convert to mm and add leg origin
        origin = self.kin.leg_origins[leg_id]
        points = []
        for joint in joints:
            point = (np.array(joint) + np.array(origin)) * 1000
            points.append(point.tolist())
        
        return points
    
    def get_robot_frame(self):
        """Get body frame coordinates"""
        origins = self.kin.leg_origins[:4, :] * 1000
        return origins.tolist()

# Initialize parameters
params = RobotParametersEnhanced()

# ===================================================================
# DASH APP SETUP
# ===================================================================

app = dash.Dash(
    __name__,
    title="BARQ Dashboard",
    suppress_callback_exceptions=True,
)

# ===================================================================
# STYLES
# ===================================================================

COLORS = {
    'bg': '#0a0a0a',
    'text': '#f0ede6',
    'accent': '#c8ff00',
    'accent_dim': '#5a7200',
    'grey': '#6b6b6b',
    'cyan': '#00f0ff',
    'border': 'rgba(240,237,230,0.15)',
}

CARD_STYLE = {
    'border': f'1px solid {COLORS["border"]}',
    'padding': '1.5rem',
    'backgroundColor': 'rgba(10,10,10,0.5)',
    'marginBottom': '1rem',
}

# ===================================================================
# LAYOUT
# ===================================================================

app.layout = html.Div([
    # Header
    html.Div([
        html.Div([
            html.H1("BARQ", style={
                'fontSize': '3rem',
                'color': COLORS['accent'],
                'margin': '0',
                'letterSpacing': '0.3em',
            }),
            html.P("Balanced Autonomous Robot Quadruped", style={
                'fontSize': '0.7rem',
                'color': COLORS['grey'],
                'letterSpacing': '0.2em',
                'margin': '0.5rem 0 0 0',
            }),
        ], style={'flex': '1'}),
        
        html.Div([
            html.Div("LIVE DASHBOARD", style={
                'fontSize': '0.6rem',
                'color': COLORS['grey'],
                'letterSpacing': '0.2em',
            }),
            html.Div(id='live-time', style={
                'fontSize': '1rem',
                'color': COLORS['accent'],
                'fontFamily': 'monospace',
            }),
        ], style={'textAlign': 'right'}),
    ], style={
        'display': 'flex',
        'justifyContent': 'space-between',
        'alignItems': 'center',
        'padding': '2rem',
        'borderBottom': f'1px solid {COLORS["accent"]}',
    }),
    
    # Main content
    html.Div([
        # Sidebar
        html.Div([
            html.Div("NAVIGATION", style={
                'fontSize': '0.5rem',
                'color': COLORS['grey'],
                'letterSpacing': '0.3em',
                'marginBottom': '1rem',
                'borderBottom': f'1px solid {COLORS["border"]}',
                'paddingBottom': '0.5rem',
            }),
            
            dcc.RadioItems(
                id='nav-tabs',
                options=[
                    {'label': '📊 OVERVIEW', 'value': 'overview'},
                    {'label': '🤖 3D VIEW', 'value': '3d'},
                    {'label': '⚙️ HARDWARE', 'value': 'hardware'},
                    {'label': '🎯 JOINTS', 'value': 'joints'},
                    {'label': '📐 KINEMATICS', 'value': 'kinematics'},
                    {'label': '📋 FULL PARAMS', 'value': 'params'},
                ],
                value='overview',
                style={'color': COLORS['text']},
                labelStyle={'display': 'block', 'padding': '0.5rem', 'cursor': 'pointer'},
            ),
            
            html.Hr(style={'borderColor': COLORS['border'], 'margin': '1.5rem 0'}),
            
            html.Div([
                html.Div("QUICK STATS", style={
                    'fontSize': '0.5rem',
                    'color': COLORS['grey'],
                    'letterSpacing': '0.2em',
                    'marginBottom': '0.5rem',
                }),
                html.Div(f"Servos: {params.servo_spec['Count']}", style={'fontSize': '0.7rem', 'marginBottom': '0.3rem'}),
                html.Div(f"DOF: 12", style={'fontSize': '0.7rem', 'marginBottom': '0.3rem'}),
                html.Div(f"Legs: 4", style={'fontSize': '0.7rem', 'marginBottom': '0.3rem'}),
                html.Div(f"Reach: {sum(params.links.values()) - params.links['Coxa (Link 1)']:.1f}mm", style={'fontSize': '0.7rem'}),
            ], style={**CARD_STYLE, 'fontSize': '0.7rem'}),
            
        ], style={
            'width': '250px',
            'padding': '2rem',
            'borderRight': f'1px solid {COLORS["border"]}',
            'minHeight': 'calc(100vh - 120px)',
        }),
        
        # Content area
        html.Div(id='tab-content', style={
            'flex': '1',
            'padding': '2rem',
            'overflowY': 'auto',
        }),
        
    ], style={'display': 'flex'}),
    
    # Interval for live updates
    dcc.Interval(id='interval', interval=1000, n_intervals=0),
    
], style={
    'backgroundColor': COLORS['bg'],
    'color': COLORS['text'],
    'fontFamily': 'SF Mono, Consolas, monospace',
    'minHeight': '100vh',
})

# ===================================================================
# CALLBACKS
# ===================================================================

@callback(
    Output('live-time', 'children'),
    Input('interval', 'n_intervals')
)
def update_time(n):
    """Update live time"""
    from datetime import datetime
    return datetime.now().strftime('%H:%M:%S')


@callback(
    Output('tab-content', 'children'),
    Input('nav-tabs', 'value')
)
def render_content(tab):
    """Render tab content"""
    
    if tab == 'overview':
        return render_overview()
    elif tab == '3d':
        return render_3d_view()
    elif tab == 'hardware':
        return render_hardware()
    elif tab == 'joints':
        return render_joints()
    elif tab == 'kinematics':
        return render_kinematics()
    elif tab == 'params':
        return render_full_params()


# ===================================================================
# TAB RENDERERS
# ===================================================================

def render_overview():
    """Overview dashboard"""
    
    # Link dimensions chart
    link_df = pd.DataFrame({
        'Link': list(params.links.keys()),
        'Length (mm)': list(params.links.values())
    })
    
    fig_links = px.bar(
        link_df, x='Link', y='Length (mm)',
        title='Link Dimensions',
        color='Length (mm)',
        color_continuous_scale='Viridis',
    )
    fig_links.update_layout(
        plot_bgcolor=COLORS['bg'],
        paper_bgcolor=COLORS['bg'],
        font=dict(color=COLORS['text'], family='SF Mono, monospace'),
        title_font=dict(color=COLORS['accent']),
    )
    
    # Body dimensions
    body_df = pd.DataFrame({
        'Dimension': list(params.body.keys()),
        'Size (mm)': list(params.body.values())
    })
    
    fig_body = px.bar(
        body_df, x='Dimension', y='Size (mm)',
        title='Body Dimensions',
        color='Size (mm)',
        color_continuous_scale='Plasma',
    )
    fig_body.update_layout(
        plot_bgcolor=COLORS['bg'],
        paper_bgcolor=COLORS['bg'],
        font=dict(color=COLORS['text'], family='SF Mono, monospace'),
        title_font=dict(color=COLORS['accent']),
    )
    
    return html.Div([
        html.H2("System Overview", style={'color': COLORS['accent']}),
        
        # Stat cards
        html.Div([
            stat_card("TOTAL SERVOS", params.servo_spec['Count'], "DS3240 MG"),
            stat_card("CONTROL FREQ", "50 Hz", "20ms cycle"),
            stat_card("MAX REACH", f"{sum(params.links.values()):.1f} mm", "Full extension"),
            stat_card("ROS DEPS", "0", "Pure Python"),
        ], style={'display': 'grid', 'gridTemplateColumns': 'repeat(4, 1fr)', 'gap': '1rem', 'marginBottom': '2rem'}),
        
        # Charts
        html.Div([
            dcc.Graph(figure=fig_links, style={'width': '50%'}),
            dcc.Graph(figure=fig_body, style={'width': '50%'}),
        ], style={'display': 'flex', 'gap': '1rem'}),
    ])


def render_3d_view():
    """3D robot visualization"""
    
    # Get robot frame
    frame = params.get_robot_frame()
    
    fig = go.Figure()
    
    # Draw body frame
    body_x = [frame[0][0], frame[3][0], frame[2][0], frame[1][0], frame[0][0]]
    body_y = [frame[0][1], frame[3][1], frame[2][1], frame[1][1], frame[0][1]]
    body_z = [0, 0, 0, 0, 0]
    
    fig.add_trace(go.Scatter3d(
        x=body_x, y=body_y, z=body_z,
        mode='lines+markers',
        name='Body Frame',
        line=dict(color=COLORS['accent'], width=6),
        marker=dict(size=8, color=COLORS['accent']),
    ))
    
    # Draw all four legs
    leg_names = ['FL', 'FR', 'RR', 'RL']
    colors = ['#ff3b30', '#0a84ff', '#00f0ff', '#c8ff00']
    
    for i, (name, color) in enumerate(zip(leg_names, colors)):
        try:
            joints = params.get_leg_skeleton(leg_id=i)
            xs = [j[0] for j in joints]
            ys = [j[1] for j in joints]
            zs = [j[2] for j in joints]
            
            fig.add_trace(go.Scatter3d(
                x=xs, y=ys, z=zs,
                mode='lines+markers',
                name=f'{name} Leg',
                line=dict(color=color, width=4),
                marker=dict(size=6, color=color),
            ))
        except:
            pass
    
    fig.update_layout(
        title='Robot 3D View - Stand Pose',
        scene=dict(
            xaxis_title='X (mm)',
            yaxis_title='Y (mm)',
            zaxis_title='Z (mm)',
            bgcolor=COLORS['bg'],
            xaxis=dict(gridcolor=COLORS['border']),
            yaxis=dict(gridcolor=COLORS['border']),
            zaxis=dict(gridcolor=COLORS['border']),
        ),
        plot_bgcolor=COLORS['bg'],
        paper_bgcolor=COLORS['bg'],
        font=dict(color=COLORS['text'], family='SF Mono, monospace'),
        title_font=dict(color=COLORS['accent']),
        height=700,
    )
    
    return html.Div([
        html.H2("3D Robot View", style={'color': COLORS['accent']}),
        dcc.Graph(figure=fig),
    ])


def render_hardware():
    """Hardware configuration"""
    
    # Channel mapping chart
    channel_df = pd.DataFrame(params.channels)
    
    fig = px.scatter(
        channel_df,
        x='Channel',
        y='Type',
        color='Type',
        size=[15]*len(channel_df),
        hover_data=['Servo', 'Leg'],
        title='PCA9685 Channel Mapping',
    )
    fig.update_layout(
        plot_bgcolor=COLORS['bg'],
        paper_bgcolor=COLORS['bg'],
        font=dict(color=COLORS['text'], family='SF Mono, monospace'),
        title_font=dict(color=COLORS['accent']),
    )
    
    return html.Div([
        html.H2("Hardware Configuration", style={'color': COLORS['accent']}),
        
        html.Div([
            # I2C Config
            html.Div([
                html.H3("I2C Bus", style={'color': COLORS['cyan'], 'fontSize': '1rem'}),
                param_table(list(params.i2c.items())),
            ], style={**CARD_STYLE, 'flex': '1'}),
            
            # Servo Config
            html.Div([
                html.H3("Servo Specs", style={'color': COLORS['cyan'], 'fontSize': '1rem'}),
                param_table(list(params.servo_spec.items())),
            ], style={**CARD_STYLE, 'flex': '1'}),
        ], style={'display': 'flex', 'gap': '1rem', 'marginBottom': '2rem'}),
        
        dcc.Graph(figure=fig),
    ])


def render_joints():
    """Joint limits and configuration"""
    
    limit_df = pd.DataFrame(params.limits)
    stand_df = pd.DataFrame(params.stand)
    
    # Joint ranges
    fig_range = px.bar(
        limit_df,
        x='Joint',
        y='Range',
        color='Type',
        title='Joint Range of Motion',
        labels={'Range': 'Range (degrees)'},
    )
    fig_range.update_layout(
        plot_bgcolor=COLORS['bg'],
        paper_bgcolor=COLORS['bg'],
        font=dict(color=COLORS['text'], family='SF Mono, monospace'),
        title_font=dict(color=COLORS['accent']),
    )
    
    # Stand pose
    fig_stand = px.bar(
        stand_df,
        x='Joint',
        y='Angle',
        color='Type',
        title='Stand Pose Angles',
    )
    fig_stand.update_layout(
        plot_bgcolor=COLORS['bg'],
        paper_bgcolor=COLORS['bg'],
        font=dict(color=COLORS['text'], family='SF Mono, monospace'),
        title_font=dict(color=COLORS['accent']),
    )
    
    return html.Div([
        html.H2("Joint Configuration", style={'color': COLORS['accent']}),
        dcc.Graph(figure=fig_range),
        dcc.Graph(figure=fig_stand),
    ])


def render_kinematics():
    """Kinematic parameters"""
    
    # Leg origins top view
    origins = params.get_robot_frame()
    
    fig = go.Figure()
    
    body_x = [origins[0][0], origins[3][0], origins[2][0], origins[1][0], origins[0][0]]
    body_y = [origins[0][1], origins[3][1], origins[2][1], origins[1][1], origins[0][1]]
    
    fig.add_trace(go.Scatter(
        x=body_x, y=body_y,
        mode='lines+markers',
        line=dict(color=COLORS['accent'], width=3),
        marker=dict(size=12, color=COLORS['accent']),
    ))
    
    # Add labels
    leg_names = ['LF', 'LR', 'RR', 'RF']
    for i, name in enumerate(leg_names):
        fig.add_annotation(
            x=origins[i][0], y=origins[i][1],
            text=name,
            showarrow=True,
            arrowcolor=COLORS['cyan'],
            font=dict(color=COLORS['cyan'], size=14),
        )
    
    fig.update_layout(
        title='Leg Origins (Top View)',
        xaxis_title='X (mm)',
        yaxis_title='Y (mm)',
        plot_bgcolor=COLORS['bg'],
        paper_bgcolor=COLORS['bg'],
        font=dict(color=COLORS['text'], family='SF Mono, monospace'),
        title_font=dict(color=COLORS['accent']),
        height=600,
    )
    fig.update_yaxes(scaleanchor="x", scaleratio=1)
    
    return html.Div([
        html.H2("Kinematic Parameters", style={'color': COLORS['accent']}),
        dcc.Graph(figure=fig),
        
        html.Div([
            html.Div([
                html.H3("Link Lengths", style={'color': COLORS['cyan'], 'fontSize': '1rem'}),
                param_table([(k, f"{v:.3f} mm") for k, v in params.links.items()]),
            ], style={**CARD_STYLE, 'flex': '1'}),
            
            html.Div([
                html.H3("Body Dimensions", style={'color': COLORS['cyan'], 'fontSize': '1rem'}),
                param_table([(k, f"{v:.3f} mm") for k, v in params.body.items()]),
            ], style={**CARD_STYLE, 'flex': '1'}),
        ], style={'display': 'flex', 'gap': '1rem', 'marginTop': '2rem'}),
    ])


def render_full_params():
    """Full parameter listing"""
    
    # Load JSON file
    json_path = Path(__file__).parent / 'robot_parameters.json'
    with open(json_path) as f:
        data = json.load(f)
    
    return html.Div([
        html.H2("Complete Parameter Export", style={'color': COLORS['accent']}),
        
        html.Pre(
            json.dumps(data, indent=2),
            style={
                **CARD_STYLE,
                'fontSize': '0.7rem',
                'overflow': 'auto',
                'maxHeight': '600px',
                'backgroundColor': '#000',
                'padding': '1rem',
            }
        ),
        
        html.A(
            "⬇️ Download JSON",
            href='/assets/robot_parameters.json',
            download='robot_parameters.json',
            style={
                'display': 'inline-block',
                'padding': '0.5rem 1rem',
                'backgroundColor': COLORS['accent'],
                'color': COLORS['bg'],
                'textDecoration': 'none',
                'borderRadius': '4px',
                'marginTop': '1rem',
            }
        ),
    ])


# ===================================================================
# HELPER FUNCTIONS
# ===================================================================

def stat_card(title, value, subtitle=""):
    """Create stat card"""
    return html.Div([
        html.Div(title, style={'fontSize': '0.6rem', 'color': COLORS['grey'], 'letterSpacing': '0.2em'}),
        html.Div(value, style={'fontSize': '1.8rem', 'color': COLORS['accent'], 'margin': '0.3rem 0'}),
        html.Div(subtitle, style={'fontSize': '0.5rem', 'color': COLORS['grey']}),
    ], style=CARD_STYLE)


def param_table(rows):
    """Create parameter table"""
    return html.Table([
        html.Tbody([
            html.Tr([
                html.Td(str(k), style={'padding': '0.5rem', 'color': COLORS['grey'], 'fontSize': '0.7rem'}),
                html.Td(str(v), style={'padding': '0.5rem', 'color': COLORS['accent'], 'fontSize': '0.8rem', 'textAlign': 'right'}),
            ]) for k, v in rows
        ])
    ], style={'width': '100%', 'borderCollapse': 'collapse'})


# ===================================================================
# RUN SERVER
# ===================================================================

if __name__ == '__main__':
    print("=" * 70)
    print("🤖 BARQ QUADRUPED DASHBOARD")
    print("=" * 70)
    print("\n✓ Dashboard starting...")
    print(f"✓ Open browser: http://127.0.0.1:8050")
    print(f"✓ Parameters loaded from codebase")
    print("\n" + "=" * 70)
    
    app.run_server(debug=True, host='0.0.0.0', port=8050)
