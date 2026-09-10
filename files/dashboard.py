import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
from shared_buffer import rolling_buffer
import serial_reader
import time
import os

# --------------------------
# CONFIG VARIABLES
# --------------------------
TIME_WINDOW = 60  # seconds for graph display
BACKGROUND_COLOR = "#4b964f"
TEXT_COLOR = "black"
BUTTON_INACTIVE_COLOR = "#2d5a2f"
BUTTON_ACTIVE_COLOR = "#7bc47f"

MAP_FOLDER = "maps"  # folder where map images are stored
MAP_CONFIG_FILE = "map_configs.json"  # optional config for coordinates

# Load map configs if file exists
import json
if os.path.exists(MAP_CONFIG_FILE):
    with open(MAP_CONFIG_FILE, "r") as f:
        MAP_CONFIGS = json.load(f)
else:
    MAP_CONFIGS = {}

# --------------------------
# DASH APP
# --------------------------
app = dash.Dash(__name__)

app.layout = html.Div([
    # ---------- TOP CONTROLS ----------
    html.Div([
        html.Div([
            html.Label("Select COM Port:", style={"fontWeight": "bold", "marginRight": "10px"}),
            dcc.Dropdown(id="com-port-dropdown", placeholder="Select COM Port", style={"width": "200px", "display": "inline-block"}),
            html.Button("Connect", id="connect-btn", n_clicks=0,
                        style={"marginLeft": "10px", "padding": "10px", "display": "inline-block"})
        ], style={"display": "inline-block"}),

        html.Div([
            html.Label("Select Map:", style={"fontWeight": "bold", "marginRight": "10px"}),
            dcc.Dropdown(id="map-dropdown",
                         options=[{"label": name, "value": name} for name in MAP_CONFIGS.keys()],
                         placeholder="Select Map",
                         style={"width": "200px"})
        ], style={"display": "inline-block", "float": "right"})
    ], style={
        "backgroundColor": BACKGROUND_COLOR,
        "padding": "10px",
        "position": "relative",
        "zIndex": 10,
        "color": TEXT_COLOR
    }),

    # ---------- CURRENT VALUES ----------
    html.Div([
        html.H1("Live Vehicle Data", style={"textAlign": "center", "fontWeight": "bold"}),
        html.Div([
            html.Div(id="Speed", style={"fontWeight": "bold", "marginBottom": "5px"}),
            html.Div(id="RPM", style={"fontWeight": "bold", "marginBottom": "5px"}),
            html.Div(id="Voltage", style={"fontWeight": "bold", "marginBottom": "5px"}),
            html.Div(id="Satellites", style={"fontWeight": "bold", "marginBottom": "5px"}),
            html.Div(id="Status", style={"fontWeight": "bold", "marginBottom": "5px"}),
        ], style={"textAlign": "center"})
    ], style={"backgroundColor": BACKGROUND_COLOR, "color": TEXT_COLOR, "padding": "20px", "marginTop": "10px"}),

    # ---------- GRAPH TOGGLE BUTTONS ----------
    html.Div([
        html.Button("Show/Hide Speed Graph", id="toggle-speed-btn", n_clicks=0,
                    style={"backgroundColor": BUTTON_INACTIVE_COLOR, "color": "white", "margin": "5px",
                           "padding": "10px", "border": "none", "borderRadius": "5px"}),
        html.Button("Show/Hide RPM Graph", id="toggle-rpm-btn", n_clicks=0,
                    style={"backgroundColor": BUTTON_INACTIVE_COLOR, "color": "white", "margin": "5px",
                           "padding": "10px", "border": "none", "borderRadius": "5px"}),
        html.Button("Show/Hide Voltage Graph", id="toggle-voltage-btn", n_clicks=0,
                    style={"backgroundColor": BUTTON_INACTIVE_COLOR, "color": "white", "margin": "5px",
                           "padding": "10px", "border": "none", "borderRadius": "5px"}),
    ], style={"padding": "10px", "backgroundColor": BACKGROUND_COLOR, "textAlign": "center", "marginTop": "10px"}),

    # ---------- GRAPH CONTAINERS ----------
    html.Div(dcc.Graph(id="speed-graph"), id="speed-graph-container",
             style={"display": "none", "backgroundColor": BACKGROUND_COLOR, "marginTop": "10px"}),
    html.Div(dcc.Graph(id="rpm-graph"), id="rpm-graph-container",
             style={"display": "none", "backgroundColor": BACKGROUND_COLOR, "marginTop": "10px"}),
    html.Div(dcc.Graph(id="voltage-graph"), id="voltage-graph-container",
             style={"display": "none", "backgroundColor": BACKGROUND_COLOR, "marginTop": "10px"}),

    # ---------- MAP IMAGE ----------
    html.Div(html.Img(id="map-image", style={"position": "absolute", "top": "60px", "right": "20px",
                                             "height": "300px", "border": "2px solid black"})),

    # ---------- INTERVAL ----------
    dcc.Interval(id="update-interval", interval=1000, n_intervals=0)
], style={"backgroundColor": BACKGROUND_COLOR, "minHeight": "100vh"})

# --------------------------
# COM PORT DROPDOWN AUTO UPDATE
# --------------------------
@app.callback(
    Output("com-port-dropdown", "options"),
    Input("update-interval", "n_intervals")
)
def update_com_ports(_):
    ports = serial_reader.get_com_ports()
    return [{"label": p, "value": p} for p in ports]

# --------------------------
# CONNECT SERIAL CALLBACK
# --------------------------
@app.callback(
    Output("connect-btn", "children"),
    Input("connect-btn", "n_clicks"),
    Input("com-port-dropdown", "value")
)
def connect_serial(n_clicks, selected_port):
    if n_clicks == 0 or not selected_port:
        return "Connect"
    serial_reader.start_serial_reader(selected_port)
    return f"Connected to {selected_port}"

# --------------------------
# GRAPH TOGGLE CALLBACKS
# --------------------------
def toggle_graph_style(n_clicks):
    if n_clicks % 2 == 1:
        return {"display": "block", "backgroundColor": BACKGROUND_COLOR}, \
               {"backgroundColor": BUTTON_ACTIVE_COLOR, "color": "white", "margin": "5px",
                "padding": "10px", "border": "none", "borderRadius": "5px"}
    return {"display": "none", "backgroundColor": BACKGROUND_COLOR}, \
           {"backgroundColor": BUTTON_INACTIVE_COLOR, "color": "white", "margin": "5px",
            "padding": "10px", "border": "none", "borderRadius": "5px"}

@app.callback(
    Output("speed-graph-container", "style"),
    Output("toggle-speed-btn", "style"),
    Input("toggle-speed-btn", "n_clicks")
)
def toggle_speed(n_clicks):
    return toggle_graph_style(n_clicks)

@app.callback(
    Output("rpm-graph-container", "style"),
    Output("toggle-rpm-btn", "style"),
    Input("toggle-rpm-btn", "n_clicks")
)
def toggle_rpm(n_clicks):
    return toggle_graph_style(n_clicks)

@app.callback(
    Output("voltage-graph-container", "style"),
    Output("toggle-voltage-btn", "style"),
    Input("toggle-voltage-btn", "n_clicks")
)
def toggle_voltage(n_clicks):
    return toggle_graph_style(n_clicks)

# --------------------------
# HELPER FOR RECENT DATA
# --------------------------
def get_recent_data():
    buffer_copy = list(rolling_buffer)
    if not buffer_copy:
        return []
    now = buffer_copy[-1]["timestamp"]
    return [d for d in buffer_copy if now - d["timestamp"] <= TIME_WINDOW]

# --------------------------
# GRAPH CREATION
# --------------------------
def create_graph(recent, y_field, y_label, line_name):
    if not recent:
        return go.Figure()
    t0 = recent[0]["timestamp"]
    times = [int(d["timestamp"] - t0) for d in recent]
    values = [d[y_field] for d in recent]
    fig = go.Figure(data=[go.Scatter(x=times, y=values, mode="lines", name=line_name)])
    fig.update_layout(
        xaxis_title="Time (s)",
        yaxis_title=y_label,
        xaxis=dict(dtick=1),
        margin=dict(l=40, r=20, t=30, b=40),
        paper_bgcolor=BACKGROUND_COLOR,
        plot_bgcolor="white",
        font=dict(color=TEXT_COLOR, family="Arial", size=14)
    )
    return fig

@app.callback(
    Output("speed-graph", "figure"),
    Input("update-interval", "n_intervals")
)
def update_speed_graph(_):
    return create_graph(get_recent_data(), "speed", "Speed (mph)", "Speed")

@app.callback(
    Output("rpm-graph", "figure"),
    Input("update-interval", "n_intervals")
)
def update_rpm_graph(_):
    return create_graph(get_recent_data(), "rpm", "RPM", "RPM")

@app.callback(
    Output("voltage-graph", "figure"),
    Input("update-interval", "n_intervals")
)
def update_voltage_graph(_):
    return create_graph(get_recent_data(), "voltage", "Voltage (V)", "Voltage")

# --------------------------
# CURRENT VALUE UPDATES
# --------------------------
@app.callback(
    Output("Speed", "children"),
    Output("RPM", "children"),
    Output("Voltage", "children"),
    Output("Satellites", "children"),
    Output("Status", "children"),
    Input("update-interval", "n_intervals")
)
def update_current_values(_):
    if not rolling_buffer:
        return "Speed: --", "RPM: --", "Voltage: --", "Satellites: --", "Status: --"
    latest = rolling_buffer[-1]
    return (f"Speed: {latest['speed']:.2f} mph",
            f"RPM: {latest['rpm']:.2f}",
            f"Voltage: {latest['voltage']:.2f} V",
            f"Satellites: {latest['satellites']}",
            f"Status: {latest['status']}")

# --------------------------
# MAP IMAGE CALLBACK
# --------------------------
@app.callback(
    Output("map-image", "src"),
    Input("map-dropdown", "value")
)
def update_map_image(selected_map):
    if not selected_map:
        return ""
    image_path = os.path.join(MAP_FOLDER, MAP_CONFIGS[selected_map]["image"])
    if not os.path.exists(image_path):
        return ""
    return app.get_asset_url(image_path)  # Ensure the image is in the assets folder


# --------------------------
# RUN APP
# --------------------------
if __name__ == "__main__":
    app.run_server(debug=False)
