from tkinter import ttk
from dash.dependencies import Output, Input
from dash import Dash,html,dcc
import plotly
import plotly.graph_objs as go
from collections import deque
import pickle



# t = deque(maxlen = 20)
# t.append(1)
  
# x = deque(maxlen = 20)
# x.append(1)

app = Dash(__name__)
  
app.layout = html.Div(
    [
        dcc.Graph(id = 'live-graph1', animate = True),
        dcc.Graph(id = 'live-graph2', animate = True),
        dcc.Graph(id = 'live-graph3', animate = True),
        dcc.Interval(
            id = 'graph-update',
            interval = 1000,
            n_intervals = 0
        ),
    ]
)
  
@app.callback(
    Output('live-graph1', 'figure'),
    [ Input('graph-update', 'n_intervals') ]
)
  
def update_graph_scatter1(n):
    t_values = []
    x_values = []
    y_values = []
    z_values = []
    yaw_values = []
    

    with open("Position.pkl", "rb") as f:
        t_values,x_values, y_values,z_values, yaw_values= pickle.load(f)
    
    data = plotly.graph_objs.Scatter(
            x=list(t_values),
            y=list(x_values),
            name='Scatter',
            mode= 'lines+markers'
    )
  
    return {'data': [data],
            'layout' : go.Layout(xaxis=dict(range=[min(t_values),max(t_values)]),yaxis = dict(range = [min(x_values),max(x_values)]),)}
  
@app.callback(
    Output('live-graph2', 'figure'),
    [ Input('graph-update', 'n_intervals') ]
)
  
def update_graph_scatter2(n):
    t_values = []
    x_values = []
    y_values = []
    z_values = []
    yaw_values = []

    with open("Position.pkl", "rb") as f:
        t_values,x_values, y_values,z_values, yaw_values= pickle.load(f)
    
    data = plotly.graph_objs.Scatter(
            x=list(t_values),
            y=list(y_values),
            name='Scatter',
            mode= 'lines+markers'
    )
  
    return {'data': [data],
            'layout' : go.Layout(xaxis=dict(range=[min(t_values),max(t_values)]),yaxis = dict(range = [min(y_values),max(y_values)]),)}
  
@app.callback(
    Output('live-graph3', 'figure'),
    [ Input('graph-update', 'n_intervals') ]
)
  
def update_graph_scatter(n):
    t_values = []
    x_values = []
    y_values = []
    z_values = []
    yaw_values = []

    with open("Position.pkl", "rb") as f:
        t_values,x_values, y_values,z_values, yaw_values= pickle.load(f)
    
    data = plotly.graph_objs.Scatter(
            x=list(t_values),
            y=list(y_values),
            name='Scatter',
            mode= 'lines+markers'
    )
  
    return {'data': [data],
            'layout' : go.Layout(xaxis=dict(range=[min(t_values),max(t_values)]),yaxis = dict(range = [min(y_values),max(y_values)]),)}
  

if __name__ == '__main__':
    app.run_server(debug=True)
