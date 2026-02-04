import os
import sys

import numpy as np

# Add 'src' to path so we can import case_studies
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import case_studies.H_hummingbird.params as P
from case_studies import H_hummingbird as H_hub
from case_studies.common import SignalGenerator

# Instantiate Dynamics
dynamics = H_hub.Dynamics_h3()

# Input Signals (Forces)
fl_gen = SignalGenerator(amplitude=2.0, frequency=0.5, y_offset=0.0)
fr_gen = SignalGenerator(amplitude=2.0, frequency=0.5, y_offset=0.0)

# Simulation Loop
x_hist = [dynamics.state]
u_hist = []
t_arr = np.arange(P.t_start, P.t_end, P.ts)

print("Starting Simulation... PLEASE WORK...")
for t in t_arr[:-1]:
    # Calculate Input
    u = np.array([fl_gen.sin(t) + 0.5, fr_gen.sin(t)]) 
    
    # Propagate Dynamics
    dynamics.update(u)
    
    # Store Data
    x_hist.append(dynamics.state.copy())
    u_hist.append(u)

# Animation
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)
viz = H_hub.Visualizer(t_arr, x_hist, u_hist)
viz.animate()
