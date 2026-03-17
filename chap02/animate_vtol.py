import numpy as np
import os
import sys

# Ensure src is in the path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import F_vtol, common

# Initialize signals: zv(t), h(t), and theta(t)
zv_gen = common.SignalGenerator(amplitude=2.0, frequency=0.05, y_offset=0.0)
h_gen = common.SignalGenerator(amplitude=1.0, frequency=0.1, y_offset=3.0)
theta_gen = common.SignalGenerator(amplitude=np.radians(10), frequency=0.15, y_offset=0.0)
u_gen = common.SignalGenerator(amplitude=5.0, frequency=0.2)

# Time parameters
ts = 0.02
time = np.arange(start=0, stop=10, step=ts, dtype=np.float64)

# Generate trajectory data
x_hist = []
u_hist = []
for t in time:
    # Generalized coordinates: [zv, h, theta]
    x = np.array([zv_gen.sin(t), h_gen.sin(t), theta_gen.sin(t), 0.0, 0.0, 0.0])
    x_hist.append(x)
    if t < time[-1]:
        # Two inputs: [fr, fl] (right and left rotor forces)
        u_hist.append(np.array([u_gen.sin(t), u_gen.sin(t)]))

# Convert to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# Visualize
viz = F_vtol.Visualizer(time, x_hist, u_hist)
viz.animate()
