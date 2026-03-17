import numpy as np
import os
import sys

# Ensure src is in the path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import E_blockbeam, common

# Initialize signals: z(t) and theta(t)
# Beam length is 1.5m; center the block at 0.75m
z_gen = common.SignalGenerator(amplitude=0.5, frequency=0.1, y_offset=0.75)
theta_gen = common.SignalGenerator(amplitude=np.radians(15), frequency=0.05, y_offset=0.0)
u_gen = common.SignalGenerator(amplitude=2.0, frequency=0.2)

# Time parameters
ts = 0.02
time = np.arange(start=0, stop=10, step=ts, dtype=np.float64)

# Generate trajectory data
x_hist = []
u_hist = []
for t in time:
    # Generalized coordinates: [z, theta]
    x = np.array([z_gen.sin(t), theta_gen.sin(t), 0.0, 0.0]) # [z, theta, z_dot, theta_dot]
    x_hist.append(x)
    if t < time[-1]:
        u_hist.append(np.array([u_gen.sin(t)]))

# Convert to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# Visualize
viz = E_blockbeam.Visualizer(time, x_hist, u_hist)
viz.animate()
