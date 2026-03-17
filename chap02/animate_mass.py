import numpy as np
import os
import sys

# Ensure src is in the path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import D_mass, common

# Initialize signals for generating data: z(t) = A*sin(2*pi*f*t) + Y
z_gen = common.SignalGenerator(amplitude=1.5, frequency=0.1, y_offset=2.0)
u_gen = common.SignalGenerator(amplitude=10.0, frequency=0.2)

# Time parameters
ts = 0.02
time = np.arange(start=0, stop=10, step=ts, dtype=np.float64)

# Generate trajectory data
x_hist = []
u_hist = []
for t in time:
    # Generalized coordinate: z
    x = np.array([z_gen.sin(t), 0.0]) # [z, z_dot]
    x_hist.append(x)
    if t < time[-1]:
        u_hist.append(np.array([u_gen.sin(t)]))

# Convert to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# Visualize
viz = D_mass.Visualizer(time, x_hist, u_hist)
viz.animate()
