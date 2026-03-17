import numpy as np
import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import E_blockbeam, common

# Initialize signals: force = 0.5*sin(2*pi*1.0*t) + 11.5
force_gen = common.SignalGenerator(amplitude=0.5, frequency=1.0, y_offset=11.5)

# Initialize dynamics
beam = E_blockbeam.Dynamics()

# Simulation loop
ts = 0.01
t_final = 10.0
time = np.arange(0, t_final, ts)

t_hist = []
x_hist = []
u_hist = []

for t in time:
    # Get input
    u = np.array([force_gen.sin(t)])
    
    # Store data
    t_hist.append(t)
    x_hist.append(beam.state.copy())
    if t < time[-1]:
        u_hist.append(u)
    
    # Propagate dynamics
    beam.update(u)

# Visualize
viz = E_blockbeam.Visualizer(np.array(t_hist), np.array(x_hist), np.array(u_hist))
viz.animate()
