import numpy as np
import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import D_mass, common

# Initialize signals
force_gen = common.SignalGenerator(amplitude=10.0, frequency=1.0)

# Initialize dynamics
mass = D_mass.Dynamics()

# Simulation loop
ts = 0.01
t_final = 10.0
time = np.arange(0, t_final, ts)

t_hist = []
x_hist = []
u_hist = []

x = np.array([D_mass.params.z0, D_mass.params.zdot0])

for t in time:
    # Get input
    u = np.array([force_gen.sin(t)])
    
    # Store data
    t_hist.append(t)
    x_hist.append(x.copy())
    u_hist.append(u)
    
    # Propagate dynamics
    x = mass.update(u)

# Visualize
viz = D_mass.Visualizer(np.array(t_hist), np.array(x_hist), np.array(u_hist))
viz.animate()
