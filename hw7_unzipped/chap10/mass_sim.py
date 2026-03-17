import numpy as np
import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import D_mass, common

# Initialize dynamics with 20% uncertainty (alpha=0.2)
mass = D_mass.Dynamics(alpha=0.2)
P = D_mass.params

# Initialize PID controller with tuned gains
ctrl = D_mass.ControllerPID()

# Simulation setup
ts = 0.01
t_final = 20.0
time = np.arange(0, t_final, ts)

t_hist, x_hist, u_hist, r_hist = [], [], [], []

for t in time:
    # Reference and Disturbance
    z_r = 1.0 if t >= 1.0 else 0.0
    d_in = 2.0 if t >= 10.0 else 0.0 # Step disturbance at t=10
    
    # Measure output y = [z]
    y = mass.h()
    
    # Update controller
    u, xhat = ctrl.update_with_measurement(z_r, y)
    
    # Store data
    t_hist.append(t)
    x_hist.append(mass.state.copy())
    r_hist.append(np.array([z_r]))
    
    if t < time[-1]:
        u_hist.append(u)
        # Apply input with disturbance and propagate
        mass.update(u + d_in)

# Visualize
viz = D_mass.Visualizer(np.array(t_hist), np.array(x_hist), np.array(u_hist), r_hist=np.array(r_hist))
viz.animate()
