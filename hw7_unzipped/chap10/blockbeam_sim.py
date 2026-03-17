import numpy as np
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import E_blockbeam, common

# Initialize with uncertainty
beam = E_blockbeam.Dynamics(alpha=0.2)
P = E_blockbeam.params

# Initialize PID controller with nested loop design
ctrl = E_blockbeam.ControllerPID()

ts = 0.01; t_final = 50.0; time = np.arange(0, t_final, ts)
t_hist, x_hist, u_hist, r_hist = [], [], [], []

for t in time:
    z_r = 0.35 if t >= 1.0 else 0.25
    d_in = 0.25 if t >= 25.0 else 0.0 # Input disturbance
    
    y = beam.h()
    u, xhat = ctrl.update_with_measurement(z_r, y)
    
    t_hist.append(t)
    x_hist.append(beam.state.copy())
    r_hist.append(np.array([z_r]))
    
    if t < time[-1]:
        u_hist.append(u)
        beam.update(u + d_in)

viz = E_blockbeam.Visualizer(np.array(t_hist), np.array(x_hist), np.array(u_hist), r_hist=np.array(r_hist))
viz.animate()
