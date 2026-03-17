import numpy as np
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import F_vtol, common

# Uncertainty
vtol = F_vtol.Dynamics(alpha=0.2)
P = F_vtol.params

# Initialize PID controller with nested loop design
ctrl = F_vtol.ControllerPID()

ts = 0.01; t_final = 40.0; time = np.arange(0, t_final, ts)
t_hist, x_hist, u_hist, r_hist = [], [], [], []

for t in time:
    r = np.array([2.0, 5.0]) if t >= 1.0 else np.array([0.0, 0.0])
    # Disturbance on altitude force
    d_in = np.array([0.5, 0.0]) if t >= 20.0 else np.array([0.0, 0.0])
    
    y = vtol.h() # Measures [z, h, theta]
    u, xhat = ctrl.update_with_measurement(r, y)
    
    t_hist.append(t)
    x_hist.append(vtol.state.copy())
    r_hist.append(r)
    
    if t < time[-1]:
        u_hist.append(u)
        vtol.update(u + d_in)

viz = F_vtol.Visualizer(np.array(t_hist), np.array(x_hist), np.array(u_hist), r_hist=np.array(r_hist))
viz.animate()
