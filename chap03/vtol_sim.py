import numpy as np
import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import F_vtol, common

# Initialize signals
# force: amplitude=0.5, frequency=1.0, y_offset=14.715
# torque: amplitude=0.001, frequency=1.0, y_offset=-0.01
force_gen = common.SignalGenerator(amplitude=0.5, frequency=1.0, y_offset=14.715)
torque_gen = common.SignalGenerator(amplitude=0.001, frequency=1.0, y_offset=-0.01)

# Initialize dynamics
vtol = F_vtol.Dynamics()
P = F_vtol.params

# Simulation loop
ts = 0.01
t_final = 10.0
time = np.arange(0, t_final, ts)

t_hist = []
x_hist = []
u_hist = []

for t in time:
    # Calculate force and torque
    f = force_gen.sin(t)
    tau = torque_gen.sin(t)
    
    # Apply mixing matrix: u = [fr, fl]
    u = P.mixing @ np.array([f, tau])
    
    # Store data
    t_hist.append(t)
    x_hist.append(vtol.state.copy())
    if t < time[-1]:
        u_hist.append(u)
    
    # Propagate dynamics
    vtol.update(u)

# Visualize
viz = F_vtol.Visualizer(np.array(t_hist), np.array(x_hist), np.array(u_hist))
viz.animate()
