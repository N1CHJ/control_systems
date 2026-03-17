import numpy as np
import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import E_blockbeam, common

class BlockbeamControllerPD:
    def __init__(self, kp, kd, limit):
        self.kp = kp
        self.kd = kd
        self.limit = limit
        
    def update(self, z_r, state):
        z = state[0]
        z_dot = state[2] # z_dot is the 3rd state
        
        # PD control law (Outer loop)
        u = self.kp * (z_r - z) - self.kd * z_dot
        
        # Saturation
        u_sat = np.clip(u, -self.limit, self.limit)
        return np.array([u_sat])

# Initialize
beam = E_blockbeam.Dynamics()
# Gains for tr=4.0, zeta=0.707
# kp_z = -0.0308, kd_z = -0.0794 (approximate)
kp = -0.0308
kd = -0.0794
ctrl = BlockbeamControllerPD(kp, kd, E_blockbeam.params.force_max)

# Simulation
ts = 0.01
t_final = 30.0
time = np.arange(0, t_final, ts)

t_hist = []
x_hist = []
u_hist = []
r_hist = []

for t in time:
    z_r = 0.5 if t >= 1.0 else 0.0
    u = ctrl.update(z_r, beam.state)
    
    t_hist.append(t)
    x_hist.append(beam.state.copy())
    u_hist.append(u)
    r_hist.append(np.array([z_r]))
    
    beam.update(u)

# Visualize
viz = E_blockbeam.Visualizer(np.array(t_hist), np.array(x_hist), np.array(u_hist), r_hist=np.array(r_hist))
viz.animate()
