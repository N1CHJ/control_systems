import numpy as np
import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import D_mass, common

# Controller implementation
class MassControllerPD:
    def __init__(self, kp, kd, limit):
        self.kp = kp
        self.kd = kd
        self.limit = limit
        
    def update(self, z_r, state):
        z = state[0]
        z_dot = state[1]
        
        # PD control law
        u = self.kp * (z_r - z) - self.kd * z_dot
        
        # Saturation (D.8)
        u_sat = np.clip(u, -self.limit, self.limit)
        return np.array([u_sat])

# Initialize
mass = D_mass.Dynamics()
# Calculated gains
kp = 3.05
kd = 7.275
ctrl = MassControllerPD(kp, kd, D_mass.params.force_max)

# Simulation
ts = 0.01
t_final = 20.0
time = np.arange(0, t_final, ts)

# Reference signal: step at t=1.0
z_ref_gen = common.SignalGenerator(amplitude=1.5, frequency=0.0)

t_hist = []
x_hist = []
u_hist = []
r_hist = []

for t in time:
    z_r = 1.0 if t >= 1.0 else 0.0
    u = ctrl.update(z_r, mass.state)
    
    t_hist.append(t)
    x_hist.append(mass.state.copy())
    u_hist.append(u)
    r_hist.append(np.array([z_r]))
    
    mass.update(u)

# Visualize
viz = D_mass.Visualizer(np.array(t_hist), np.array(x_hist), np.array(u_hist), r_hist=np.array(r_hist))
viz.animate()
