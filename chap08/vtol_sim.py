import numpy as np
import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import F_vtol, common

class VTOLControllerPD:
    def __init__(self, kp_h, kd_h, kp_z, kd_z, kp_th, kd_th, limit):
        self.kp_h = kp_h
        self.kd_h = kd_h
        self.kp_z = kp_z
        self.kd_z = kd_z
        self.kp_th = kp_th
        self.kd_th = kd_th
        self.limit = limit
        self.P = F_vtol.params
        
    def update(self, h_r, z_r, state):
        zv, h, th, zv_dot, h_dot, th_dot = state
        
        # Altitude loop (PD)
        F = self.kp_h * (h_r - h) - self.kd_h * h_dot + (self.P.M * self.P.g)
        
        # Lateral loop (Outer) -> outputs desired theta
        th_r = self.kp_z * (z_r - zv) - self.kd_z * zv_dot
        
        # Roll loop (Inner) -> outputs torque
        tau = self.kp_th * (th_r - th) - self.kd_th * th_dot
        
        # Mix and Saturate
        u = self.P.mixing @ np.array([F, tau])
        u_sat = np.clip(u, -self.limit, self.limit)
        return u_sat

# Initialize
vtol = F_vtol.Dynamics()
# Gains derived previously + inner loop guesses
kp_h = 1.815; kd_h = 2.333
kp_z = -0.1235; kd_z = -0.1587
kp_th = 0.5; kd_th = 0.1
ctrl = VTOLControllerPD(kp_h, kd_h, kp_z, kd_z, kp_th, kd_th, F_vtol.params.force_max)

# Simulation
ts = 0.01
t_final = 30.0
time = np.arange(0, t_final, ts)

t_hist = []
x_hist = []
u_hist = []
r_hist = []

for t in time:
    h_r = 5.0 if t >= 1.0 else 0.0
    z_r = 2.0 if t >= 5.0 else 0.0
    u = ctrl.update(h_r, z_r, vtol.state)
    
    t_hist.append(t)
    x_hist.append(vtol.state.copy())
    u_hist.append(u)
    r_hist.append(np.array([z_r, h_r]))
    
    vtol.update(u)

# Visualize
viz = F_vtol.Visualizer(np.array(t_hist), np.array(x_hist), np.array(u_hist), r_hist=np.array(r_hist))
viz.animate()
