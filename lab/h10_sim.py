import os
import sys

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import case_studies.H_hummingbird.params as P
from case_studies import H_hummingbird as H_hub
from case_studies.common import SignalGenerator

# Add 20% uncertainty to the model parameters
dynamics = H_hub.Dynamics(alpha=0.2)

controller = H_hub.ControllerPID(
    tr_theta=1.0, zeta_theta=0.707, ki_theta=0.25,
    tr_psi=1.5, zeta_psi=0.707, ki_psi=0.1,
    M_bandwidth=10.0, zeta_phi=0.707,
    sigma=0.05
)

psi_ref_gen = SignalGenerator(amplitude=np.radians(45), frequency=0.05)
phi_ref_input = 0.0
theta_ref = 0.0

print("Specs: tr_psi = 1.5s, M = 10, Amp = 30 deg, Freq = 0.05 Hz")
print("Uncertainty: alpha = 0.2")
print("Disturbance: 0.1 N force (0.05 N per motor)")

# Simulation parameters
t_final = 60.0
dt = P.ts
time = np.arange(0, t_final, dt)

# Data histories
x_hist = [np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])]
u_hist = []
r_hist = [np.array([np.nan, np.nan, 0.0])]

# Initial state setup
dynamics.state = x_hist[0].copy()
disturbance_val = 0.05

for i in range(len(time) - 1):
    t = time[i]
    
    psi_ref = psi_ref_gen.square(t)
    r = np.array([phi_ref_input, theta_ref, psi_ref])
    
    y = dynamics.h()
    
    u, xhat = controller.update_with_measurement(r, y)
    
    # disturbance step at t=5.0
    disturbance = disturbance_val if t >= 5.0 else 0.0
    dynamics.update(u, disturbance=disturbance)
    
    x_hist.append(dynamics.state.copy())
    u_hist.append(u)
    
    psi_ref_next = psi_ref_gen.square(time[i+1])
    r_next = np.array([np.nan, np.nan, psi_ref_next])
    r_hist.append(r_next)

x_hist = np.array(x_hist)
u_hist = np.array(u_hist)
r_hist = np.array(r_hist)

viz = H_hub.Visualizer(time, x_hist, u_hist, r_hist)
# viz.plot()
viz.animate()
