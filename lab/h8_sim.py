import os
import sys

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import case_studies.H_hummingbird.params as P
from case_studies import H_hummingbird as H_hub
from case_studies.common import SignalGenerator

# H8: Full Lateral PD Control (Successive Loop Closure)
# We also include Longitudinal (Pitch) control to keep theta near zero.

# tr_psi = 1.0, M = 10.0 => tr_phi = 0.1
# Adjust gains as needed for good performance.
dynamics = H_hub.Dynamics()
controller = H_hub.ControllerFullPD(
    tr_theta=1.0, zeta_theta=0.707,
    tr_psi=1.5, zeta_psi=0.707,
    M_bandwidth=10.0, zeta_phi=0.707,
    sigma=0.05
)

# Reference generator for yaw (psi)
psi_ref_gen = SignalGenerator(amplitude=np.radians(30), frequency=0.05)
# Zero references for phi and theta
phi_ref_input = 0.0
theta_ref = 0.0

print("Specs: tr_psi = 1.5s, M = 10, Amp = 30 deg, Freq = 0.05 Hz")

# Simulation parameters
t_final = 40.0 
dt = P.ts
time = np.arange(0, t_final, dt)

# Data histories
# State: [phi, theta, psi, phidot, thetadot, psidot]
x_hist = [np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])]
u_hist = []
# Reference history: [phi_ref, theta_ref, psi_ref]
# Set phi and theta to NaN so they are not plotted
r_hist = [np.array([np.nan, np.nan, 0.0])]

# Initial state setup
dynamics.state = x_hist[0].copy()

for i in range(len(time) - 1):
    t = time[i]
    
    psi_ref = psi_ref_gen.square(t)
    r = np.array([phi_ref_input, theta_ref, psi_ref])
    
    y = dynamics.h()
    
    u, xhat = controller.update_with_measurement(r, y)
    
    dynamics.update(u)
    
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
