import os
import sys

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import case_studies.H_hummingbird.params as P
from case_studies import H_hummingbird as H_hub
from case_studies.common import SignalGenerator

# tr = 1.0, zeta = 0.707. Feel free to adjust.
dynamics = H_hub.Dynamics()
controller = H_hub.ControllerLonPD(tr=1.0, zeta=0.707, sigma=0.05)
theta_ref_gen = SignalGenerator(amplitude=np.radians(30), frequency=0.1)

print("Specs: tr = 1.0s, zeta = 0.707, Amp = 30 deg, Freq = 0.1 Hz")

# Simulation parameters
t_final = 20.0 
dt = P.ts
time = np.arange(0, t_final, dt)

# Data histories
x_hist = [np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])]
u_hist = []

r_hist = [np.array([0.0, theta_ref_gen.square(0.0), 0.0])]

# Initial state setup
dynamics.state = x_hist[0].copy()

for i in range(len(time) - 1):
    t = time[i]
    
    theta_ref = theta_ref_gen.square(t)
    r = np.array([theta_ref])
    
    y = dynamics.h()
    
    u, xhat = controller.update_with_measurement(r, y)
    
    dynamics.update(u)
    
    dynamics.state[0] = 0.0 # phi
    dynamics.state[2] = 0.0 # psi
    dynamics.state[3] = 0.0 # phidot
    dynamics.state[5] = 0.0 # psidot
    
    x_hist.append(dynamics.state.copy())
    u_hist.append(u)
    
    r_next = np.array([0.0, theta_ref_gen.square(time[i+1]), 0.0])
    r_hist.append(r_next)

# Convert to numpy arrays for visualization
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)
r_hist = np.array(r_hist)

viz = H_hub.Visualizer(time, x_hist, u_hist, r_hist)
# viz.plot()
viz.animate()
