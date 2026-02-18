import os
import sys

import numpy as np

# Add 'src' to path so we can import case_studies
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import case_studies.H_hummingbird.params as P
from case_studies import H_hummingbird as H_hub
from case_studies.common import SignalGenerator

# Instantiate Dynamics (use full EOM model)
dynamics = H_hub.Dynamics()

# Input Signals (phi, theta, psi)
F_hover = (P.m1 * P.l1 + P.m2 * P.l2) * P.g / P.lT

# phi_sig = 0.0
theta_sig = 0.0
psi_sig = 0.0

phi_sig = SignalGenerator(amplitude=np.radians(0.01), frequency=1)
# theta_sig = SignalGenerator(amplitude=np.radians(0.01), frequency=1)
# psi_sig = SignalGenerator(amplitude=np.radians(0.1), frequency=1)

# Simulation Loop
x_hist = [dynamics.state]
u_hist = []
t_arr = np.arange(P.t_start, P.t_end, P.ts)

def signal_value(sig, t):
    return sig.sin(t) if hasattr(sig, "sin") else float(sig)


print("PLEASE WORK...")
for t in t_arr[:-1]:
    d_theta = signal_value(theta_sig, t)
    d_phi = signal_value(phi_sig, t)
    d_psi = signal_value(psi_sig, t)

    dF = d_theta
    dTau = d_phi + d_psi
    fl = 0.5 * (F_hover + dF) + dTau
    fr = 0.5 * (F_hover + dF) - dTau
    u = np.array([fl, fr])
    u = np.clip(u, 0.0, None)
    
    # Propagate Dynamics
    dynamics.update(u)
    
    # Store Data
    x_hist.append(dynamics.state.copy())
    u_hist.append(u)

# Animation
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)
viz = H_hub.Visualizer(t_arr, x_hist, u_hist)
viz.animate()
