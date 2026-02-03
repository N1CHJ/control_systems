import os
import sys

import numpy as np

# Add 'src' to path so we can import case_studies
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import case_studies.H_hummingbird.params as P
from case_studies import H_hummingbird, common

# initialize signals for generating data
# phi (roll), theta (pitch), psi (yaw)
phi_gen = common.SignalGenerator(amplitude=np.radians(30), frequency=0.2)
theta_gen = common.SignalGenerator(amplitude=np.radians(30), frequency=0.3)
psi_gen = common.SignalGenerator(amplitude=np.radians(30), frequency=0.1)

ul_gen = common.SignalGenerator(amplitude=0.5, frequency=0.5)
ur_gen = common.SignalGenerator(amplitude=0.5, frequency=0.5)

# initialize data storage
#### [phi, theta, psi, phidot, thetadot, psidot]
x0 = np.zeros(6)
x_hist = [x0]
u_hist = []

# loop over time
time = np.arange(start=P.t_start, stop=P.t_end, step=P.ts, dtype=np.float64)

for t in time[1:]:
    # generate fake state data
    x = np.zeros(6)
    x[0] = phi_gen.sin(t)
    x[1] = theta_gen.square(t)
    x[2] = psi_gen.sin(t)

    # generate fake input data
    u = np.array([ul_gen.sin(t), ur_gen.sin(t)])

    # store data
    x_hist.append(x)
    u_hist.append(u)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize generated data
viz = H_hummingbird.Visualizer(time, x_hist, u_hist)
viz.animate()
