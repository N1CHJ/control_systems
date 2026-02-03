import numpy as np
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies import D_mass, common

# initialize signals for generating data
z_gen = common.SignalGenerator(amplitude=1.0, frequency=0.5)
u_gen = common.SignalGenerator(amplitude=2.0, frequency=0.3)


# initialize data storage
x0 = np.zeros(2)
x_hist = [x0]
u_hist = []

# loop over time
time = np.arange(start=0, stop=10, step=0.02, dtype=np.float64)
for t in time[1:]:
    # generate fake state and input data
    x = np.empty(2)
    x[0] = z_gen.sin(t)
    x[1] = 0.0  # z_dot (dummy)
    u = np.array([u_gen.sawtooth(t)])
    # store data for visualization
    x_hist.append(x)
    u_hist.append(u)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize generated data
viz = D_mass.Visualizer(time, x_hist, u_hist)
viz.animate()
