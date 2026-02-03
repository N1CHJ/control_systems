import numpy as np

import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src')) #I had to add this to fix the import for some reason.
from case_studies import F_vtol, common

# initialize signals for generating data
zv_gen = common.SignalGenerator(amplitude=1.0, frequency=0.2)
h_gen = common.SignalGenerator(amplitude=0.5, frequency=0.3, y_offset=1.0)
th_gen = common.SignalGenerator(amplitude=0.2, frequency=0.5)
u_gen = common.SignalGenerator(amplitude=1.0, frequency=0.4)


# initialize data storage
x0 = np.zeros(6)
x_hist = [x0]
u_hist = []

# loop over time
time = np.arange(start=0, stop=10, step=0.02, dtype=np.float64)
for t in time[1:]:
    # generate fake state and input data
    x = np.empty(6)
    x[0] = zv_gen.sin(t)
    x[1] = h_gen.sin(t)
    x[2] = th_gen.sin(t)
    x[3] = 0.0  # zv_dot (dummy)
    x[4] = 0.0  # h_dot (dummy)
    x[5] = 0.0  # th_dot (dummy)
    u = np.array([u_gen.sawtooth(t), u_gen.sawtooth(t)])  # dummy vals
    # store data for visualization
    x_hist.append(x)
    u_hist.append(u)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize generated data
viz = F_vtol.Visualizer(time, x_hist, u_hist)
viz.animate()
