import os
import sys

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))


from case_studies import E_blockbeam, common

# initialize signals for generating data
z_gen = common.SignalGenerator(amplitude=0.5, frequency=0.3)
th_gen = common.SignalGenerator(amplitude=0.2, frequency=0.7)
u_gen = common.SignalGenerator(amplitude=1.0, frequency=0.5)


# initialize data storage
x0 = np.zeros(4)
x_hist = [x0]
u_hist = []

# loop over time
time = np.arange(start=0, stop=10, step=0.02, dtype=np.float64)
for t in time[1:]:
    # generate fake state and input data
    x = np.empty(4)
    x[0] = z_gen.sin(t)
    x[1] = th_gen.sin(t)
    x[2] = 0.0  # z_dot (dummy)
    x[3] = 0.0  # th_dot (dummy)
    u = np.array([u_gen.sawtooth(t)])
    # store data for visualization
    x_hist.append(x)
    u_hist.append(u)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize generated data
viz = E_blockbeam.Visualizer(time, x_hist, u_hist)
viz.animate()
