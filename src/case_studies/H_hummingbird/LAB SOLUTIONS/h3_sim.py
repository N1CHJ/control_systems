# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import H_hummingbird


# alias for parameters
P = H_hummingbird.params

# initialize system
hummingbird = H_hummingbird.Dynamics()

# initialize data storage
x_hist = [hummingbird.state]
u_hist = []

# loop over time
time = np.arange(start=0.0, stop=20.0, step=P.ts, dtype=np.float64)
for t in time[1:]:
    # Could use SignalGenerator to make time-varying inputs, but the system is
    # pretty sensitive and constant inputs help show the behavior more clearly

    # generate input signal [f_l, f_r]
    u = np.ones(2) * 0.45  # roughly hovers
    # u = np.array([0.005, 0.0])  # falls with a little (positive) spin

    # simulate system response
    y = hummingbird.update(u)

    # store data for visualization
    u_hist.append(u)
    x_hist.append(hummingbird.state)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize
viz = H_hummingbird.Visualizer(time, x_hist, u_hist)
viz.animate()  # could also just call viz.plot()
