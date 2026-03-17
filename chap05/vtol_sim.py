
import os
import sys

import numpy as np

# Add the parent directory to sys.path to enable importing from src
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from case_studies import F_vtol, common

# initialize system
vtol = F_vtol.Dynamics()

force = common.SignalGenerator(amplitude=0.5, frequency=1.0, y_offset=14.715)
torque = common.SignalGenerator(amplitude=0.001, frequency=1.0, y_offset=0.01)


# initialize data storage
x_hist = [vtol.state]
u_hist = []

# loop over time
time = np.arange(start=0.0, stop=15.0, step=F_vtol.params.ts, dtype=np.float64)
for t in time[1:]:
    # generate input signal
    # u = P.mixing @ np.array([[force.sin(t)], [torque.sin(t)]])
    inputs = np.array([[force.sin(t)], [torque.sin(t)]])
    u = F_vtol.params.mixing @ inputs
    u = u.flatten() # Make sure it is 1D array [fr, fl]

    # simulate system response
    y = vtol.update(u)

    # store data
    u_hist.append(u)
    x_hist.append(vtol.state)

# convert to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize
viz = F_vtol.Visualizer(time, x_hist, u_hist)
viz.animate()
