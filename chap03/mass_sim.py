
import os
import sys

import numpy as np

# Add the parent directory to sys.path to enable importing from src
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from case_studies import D_mass, common

# initialize system
mass = D_mass.Dynamics()

# force = signalGenerator(amplitude=10.0, frequency=1)
force = common.SignalGenerator(amplitude=10.0, frequency=1.0)

# initialize data storage
x_hist = [mass.state]
u_hist = []

# loop over time
time = np.arange(start=0.0, stop=50.0, step=D_mass.params.ts, dtype=np.float64)
for t in time[1:]:
    # generate input signal
    u = np.array([force.sin(t)])

    # simulate system response
    y = mass.update(u)

    # store data
    u_hist.append(u)
    x_hist.append(mass.state)

# convert to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize
viz = D_mass.Visualizer(time, x_hist, u_hist)
viz.animate()
