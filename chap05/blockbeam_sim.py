
import os
import sys

import numpy as np

# Add the parent directory to sys.path to enable importing from src
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from case_studies import E_blockbeam, common

# initialize system and input generator
blockbeam = E_blockbeam.Dynamics()
force_gen = common.SignalGenerator(amplitude=0.5, frequency=1.0, y_offset=11.5)
# initialize data storage
x_hist = [blockbeam.state]
u_hist = []
# loop over time
time = np.arange(start=0.0, stop=5.0, step=E_blockbeam.params.ts, dtype=np.float64)
for t in time[1:]:
 # generate input signal
 u = np.array([force_gen.sin(t)])
 # simulate system
 y = blockbeam.update(u)
 # store data for visualization
 u_hist.append(u)
 x_hist.append(blockbeam.state)
# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)
# visualize
viz = E_blockbeam.Visualizer(time, x_hist, u_hist)
viz.animate() # could also just call viz.plot()
