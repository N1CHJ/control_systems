import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import E_blockbeam, common

beam = E_blockbeam.Dynamics()
controller = E_blockbeam.ControllerSS()
z_ref = common.SignalGenerator(amplitude=0.25, frequency=0.05)

time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    beam,
    [z_ref],
    controller,
    controller_input="state",
    t_final=40,
    dt=E_blockbeam.params.ts,
)

viz = E_blockbeam.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
#viz.plot()
viz.animate()
