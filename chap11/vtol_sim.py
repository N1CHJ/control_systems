import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import F_vtol, common

vtol = F_vtol.Dynamics()
controller = F_vtol.ControllerSS()
z_ref = common.SignalGenerator(amplitude=2.0, frequency=0.03)
h_ref = common.SignalGenerator(amplitude=2.0, frequency=0.03, y_offset=5.0)

time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    vtol,
    [z_ref, h_ref],
    controller,
    controller_input="state",
    t_final=40,
    dt=F_vtol.params.ts,
)

viz = F_vtol.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
#viz.plot()
viz.animate()
