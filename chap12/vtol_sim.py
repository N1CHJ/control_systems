import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import F_vtol, common

vtol = F_vtol.Dynamics()
controller = F_vtol.ControllerSSI(separate_integrator=False)
z_ref = common.SignalGenerator(amplitude=1.5, frequency=0.08)
h_ref = common.SignalGenerator(amplitude=1.0, frequency=0.12)
d_force = np.array([0.1, 0.1])

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    vtol,
    [z_ref, h_ref],
    controller,
    controller_input="state",
    input_disturbance=d_force,
    t_final=50, # increased time
    dt=F_vtol.params.ts,
)

viz = F_vtol.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist)
# viz.plot()
viz.animate()
