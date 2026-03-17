import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import D_mass, common

mass = D_mass.Dynamics()
controller = D_mass.ControllerSS()
z_ref = common.SignalGenerator(amplitude=1.5, frequency=0.05)

time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    mass,
    [z_ref],
    controller,
    controller_input="state",
    t_final=20,
    dt=D_mass.params.ts,
)

viz = D_mass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
# viz.plot()
viz.animate()
