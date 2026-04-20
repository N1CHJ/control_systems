import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import D_mass, common

mass = D_mass.Dynamics()
controller = D_mass.ControllerSSI(separate_integrator=False)
z_ref = common.SignalGenerator(amplitude=0.5, frequency=0.1) # increased frequency
d_force = np.array([0.25])

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    mass,
    [z_ref],
    controller,
    controller_input="state",
    input_disturbance=d_force,
    t_final=50, # increased time
    dt=D_mass.params.ts,
)

viz = D_mass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist)
# viz.plot()
viz.animate()
