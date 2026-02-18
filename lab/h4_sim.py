import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import case_studies.H_hummingbird.params as P
from case_studies import H_hummingbird as H_hub
from case_studies.common import SignalGenerator, run_simulation

dynamics = H_hub.Dynamics()
controller = H_hub.ControllerEquilibrium()
ref = SignalGenerator(amplitude=0.0, frequency=0.1)

# Run simulation
time, x_hist, u_hist, r_hist, *_ = run_simulation(
    sys=dynamics,
    refs=[ref],
    controller=controller,
    controller_input="state",
    t_final=P.t_end,
    dt=P.ts,
)

# Animation
viz = H_hub.Visualizer(time, x_hist, u_hist)
viz.animate()
