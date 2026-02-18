import os
import sys

# 3rd-party
import numpy as np

# Add 'src' to path so we can import case_studies
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'src'))

import case_studies.H_hummingbird.params as P
from case_studies import H_hummingbird as H_hub

# initialize system and controller
hummingbird = H_hub.Dynamics()
controller = H_hub.ControllerEquilibrium()

# Run simulation with equilibrium controller.
# No reference signal needed — the controller outputs constant equilibrium PWM.
# With initial conditions all zero, the system should remain stationary.
x_hist = [hummingbird.state.copy()]
u_hist = []
t_arr = np.arange(P.t_start, P.t_end, P.ts)

for t in t_arr[:-1]:
    r = np.array([0.0])  # reference (unused by equilibrium controller)
    pwm = controller.update_with_state(r, hummingbird.state)
    hummingbird.update(pwm)
    x_hist.append(hummingbird.state.copy())
    u_hist.append(pwm)

x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# Print max deviation from zero to verify equilibrium
max_deviation = np.max(np.abs(x_hist))
print(f"Max state deviation from zero: {max_deviation:.2e}")
if max_deviation < 1e-10:
    print("SUCCESS: System remains at equilibrium.")
else:
    print("WARNING: System deviates from equilibrium!")

# visualize
viz = H_hub.Visualizer(t_arr, x_hist, u_hist)
viz.plot()
# viz.animate()
