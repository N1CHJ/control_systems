"""
Practice Final Exam - Part 4: State-Space Control with Observers
Rod-Mass System

This script demonstrates:
- Part 4.2: State-space control with integrator (observer-based)
- Part 4.5: Observer-based control with input disturbance
- Part 4.6: Observer estimation error analysis
- Part 4.7: LQR controller for faster response
"""
# 3rd-party
import numpy as np
import matplotlib.pyplot as plt

# local (controlbook)
from .. import common
from .dynamics import RodMassDynamics as Dynamics
from .ssi_controller import RodMassSSIController as SSIController
from .ssi_dist_obs_controller import RodMassSSIDOController as SSIDOController
from .lqr_controller import RodMassLQRController as LQRController
from .visualizer import RodMassVisualizer as Visualizer
from . import params as P

print("\n" + "="*60)
print("Part 4: State-Space Control with Observers")
print("="*60)

# Reference signal - 20 degree square wave at 0.1 Hz
reference = common.SignalGenerator(amplitude=20*np.pi/180, frequency=0.1)

#=========================================================================
# Part 4.2: State-Space Control with Integrator (Full State Feedback)
#=========================================================================
print("\n--- Part 4.2: State-Space Control (Full State) ---")

# Instantiate nominal system and SSI controller
system_ssi = Dynamics(alpha=0.0)
controller_ssi = SSIController()

# Run simulation (no input disturbance in this section)
time_ssi, x_ssi, u_ssi, r_ssi, xhat_ssi, d_ssi, dhat_ssi = common.run_simulation(
    system_ssi,
    [reference],
    controller_ssi,
    controller_input="state",
    t_final=20.0,
    dt=P.ts
)

# Visualize
viz_ssi = Visualizer(time_ssi, x_ssi, u_ssi, r_ssi, xhat_ssi)
viz_ssi.plot()


#=========================================================================
# Parts 4.5 and 4.6: Observer-Based Control WITH Input Disturbance
#=========================================================================
print("\n--- Parts 4.5 and 4.6: Observer with Input Disturbance ---")

# Instantiate uncertain system (alpha=0.1) and SSIDO controller
system_dist = Dynamics(alpha=0.1)
controller_dist = SSIDOController()

# Constant input disturbance of 0.5 N-m
input_disturbance = np.array([0.5])

# Run simulation with disturbance and measurements (requires observer)
time_dist, x_dist, u_dist, r_dist, xhat_dist, d_dist, dhat_dist = common.run_simulation(
    system_dist,
    [reference],
    controller_dist,
    controller_input="measurement",
    input_disturbance=input_disturbance,
    t_final=20.0,
    dt=P.ts
)

# Visualize with disturbance plots
viz_dist = Visualizer(time_dist, x_dist, u_dist, r_dist, xhat_dist, d_dist, dhat_dist)
viz_dist.plot()

#=========================================================================
# Part 4.7: LQR Controller for Faster Response
#=========================================================================
print("\n--- Part 4.7: LQR Controller ---")

# Instantiate system and LQR controller
system_lqr = Dynamics(alpha=0.1)
controller_lqr = LQRController()

# Run simulation with disturbance
time_lqr, x_lqr, u_lqr, r_lqr, xhat_lqr, d_lqr, dhat_lqr = common.run_simulation(
    system_lqr,
    [reference],
    controller_lqr,
    controller_input="measurement",
    input_disturbance=input_disturbance,
    t_final=20.0,
    dt=P.ts
)

# Visualize LQR results
viz_lqr = Visualizer(time_lqr, x_lqr, u_lqr, r_lqr, xhat_lqr, d_lqr, dhat_lqr)
viz_lqr.plot()
# viz_lqr.animate()

print("\n" + "="*60)
print("State-Space Control Complete")
print("="*60)
plt.show()
