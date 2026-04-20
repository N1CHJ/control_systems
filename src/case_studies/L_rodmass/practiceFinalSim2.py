"""
Practice Final Exam - Part 2: Equilibrium Verification
Rod-Mass System

This script verifies that the equilibrium torque holds the system at rest.
Students implement dynamics in dynamics.py, then run this to verify.
"""
# 3rd-party
import numpy as np
import matplotlib.pyplot as plt

# local (controlbook)
from .. import common
from . import params as P
from .dynamics import RodMassDynamics as Dynamics
from .visualizer import RodMassVisualizer as Visualizer
from .constant_controller import ConstantController

# Instantiate system and controller
system = Dynamics(alpha=0.0)  # no parameter uncertainty
reference = common.SignalGenerator(amplitude=0.0)  # zero reference

# Equilibrium torque calculated from design_model_tool.py for theta_e = 0.0
# (For the practice final theta_e = 0, u_e is approximately 0.245)
u_e = 0.245 
controller = ConstantController(u_e)

# Run simulation
time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    system,
    [reference],
    controller,
    controller_input="measurement",
    t_final=5.0,
    dt=P.ts
)

# Print verification results
print(f"Final state at equilibrium test: {x_hist[-1]}")

# Visualize results
viz = Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
viz.plot()
viz.animate()

