"""
Practice Final Exam - Part 3: PID Control
Rod-Mass System

This script demonstrates PID control with and without parameter uncertainty.
- First run: PD control (no integrator needed) with nominal parameters
- Second run: PID control with 10% parameter uncertainty
"""

# 3rd-party
import numpy as np
import matplotlib.pyplot as plt

# local (controlbook)
from .. import common
from .dynamics import RodMassDynamics as Dynamics
from .pid_controller import RodMassControllerPID as Controller
from .visualizer import RodMassVisualizer as Visualizer
from . import params as P

print("\n" + "="*60)
print("Part 3: PID Control")
print("="*60)

# Reference signal - 20 degree square wave at 0.1 Hz
reference = common.SignalGenerator(amplitude=20*np.pi/180, frequency=0.1)

#=========================================================================
# Part 3.5: PD Control with Nominal Parameters
#=========================================================================
print("\n--- Part 3.5: PD Control (Nominal) ---")

# Instantiate system with nominal parameters (alpha=0.0)
system_pd = Dynamics(alpha=0.0)

# Define PD controller (ki=0)
# Tuned for reasonable performance
kp = 1.0
kd = 0.5
controller_pd = Controller(kp=kp, kd=kd, ki=0.0, ts=P.ts)

# Run simulation
time_pd, x_pd, u_pd, r_pd, xhat_pd, *_ = common.run_simulation(
    system_pd,
    [reference],
    controller_pd,
    controller_input="measurement",
    t_final=20.0,
    dt=P.ts
)

# Visualize
viz_pd = Visualizer(time_pd, x_pd, u_pd, r_pd, xhat_pd)
viz_pd.plot()


#=========================================================================
# Part 3.6: PID Control with 10% Parameter Uncertainty
#=========================================================================
print("\n--- Part 3.6: PID Control with 10% Uncertainty ---")

# Instantiate system with 10% uncertainty
system_pid = Dynamics(alpha=0.1)

# Define PID controller (ki > 0 to reject steady-state error from uncertainty)
ki = 0.5
controller_pid = Controller(kp=kp, kd=kd, ki=ki, ts=P.ts)

# Run simulation
time_pid, x_pid, u_pid, r_pid, xhat_pid, *_ = common.run_simulation(
    system_pid,
    [reference],
    controller_pid,
    controller_input="measurement",
    t_final=20.0,
    dt=P.ts
)

# Visualize
viz_pid = Visualizer(time_pid, x_pid, u_pid, r_pid, xhat_pid)
viz_pid.plot()
# viz_pid.animate() # Optional: uncomment if animation is needed

print("\n" + "="*60)
print("PID Control Complete")
print("="*60)
plt.show()
