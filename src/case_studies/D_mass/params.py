import numpy as np

# ...existing code...
# Physical parameters
m = 5.0  # mass (kg)
k = 3.0  # spring constant (N/m)
b = 0.5  # damping coefficient (Ns/m)

# Initial Conditions
z0 = 0.0
zdot0 = 0.0

# Simulation parameters
ts = 0.01

# Input constraint
force_max = 100.0

##### Chapter 4 / 11-14
# Linearization/equilibrium point
x_eq = np.array([z0, zdot0])
u_eq = np.array([0.0])
r_eq = np.array([z0])

##### Chapter 6 / 11-14
# State space
A = np.array([[0.0, 1.0], [-k/m, -b/m]])
B = np.array([[0.0, 1/m]]).T
Cm = np.array([[1.0, 0.0]])  # measure position
Cr = Cm
D = np.array([[0.0]])

class Params:
	"""Minimal params for animation only."""
	def __init__(self):
		pass
