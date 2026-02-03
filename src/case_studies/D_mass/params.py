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
F_max = 100.0

class Params:
	"""Minimal params for animation only."""
	def __init__(self):
		pass
