import numpy as np

# Physical parameters
m1 = 2.0
m2 = 5.0
L = 1.5
g = 9.8
I_beam = 2.0

# Initial Conditions
z0 = 0.5
theta0 = 0.0
zdot0 = 0.0
thetadot0 = 0.0

# Simulation parameters
t0 = 0.0
tf = 50.0
ts = 0.01

# Input constraint
F_max = 20.0

class Params:
	"""Minimal params for animation only."""
	def __init__(self):
		pass
