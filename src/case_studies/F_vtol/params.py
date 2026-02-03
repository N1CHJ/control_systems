import numpy as np

# Physical parameters
mc = 1.0  # center mass
mr = 0.25 # rotor mass
Jc = 0.0042 # center inertia
d = 0.3   # distance to rotor
mu = 0.1  # drag
g = 9.8   # gravity

# Derived parameters
M = mc + 2 * mr
J = Jc + 2 * mr * d**2

# Mixing matrix
# f_r = 0.5 * F + 0.5/d * tau
# f_l = 0.5 * F - 0.5/d * tau
mixing = np.array([[0.5, 0.5/d], [0.5, -0.5/d]])

# Unmixing matrix for visualization (convert [fr, fl] back to [F, tau])
unmixer = np.linalg.inv(mixing)

# Initial Conditions
zv0 = 0.0
h0 = 0.0
theta0 = 0.0
zvdot0 = 0.0
hdot0 = 0.0
thetadot0 = 0.0

# Simulation parameters
t0 = 0.0
tf = 50.0
ts = 0.01

# Input constraints
f_max = 10.0

class Params:
	"""Minimal params for animation only."""
	def __init__(self):
		pass