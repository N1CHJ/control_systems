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
z0 = 0.0
h0 = 0.0
theta0 = 0.0
zdot0 = 0.0
hdot0 = 0.0
thetadot0 = 0.0

# Simulation parameters
t0 = 0.0
tf = 50.0
ts = 0.01

# Input constraints
force_max = 10.0

##### Chapter 4 / 11-14
# Linearization/equilibrium point
x_eq = np.array([z0, h0, theta0, zdot0, hdot0, thetadot0])
u_eq = np.array([M * g / 2, M * g / 2])
r_eq = np.array([z0, h0])

##### Chapter 6 / 11-14
# State space
# x = [zv, h, theta, zvdot, hdot, thetadot]
A = np.zeros((6, 6))
A[0, 3] = 1.0
A[1, 4] = 1.0
A[2, 5] = 1.0
A[3, 2] = -g
A[3, 3] = -mu / M
B = np.zeros((6, 2))
B[4, 0] = 1 / M
B[4, 1] = 1 / M
B[5, 0] = d / J
B[5, 1] = -d / J
# measure zv and h
Cm = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]])
Cr = Cm
D = np.zeros((2, 2))

class Params:
	"""Minimal params for animation only."""
	def __init__(self):
		pass