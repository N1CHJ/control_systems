import numpy as np

# Physical parameters
m1 = 0.35
m2 = 2.0
length = 0.5
g = 9.8
I_beam = 0.05

# Initial Conditions
z0 = 0.25
theta0 = 0.0
zdot0 = 0.0
thetadot0 = 0.0

# Simulation parameters
t0 = 0.0
tf = 50.0
ts = 0.01

# Input constraint
force_max = 20.0

##### Chapter 4 / 11-14
# Linearization/equilibrium point
x_eq = np.array([z0, theta0, zdot0, thetadot0])
u_eq = np.array([(m1 * z0 + m2 * length / 2) * g / length])
r_eq = np.array([z0])

##### Chapter 6 / 11-14
# State space
# x = [z, theta, zdot, thetadot]
A = np.array([
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, -g, 0.0, 0.0],
    [-m1 * g / (I_beam + m1 * z0**2), 0.0, 0.0, 0.0]
])
B = np.array([[0.0, 0.0, 0.0, length / (I_beam + m1 * z0**2)]]).T
Cm = np.array([[1.0, 0.0, 0.0, 0.0]])  # measure position z
Cr = Cm
D = np.array([[0.0]])

class Params:
	"""Minimal params for animation only."""
	def __init__(self):
		pass
