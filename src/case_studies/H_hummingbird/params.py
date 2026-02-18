import numpy as np

# Physical Parameters
g = 9.81  # m/s^2

# Lengths
l1 = 0.247   # m
l2 = -0.039  # m
l3x = -0.007  # m
l3y = -0.007  # m
l3z = 0.018  # m
lT = 0.355   # m
d = 0.12     # m

# Masses
m1 = 0.108862  # kg
m2 = 0.4717    # kg
m3 = 0.1905    # kg

# Inertias (J = diag(Jx, Jy, Jz))
J1x = 0.000189   # kg-m^2
J1y = 0.001953   # kg-m^2
J1z = 0.001894   # kg-m^2

J2x = 0.00231    # kg-m^2
J2y = 0.003274   # kg-m^2
J2z = 0.003416   # kg-m^2

J3x = 0.0002222  # kg-m^2
J3y = 0.0001956  # kg-m^2
J3z = 0.000027   # kg-m^2

# Damping/Friction (Chapter 3)
beta = 0.001
b_phi = beta
b_theta = beta
b_psi = beta

# Initial Conditions (Lab H.2)
phi0 = 0.0
theta0 = 0.0
psi0 = 0.0
phidot0 = 0.0
thetadot0 = 0.0
psidot0 = 0.0

# Simulation parameters
t_start = 0.0
t_end = 50.0
ts = 0.01

# Aliases
ell1 = l1
ell2 = l2
ell3x = l3x
ell3y = l3y
ell3z = l3z
ellT = lT

##### Chapter 4
# Mixing matrices (see end of Chapter 4 in lab manual)
# Mixing is a UAV term for taking body forces/torques to individual motor forces
unmixer = np.array([[1.0, 1.0], [d, -d]])  # [F, tau] = unmixer @ [fl, fr]
mixer = np.linalg.inv(unmixer)  # [fl, fr] = mixer @ [F, tau]

