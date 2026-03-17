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

# Motor constant: maps PWM duty cycle [0,1] to force
# Derived from equilibrium: Fe = km*(ul_e + ur_e), solving for km
km = g * (m1 * l1 + m2 * l2) / lT

# Equilibrium values
F_e = (m1 * l1 + m2 * l2) * g / lT  # equilibrium total force
tau_e = 0.0  # equilibrium torque
theta_e = 0.0
phi_e = 0.0
psi_e = 0.0

# Equilibrium PWM (each motor produces half the equilibrium force)
u_e = F_e / (2.0 * km)  # equilibrium PWM per motor

# Linearized longitudinal dynamics: theta_ddot = b_theta * F_ctrl  (Eqn 4.3-4.4)
Jy_eff = m1 * l1**2 + m2 * l2**2 + J1y + J2y
b_theta = lT / Jy_eff

# Linearized lateral dynamics (Eqns 4.7-4.8)
# phi_ddot = (1/J1x) * tau_tilde
# psi_ddot = (F_e * lT / J_psi) * phi_tilde
J_psi = J1z + J2z + J3z + l1**2 * m1 + l2**2 * m2 + l3x**2 * m3 + l3y**2 * m3
b_phi = 1.0 / J1x
a_psi = F_e * lT / J_psi

##### Chapter 5: Transfer Functions
# P_theta(s) = b_theta / s^2
# P_phi(s) = b_phi / s^2
# P_psi(s) = a_psi / s^2  (psi/phi)
# P_psi_tau(s) = (a_psi * b_phi) / s^4

##### Chapter 6: State Space Matrices
# Longitudinal State-Space: x_lon = [theta, thetadot]
A_lon = np.array([[0.0, 1.0], [0.0, 0.0]])
B_lon = np.array([[0.0], [b_theta]])
C_lon = np.array([[1.0, 0.0]])
D_lon = np.array([[0.0]])

# Lateral State-Space: x_lat = [phi, psi, phidot, psidot]
A_lat = np.array([[0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0],
                  [0.0, 0.0, 0.0, 0.0],
                  [a_psi, 0.0, 0.0, 0.0]])
B_lat = np.array([[0.0],
                  [0.0],
                  [b_phi],
                  [0.0]])
C_lat = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0]])
D_lat = np.array([[0.0],
                  [0.0]])

##### Chapter 7: PD Tuning Parameters
tr = 1.0
zeta = 0.707
sigma = 0.05

