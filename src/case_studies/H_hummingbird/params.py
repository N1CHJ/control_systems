# Simulation parameters
t_start = 0.0  # Start time of simulation
t_end = 20.0   # End time of simulation
ts = 0.01       # Sampling time (seconds) - Faster for simulation

# Physical Parameters
g = 9.81        # Gravity (m/s^2)

# Geometry
l1 = 0.3        # Distance to body (m)
l2 = 0.3        # Distance to counterweight (m)
lT = 0.3        # Distance to motors (m, assumed same as l1)
d = 0.1         # Motor separation distance from arm axis (m)

# Mass and Inertia
m1 = 1.0        # Mass of body (kg)
m2 = 1.0        # Mass of counterweight (kg)

# Body Inertia (kg*m^2)
J1x = 0.1
J1y = 0.1
J1z = 0.1

# Counterweight Inertia (kg*m^2)
J2x = 0.1
J2y = 0.1
J2z = 0.1

# Damping
b_phi = 0.1     # Roll damping
b_theta = 0.1   # Pitch damping
b_psi = 0.1     # Yaw damping

# Initial Conditions
phi0 = 0.0
theta0 = 0.0
psi0 = 0.0
phidot0 = 0.0
thetadot0 = 0.0
psidot0 = 0.0
import numpy as np

# g = 
# ell1 = 
# ell2 = 
# ell3x = 
# ell3y = 
# ell3z = 
# ellT = 
# d = 
# m1 = 
# J1x = 
# J1y = 
# J1z = 
# m2 = 
# J2x = 
# J2y = 
# J2z = 
# m3 = 
# J3x = 
# J3y = 
# J3z = 


##### Chapter 4
# mixing matrices (see end of Chapter 4 in lab manual)
# mixing is a UAV term for taking body forces/torques to individual motor forces
unmixer = np.array([[1.0, 1.0], [d, -d]])  # [F, tau] = unmixer @ [fl, fr]
mixer = np.linalg.inv(unmixer)  # [fl, fr] = mixer @ [F, tau]

