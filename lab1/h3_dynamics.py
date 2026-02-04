import os
import sys

import numpy as np
import sympy as sp
from IPython.display import Math, display
from sympy.physics.vector.printing import vlatex

# Setup path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from case_studies.common.sym_utils import printeq

# ----------------- Defines -----------------
t = sp.symbols('t')
# States: phi (roll), theta (pitch), psi (yaw)
phi, theta, psi = sp.Function('phi')(t), sp.Function('theta')(t), sp.Function('psi')(t)
phid, thetad, psid = phi.diff(t), theta.diff(t), psi.diff(t)
q = sp.Matrix([phi, theta, psi])
qd = sp.Matrix([phid, thetad, psid])

# Physical Parameters
# Use latex-friendly symbol names with underscores for subscripts
l1, l2, lT, d = sp.symbols('l_1 l_2 l_T d', real=True, positive=True)
l3x, l3y, l3z = sp.symbols('l_{3x} l_{3y} l_{3z}', real=True, positive=True)
m1, m2, m3, g = sp.symbols('m_1 m_2 m_3 g', real=True, positive=True)
J1x, J1y, J1z = sp.symbols('J_{1x} J_{1y} J_{1z}', real=True, positive=True)
J2x, J2y, J2z = sp.symbols('J_{2x} J_{2y} J_{2z}', real=True, positive=True)
J3x, J3y, J3z = sp.symbols('J_{3x} J_{3y} J_{3z}', real=True, positive=True)
fl, fr = sp.symbols('f_l f_r', real=True) # Motor forces
b_phi, b_theta, b_psi = sp.symbols('b_phi b_theta b_psi', real=True) # Damping

# ----------------- Kinematics -----------------
# Rotation Matrices (Body-3-2-1 Euler Sequence: Yaw(psi) -> Pitch(theta) -> Roll(phi))
# Frame 1: Rotated by psi about Z0
R1_0 = sp.Matrix([[sp.cos(psi), -sp.sin(psi), 0], [sp.sin(psi), sp.cos(psi), 0], [0, 0, 1]])
# Frame 2: Rotated by theta about Y1 (Arm Frame)
R2_1 = sp.Matrix([[sp.cos(theta), 0, sp.sin(theta)], [0, 1, 0], [-sp.sin(theta), 0, sp.cos(theta)]])
# Frame 3: Rotated by phi about X2 (Body Frame)
R3_2 = sp.Matrix([[1, 0, 0], [0, sp.cos(phi), -sp.sin(phi)], [0, sp.sin(phi), sp.cos(phi)]])

# Angular Velocities
w1_1 = sp.Matrix([0, 0, psid])              # Frame 1 w.r.t Inertial
w2_2 = R2_1.T @ w1_1 + sp.Matrix([0, thetad, 0]) # Frame 2 w.r.t Inertial
w3_3 = R3_2.T @ w2_2 + sp.Matrix([phid, 0, 0])   # Frame 3 w.r.t Inertial

# Linear Velocities
# Body COM (Frame 3, distance l1 along x2, rotates with phi? No, COM is on axis)
r_body_0 = R1_0 @ R2_1 @ sp.Matrix([l1, 0, 0])
v_body_0 = r_body_0.diff(t)
# Counterweight COM (Frame 2, distance -l2 along x2)
r_cw_0 = R1_0 @ R2_1 @ sp.Matrix([-l2, 0, 0])
v_cw_0 = r_cw_0.diff(t)
# Base COM (Frame 1, distance l3x, l3y, l3z) - rotates with psi only
r_base_0 = R1_0 @ sp.Matrix([l3x, l3y, l3z])
v_base_0 = r_base_0.diff(t)

# ----------------- Energy -----------------
# Kinetic Energy 12mvt*v + RcJRt
T_trans = sp.Rational(1, 2) * m1 * (v_body_0.T @ v_body_0)[0] + \
          sp.Rational(1, 2) * m2 * (v_cw_0.T @ v_cw_0)[0] + \
          sp.Rational(1, 2) * m3 * (v_base_0.T @ v_base_0)[0]
T_rot = sp.Rational(1, 2) * (w3_3.T @ sp.Matrix(np.diag([J1x, J1y, J1z])) @ w3_3)[0] + \
        sp.Rational(1, 2) * (w2_2.T @ sp.Matrix(np.diag([J2x, J2y, J2z])) @ w2_2)[0] + \
        sp.Rational(1, 2) * (w1_1.T @ sp.Matrix(np.diag([J3x, J3y, J3z])) @ w1_1)[0]

# Potential Energy
V = m1 * g * r_body_0[2] + m2 * g * r_cw_0[2] + m3 * g * r_base_0[2]

# Lagrangian
L = sp.simplify(T_trans + T_rot - V)

# ----------------- Generalized Forces -----------------
# Torques derived from geometry of motors
# Roll torque: Differential thrust
tau_phi = -d * (fl - fr) - b_phi * phid
# Pitch torque: Thrust vector projection * lT
tau_theta = lT * (fl + fr) * sp.cos(phi) - b_theta * thetad
# Yaw torque: Thrust vector projection * lT
tau_psi = lT * (fl + fr) * sp.sin(phi) - b_psi * psid

Q = sp.Matrix([tau_phi, tau_theta, tau_psi])

# ----------------- EOM -----------------
print("Deriving EOM...")
EOM = sp.Matrix([L.diff(qd[i]).diff(t) - L.diff(q[i]) - Q[i] for i in range(3)])
EOM = sp.simplify(EOM)

#printeq(EOM, "Equations of Motion (LHS - RHS = 0)")
#sp.pprint(EOM)
#display(Math(vlatex(EOM)))

# ----------------- Mass Matrix M
M = sp.hessian(L, qd)
M = sp.simplify(M)
M = sp.trigsimp(M)





# Textbook M(q) components for verification
# M22 = J1y*c^2(phi) + J1z*s^2(phi) + J2y + m1*l1^2 + m2*l2^2
M22_textbook = J1y * sp.cos(phi)**2 + J1z * sp.sin(phi)**2 + J2y + m1 * l1**2 + m2 * l2**2
# M23 = (J1y - J1z)*s(phi)c(phi)c(theta)
M23_textbook = (J1y - J1z) * sp.sin(phi) * sp.cos(phi) * sp.cos(theta)
# M33 = J3z + l3x^2*m3 + l3y^2*m3 + (J1y*s^2(phi) + J1z*c^2(phi) + J2z + m1*l1^2 + m2*l2^2)*c^2(theta) + (J1x + J2x)*s^2(theta)
M33_textbook = J3z + (l3x**2 + l3y**2) * m3 + \
               (J1y * sp.sin(phi)**2 + J1z * sp.cos(phi)**2 + J2z + m1 * l1**2 + m2 * l2**2) * sp.cos(theta)**2 + \
               (J1x + J2x) * sp.sin(theta)**2

assert sp.simplify(M[1, 1] - M22_textbook) == 0, "M22 does not match!"
assert sp.simplify(M[1, 2] - M23_textbook) == 0, "M23 does not match!"
# We allow some rearrangement in M33, checking diff simplification
m33_diff = sp.simplify(M[2, 2] - M33_textbook)
if m33_diff != 0:
    print("M33 Diff:")
    sp.pprint(m33_diff)
assert m33_diff == 0, "M33 does not match!"



# Construct symbolic matrix for display matching the textbook form
M22_sym, M23_sym, M33_sym = sp.symbols('M_{22} M_{23} M_{33}')
M_display = sp.Matrix([
    [J1x, 0, -J1x * sp.sin(theta)],
    [0, M22_sym, M23_sym],
    [-J1x * sp.sin(theta), M23_sym, M33_sym]
])

print("\nM(q) =")
sp.pprint(M_display)
print("\nWhere:")

print("\nM22 =")
sp.pprint(M22_textbook)
print("\nM23 =")
sp.pprint(M23_textbook)
print("\nM33 =")
sp.pprint(M33_textbook)


# display(Math('M(q) = ' + vlatex(M_display)))
# display(Math(r'\text{Where:}'))
# display(Math('M_{22} = ' + vlatex(M22_textbook)))
# display(Math('M_{23} = ' + vlatex(M23_textbook)))
# display(Math('M_{33} = ' + vlatex(M33_textbook)))
 