"""Homework E.4 and E.6

E.4: State-space EOM for the block-beam system.
E.6: Jacobian linearization and final A, B, C, D matrices.
"""

from __future__ import annotations

import os
import sys

import sympy as sp

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC = os.path.join(ROOT, "src")
if SRC not in sys.path:
    sys.path.append(SRC)

from case_studies.E_blockbeam import params as P  # noqa: E402


def print_section(title: str) -> None:
    bar = "=" * len(title)
    print(f"\n{title}\n{bar}")


def print_matrix(label: str, mat: sp.Matrix) -> None:
    print(f"\n{label}:")
    sp.pprint(mat)


# -----------------------------
# Symbols
# -----------------------------
z, theta, zdot, thetadot, F = sp.symbols("z theta zdot thetadot F")
m1, m2, L, g, I_beam = sp.symbols("m1 m2 L g I_beam")

# State, input
x = sp.Matrix([z, theta, zdot, thetadot])
u = sp.Matrix([F])

# EOM from dynamics.py
zddot = z * thetadot**2 - g * sp.sin(theta)

num = F * L * sp.cos(theta) - 2 * m1 * z * zdot * thetadot - (m1 * z + m2 * L / 2) * g * sp.cos(theta)
den = I_beam + m1 * z**2
thetaddot = num / den

xdot = sp.Matrix([zdot, thetadot, zddot, thetaddot])

# Output (z and theta)
y = sp.Matrix([z, theta])

# -----------------------------
# Jacobian linearization
# -----------------------------
A = xdot.jacobian(x)
B = xdot.jacobian(u)
C = y.jacobian(x)
D = y.jacobian(u)

# Equilibrium point (theta=0, zdot=0, thetadot=0)
# Choose z_e as the nominal block position (defaults to P.z0)
z_e = P.z0

theta_e = 0
zdot_e = 0
thetadot_e = 0

# Solve for F_e that makes thetaddot = 0 at equilibrium
F_e_expr = (m1 * z + m2 * L / 2) * g / L
F_e = F_e_expr.subs({z: z_e})

subs_eq = {z: z_e, theta: theta_e, zdot: zdot_e, thetadot: thetadot_e, F: F_e}
subs_params = {m1: P.m1, m2: P.m2, L: P.L, g: P.g, I_beam: P.I_beam}

A_lin = sp.simplify(A.subs(subs_eq).subs(subs_params))
B_lin = sp.simplify(B.subs(subs_eq).subs(subs_params))
C_lin = sp.simplify(C.subs(subs_eq).subs(subs_params))
D_lin = sp.simplify(D.subs(subs_eq).subs(subs_params))

# Linearized deviation variables
z_t, theta_t, zdot_t, thetadot_t, F_t = sp.symbols("z_t theta_t zdot_t thetadot_t F_t")
dx = sp.Matrix([z_t, theta_t, zdot_t, thetadot_t])
du = sp.Matrix([F_t])

# -----------------------------
# Printouts
# -----------------------------
print_section("E.4 Block-Beam EOM (State-Space Form)")
print_matrix("x", x)
print_matrix("u", u)
print_matrix("Nonlinear xdot", xdot)
print_matrix("Output y", y)

print_section("E.6 Jacobian Linearization")
print_matrix("A (symbolic)", A)
print_matrix("B (symbolic)", B)
print_matrix("C (symbolic)", C)
print_matrix("D (symbolic)", D)

print_section("Equilibrium Point")
print(f"z_e = {z_e}")
print(f"theta_e = {theta_e}")
print(f"F_e = {sp.simplify(F_e.subs(subs_params))}")

print_section("Final A, B, C, D (evaluated at equilibrium + parameters)")
print_matrix("A", A_lin)
print_matrix("B", B_lin)
print_matrix("C", C_lin)
print_matrix("D", D_lin)

print_section("Linearized EOM (deviation form)")
print("xdot ≈ A * dx + B * du")
print_matrix("dx", dx)
print_matrix("du", du)
print_matrix("A*dx + B*du", sp.simplify(A_lin * dx + B_lin * du))
