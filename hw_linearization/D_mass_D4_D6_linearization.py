"""Homework D.4 and D.6

D.4: State-space EOM for mass-spring-damper.
D.6: Jacobian linearization and final A, B, C, D matrices.
"""

from __future__ import annotations

import os
import sys

import sympy as sp

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC = os.path.join(ROOT, "src")
if SRC not in sys.path:
    sys.path.append(SRC)

from case_studies.D_mass import params as P  # noqa: E402


def print_section(title: str) -> None:
    bar = "=" * len(title)
    print(f"\n{title}\n{bar}")


def print_matrix(label: str, mat: sp.Matrix) -> None:
    print(f"\n{label}:")
    sp.pprint(mat)


# -----------------------------
# Symbols
# -----------------------------
z, zdot, F = sp.symbols("z zdot F")
m, k, b = sp.symbols("m k b")

# State, input
x = sp.Matrix([z, zdot])
u = sp.Matrix([F])

# EOM (nonlinear == linear for this system)
zddot = (F - b * zdot - k * z) / m
xdot = sp.Matrix([zdot, zddot])

# Output (position)
y = sp.Matrix([z])

# -----------------------------
# Jacobian linearization
# -----------------------------
A = xdot.jacobian(x)
B = xdot.jacobian(u)
C = y.jacobian(x)
D = y.jacobian(u)

# Equilibrium point (z=0, zdot=0, F=0)
z_e = 0
zdot_e = 0
F_e = 0

subs_eq = {z: z_e, zdot: zdot_e, F: F_e}
subs_params = {m: P.m, k: P.k, b: P.b}

A_lin = sp.simplify(A.subs(subs_eq).subs(subs_params))
B_lin = sp.simplify(B.subs(subs_eq).subs(subs_params))
C_lin = sp.simplify(C.subs(subs_eq).subs(subs_params))
D_lin = sp.simplify(D.subs(subs_eq).subs(subs_params))

# Linearized deviation variables
z_t, zdot_t, F_t = sp.symbols("z_t zdot_t F_t")
dx = sp.Matrix([z_t, zdot_t])
du = sp.Matrix([F_t])

# -----------------------------
# Printouts
# -----------------------------
print_section("D.4 Mass-Spring-Damper EOM (State-Space Form)")
print_matrix("x", x)
print_matrix("u", u)
print_matrix("Nonlinear xdot", xdot)
print_matrix("Output y", y)

print_section("D.6 Jacobian Linearization")
print_matrix("A (symbolic)", A)
print_matrix("B (symbolic)", B)
print_matrix("C (symbolic)", C)
print_matrix("D (symbolic)", D)

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
