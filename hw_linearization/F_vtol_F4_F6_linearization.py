"""Homework F.4 and F.6

F.4: State-space EOM for the VTOL system.
F.6: Jacobian linearization and final A, B, C, D matrices.
"""

from __future__ import annotations

import os
import sys

import sympy as sp

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC = os.path.join(ROOT, "src")
if SRC not in sys.path:
    sys.path.append(SRC)

from case_studies.F_vtol import params as P  # noqa: E402


def print_section(title: str) -> None:
    bar = "=" * len(title)
    print(f"\n{title}\n{bar}")


def print_matrix(label: str, mat: sp.Matrix) -> None:
    print(f"\n{label}:")
    sp.pprint(mat)


# -----------------------------
# Symbols
# -----------------------------
zv, h, theta, zvdot, hdot, thetadot = sp.symbols("zv h theta zvdot hdot thetadot")
fr, fl = sp.symbols("fr fl")
M, J, d, mu, g = sp.symbols("M J d mu g")

# State, input
x = sp.Matrix([zv, h, theta, zvdot, hdot, thetadot])
u = sp.Matrix([fr, fl])

# EOM from dynamics.py
zvddot = (-(fr + fl) * sp.sin(theta) - mu * zvdot) / M
hddot = ((fr + fl) * sp.cos(theta) - M * g) / M
thetaddot = d * (fr - fl) / J

xdot = sp.Matrix([zvdot, hdot, thetadot, zvddot, hddot, thetaddot])

# Output (zv, h, theta)
y = sp.Matrix([zv, h, theta])

# -----------------------------
# Jacobian linearization
# -----------------------------
A = xdot.jacobian(x)
B = xdot.jacobian(u)
C = y.jacobian(x)
D = y.jacobian(u)

# Equilibrium point (hover): theta=0, rates=0, fr=fl=Mg/2
zv_e = 0
h_e = 0
theta_e = 0
zvdot_e = 0
hdot_e = 0
thetadot_e = 0

fr_e = M * g / 2
fl_e = M * g / 2

subs_eq = {
    zv: zv_e,
    h: h_e,
    theta: theta_e,
    zvdot: zvdot_e,
    hdot: hdot_e,
    thetadot: thetadot_e,
    fr: fr_e,
    fl: fl_e,
}
subs_params = {M: P.M, J: P.J, d: P.d, mu: P.mu, g: P.g}

A_lin = sp.simplify(A.subs(subs_eq).subs(subs_params))
B_lin = sp.simplify(B.subs(subs_eq).subs(subs_params))
C_lin = sp.simplify(C.subs(subs_eq).subs(subs_params))
D_lin = sp.simplify(D.subs(subs_eq).subs(subs_params))

# Linearized deviation variables
zv_t, h_t, theta_t, zvdot_t, hdot_t, thetadot_t, fr_t, fl_t = sp.symbols(
    "zv_t h_t theta_t zvdot_t hdot_t thetadot_t fr_t fl_t"
)

dx = sp.Matrix([zv_t, h_t, theta_t, zvdot_t, hdot_t, thetadot_t])
du = sp.Matrix([fr_t, fl_t])

# -----------------------------
# Printouts
# -----------------------------
print_section("F.4 VTOL EOM (State-Space Form)")
print_matrix("x", x)
print_matrix("u", u)
print_matrix("Nonlinear xdot", xdot)
print_matrix("Output y", y)

print_section("F.6 Jacobian Linearization")
print_matrix("A (symbolic)", A)
print_matrix("B (symbolic)", B)
print_matrix("C (symbolic)", C)
print_matrix("D (symbolic)", D)

print_section("Equilibrium Point")
print(f"theta_e = {theta_e}")
print(f"fr_e = {sp.simplify(fr_e.subs(subs_params))}")
print(f"fl_e = {sp.simplify(fl_e.subs(subs_params))}")

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
