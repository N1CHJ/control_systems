import argparse
import os
import sys

import numpy as np
import sympy as sp

THIS_DIR = os.path.dirname(__file__)
SRC_DIR = os.path.abspath(os.path.join(THIS_DIR, "..", "src"))
if SRC_DIR not in sys.path:
    sys.path.append(SRC_DIR)

from case_studies.H_hummingbird import eom_generated
from case_studies.H_hummingbird import params as P


def build_params():
    return {
        "m1": P.m1,
        "m2": P.m2,
        "m3": P.m3,
        "J_1x": P.J1x,
        "J_1y": P.J1y,
        "J_1z": P.J1z,
        "J_2x": P.J2x,
        "J_2y": P.J2y,
        "J_2z": P.J2z,
        "J_3x": P.J3x,
        "J_3y": P.J3y,
        "J_3z": P.J3z,
        "ell_1": P.l1,
        "ell_2": P.l2,
        "ell_3x": P.l3x,
        "ell_3y": P.l3y,
        "ell_3z": P.l3z,
        "ell_T": P.lT,
        "d": P.d,
        "g": P.g,
    }


def pretty_print_all(x, u):
    p = build_params()

    np.set_printoptions(precision=10, suppress=True)

    M = eom_generated.calculate_M(x, **p)
    C = eom_generated.calculate_C(x, **p)
    dP_dq = eom_generated.calculate_dP_dq(x, **p)
    tau = eom_generated.calculate_tau(x, u, **p)
    N33 = eom_generated.calculate_N33(x, **p)

    print("M =")
    print(M)
    print("\nC =")
    print(C)
    print("\ndP_dq =")
    print(dP_dq)
    print("\ntau =")
    print(tau)
    print("\nN33 =")
    print(N33)


def main():
    x = np.array([P.phi0, P.theta0, P.psi0, P.phidot0, P.thetadot0, P.psidot0], dtype=float)
    # Match the example input used in generate_state_variable_form.py
    u = np.array([0.24468 / 2, 0.22468 / 2], dtype=float)
    pretty_print_all(x, u)
    print("SYMBOLIC EXPRESSION 3.17")
    # pretty_print_symbolic()


def pretty_print_symbolic():
    phi, theta, psi = sp.symbols("phi theta psi", real=True)
    phid, thetad, psid = sp.symbols("phid thetad psid", real=True)
    f_l, f_r = sp.symbols("f_l f_r", real=True)

    m1, m2, m3 = sp.symbols("m1 m2 m3", positive=True)
    J1x, J1y, J1z = sp.symbols("J_1x J_1y J_1z", positive=True)
    J2x, J2y, J2z = sp.symbols("J_2x J_2y J_2z", positive=True)
    J3x, J3y, J3z = sp.symbols("J_3x J_3y J_3z", positive=True)
    ell_1, ell_2, ell_3x, ell_3y, ell_3z, ell_T = sp.symbols(
        "ell_1 ell_2 ell_3x ell_3y ell_3z ell_T", real=True
    )
    d, g = sp.symbols("d g", positive=True)

    c_phi = sp.cos(phi)
    s_phi = sp.sin(phi)
    c_theta = sp.cos(theta)
    s_theta = sp.sin(theta)

    M22 = J1y * c_phi**2 + J1z * s_phi**2 + J2y + m1 * ell_1**2 + m2 * ell_2**2
    M23 = (J1y - J1z) * s_phi * c_phi * c_theta
    M33 = (
        J3z
        + (ell_3x**2 + ell_3y**2) * m3
        + (J1x + J2x) * s_theta**2
        + (J1y * s_phi**2 + J1z * c_phi**2 + J2z + m1 * ell_1**2 + m2 * ell_2**2)
        * c_theta**2
    )

    M = sp.Matrix(
        [
            [J1x, 0, -J1x * s_theta],
            [0, M22, M23],
            [-J1x * s_theta, M23, M33],
        ]
    )

    dP_dq = sp.Matrix([0, (m1 * ell_1 + m2 * ell_2) * g * c_theta, 0])

    tau = sp.Matrix(
        [
            d * (f_l - f_r),
            ell_T * (f_l + f_r) * c_phi,
            ell_T * (f_l + f_r) * c_theta * s_phi - d * (f_l - f_r) * s_theta,
        ]
    )

    M_dot_22 = 2 * (J1z - J1y) * s_phi * c_phi * phid
    M_dot_23 = (J1y - J1z) * (
        -2 * s_phi**2 * c_theta * phid - s_phi * s_theta * c_phi * thetad + c_theta * phid
    )
    dM33_dphi = 2 * (J1y - J1z) * s_phi * c_phi * c_theta**2
    dM33_dtheta = (
        2 * (J1x + J2x) * s_theta * c_theta
        + 2
        * (J1y * s_phi**2 + J1z * c_phi**2 + J2z + m1 * ell_1**2 + m2 * ell_2**2)
        * c_theta
        * (-s_theta)
    )
    M_dot_33 = dM33_dphi * phid + dM33_dtheta * thetad
    M_dot = sp.Matrix(
        [
            [0, 0, -J1x * c_theta * thetad],
            [0, M_dot_22, M_dot_23],
            [-J1x * c_theta * thetad, M_dot_23, M_dot_33],
        ]
    )

    dM_dphi = sp.Matrix(
        [
            [0, 0, 0],
            [0, 2 * (J1z - J1y) * s_phi * c_phi, (J1y - J1z) * (c_phi**2 - s_phi**2) * c_theta],
            [0, (J1y - J1z) * (c_phi**2 - s_phi**2) * c_theta, 2 * (J1y - J1z) * s_phi * c_phi * c_theta**2],
        ]
    )

    N33 = 2 * (
        J1x
        + J2x
        - (J1y * s_phi**2 + J1z * c_phi**2 + J2z + m1 * ell_1**2 + m2 * ell_2**2)
    ) * s_theta * c_theta
    dM_dtheta = sp.Matrix(
        [
            [0, 0, -J1x * c_theta],
            [0, 0, -(J1y - J1z) * s_phi * c_phi * s_theta],
            [-J1x * c_theta, -(J1y - J1z) * s_phi * c_phi * s_theta, N33],
        ]
    )

    qdot = sp.Matrix([phid, thetad, psid])
    grad_term = sp.Matrix(
        [
            sp.Rational(1, 2) * (qdot.T * dM_dphi * qdot)[0],
            sp.Rational(1, 2) * (qdot.T * dM_dtheta * qdot)[0],
            0,
        ]
    )
    C = M_dot * qdot - grad_term

    print("\nSymbolic expressions (Eq. 3.17 form):")
    print("\nM =")
    sp.pprint(M)
    print("\nC =")
    sp.pprint(C)
    print("\ndP_dq =")
    sp.pprint(dP_dq)
    print("\ntau =")
    sp.pprint(tau)


if __name__ == "__main__":
    main()
