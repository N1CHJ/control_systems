import numpy as np


def _to_scalar(value):
    return float(np.squeeze(value))


def calculate_M(x, **p):
    """
    Gets M
    x: state vector [phi, theta, psi, phidot, thetadot, psidot]
    p: dictionary of parameters
    """
    phi, theta = _to_scalar(x[0]), _to_scalar(x[1])

    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)

    # Extract params
    m1, m2, m3 = p["m1"], p["m2"], p["m3"]
    l1, l2 = p["ell_1"], p["ell_2"]
    l3x, l3y = p["ell_3x"], p["ell_3y"]
    J1x, J1y, J1z = p["J_1x"], p["J_1y"], p["J_1z"]
    J2x, J2y, J2z = p["J_2x"], p["J_2y"], p["J_2z"]
    J3z = p["J_3z"]

    # M22
    M22 = J1y * c_phi**2 + J1z * s_phi**2 + J2y + m1 * l1**2 + m2 * l2**2

    # M23
    M23 = (J1y - J1z) * s_phi * c_phi * c_theta

    # M33
    M33 = (
        J3z
        + (l3x**2 + l3y**2) * m3
        + (J1x + J2x) * s_theta**2
        + (J1y * s_phi**2 + J1z * c_phi**2 + J2z + m1 * l1**2 + m2 * l2**2)
        * c_theta**2
    )

    M = np.array(
        [
            [J1x, 0.0, -J1x * s_theta],
            [0.0, M22, M23],
            [-J1x * s_theta, M23, M33],
        ]
    )
    return M


def calculate_dP_dq(x, **p):
    """
    Calculates partial P / partial q (gravity vector).
    """
    theta = _to_scalar(x[1])
    c_theta = np.cos(theta)

    m1, m2 = p["m1"], p["m2"]
    l1, l2 = p["ell_1"], p["ell_2"]
    g = p["g"]

    dP_dq = np.array(
        [
            0.0,
            (m1 * l1 + m2 * l2) * g * c_theta,
            0.0,
        ]
    )
    return dP_dq


def calculate_tau(x, u, **p):
    """
    generalized forces tau.
    u: input forces [f_l, f_r]
    """
    phi, theta = _to_scalar(x[0]), _to_scalar(x[1])
    c_phi, s_phi = np.cos(phi), np.sin(phi)
    c_theta, s_theta = np.cos(theta), np.sin(theta)

    fl, fr = _to_scalar(u[0]), _to_scalar(u[1])
    d = p["d"]
    lT = p["ell_T"]

    tau = np.array(
        [
            d * (fl - fr),
            lT * (fl + fr) * c_phi,
            lT * (fl + fr) * c_theta * s_phi - d * (fl - fr) * s_theta,
        ]
    )
    return tau


def calculate_C(x, **p):
    """
    C = M_dot * qdot - 0.5 * grad(qdot.T * M * qdot)
    """
    phi, theta = _to_scalar(x[0]), _to_scalar(x[1])
    phid, thetad, psid = _to_scalar(x[3]), _to_scalar(x[4]), _to_scalar(x[5])

    c_phi, s_phi = np.cos(phi), np.sin(phi)
    c_theta, s_theta = np.cos(theta), np.sin(theta)

    # Extract params
    m1, m2 = p["m1"], p["m2"]
    l1, l2 = p["ell_1"], p["ell_2"]
    J1x, J1y, J1z = p["J_1x"], p["J_1y"], p["J_1z"]
    J2x, J2y, J2z = p["J_2x"], p["J_2y"], p["J_2z"]

    # --- M_dot calculation ---
    M_dot_22 = 2 * (J1z - J1y) * s_phi * c_phi * phid

    M_dot_23 = (J1y - J1z) * (
        -2 * s_phi**2 * c_theta * phid - s_phi * s_theta * c_phi * thetad + c_theta * phid
    )

    dM33_dphi = 2 * (J1y - J1z) * s_phi * c_phi * c_theta**2
    dM33_dtheta = (
        2 * (J1x + J2x) * s_theta * c_theta
        + 2
        * (J1y * s_phi**2 + J1z * c_phi**2 + J2z + m1 * l1**2 + m2 * l2**2)
        * c_theta
        * (-s_theta)
    )

    M_dot_33 = dM33_dphi * phid + dM33_dtheta * thetad

    M_dot = np.array(
        [
            [0.0, 0.0, -J1x * c_theta * thetad],
            [0.0, M_dot_22, M_dot_23],
            [-J1x * c_theta * thetad, M_dot_23, M_dot_33],
        ]
    )

    # --- Partial derivatives ---
    dM_dphi = np.zeros((3, 3))
    dM_dphi[1, 1] = 2 * (J1z - J1y) * s_phi * c_phi
    dM_dphi[1, 2] = (J1y - J1z) * (c_phi**2 - s_phi**2) * c_theta
    dM_dphi[2, 1] = dM_dphi[1, 2]
    dM_dphi[2, 2] = 2 * (J1y - J1z) * s_phi * c_phi * c_theta**2

    N33 = 2 * (
        J1x
        + J2x
        - (J1y * s_phi**2 + J1z * c_phi**2 + J2z + m1 * l1**2 + m2 * l2**2)
    ) * s_theta * c_theta

    dM_dtheta = np.array(
        [
            [0.0, 0.0, -J1x * c_theta],
            [0.0, 0.0, -(J1y - J1z) * s_phi * c_phi * s_theta],
            [-J1x * c_theta, -(J1y - J1z) * s_phi * c_phi * s_theta, N33],
        ]
    )

    q_dot_vec = np.array([phid, thetad, psid])

    grad_term = np.array(
        [
            0.5 * q_dot_vec @ dM_dphi @ q_dot_vec,
            0.5 * q_dot_vec @ dM_dtheta @ q_dot_vec,
            0.0,
        ]
    )

    C = M_dot @ q_dot_vec - grad_term
    return C


def calculate_N33(x, **p):
    """
    Eq. 3.17
    """
    phi, theta = _to_scalar(x[0]), _to_scalar(x[1])

    s_phi = np.sin(phi)
    c_phi = np.cos(phi)
    s_theta = np.sin(theta)
    c_theta = np.cos(theta)

    m1, m2 = p["m1"], p["m2"]
    l1, l2 = p["ell_1"], p["ell_2"]
    J1x, J1y, J1z = p["J_1x"], p["J_1y"], p["J_1z"]
    J2x, J2y, J2z = p["J_2x"], p["J_2y"], p["J_2z"]

    N33 = 2 * (
        J1x
        + J2x
        - (J1y * s_phi**2 + J1z * c_phi**2 + J2z + m1 * l1**2 + m2 * l2**2)
    ) * s_theta * c_theta

    return N33
