# 3rd-party
import numpy as np

from ..common.dynamics_base import DynamicsBase
# local (controlbook)
from . import eom_generated
from . import params as P


class HummingbirdDynamics(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            state0=np.array(
                [P.phi0, P.theta0, P.psi0, P.phidot0, P.thetadot0, P.psidot0]
            ),
            u_max=np.inf,
            u_min=0.0,
            dt=P.ts,
        )

        self.eom_params = {
            "m1": self.randomize_parameter(P.m1, alpha),
            "m2": self.randomize_parameter(P.m2, alpha),
            "m3": self.randomize_parameter(P.m3, alpha),
            "J_1x": self.randomize_parameter(P.J1x, alpha),
            "J_1y": self.randomize_parameter(P.J1y, alpha),
            "J_1z": self.randomize_parameter(P.J1z, alpha),
            "J_2x": self.randomize_parameter(P.J2x, alpha),
            "J_2y": self.randomize_parameter(P.J2y, alpha),
            "J_2z": self.randomize_parameter(P.J2z, alpha),
            "J_3x": self.randomize_parameter(P.J3x, alpha),
            "J_3y": self.randomize_parameter(P.J3y, alpha),
            "J_3z": self.randomize_parameter(P.J3z, alpha),
            "ell_1": self.randomize_parameter(P.l1, alpha),
            "ell_2": self.randomize_parameter(P.l2, alpha),
            "ell_3x": self.randomize_parameter(P.l3x, alpha),
            "ell_3y": self.randomize_parameter(P.l3y, alpha),
            "ell_3z": self.randomize_parameter(P.l3z, alpha),
            "ell_T": self.randomize_parameter(P.lT, alpha),
            "d": self.randomize_parameter(P.d, alpha),
            "g": P.g,
        }

        self.B = np.diag([P.b_phi, P.b_theta, P.b_psi])

    def calculate_M(self, x):
        return eom_generated.calculate_M(x, **self.eom_params)

    def calculate_C(self, x):
        return eom_generated.calculate_C(x, **self.eom_params)

    def calculate_dP_dq(self, x):
        return eom_generated.calculate_dP_dq(x, **self.eom_params)

    def calculate_tau(self, x, u):
        return eom_generated.calculate_tau(x, u, **self.eom_params)

    def f(self, x, u):
        """
        x = [phi, theta, psi, phidot, thetadot, psidot]
        u = [f_l, f_r]
        """
        x = x.flatten()
        u = u.flatten()

        qdot = x[3:6]

        M = self.calculate_M(x)
        C = self.calculate_C(x)
        dP_dq = self.calculate_dP_dq(x)
        tau = self.calculate_tau(x, u)

        friction = self.B @ qdot
        rhs = tau - C - dP_dq - friction
        qddot = np.linalg.solve(M, rhs)

        xdot = np.concatenate((qdot, qddot))
        return xdot

    def h(self):
        """Get h"""
        return self.state[0:3]

    def update(self, u):
        """
        Numerical integration step.
            u (NDArray[np.float64]): input vector [f_l, f_r].
        """
        return super().update(u)
