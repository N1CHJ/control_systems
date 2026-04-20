# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class BlockbeamSSIController(ControllerBase):
    def __init__(self, separate_integrator=True):
        # tuning parameters
        tr = 2.0
        zeta = 0.85
        integrator_pole = [-1.5]

        # augmented system
        A1 = np.block([[P.A, np.zeros((4, 1))], [-P.Cr, np.zeros((1, 1))]])
        B1 = np.vstack((P.B, 0))

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
            raise ValueError("System not controllable")

        # compute gains
        wn = 2.2 / tr  # assumes zeta = 0.707
        p_main = np.roots([1, 2 * zeta * wn, wn**2])
        p_add = np.array([-5.0 * wn, -5.1 * wn])
        des_poles = np.concatenate((p_main, p_add, integrator_pole))
        
        self.K1 = cnt.place(A1, B1, des_poles)
        self.K = self.K1[:, :4]
        self.ki = self.K1[:, 4:]
        print("des_poles:", des_poles)
        print("K1:", self.K1)
        print("K:", self.K)
        print("ki:", self.ki)

        # linearization point
        self.x_eq = P.x_eq
        self.u_eq = P.u_eq
        self.r_eq = P.r_eq

        # integrator variables
        self.error_prev = 0.0
        self.error_integral = 0.0
        self.separate_integrator = separate_integrator

    def update_with_state(self, r, x):
        # convert to linearization (tilde) variables
        x_tilde = x - self.x_eq

        # integrate error
        error = r - P.Cr @ x
        self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        # compute feedback control
        if self.separate_integrator:
            u_tilde = -self.K @ x_tilde - self.ki @ self.error_integral
        else:
            x1_tilde = np.hstack((x_tilde, self.error_integral))
            u_tilde = -self.K1 @ x1_tilde

        # convert back to original variables (feedback linearization)
        u_unsat = u_tilde[0] + self.u_eq[0]
        u = self.saturate(u_unsat, u_max=P.force_max)
        return np.array([u])
