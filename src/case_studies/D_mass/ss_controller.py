# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class MassSSController(ControllerBase):
    def __init__(self):
        # tuning parameters
        tr = 2.0
        zeta = 0.707

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(P.A, P.B)) != 2:
            raise ValueError("System not controllable")

        # compute gains
        wn = 2.2 / tr  # assumes zeta = 0.707
        des_char_poly = [1, 2 * zeta * wn, wn**2]
        des_poles = np.roots(des_char_poly)
        self.K = cnt.place(P.A, P.B, des_poles)
        self.kr = -1.0 / (P.Cr @ np.linalg.inv(P.A - P.B @ self.K) @ P.B)
        print("des_poles:", des_poles)
        print("K:", self.K)
        print("kr:", self.kr)

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.r_eq

    def update_with_state(self, r, x):
        # convert to linearization (tilde) variables
        x_tilde = x - self.x_eq
        r_tilde = r - self.r_eq

        # compute state feedback control
        u_tilde = -self.K @ x_tilde + self.kr @ r_tilde

        # saturate and return
        u_unsat = u_tilde[0]
        u = self.saturate(u_unsat, u_max=P.force_max)
        return np.array([u])
