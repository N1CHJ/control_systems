# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class BlockbeamSSController(ControllerBase):
    def __init__(self):
        # tuning parameters
        tr = 4.0
        zeta = 0.707

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(P.A, P.B)) != 4:
            raise ValueError("System not controllable")

        # compute gains
        wn = 2.2 / tr  # assumes zeta = 0.707
        
        # main poles
        p_main = np.roots([1, 2 * zeta * wn, wn**2])
        # additional poles (must be distinct for single-input systems)
        p_add = np.array([-5.0 * wn, -5.1 * wn])
        des_poles = np.concatenate((p_main, p_add))
        
        self.K = cnt.place(P.A, P.B, des_poles)
        self.kr = -1.0 / (P.Cr @ np.linalg.inv(P.A - P.B @ self.K) @ P.B)
        print("des_poles:", des_poles)
        print("K:", self.K)
        print("kr:", self.kr)

        # linearization point
        self.x_eq = P.x_eq
        self.u_eq = P.u_eq
        self.r_eq = P.r_eq

    def update_with_state(self, r, x):
        # convert to linearization (tilde) variables
        x_tilde = x - self.x_eq
        r_tilde = r - self.r_eq

        # compute state feedback control
        u_tilde = -self.K @ x_tilde + self.kr @ r_tilde

        # convert back to original variables (feedback linearization)
        u_unsat = u_tilde[0] + self.u_eq[0]
        u = self.saturate(u_unsat, u_max=P.force_max)
        return np.array([u])
