# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class VTOLControllerSS(ControllerBase):
    def __init__(self):
        # tuning parameters
        tr_h = 2.0
        tr_z = 4.0
        tr_th = 0.5
        zeta = 0.707

        # compute natural frequencies
        wn_h = 2.2 / tr_h
        wn_z = 2.2 / tr_z
        wn_th = 2.2 / tr_th

        # ensure poles are slightly distinct to avoid placement errors
        p_h = np.array([-wn_h * (zeta + 1j * np.sqrt(1 - zeta**2)),
                        -wn_h * (zeta - 1j * np.sqrt(1 - zeta**2))])
        p_z = np.array([-wn_z * (zeta + 1j * np.sqrt(1 - zeta**2)),
                        -wn_z * (zeta - 1j * np.sqrt(1 - zeta**2))])
        p_th = np.array([-wn_th * (zeta + 1j * np.sqrt(1 - zeta**2)),
                         -wn_th * (zeta - 1j * np.sqrt(1 - zeta**2))])
        
        # apply slight offsets to duplicate frequencies if any
        p_z *= 1.01 
        p_th *= 1.02
        
        des_poles = np.concatenate((p_h, p_z, p_th))
        
        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(P.A, P.B)) != 6:
            raise ValueError("System not controllable")

        self.K = cnt.place(P.A, P.B, des_poles)
        self.kr = np.linalg.inv(-P.Cr @ np.linalg.inv(P.A - P.B @ self.K) @ P.B)
        
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

        # convert back to original variables
        u_unsat = u_tilde + self.u_eq
        
        # saturate (both rotors)
        u = np.clip(u_unsat, 0.0, P.force_max) # rotors can't push back
        return u
