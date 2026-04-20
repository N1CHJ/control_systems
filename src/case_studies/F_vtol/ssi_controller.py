# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class VTOLControllerSSI(ControllerBase):
    def __init__(self, separate_integrator=True):
        # tuning parameters
        tr_h = 1.0
        tr_z = 2.0
        tr_th = 0.25
        zeta = 0.707
        integrator_poles = np.array([-1.5, -1.6])

        # augmented system
        A1 = np.block([[P.A, np.zeros((6, 2))], [-P.Cr, np.zeros((2, 2))]])
        B1 = np.vstack((P.B, np.zeros((2, 2))))

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 8:
            raise ValueError("System not controllable")

        # compute natural frequencies
        wn_h = 2.2 / tr_h
        wn_z = 2.2 / tr_z
        wn_th = 2.2 / tr_th

        # main poles
        p_h = np.array([-wn_h * (zeta + 1j * np.sqrt(1 - zeta**2)),
                        -wn_h * (zeta - 1j * np.sqrt(1 - zeta**2))])
        p_z = np.array([-wn_z * (zeta + 1j * np.sqrt(1 - zeta**2)),
                        -wn_z * (zeta - 1j * np.sqrt(1 - zeta**2))])
        p_th = np.array([-wn_th * (zeta + 1j * np.sqrt(1 - zeta**2)),
                         -wn_th * (zeta - 1j * np.sqrt(1 - zeta**2))])
        
        # apply slight offsets
        p_z *= 1.01 
        p_th *= 1.02
        
        des_poles = np.concatenate((p_h, p_z, p_th, integrator_poles))
        
        self.K1 = cnt.place(A1, B1, des_poles)
        self.K = self.K1[:, :6]
        self.ki = self.K1[:, 6:]
        print("des_poles:", des_poles)
        print("K1:", self.K1)
        print("K:", self.K)
        print("ki:", self.ki)

        # linearization point
        self.x_eq = P.x_eq
        self.u_eq = P.u_eq
        self.r_eq = P.r_eq

        # integrator variables
        self.error_prev = np.zeros(2)
        self.error_integral = np.zeros(2)
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

        # convert back to original variables
        u_unsat = u_tilde + self.u_eq
        
        # saturate (both rotors)
        u = np.clip(u_unsat, 0.0, P.force_max)
        return u
