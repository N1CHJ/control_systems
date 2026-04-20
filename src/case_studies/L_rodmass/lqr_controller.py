# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassLQRController(ControllerBase):
    """
    LQR-based state-space integral control with disturbance observer for rod-mass system.
    """

    def __init__(self, Q=None, R=None):
        """
        Initialize LQR controller with tunable Q and R matrices.
        """
        super().__init__()
        
        # 1. Physical System Model (Linearized)
        self.A = np.array([[0.0, 1.0],
                          [-P.k1/(P.m*P.ell**2), -P.b/(P.m*P.ell**2)]])
        self.B = np.array([[0.0],
                          [1.0/(P.m*P.ell**2)]])
        self.C = np.array([[1.0, 0.0]])
        
        # 2. Design Controller Gains (LQR on Augmented System)
        A1 = np.vstack([
            np.hstack([self.A, np.zeros((2, 1))]),
            np.hstack([-self.C, np.zeros((1, 1))])
        ])
        B1 = np.vstack([self.B, [[0.0]]])
        
        if Q is None:
            # Default Q: Penalize position error and velocity
            Q = np.diag([10.0, 1.0, 100.0])
        if R is None:
            # Default R: Penalize control effort
            R = np.array([[0.1]])
            
        # K1 = [K, -ki]
        K1, S, E = cnt.lqr(A1, B1, Q, R)
        self.K = K1[0, 0:2]
        self.ki = -K1[0, 2]
        
        # 3. Design Observer Gains (Same as SSIDO for now)
        self.A_obs = np.vstack([
            np.hstack([self.A, self.B]),
            np.hstack([np.zeros((1, 2)), [[0.0]]])
        ])
        self.C_obs = np.hstack([self.C, [[0.0]]])
        
        # LQR can also be used for observer design (Kalman Filter)
        # But we'll stick to pole placement to be consistent with Part 4.3
        obs_poles = np.array([-30.0, -31.0, -32.0]) # Slightly faster for LQR
        L_total = cnt.place(self.A_obs.T, self.C_obs.T, obs_poles).T
        self.L = L_total[0:2, :]
        self.L2 = L_total[2:3, :]
        
        # 4. Initialize States
        self.xhat = np.zeros((2, 1))
        self.dhat = 0.0
        self.integrator = 0.0
        self.error_prev = 0.0
        self.u_prev = 0.0
        self.u_e = P.m * P.g * P.ell

    def update_with_measurement(self, r, y):
        """
        Update observer and compute control law.
        """
        y_error = y[0] - (self.C @ self.xhat)[0, 0]
        
        xhat_dot = self.A @ self.xhat + self.B * (self.u_prev + self.dhat) + self.L * y_error
        dhat_dot = self.L2 * y_error
        
        self.xhat += xhat_dot * P.ts
        self.dhat += dhat_dot[0] * P.ts
        
        error = r[0] - self.xhat[0, 0]
        self.integrator += (P.ts / 2.0) * (error + self.error_prev)
        self.error_prev = error
        
        u_unsat = np.array([self.u_e + self.K[0]*error - self.K[1]*self.xhat[1,0] + self.ki * self.integrator - self.dhat])
        u = self.saturate(u_unsat, P.tau_max)
        
        if self.ki != 0:
            self.integrator += (P.ts / self.ki) * (u[0] - u_unsat[0])
            
        self.u_prev = u[0]
        
        return u, self.xhat.flatten(), self.dhat
