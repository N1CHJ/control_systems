# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassSSIDOController(ControllerBase):
    """
    State-space integral control with disturbance observer for rod-mass system.
    
    Implements:
    - SSI control (K and ki gains)
    - Full-state observer (L gain)
    - Disturbance observer (L2 gain)
    """

    def __init__(self):
        """
        Initialize controller and observer with pole placement design.
        """
        super().__init__()
        
        # 1. Physical System Model (Linearized)
        self.A = np.array([[0.0, 1.0],
                          [-P.k1/(P.m*P.ell**2), -P.b/(P.m*P.ell**2)]])
        self.B = np.array([[0.0],
                          [1.0/(P.m*P.ell**2)]])
        self.C = np.array([[1.0, 0.0]])
        
        # 2. Design Controller Gains (SSI)
        A1 = np.vstack([
            np.hstack([self.A, np.zeros((2, 1))]),
            np.hstack([-self.C, np.zeros((1, 1))])
        ])
        B1 = np.vstack([self.B, [[0.0]]])
        
        # Desired controller poles
        ctrl_poles = np.array([-5.0, -5.1, -10.0])
        K1 = cnt.place(A1, B1, ctrl_poles)
        self.K = K1[0, 0:2]
        self.ki = -K1[0, 2]
        
        # 3. Design Observer Gains (State + Disturbance)
        # Observer augmented state: x_obs = [x, d]
        # x_obs_dot = A_obs * x_obs + B_obs * u
        # y = C_obs * x_obs
        self.A_obs = np.vstack([
            np.hstack([self.A, self.B]), # d enters system like u
            np.hstack([np.zeros((1, 2)), [[0.0]]]) # d_dot = 0 (constant dist assumption)
        ])
        self.B_obs = np.vstack([self.B, [[0.0]]])
        self.C_obs = np.hstack([self.C, [[0.0]]])
        
        # Desired observer poles (usually 2-10x faster than controller)
        # Part 4.3: 5x faster than A-BK poles
        # For simplicity, let's pick poles at -25, -26, -27
        obs_poles = np.array([-25.0, -26.0, -27.0])
        
        # L_total = [L; L2]
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
        # 1. Update Observer (Euler Integration)
        # xhat_dot = A*xhat + B*(u + dhat) + L*(y - C*xhat)
        # dhat_dot = L2*(y - C*xhat)
        
        y_error = y[0] - (self.C @ self.xhat)[0, 0]
        
        xhat_dot = self.A @ self.xhat + self.B * (self.u_prev + self.dhat) + self.L * y_error
        dhat_dot = self.L2 * y_error
        
        self.xhat += xhat_dot * P.ts
        self.dhat += dhat_dot[0] * P.ts
        
        # 2. Update Integrator
        error = r[0] - self.xhat[0, 0]
        self.integrator += (P.ts / 2.0) * (error + self.error_prev)
        self.error_prev = error
        
        # 3. Compute Control Law
        # u = u_e + K*(x_r - xhat) + ki*int - dhat
        u_unsat = np.array([self.u_e + self.K[0]*error - self.K[1]*self.xhat[1,0] + self.ki * self.integrator - self.dhat])
        
        # 4. Saturate and Anti-windup
        u = self.saturate(u_unsat, P.tau_max)
        
        if self.ki != 0:
            self.integrator += (P.ts / self.ki) * (u[0] - u_unsat[0])
            
        self.u_prev = u[0]
        
        return u, self.xhat.flatten(), self.dhat
