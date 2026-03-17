import numpy as np
from . import params as P
from .. import common

class VTOLControllerPID(common.ControllerBase):
    def __init__(self, tr_h=2.0, zeta_h=0.707, tr_th=0.2, zeta_th=0.707, tr_z=2.0, zeta_z=0.707):
        # Altitude Loop (h)
        # h_ddot = (F - Mg)/M
        wn_h = 2.2 / tr_h
        self.kp_h = P.M * wn_h**2
        self.kd_h = P.M * 2.0 * zeta_h * wn_h
        self.ki_h = 0.5 # small integrator
        
        # Inner loop (Roll angle theta)
        # theta_ddot = tau / J
        J = P.Jc + 2 * P.mr * P.d**2
        wn_th = 2.2 / tr_th
        self.kp_th = J * wn_th**2
        self.kd_th = J * 2.0 * zeta_th * wn_th
        
        # Outer loop (Lateral position z)
        # z_ddot = -g * theta
        wn_z = 2.2 / tr_z
        self.kp_z = -wn_z**2 / P.g
        self.kd_z = -2.0 * zeta_z * wn_z / P.g
        self.ki_z = -0.1 * wn_z / P.g
        
        # Dirty derivative variables
        self.sigma = 0.05
        self.beta = (2.0 * self.sigma - P.ts) / (2.0 * self.sigma + P.ts)
        self.prev_y = np.array([P.z0, P.h0, P.theta0])
        self.ydot_hat = np.zeros(3)
        
        # Integrators
        self.h_error_int = 0.0; self.h_error_prev = 0.0
        self.z_error_int = 0.0; self.z_error_prev = 0.0
        
    def update_with_measurement(self, r, y):
        z, h, th = y
        z_r, h_r = r
        
        # 1. Estimate velocities
        ydot = (y - self.prev_y) / P.ts
        self.ydot_hat = self.beta * self.ydot_hat + (1 - self.beta) * ydot
        self.prev_y = y
        zv_dot, h_dot, th_dot = self.ydot_hat
        
        # 2. Altitude loop (PID)
        h_error = h_r - h
        if abs(h_dot) < 0.2:
            self.h_error_int += P.ts * (h_error + self.h_error_prev) / 2.0
        self.h_error_prev = h_error
        # F_tilde produces h_ddot, F_eq = Mg
        F_tilde = self.kp_h * h_error + self.ki_h * self.h_error_int - self.kd_h * h_dot
        F = F_tilde + (P.M * P.g)
        
        # 3. Outer Lateral Loop: PID on z -> outputs desired theta
        z_error = z_r - z
        if abs(zv_dot) < 0.2:
            self.z_error_int += P.ts * (z_error + self.z_error_prev) / 2.0
        self.z_error_prev = z_error
        th_r = self.kp_z * z_error + self.ki_z * self.z_error_int - self.kd_z * zv_dot
        # Limit tilt angle
        th_r = np.clip(th_r, np.radians(-15), np.radians(15))
        
        # 4. Inner Roll Loop: PD on theta -> outputs torque tau
        th_error = th_r - th
        tau = self.kp_th * th_error - self.kd_th * th_dot
        
        # 5. Mix and Saturate
        # u = [fr, fl]
        u_unsat = P.mixing @ np.array([F, tau])
        # Rotor force cannot be negative for real VTOL
        u_sat = np.clip(u_unsat, 0.0, P.force_max)
        
        return u_sat, np.concatenate([y, self.ydot_hat])
        
