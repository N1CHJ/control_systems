import numpy as np
from . import params as P
from .. import common

class BlockbeamControllerPID(common.ControllerBase):
    def __init__(self, tr_inner=0.2, zeta_inner=0.707, tr_outer=2.0, zeta_outer=0.707):
        # Inner loop (angle theta) gains
        J = (P.m2 * P.length**2 / 3.0) + (P.m1 * (P.length/2.0)**2)
        wn_in = 2.2 / tr_inner
        self.kp_th = J * wn_in**2
        self.kd_th = J * 2.0 * zeta_inner * wn_in
        
        # Outer loop (position z) gains
        # z_ddot = -g * theta
        wn_out = 2.2 / tr_outer
        self.kp_z = -wn_out**2 / P.g
        self.kd_z = -2.0 * zeta_outer * wn_out / P.g
        self.ki_z = -0.1 * wn_out / P.g
        
        # Dirty derivative variables
        self.sigma = 0.05
        self.beta = (2.0 * self.sigma - P.ts) / (2.0 * self.sigma + P.ts)
        self.z_prev = P.z0
        self.th_prev = P.theta0
        self.zdot_hat = 0.0
        self.thdot_hat = 0.0
        
        # Integrator
        self.z_error_int = 0.0
        self.z_error_prev = 0.0
        
    def update_with_measurement(self, z_r, y):
        z, th = y
        
        # 1. Estimate velocities
        zdot = (z - self.z_prev) / P.ts
        self.zdot_hat = self.beta * self.zdot_hat + (1 - self.beta) * zdot
        thdot = (th - self.th_prev) / P.ts
        self.thdot_hat = self.beta * self.thdot_hat + (1 - self.beta) * thdot
        self.z_prev, self.th_prev = z, th
        
        # 2. Outer loop: PID on z -> outputs desired theta
        z_error = z_r - z
        # Anti-windup
        if abs(self.zdot_hat) < 0.2:
            self.z_error_int += P.ts * (z_error + self.z_error_prev) / 2.0
        self.z_error_prev = z_error
        
        th_r = self.kp_z * z_error + self.ki_z * self.z_error_int - self.kd_z * self.zdot_hat
        th_r = np.clip(th_r, np.radians(-15), np.radians(15))
        
        # 3. Inner loop: PD on theta -> outputs force F
        th_error = th_r - th
        tau = self.kp_th * th_error - self.kd_th * self.thdot_hat
        
        # Equilibrium force: balances gravity moments
        # torque_eq = (m1*g*z + m2*g*L/2)*cos(theta)
        # actuator_torque = F * L * cos(theta)
        # F_eq = (m1*g*z/L + m2*g/2)
        F_eq = (P.m1 * P.g * z / P.length) + (P.m2 * P.g / 2.0)
        
        # F_tilde = tau / (L * cos(theta))
        F_tilde = tau / (P.length * np.cos(th))
        F = F_tilde + F_eq
        
        u_sat = np.clip(F, -P.force_max, P.force_max)
        return np.array([u_sat]), np.array([z, th, self.zdot_hat, self.thdot_hat])
