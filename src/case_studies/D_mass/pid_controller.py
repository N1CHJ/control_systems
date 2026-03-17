import numpy as np
from . import params as P
from .. import common

class MassControllerPID(common.ControllerBase):
    def __init__(self, kp=3.9, kd=7.8, ki=0.665):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        
        # Dirty derivative variables
        self.sigma = 0.05
        self.beta = (2.0 * self.sigma - P.ts) / (2.0 * self.sigma + P.ts)
        self.z_prev = P.z0
        self.zdot_hat = 0.0
        
        # Integration variables
        self.error_integral = 0.0
        self.error_prev = 0.0
        
    def update_with_measurement(self, z_r, y):
        z = y[0]
        
        # Calculate dirty derivative of z
        z_diff = (z - self.z_prev) / P.ts
        self.zdot_hat = self.beta * self.zdot_hat + (1 - self.beta) * z_diff
        self.z_prev = z
        
        # Error and integration
        error = z_r - z
        # Anti-windup
        if abs(self.zdot_hat) < 0.2: 
            self.error_integral += P.ts * (error + self.error_prev) / 2.0
        self.error_prev = error
        
        # Control Law
        u_unsat = self.kp * error + self.ki * self.error_integral - self.kd * self.zdot_hat
        u_sat = np.clip(u_unsat, -P.force_max, P.force_max)
        
        return np.array([u_sat]), np.array([z, self.zdot_hat])
