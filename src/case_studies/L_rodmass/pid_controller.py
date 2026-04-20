# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassControllerPID(ControllerBase):
    """
    PID controller for the rod-mass system.
    
    Implements:
    - Proportional control
    - Dirty derivative for D-control (to handle sensor noise)
    - Integrator with anti-windup for steady-state error rejection
    """

    def __init__(self, kp=0.0, kd=0.0, ki=0.0, sigma=0.05, ts=0.01):
        """
        Initialize PID controller with specified gains.
        
        Args:
            kp: Proportional gain
            kd: Derivative gain 
            ki: Integral gain
            sigma: Low-pass filter coefficient for dirty derivative
            ts: Sampling period
        """
        super().__init__()
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.sigma = sigma
        self.ts = ts

        # State variables for dirty derivative and integrator
        self.error_dot = 0.0
        self.error_prev = 0.0
        self.integrator = 0.0
        self.y_prev = 0.0
        self.y_dot = 0.0

        # Feedforward equilibrium torque (for theta_e = 0)
        self.u_e = P.m * P.g * P.ell 

    def update_with_measurement(self, r, y):
        """
        Update the PID control law using output measurements.
        
        Args:
            r: Reference angle (theta_r)
            y: Measured angle (theta)
            
        Returns:
            u: Control torque
            xhat: Estimated state [theta, thetadot]
        """
        theta = y[0]
        theta_r = r[0]

        # 1. Compute error
        error = theta_r - theta

        # 2. Compute dirty derivative of error (or y)
        # We'll differentiate the output y to avoid "derivative kick" on reference steps
        self.y_dot = (2.0 * self.sigma - self.ts) / (2.0 * self.sigma + self.ts) * self.y_dot + \
                     (2.0 / (2.0 * self.sigma + self.ts)) * (theta - self.y_prev)
        self.y_prev = theta

        # 3. Compute integrator (trapezoidal integration)
        self.integrator = self.integrator + (self.ts / 2.0) * (error + self.error_prev)
        self.error_prev = error

        # 4. PID control law + Feedforward
        # u = u_e + Kp*error - Kd*y_dot + Ki*integrator
        u_unsat = np.array([self.u_e + self.kp * error - self.kd * self.y_dot + self.ki * self.integrator])

        # 5. Saturate and Anti-windup
        u = self.saturate(u_unsat, P.tau_max)
        
        # Integrator anti-windup
        if self.ki != 0:
            self.integrator = self.integrator + (self.ts / self.ki) * (u[0] - u_unsat[0])

        # Estimated state for logging/visualization
        xhat = np.array([theta, self.y_dot])
        
        return u, xhat

    def update_with_state(self, r, x):
        """
        Update PID control law assuming full state access.
        """
        theta = x[0]
        thetadot = x[1]
        theta_r = r[0]
        
        error = theta_r - theta
        
        # No dirty derivative needed if we have thetadot
        self.integrator = self.integrator + (self.ts / 2.0) * (error + self.error_prev)
        self.error_prev = error

        u_unsat = self.u_e + self.kp * error - self.kd * thetadot + self.ki * self.integrator
        u = self.saturate(u_unsat, P.tau_max)
        
        if self.ki != 0:
            self.integrator = self.integrator + (self.ts / self.ki) * (u[0] - u_unsat[0])

        return u
