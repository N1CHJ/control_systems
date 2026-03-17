# 3rd-party
import numpy as np

# local (controlbook)
from ..common import ControllerBase
from . import params as P


class HummingbirdControllerPID(ControllerBase):
    """
    PID controller for the Hummingbird.
    Can control pitch, roll, or yaw depending on the configuration.
    For H7, it is used for pitch (theta).
    """

    def __init__(self, kp: float, ki: float, kd: float, Ts: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts

        # PID state
        self.error_sum = 0.0
        self.error_prev = 0.0

        # Equilibrium values
        self.u_e = P.u_e
        self.km = P.km

    def update_with_state(self, r, x):
        # For H7, r is theta_ref, x is the full state
        # In Hummingbird, state is [phi, theta, psi, phidot, thetadot, psidot]
        theta_ref = r[0]
        theta = x[1]
        thetadot = x[4]

        # Compute error
        error = theta_ref - theta

        # Update integral term
        self.error_sum += error * self.Ts

        # PID control law for F_tilde
        # Note: using thetadot directly instead of numerical derivative of error
        F_tilde = self.kp * error + self.ki * self.error_sum - self.kd * thetadot

        # Total PWM for longitudinal control
        u_val = self.u_e + F_tilde / (2.0 * self.km)

        # Output PWM for both motors
        u = np.array([u_val, u_val])

        # Saturate PWM to [0, 1]
        u = self.saturate(u, u_max=1.0, u_min=0.0)

        # Reset integral if saturated (Anti-windup - basic)
        # if np.any(u >= 1.0) or np.any(u <= 0.0):
        #    self.error_sum -= error * self.Ts

        return u
