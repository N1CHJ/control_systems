# 3rd-party
import numpy as np

# local (controlbook)
from ..common import ControllerBase
from . import params as P


class HummingbirdControllerLonPD(ControllerBase):
    """
    Longitudinal PD controller for the Hummingbird.
    Controls pitch (theta) using total force (F).
    """

    def __init__(self, tr: float = P.tr, zeta: float = P.zeta, sigma: float = P.sigma):
        """
        Initialize the PD controller.
        
        Args:
            tr (float): Desired rise time (seconds)
            zeta (float): Desired damping ratio
            sigma (float): Dirty derivative filter parameter
        """
        # Desired characteristic equation from rise time and damping ratio
        wn = 2.2 / tr
        alpha1 = 2.0 * zeta * wn
        alpha0 = wn**2

        # Longitudinal dynamics: theta_ddot = b_theta * F_tilde
        # Characteristic equation: s^2 + b_theta*kd*s + b_theta*kp = s^2 + alpha1*s + alpha0
        self.kp = alpha0 / P.b_theta
        self.kd = alpha1 / P.b_theta

        print(f"Longitudinal PD Gains: kp = {self.kp:.4f}, kd = {self.kd:.4f}")

        # Equilibrium values
        self.u_e = P.u_e  # equilibrium PWM per motor
        self.km = P.km
        
        # Digital PD state (dirty derivative)
        self.theta_dot = 0.0
        self.theta_prev = 0.0
        self.sigma = sigma
        self.Ts = P.ts

    def update_with_state(self, r, x):
        """
        PD control law using full state feedback (idealized).
        """
        # Unpack reference and state
        theta_ref = r[0]
        theta = x[1]
        thetadot = x[4]  # In Hummingbird, state is [phi, theta, psi, phidot, thetadot, psidot]

        # Compute error
        error = theta_ref - theta

        # PD control law for F_tilde
        F_tilde = self.kp * error - self.kd * thetadot

        # Total PWM for longitudinal control (equal for both motors)
        # Using feedback linearization force F_fl = F_e * cos(theta)
        # u_fl = F_fl / (2*km) = (F_e / (2*km)) * cos(theta) = u_e * cos(theta)
        u_val = self.u_e * np.cos(theta) + F_tilde / (2.0 * self.km)

        # Output PWM for both motors
        u = np.array([u_val, u_val])

        # Saturate PWM to [0, 1]
        u = self.saturate(u, u_max=1.0, u_min=0.0)

        # Return u and estimated state (thetadot)
        # Note: run_simulation expects at least two return values for measurement-based control
        return u, np.array([thetadot])
    
    def update_with_measurement(self, r, y):
        """
        Digital PD control law using output measurement and dirty derivative.
        
        Args:
            r (NDArray): Reference vector [theta_ref]
            y (NDArray): Measurement vector [phi, theta, psi]
        """
        # Unpack reference and measurement
        theta_ref = r[0]
        theta = y[1]

        # Compute error
        error = theta_ref - theta

        # Dirty derivative to estimate thetadot
        # thetadot[k] = (2*sigma - Ts)/(2*sigma + Ts) * thetadot[k-1] 
        #               + 2/(2*sigma + Ts) * (theta[k] - theta[k-1])
        a1 = (2.0 * self.sigma - self.Ts) / (2.0 * self.sigma + self.Ts)
        a2 = 2.0 / (2.0 * self.sigma + self.Ts)
        
        self.theta_dot = a1 * self.theta_dot + a2 * (theta - self.theta_prev)
        self.theta_prev = theta

        # PD control law for F_tilde
        F_tilde = self.kp * error - self.kd * self.theta_dot

        # Total PWM for longitudinal control
        u_val = self.u_e * np.cos(theta) + F_tilde / (2.0 * self.km)

        # Output PWM for both motors
        u = np.array([u_val, u_val])

        # Saturate PWM to [0, 1]
        u = self.saturate(u, u_max=1.0, u_min=0.0)

        return u, np.array([self.theta_dot])
