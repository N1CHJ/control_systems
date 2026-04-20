"""
Equilibrium Controller for Rod-Mass System
==========================================
This controller provides the feedforward torque u_e required to maintain 
the system at a specific equilibrium angle theta_e.

The equilibrium torque is derived from the equations of motion:
0 = tau_e - b*thetadot - m*g*ell*cos(theta_e) - k1*theta_e - k2*theta_e^3
Since at equilibrium thetadot = 0:
tau_e = m*g*ell*cos(theta_e) + k1*theta_e + k2*theta_e^3
"""

import numpy as np
from ..common import ControllerBase
from . import params as P

class EquilibriumController(ControllerBase):
    def __init__(self, theta_e=0.0):
        """
        Initialize the equilibrium controller.
        
        Args:
            theta_e: The desired equilibrium angle in radians.
        """
        super().__init__()
        self.theta_e = theta_e
        
        # Calculate u_e based on the system parameters and theta_e
        self.u_e = self.calculate_ue(theta_e)
        
    def calculate_ue(self, theta_e):
        """
        Calculates the equilibrium torque u_e for a given angle theta_e.
        
        Physics derivation:
        Sum of torques = 0 at equilibrium (theta_ddot = 0, theta_dot = 0)
        tau_e - m*g*ell*cos(theta_e) - k1*theta_e - k2*theta_e^3 = 0
        """
        u_e = P.m * P.g * P.ell * np.cos(theta_e) + P.k1 * theta_e + P.k2 * (theta_e**3)
        return np.array([u_e])

    def update_with_measurement(self, r, y):
        """
        Returns the constant equilibrium torque.
        r and y are ignored as this is a feedforward-only controller.
        """
        return self.u_e, np.zeros(2)

    def update_with_state(self, r, x):
        """
        Returns the constant equilibrium torque.
        r and x are ignored.
        """
        return self.u_e

if __name__ == "__main__":
    # Test calculation for theta_e = 0
    ctrl = EquilibriumController(theta_e=0.0)
    print(f"Equilibrium torque for theta_e=0: {ctrl.u_e[0]:.4f} N-m")
    # Expected: 0.1 * 9.8 * 0.25 = 0.245
