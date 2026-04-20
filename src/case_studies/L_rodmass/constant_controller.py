"""
Constant Torque Controller
==========================
Simple controller that outputs a constant torque u_e.
Used to verify equilibrium in Part 2.
"""
import numpy as np
from ..common import ControllerBase

class ConstantController(ControllerBase):
    def __init__(self, u_e):
        self.u_e = np.array([u_e])

    def update_with_measurement(self, r, y):
        # Simply returns the constant equilibrium torque regardless of feedback
        # xhat is returned as zeros because this controller doesn't use an observer
        return self.u_e, np.zeros(2)

    def update_with_state(self, r, x):
        # Simply returns the constant equilibrium torque regardless of feedback
        return self.u_e
