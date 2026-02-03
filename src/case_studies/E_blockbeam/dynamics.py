# 3rd-party
import numpy as np

from ..common.dynamics_base import DynamicsBase
# local (controlbook)
from . import params as P


class BlockbeamDynamics(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            state0=np.array([P.z0, P.theta0, P.zdot0, P.thetadot0]),
            u_max=P.F_max,
            u_min=-P.F_max,
            dt=P.ts,
        )
        self.m1 = self.randomize_parameter(P.m1, alpha)
        self.m2 = self.randomize_parameter(P.m2, alpha)
        self.L = self.randomize_parameter(P.L, alpha)
        self.g = P.g
        self.I_beam = self.randomize_parameter(P.I_beam, alpha)

    def f(self, x, u):
        z = x[0]
        theta = x[1]
        zdot = x[2]
        thetadot = x[3]
        F = u[0]
        
        zddot = z * thetadot**2 - self.g * np.sin(theta)
        
        num = F * self.L * np.cos(theta) \
              - 2 * self.m1 * z * zdot * thetadot \
              - (self.m1 * z + self.m2 * self.L / 2) * self.g * np.cos(theta)
        den = self.I_beam + self.m1 * z**2
        thetaddot = num / den
        
        return np.array([zdot, thetadot, zddot, thetaddot])

    def h(self):
        return self.state[0:2]
