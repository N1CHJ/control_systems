# 3rd-party
import numpy as np

from ..common.dynamics_base import DynamicsBase
# local (controlbook)
from . import params as P


class VTOLDynamics(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            state0=np.array([P.zv0, P.h0, P.theta0, P.zvdot0, P.hdot0, P.thetadot0]),
            u_max=P.f_max,
            u_min=0.0, # Rotor force usually non-negative
            dt=P.ts,
        )
        self.M = self.randomize_parameter(P.M, alpha)
        self.J = self.randomize_parameter(P.J, alpha)
        self.d = self.randomize_parameter(P.d, alpha)
        self.mu = self.randomize_parameter(P.mu, alpha)
        self.g = P.g

    def f(self, x, u):
        zv = x[0]
        h = x[1]
        theta = x[2]
        zvdot = x[3]
        hdot = x[4]
        thetadot = x[5]
        
        fr = u[0]
        fl = u[1]
        
        zvddot = (-(fr + fl) * np.sin(theta) - self.mu * zvdot) / self.M
        hddot = ((fr + fl) * np.cos(theta) - self.M * self.g) / self.M
        thetaddot = self.d * (fr - fl) / self.J
        
        return np.array([zvdot, hdot, thetadot, zvddot, hddot, thetaddot])

    def h(self):
        return self.state[0:3]
