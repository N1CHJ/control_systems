# 3rd-party
import numpy as np

from ..common.dynamics_base import DynamicsBase
# local (controlbook)
from . import params as P


class MassDynamics(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            state0=np.array([P.z0, P.zdot0]),
            u_max=P.F_max,
            u_min=-P.F_max,
            dt=P.ts,
        )
        self.m = self.randomize_parameter(P.m, alpha)
        self.k = self.randomize_parameter(P.k, alpha)
        self.b = self.randomize_parameter(P.b, alpha)

    def f(self, x, u):
        z = x[0]
        zdot = x[1]
        F = u[0]
        zddot = (F - self.b * zdot - self.k * z) / self.m
        return np.array([zdot, zddot])

    def h(self):
        z = self.state[0]
        return np.array([z])
