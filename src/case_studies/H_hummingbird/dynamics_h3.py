# 3rd-party
import numpy as np

from ..common.dynamics_base import DynamicsBase
# local
from . import params as P


class HummingbirdDynamics_h3(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            # Initial state conditions
            state0=np.array([P.phi0, P.theta0, P.psi0, P.phidot0, P.thetadot0, P.psidot0]),
            u_max=10.0,
            u_min=-10.0,
            dt=P.ts,
        )
        # Parameters from appendix with uncertainty
        self.m1 = P.m1 * (1.0 + alpha * (2 * np.random.rand() - 1))
        self.m2 = P.m2 * (1.0 + alpha * (2 * np.random.rand() - 1))
        self.l1 = P.l1 * (1.0 + alpha * (2 * np.random.rand() - 1))
        self.l2 = P.l2 * (1.0 + alpha * (2 * np.random.rand() - 1))
        self.lT = P.lT * (1.0 + alpha * (2 * np.random.rand() - 1))
        self.d = P.d * (1.0 + alpha * (2 * np.random.rand() - 1))
        
        self.J1x = P.J1x * (1.0 + alpha * (2 * np.random.rand() - 1))
        self.J1y = P.J1y * (1.0 + alpha * (2 * np.random.rand() - 1))
        self.J1z = P.J1z * (1.0 + alpha * (2 * np.random.rand() - 1))
        self.J2x = P.J2x * (1.0 + alpha * (2 * np.random.rand() - 1))
        self.J2y = P.J2y * (1.0 + alpha * (2 * np.random.rand() - 1))
        self.J2z = P.J2z * (1.0 + alpha * (2 * np.random.rand() - 1))

        self.b_phi = P.b_phi
        self.b_theta = P.b_theta
        self.b_psi = P.b_psi
        self.g = P.g

    def f(self, x, u):
        """
        Return xdot = f(x,u).
        x = [phi, theta, psi, phidot, thetadot, psidot]
        u = [fl, fr]
        """
        phi, theta, psi, phidot, thetadot, psidot = x
        fl, fr = u
        cpeak = np.cos(theta)
        speak = np.sin(theta)
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        
        # Generalized Forces
        tau_phi = -self.d * (fl - fr) - self.b_phi * phidot
        tau_theta = self.lT * (fl + fr) * cphi - self.b_theta * thetadot
        tau_psi = self.lT * (fl + fr) * sphi - self.b_psi * psidot
        
        # M11 * phidd + ... = tau_phi
        # M22 * thetadd + ... = tau_theta
        # M33 * psidd + ... = tau_psi
        
        # 1. Roll Equation
        # M_phi = J1x
        # J1x * phidd = tau_phi
        phidd = tau_phi / self.J1x
        
        # 2. Pitch Equation
        # Inertia about y-axis m1*l1^2 + m2*l2^2 + J1y + J2y
        J_pitch = self.J1y + self.J2y + self.m1 * self.l1**2 + self.m2 * self.l2**2
        # Gravity moment: m1*g*l1*cos(theta) - m2*g*l2*cos(theta)
        g_moment = (self.m1 * self.l1 - self.m2 * self.l2) * self.g * cpeak
        # J_pitch * thetadd + g_moment = tau_theta
        thetadd = (tau_theta - -g_moment) / J_pitch
        # Gravity torque = -(m1*l1 - m2*l2)*g*cos(theta).
        # Equation: J*thetadd - GravityTorque = ExternalTorque
        # J*thetadd - (-(m1 l1 - m2 l2) g cos(theta)) = tau
        thetadd = (tau_theta + (self.m1 * self.l1 - self.m2 * self.l2) * self.g * cpeak) / J_pitch

        # 3. Yaw Equation
        # J_yaw = m1 * (l1 cos(theta))^2 + m2 * (l2 cos(theta))^2 + J1z + J2z
        J_yaw = self.J1z + self.J2z + (self.m1 * self.l1**2 + self.m2 * self.l2**2) * cpeak**2
        psidd = tau_psi / J_yaw


        # State Derivats
        xdot = np.array([
            phidot,
            thetadot,
            psidot,
            phidd,
            thetadd,
            psidd
        ])
        
        return xdot

    def h(self):
        phi = self.state[0]
        theta = self.state[1]
        psi = self.state[2]
        y = np.array([phi, theta, psi])
        return y
