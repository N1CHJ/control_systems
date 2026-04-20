# 3rd-party
import numpy as np

from ..common import ControllerBase
from . import params as P


class HummingbirdControllerFullPD(ControllerBase):

    def __init__(self, 
                 tr_theta: float = 1.0, zeta_theta: float = 0.707,
                 tr_psi: float = 1.0, zeta_psi: float = 0.707,
                 M_bandwidth: float = 10.0, zeta_phi: float = 0.707,
                 sigma: float = 0.05):
        """
        Initialize the Full PD controller.
        sigma (float): Dirty derivative
        """
        wn_theta = 2.2 / tr_theta
        self.kp_theta = wn_theta**2 / P.b_theta
        self.kd_theta = (2.0 * zeta_theta * wn_theta) / P.b_theta

        wn_psi = 2.2 / tr_psi
        self.kp_psi = wn_psi**2 / P.a_psi
        self.kd_psi = (2.0 * zeta_psi * wn_psi) / P.a_psi

        tr_phi = tr_psi / M_bandwidth
        wn_phi = 2.2 / tr_phi
        self.kp_phi = wn_phi**2 / P.b_phi
        self.kd_phi = (2.0 * zeta_phi * wn_phi) / P.b_phi

        print(f"Pitch Gains: kp = {self.kp_theta:.4f}, kd = {self.kd_theta:.4f}")
        print(f"Yaw Gains:   kp = {self.kp_psi:.4f}, kd = {self.kd_psi:.4f}")
        print(f"Roll Gains:  kp = {self.kp_phi:.4f}, kd = {self.kd_phi:.4f}")

        self.u_e = P.u_e
        self.km = P.km
        self.mixer = P.mixer  # [[0.5, 0.5/d], [0.5, -0.5/d]]

        self.phi_dot = 0.0
        self.theta_dot = 0.0
        self.psi_dot = 0.0
        self.phi_prev = 0.0
        self.theta_prev = 0.0
        self.psi_prev = 0.0
        
        self.sigma = sigma
        self.Ts = P.ts

    def update_with_state(self, r, x):
        """
        Full PD control law using full state feedback.
        r = [phi_ref, theta_ref, psi_ref]
        x = [phi, theta, psi, phidot, thetadot, psidot]
        """
        phi_ref_input, theta_ref, psi_ref = r
        phi, theta, psi, phidot, thetadot, psidot = x

        F_tilde = self.kp_theta * (theta_ref - theta) - self.kd_theta * thetadot
        # Total Force F = F_e * cos(theta) + F_tilde
        F = P.F_e * np.cos(theta) + F_tilde
        phi_ref_outer = self.kp_psi * (psi_ref - psi) - self.kd_psi * psidot
        phi_ref = phi_ref_input + phi_ref_outer

        tau = self.kp_phi * (phi_ref - phi) - self.kd_phi * phidot

        # [fl, fr]^T = mixer @ [F, tau]^T
        f_motors = self.mixer @ np.array([F, tau])
        u = f_motors / self.km
        u = self.saturate(u, u_max=1.0, u_min=0.0)

        return u, np.array([phidot, thetadot, psidot])

    def update_with_measurement(self, r, y):
        phi_ref_input, theta_ref, psi_ref = r
        phi, theta, psi = y

        # Dirty derivative
        a1 = (2.0 * self.sigma - self.Ts) / (2.0 * self.sigma + self.Ts)
        a2 = 2.0 / (2.0 * self.sigma + self.Ts)

        # Estimate rates
        self.phi_dot = a1 * self.phi_dot + a2 * (phi - self.phi_prev)
        self.theta_dot = a1 * self.theta_dot + a2 * (theta - self.theta_prev)
        self.psi_dot = a1 * self.psi_dot + a2 * (psi - self.psi_prev)
        
        self.phi_prev = phi
        self.theta_prev = theta
        self.psi_prev = psi

        x_hat = np.array([phi, theta, psi, self.phi_dot, self.theta_dot, self.psi_dot])
        return self.update_with_state(r, x_hat)
