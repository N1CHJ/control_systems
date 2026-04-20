# 3rd-party
import numpy as np

# local (controlbook)
from ..common import ControllerBase
from . import params as P


class HummingbirdControllerPID(ControllerBase):
    """
    Full PID controller for the Hummingbird.
    Longitudinal: PID for theta.
    Lateral: Inner PD for phi, Outer PID for psi.
    Includes anti-windup and dirty derivatives.
    """

    def __init__(self, 
                 tr_theta: float = 1.0, zeta_theta: float = 0.707, ki_theta: float = 0.5,
                 tr_psi: float = 1.0, zeta_psi: float = 0.707, ki_psi: float = 0.1,
                 M_bandwidth: float = 10.0, zeta_phi: float = 0.707,
                 sigma: float = 0.05):
        """
        Initialize the PID controller.
        
        Args:
            tr_theta (float): Rise time for pitch (longitudinal)
            zeta_theta (float): Damping ratio for pitch
            ki_theta (float): Integral gain for pitch
            tr_psi (float): Rise time for yaw (outer lateral)
            zeta_psi (float): Damping ratio for yaw
            ki_psi (float): Integral gain for yaw
            M_bandwidth (float): Bandwidth separation factor (tr_psi / tr_phi)
            zeta_phi (float): Damping ratio for roll (inner lateral)
            sigma (float): Dirty derivative filter parameter
        """
        # --- Pitch (Longitudinal) Gains ---
        wn_theta = 2.2 / tr_theta
        self.kp_theta = wn_theta**2 / P.b_theta
        self.kd_theta = (2.0 * zeta_theta * wn_theta) / P.b_theta
        self.ki_theta = ki_theta

        # --- Yaw (Outer Lateral) Gains ---
        wn_psi = 2.2 / tr_psi
        self.kp_psi = wn_psi**2 / P.a_psi
        self.kd_psi = (2.0 * zeta_psi * wn_psi) / P.a_psi
        self.ki_psi = ki_psi

        # --- Roll (Inner Lateral) Gains ---
        tr_phi = tr_psi / M_bandwidth
        wn_phi = 2.2 / tr_phi
        self.kp_phi = wn_phi**2 / P.b_phi
        self.kd_phi = (2.0 * zeta_phi * wn_phi) / P.b_phi

        print(f"Pitch Gains: kp = {self.kp_theta:.4f}, kd = {self.kd_theta:.4f}, ki = {self.ki_theta:.4f}")
        print(f"Yaw Gains:   kp = {self.kp_psi:.4f}, kd = {self.kd_psi:.4f}, ki = {self.ki_psi:.4f}")
        print(f"Roll Gains:  kp = {self.kp_phi:.4f}, kd = {self.kd_phi:.4f}")

        # Equilibrium and Mixer parameters
        self.u_e = P.u_e
        self.km = P.km
        self.mixer = P.mixer

        # State for Integrators and Dirty Derivatives
        self.integrator_theta = 0.0
        self.integrator_psi = 0.0
        
        self.error_theta_prev = 0.0
        self.error_psi_prev = 0.0
        
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
        PID control law using full state feedback.
        Note: Typically integral control is implemented in update_with_measurement
        for digital systems, but we provide this for consistency.
        """
        # This controller is primarily intended for update_with_measurement
        # because of the integrators and digital implementation.
        return self.update_with_measurement(r, x[:3])

    def update_with_measurement(self, r, y):
        """
        Digital PID control law using output measurement, integrators, and dirty derivative.
        r = [phi_ref_input, theta_ref, psi_ref]
        y = [phi, theta, psi]
        """
        phi_ref_input, theta_ref, psi_ref = r
        phi, theta, psi = y

        #Dirty derivative to estimate rates
        a1 = (2.0 * self.sigma - self.Ts) / (2.0 * self.sigma + self.Ts)
        a2 = 2.0 / (2.0 * self.sigma + self.Ts)

        self.phi_dot = a1 * self.phi_dot + a2 * (phi - self.phi_prev)
        self.theta_dot = a1 * self.theta_dot + a2 * (theta - self.theta_prev)
        self.psi_dot = a1 * self.psi_dot + a2 * (psi - self.psi_prev)
        
        self.phi_prev = phi
        self.theta_prev = theta
        self.psi_prev = psi

        # Longitudinal Control (Pitch PID)
        error_theta = theta_ref - theta
        # Integration
        self.integrator_theta += (self.Ts / 2.0) * (error_theta + self.error_theta_prev)
        self.error_theta_prev = error_theta
        self.integrator_theta = np.clip(self.integrator_theta, -2.0, 2.0)
        
        F_tilde = (self.kp_theta * error_theta + 
                   self.ki_theta * self.integrator_theta - 
                   self.kd_theta * self.theta_dot)
        
        F = P.F_e * np.cos(theta) + F_tilde

        # Outer Loop Lateral Control (Yaw PID to Roll Reference)
        error_psi = psi_ref - psi
        # Integration
        self.integrator_psi += (self.Ts / 2.0) * (error_psi + self.error_psi_prev)
        self.error_psi_prev = error_psi
        
        self.integrator_psi = np.clip(self.integrator_psi, -1.0, 1.0)

        phi_ref_outer = (self.kp_psi * error_psi + 
                         self.ki_psi * self.integrator_psi - 
                         self.kd_psi * self.psi_dot)
        phi_ref = phi_ref_input + phi_ref_outer

        tau = self.kp_phi * (phi_ref - phi) - self.kd_phi * self.phi_dot

        # [fl, fr]^T = mixer @ [F, tau]^T
        f_motors = self.mixer @ np.array([F, tau])
        u_unsat = f_motors / self.km
        
        # Saturate PWM to [0, 1]
        u = np.clip(u_unsat, 0.0, 1.0)

        # Anti-windup via back-calculation
        kb = 5.0 
        
        sat_error = u - u_unsat
        
        # Correct integrators
        self.integrator_theta += self.Ts * kb * np.mean(sat_error)
        self.integrator_psi += self.Ts * kb * (sat_error[0] - sat_error[1])

        return u, np.array([self.phi_dot, self.theta_dot, self.psi_dot])
