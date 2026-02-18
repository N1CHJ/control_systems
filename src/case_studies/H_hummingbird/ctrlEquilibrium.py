# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common
from case_studies.H_hummingbird import params as P


class HummingbirdControllerEquilibrium(common.ControllerBase):
    """
    Equilibrium controller for Lab H.4.

    Outputs constant PWM commands [u_l, u_r] that hold the hummingbird at
    the equilibrium point (phi=0, theta=0, psi=0) with no angular velocities.

    This controller verifies that F_e = (m1*l1 + m2*l2)*g / l_T is the correct
    equilibrium force.  With zero initial conditions the system should remain
    stationary.
    """

    def __init__(self):
        self.F_e = P.F_e
        self.tau_e = P.tau_e
        self.km = P.km
        self.mixer = P.mixer  # [fl, fr] = mixer @ [F, tau]

    def update_with_state(self, r, x):
        """
        Returns equilibrium PWM commands regardless of state or reference.

        Args:
            r: reference vector (ignored)
            x: state vector (ignored)
        Returns:
            pwm: numpy array [u_l, u_r] in range [0, 1]
        """
        F = self.F_e
        tau = self.tau_e  # = 0

        # Convert [F, tau] to individual motor forces [fl, fr]
        forces = self.mixer @ np.array([F, tau])

        # Convert forces to PWM duty cycle
        pwm = forces / self.km

        # Clip to valid range
        pwm = np.clip(pwm, 0.0, 1.0)

        return pwm
