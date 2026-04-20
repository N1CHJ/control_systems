"""
Symbolic Dynamics Deriver (Euler-Lagrange Method)
=================================================
This file provides a generalized class to derive the equations of motion (EOM)
for any mechanical system using the Euler-Lagrange equations, SymPy, and NumPy.

You can adapt this class for your final exam by plugging in the appropriate
kinetic energy (K), potential energy (P), and generalized forces (Q) for the 
specific system you are given.

Dependencies:
- sympy
- numpy
"""

import sympy as sp
import numpy as np

class EulerLagrangeDeriver:
    def __init__(self):
        """
        Initialize the symbolic variables. 
        Modify this method to include the specific variables for your system.
        """
        # Define time variable
        self.t = sp.symbols('t')
        
        # 1. Define generalized coordinates (q) as functions of time
        # Example for a simple pendulum or rod:
        self.theta = sp.Function('theta')(self.t)
        
        # 2. Define their derivatives (q_dot and q_ddot)
        self.theta_dot = sp.diff(self.theta, self.t)
        self.theta_ddot = sp.diff(self.theta_dot, self.t)
        
        # 3. Define system parameters (masses, lengths, spring constants, etc.)
        # Add any constants given in the problem statement here
        self.m, self.ell, self.g = sp.symbols('m ell g', positive=True)
        self.k1, self.k2, self.b = sp.symbols('k1 k2 b', positive=True)
        
        # 4. Define inputs / external forces
        self.tau = sp.symbols('tau')

    def define_energies(self):
        """
        Define the kinetic energy (K) and potential energy (P) of the system.
        Modify these equations based on the specific geometry of your exam problem.
        """
        # --- KINETIC ENERGY (K) ---
        # For a point mass at the end of a rotating rod: K = 1/2 * m * v^2
        # Velocity v = ell * theta_dot
        self.K = 0.5 * self.m * (self.ell * self.theta_dot)**2
        
        # --- POTENTIAL ENERGY (P) ---
        # 1. Gravitational Potential Energy: Vg = m * g * h
        # Assuming height h = ell * sin(theta) relative to the joint
        self.Vg = self.m * self.g * self.ell * sp.sin(self.theta)
        
        # 2. Spring Potential Energy (Nonlinear spring given in Part 1)
        # Vspring = 1/2 * k1 * theta^2 + 1/4 * k2 * theta^4
        self.Vspring = 0.5 * self.k1 * self.theta**2 + 0.25 * self.k2 * self.theta**4
        
        # Total Potential Energy
        self.P = self.Vg + self.Vspring

        # --- LAGRANGIAN (L) ---
        self.L = self.K - self.P

    def define_generalized_forces(self):
        """
        Define the non-conservative generalized forces (Q) corresponding to 
        each generalized coordinate.
        """
        # For the theta coordinate, the generalized force includes the 
        # input torque (tau) and the viscous damping friction (-b * theta_dot)
        self.Q_theta = self.tau - self.b * self.theta_dot

    def derive_eom(self):
        """
        Apply the Euler-Lagrange equation to derive the equations of motion:
        d/dt (dL/dq_dot) - dL/dq = Q
        """
        print("Deriving Equations of Motion...")
        
        # 1. Calculate dL/dq_dot
        dL_dtheta_dot = sp.diff(self.L, self.theta_dot)
        
        # 2. Calculate d/dt (dL/dq_dot)
        ddt_dL_dtheta_dot = sp.diff(dL_dtheta_dot, self.t)
        
        # 3. Calculate dL/dq
        dL_dtheta = sp.diff(self.L, self.theta)
        
        # 4. Form the Euler-Lagrange equation
        # Left-Hand Side (LHS) = Right-Hand Side (RHS)
        self.eom_lhs = ddt_dL_dtheta_dot - dL_dtheta
        self.eom_rhs = self.Q_theta
        
        # Define the equation: LHS - RHS = 0
        self.eom_eq = sp.Eq(self.eom_lhs, self.eom_rhs)
        
        print(f"\nEuler-Lagrange Equation for theta:")
        sp.pprint(self.eom_eq)
        
        # 5. Solve for the highest derivative (theta_ddot)
        self.accelerations = sp.solve(self.eom_eq, self.theta_ddot)
        
        print(f"\nSolved Equation of Motion (theta_ddot = ...):")
        sp.pprint(self.accelerations[0])
        
        return self.accelerations[0]

    def substitute_parameters(self, theta_ddot_expr):
        """
        Substitute numerical values into the symbolic equation for verification
        or for creating a numerical function.
        """
        # Define the numerical values from the problem
        param_values = {
            self.m: 0.1,
            self.ell: 0.25,
            self.g: 9.8,
            self.k1: 0.02,
            self.k2: 0.01,
            self.b: 0.1
        }
        
        numerical_eom = theta_ddot_expr.subs(param_values)
        print(f"\nNumerical Equation of Motion (parameters substituted):")
        sp.pprint(numerical_eom)
        
        return numerical_eom

# --- How to use this generic script ---
if __name__ == "__main__":
    # 1. Instantiate the deriver
    deriver = EulerLagrangeDeriver()
    
    # 2. Define energies based on your specific system
    deriver.define_energies()
    
    # 3. Define generalized forces
    deriver.define_generalized_forces()
    
    # 4. Derive the Equations of Motion
    theta_ddot = deriver.derive_eom()
    
    # 5. Substitute numerical values (optional, for verification)
    deriver.substitute_parameters(theta_ddot)
