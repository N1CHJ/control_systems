"""
Part 2: Design Models - Linearization and Equilibrium Analysis
==============================================================
This script provides a robust and generalized framework for:
1. Finding the equilibrium input (u_e) for a desired state (x_e).
2. Performing numerical linearization of non-linear dynamics.
3. Converting state-space models to transfer functions.
4. Comparing custom dynamics implementation against obfuscated/compiled versions.

This code is designed to be easily adaptable for the final exam.
"""

import numpy as np
import control as cnt
import sympy as sp
from scipy.optimize import fsolve

# Import the system parameters and dynamics
# Note: In the final, you would change these imports to match the new system
try:
    from L_rodmass import params as P
    from L_rodmass.dynamics import RodMassDynamics as Dynamics
    # To use the obfuscated version for verification, uncomment the next line:
    # from L_rodmass.dynamics_compiled import RodMassDynamics as Dynamics_Verified
except ImportError:
    print("Warning: Could not import L_rodmass modules. Ensure you are in the project root.")

class SystemDesignTool:
    def __init__(self, dynamics_obj):
        """
        Initialize with a dynamics object that has f(x, u) and h(x) methods.
        """
        self.sys = dynamics_obj

    def find_equilibrium(self, x_e, u_guess):
        """
        Finds the equilibrium input u_e such that f(x_e, u_e) = 0.
        
        Args:
            x_e: Desired equilibrium state [theta_e, thetadot_e]
            u_guess: Initial guess for the equilibrium torque/force
            
        Returns:
            u_e: The calculated equilibrium input
        """
        # Objective function: we want f(x_e, u) to return a zero vector
        # Since f returns [x1_dot, x2_dot], we minimize the norm or find root
        def objective(u):
            # u is passed as a list/array by fsolve, dynamics expects array-like
            xdot = self.sys.f(x_e, np.array([u]))
            return xdot

        # We use fsolve to find where xdot is zero. 
        # Note: If xdot has multiple components, fsolve handles it.
        # For our system, we specifically care about making thetadot_dot = 0.
        u_e = fsolve(objective, u_guess)
        return u_e[0]

    def compute_ss_matrices(self, x_e, u_e, eps=1e-6):
        """
        Numerically linearizes the system around (x_e, u_e).
        x_dot = A(x - x_e) + B(u - u_e)
        y = C(x - x_e) + D(u - u_e)
        """
        n = len(x_e)
        m = 1 # Assuming single input for this exam
        p = len(self.sys.h(x_e)) # Number of outputs

        A = np.zeros((n, n))
        B = np.zeros((n, m))
        C = np.zeros((p, n))
        D = np.zeros((p, m))

        # --- Calculate A matrix (df/dx) ---
        for i in range(n):
            x_plus = np.array(x_e, dtype=float)
            x_plus[i] += eps
            f_plus = self.sys.f(x_plus, np.array([u_e]))
            
            x_minus = np.array(x_e, dtype=float)
            x_minus[i] -= eps
            f_minus = self.sys.f(x_minus, np.array([u_e]))
            
            A[:, i] = (f_plus - f_minus) / (2 * eps)

        # --- Calculate B matrix (df/du) ---
        u_plus = u_e + eps
        f_plus = self.sys.f(x_e, np.array([u_plus]))
        
        u_minus = u_e - eps
        f_minus = self.sys.f(x_e, np.array([u_minus]))
        
        B[:, 0] = (f_plus - f_minus) / (2 * eps)

        # --- Calculate C matrix (dh/dx) ---
        for i in range(n):
            x_plus = np.array(x_e, dtype=float)
            x_plus[i] += eps
            h_plus = self.sys.h(x_plus)
            
            x_minus = np.array(x_e, dtype=float)
            x_minus[i] -= eps
            h_minus = self.sys.h(x_minus)
            
            C[:, i] = (h_plus - h_minus) / (2 * eps)

        # --- Calculate D matrix (dh/du) ---
        # (Usually zero for mechanical systems where output is position)
        D = np.zeros((p, m)) 

        return A, B, C, D

    def get_transfer_function(self, A, B, C, D):
        """
        Converts State-Space matrices to a Transfer Function.
        """
        ss_sys = cnt.ss(A, B, C, D)
        tf_sys = cnt.ss2tf(ss_sys)
        return tf_sys

def verify_dynamics_implementation():
    """
    Compares the student's dynamics against the obfuscated 'verified' version.
    This is useful to ensure Part 1 was done correctly before starting Part 2.
    """
    print("\n--- Dynamics Verification ---")
    try:
        from L_rodmass.dynamics import RodMassDynamics as DynStudent
        from L_rodmass.dynamics_compiled import RodMassDynamics as DynVerified
        
        s_student = DynStudent()
        s_verified = DynVerified()
        
        test_x = np.array([0.1, 0.5])
        test_u = np.array([1.0])
        
        dot_s = s_student.f(test_x, test_u)
        dot_v = s_verified.f(test_x, test_u)
        
        if np.allclose(dot_s, dot_v):
            print("SUCCESS: Your dynamics implementation matches the verified version!")
        else:
            print("FAILURE: Your dynamics implementation differs from the verified version.")
            print(f"Student x_dot: {dot_s}")
            print(f"Verified x_dot: {dot_v}")
    except Exception as e:
        print(f"Verification could not run: {e}")

if __name__ == "__main__":
    # 1. Verify dynamics before proceeding
    verify_dynamics_implementation()

    # 2. Setup the tool
    system_dynamics = Dynamics(alpha=0.0)
    design_tool = SystemDesignTool(system_dynamics)

    # 3. Define the Equilibrium Point (Target for linearization)
    # Example: theta_e = 0 degrees
    theta_e = 0.0 
    x_e = np.array([theta_e, 0.0]) # thetadot is 0 at equilibrium

    # 4. Find Equilibrium Torque (u_e)
    u_e = design_tool.find_equilibrium(x_e, u_guess=0.0)
    print(f"\nEquilibrium Analysis for theta_e = {theta_e} rad:")
    print(f"Calculated u_e (tau_e): {u_e:.6f} N-m")

    # 5. Perform Linearization
    A, B, C, D = design_tool.compute_ss_matrices(x_e, u_e)
    print("\n" + "="*50)
    print("STATE SPACE MODEL (For Exam Question 2.5)")
    print("="*50)
    print(f"Equilibrium: theta_e = {theta_e}, u_e = {u_e:.4f}")
    print("\nMatrix A:")
    print(A)
    print("\nMatrix B:")
    print(B)
    print("\nMatrix C:")
    print(C)
    print("\nMatrix D:")
    print(D)
    
    print("\nForm x_dot = A*x_tilde + B*u_tilde:")
    print(f"[theta_dot ]   [{A[0,0]:.4f}  {A[0,1]:.4f}] [theta_tilde    ]   [{B[0,0]:.4f}]")
    print(f"[theta_ddot] = [{A[1,0]:.4f}  {A[1,1]:.4f}] [thetadot_tilde ] + [{B[1,0]:.4f}] * tau_tilde")
    
    print("\nForm y_tilde = C*x_tilde + D*u_tilde:")
    print(f"y_tilde = [{C[0,0]:.4f}  {C[0,1]:.4f}] [theta_tilde, thetadot_tilde]^T")

    # 6. Find Transfer Function
    tf = design_tool.get_transfer_function(A, B, C, D)
    print("\n" + "="*50)
    print("TRANSFER FUNCTION P(s) (For Exam Question 2.4)")
    print("="*50)
    print(tf)
    print("\nCopy-paste friendly denominator coefficients:")
    print(f"s^2 + {tf.den[0][0][1]:.4f}s + {tf.den[0][0][2]:.4f}")
    print(f"Numerator: {tf.num[0][0][2]:.4f}")


    # 7. Quick access for Final Exam pattern matching:
    # If the system is theta_ddot = (1/J) * (tau - b*theta_dot - mgl*cos(theta))
    # The A matrix should look like:
    # [[0, 1],
    #  [-(mgl*sin(theta_e))/J, -b/J]]
    # (Note: For the practice final, it also includes spring terms k1, k2)
