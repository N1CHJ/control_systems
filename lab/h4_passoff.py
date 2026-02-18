import os
import sys

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import case_studies.H_hummingbird.params as P
from case_studies import H_hummingbird as H_hub

sep = "=" * 60

# ─── 1. Motor constant km ────────────────────────────────────
print(sep)
print("  Motor Constant")
print(sep)
print(f"  km = g*(m1*l1 + m2*l2) / lT")
print(f"     = {P.g}*({P.m1}*{P.l1} + {P.m2}*{P.l2}) / {P.lT}")
print(f"     = {P.km:.6f}  N per unit PWM")

# ─── 2. Equilibrium values ───────────────────────────────────
print(f"\n{sep}")
print("  Equilibrium Values")
print(sep)
print(f"  F_e   = {P.F_e:.6f} N   (total lift to hover)")
print(f"  tau_e = {P.tau_e:.6f} N·m (no torque at equilibrium)")
print(f"  u_e   = {P.u_e:.4f}       (each motor PWM)")
print(f"  θ_e = {P.theta_e},  φ_e = {P.phi_e},  ψ_e = {P.psi_e}")

# ─── 3. Linearized equations ─────────────────────────────────
print(f"\n{sep}")
print("  Linearized Longitudinal Dynamics  (Eqn 4.3)")
print(sep)
Jy_eff = P.m1*P.l1**2 + P.m2*P.l2**2 + P.J1y + P.J2y
print(f"  θ̈  = b_θ · F̃")
print(f"  b_θ = ℓ_T / J_y_eff")
print(f"      = {P.lT} / ({P.m1}·{P.l1}² + {P.m2}·{P.l2}² + {P.J1y} + {P.J2y})")
print(f"      = {P.lT} / {Jy_eff:.8f}")
print(f"      = {P.b_theta:.6f}")

print(f"\n{sep}")
print("  Linearized Lateral Dynamics  (Eqns 4.7–4.8)")
print(sep)
print(f"  φ̈  = b_φ · τ̃")
print(f"  b_φ = 1 / J_1x = 1 / {P.J1x} = {P.b_phi:.6f}")
print()
J_psi = (P.J1z + P.J2z + P.J3z
         + P.l1**2*P.m1 + P.l2**2*P.m2
         + P.l3x**2*P.m3 + P.l3y**2*P.m3)
print(f"  ψ̈  = a_ψ · φ̃")
print(f"  a_ψ = F_e·ℓ_T / J_ψ")
print(f"      = {P.F_e:.6f}·{P.lT} / {J_psi:.8f}")
print(f"      = {P.a_psi:.6f}")

# ─── 4. Mixing matrices ──────────────────────────────────────
print(f"\n{sep}")
print("  Mixing Matrices")
print(sep)
print(f"  unmixer (forces → F,τ):  {P.unmixer.tolist()}")
print(f"  mixer   (F,τ → forces):  {P.mixer.tolist()}")
