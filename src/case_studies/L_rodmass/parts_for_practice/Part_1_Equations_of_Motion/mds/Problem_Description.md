# Part 1: Equations of Motion - Simulation Model

## Problem Description
A point mass $m$ is connected to a massless rod of length $\ell$. The rod is connected to a wall with a nonlinear spring and a damper. The system is subject to gravity $g$.

### Physical Parameters
- Gravity: $g = 9.8$ m/s²
- Rod length: $\ell = 0.25$ m
- Mass: $m = 0.1$ kg
- Nonlinear spring constants: $k_1 = 0.02$, $k_2 = 0.01$
- Viscous friction (damping): $b = 0.1$
- Input torque: $\tau$ (limited to $\pm 3.0$ N-m)

### Configuration Variable
- $q = \theta$ (angle of the rod from the horizontal, as shown in the diagram)

### Tasks
1. Find the kinetic energy $K$.
2. Find the potential energy $P$.
3. Find the Lagrangian $L = K - P$.
4. Find the generalized forces.
5. Derive the equations of motion using Euler-Lagrange.
