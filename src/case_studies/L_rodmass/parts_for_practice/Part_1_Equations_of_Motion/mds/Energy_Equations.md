# Energy Equations for Part 1

## Kinetic Energy ($K$)
For a point mass $m$ at the end of a rod of length $\ell$ rotating with angular velocity $\dot{\theta}$:
- Velocity magnitude: $v = \ell \dot{\theta}$
- Kinetic Energy: $K = \frac{1}{2} m v^2 = \frac{1}{2} m \ell^2 \dot{\theta}^2$

## Potential Energy ($P$)
The total potential energy is the sum of gravitational potential energy and the energy stored in the nonlinear spring.

### Gravitational Potential Energy ($V_g$)
Relative to the joint (origin):
- $y = \ell \sin(\theta)$
- $V_g = m g y = m g \ell \sin(\theta)$

### Nonlinear Spring Potential Energy ($V_{spring}$)
As given in the problem:
- $V_{spring} = \frac{1}{2} k_1 \theta^2 + \frac{1}{4} k_2 \theta^4$

### Total Potential Energy
- $P = V_g + V_{spring} = m g \ell \sin(\theta) + \frac{1}{2} k_1 \theta^2 + \frac{1}{4} k_2 \theta^4$

## Lagrangian ($L$)
- $L = K - P$
- $L = \frac{1}{2} m \ell^2 \dot{\theta}^2 - \left( m g \ell \sin(\theta) + \frac{1}{2} k_1 \theta^2 + \frac{1}{4} k_2 \theta^4 \right)$
