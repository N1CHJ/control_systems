# Steps to Derive Equations of Motion

## 1. Define Generalized Coordinates
Identify the degrees of freedom and choose the appropriate generalized coordinate(s) $q$.
- For this problem: $q = \theta$

## 2. Identify Kinetic and Potential Energy
- $K = \text{Kinetic Energy}$
- $P = \text{Potential Energy}$

## 3. Form the Lagrangian
- $L = K - P$

## 4. Identify Generalized Forces ($Q$)
Generalized forces include non-conservative forces like damping and external torques.
- $Q = \tau - b \dot{\theta}$
  - $\tau$ is the input torque.
  - $-b \dot{\theta}$ is the damping torque (viscous friction).

## 5. Apply Euler-Lagrange Equation
For each generalized coordinate $q_i$:
$$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}_i} \right) - \frac{\partial L}{\partial q_i} = Q_i$$

### Detailed Calculation for $\theta$:
1. Compute $\frac{\partial L}{\partial \dot{\theta}}$:
   - $\frac{\partial L}{\partial \dot{\theta}} = m \ell^2 \dot{\theta}$
2. Compute the time derivative $\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right)$:
   - $\frac{d}{dt} (m \ell^2 \dot{\theta}) = m \ell^2 \ddot{\theta}$
3. Compute $\frac{\partial L}{\partial \theta}$:
   - $\frac{\partial L}{\partial \theta} = - (m g \ell \cos(\theta) + k_1 \theta + k_2 \theta^3)$
4. Combine into Euler-Lagrange:
   - $m \ell^2 \ddot{\theta} - [- (m g \ell \cos(\theta) + k_1 \theta + k_2 \theta^3)] = \tau - b \dot{\theta}$
   - $m \ell^2 \ddot{\theta} + m g \ell \cos(\theta) + k_1 \theta + k_2 \theta^3 = \tau - b \dot{\theta}$

## 6. Solve for $\ddot{\theta}$
Rearrange the equation to solve for the highest derivative:
$$\ddot{\theta} = \frac{1}{m \ell^2} \left( \tau - b \dot{\theta} - m g \ell \cos(\theta) - k_1 \theta - k_2 \theta^3 \right)$$
