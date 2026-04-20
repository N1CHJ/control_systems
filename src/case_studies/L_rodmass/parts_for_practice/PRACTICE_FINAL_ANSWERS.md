# Practice Final Exam - Rod-Mass System: Answers and Derivations

This document contains all the derivations, calculations, and design values required for the practice final exam.

---

## Part 1: Equations of Motion

### 1.1 Kinetic Energy ($K$)
The mass is a point mass at distance $\ell$. Velocity $v = \ell \dot{\theta}$.
$$K = \frac{1}{2} m v^2 = \frac{1}{2} m \ell^2 \dot{\theta}^2$$

### 1.2 Potential Energy ($P$)
Includes gravity and the nonlinear spring.
- $P_{gravity} = m g h = m g \ell \sin\theta$ (assuming $h=0$ at horizontal)
- $P_{spring} = \frac{1}{2} k_1 \theta^2 + \frac{1}{4} k_2 \theta^4$
$$P = m g \ell \sin\theta + \frac{1}{2} k_1 \theta^2 + \frac{1}{4} k_2 \theta^4$$

### 1.3 Lagrangian ($L = K - P$)
$$L = \frac{1}{2} m \ell^2 \dot{\theta}^2 - m g \ell \sin\theta - \frac{1}{2} k_1 \theta^2 - \frac{1}{4} k_2 \theta^4$$

### 1.4 Generalized Forces ($Q$)
Includes input torque $\tau$ and viscous friction $-b\dot{\theta}$.
$$Q = \tau - b \dot{\theta}$$

### 1.5 Euler-Lagrange Equations
$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}}\right) - \frac{\partial L}{\partial \theta} = Q$$
- $\frac{\partial L}{\partial \dot{\theta}} = m \ell^2 \dot{\theta} \implies \frac{d}{dt}(\cdot) = m \ell^2 \ddot{\theta}$
- $\frac{\partial L}{\partial \theta} = -m g \ell \cos\theta - k_1 \theta - k_2 \theta^3$
$$m \ell^2 \ddot{\theta} + m g \ell \cos\theta + k_1 \theta + k_2 \theta^3 = \tau - b \dot{\theta}$$
$$\ddot{\theta} = \frac{1}{m \ell^2} (\tau - b \dot{\theta} - m g \ell \cos\theta - k_1 \theta - k_2 \theta^3)$$

---

## Part 2: Design Models

### 2.1 Equilibrium Torque ($\tau_e$)
At rest ($\dot{\theta}=0, \ddot{\theta}=0$) at angle $\theta_e$:
$$\tau_e = m g \ell \cos\theta_e + k_1 \theta_e + k_2 \theta_e^3$$
For $\theta_e = 0$:
$$\tau_e = m g \ell = (0.1)(9.8)(0.25) = 0.245 \text{ N-m}$$

### 2.3 Linearization
Linearizing around $\theta_e = 0, \tau_e = m g \ell$:
$$\Delta \ddot{\theta} + \frac{b}{m \ell^2} \Delta \dot{\theta} + \frac{k_1}{m \ell^2} \Delta \theta = \frac{1}{m \ell^2} \Delta \tau$$

### 2.4 Transfer Function
$$P(s) = \frac{\Theta(s)}{T(s)} = \frac{1/m\ell^2}{s^2 + \frac{b}{m\ell^2} s + \frac{k_1}{m\ell^2}}$$
With nominal parameters ($m\ell^2 = 0.00625$):
$$P(s) = \frac{160}{s^2 + 16s + 3.2}$$

### 2.5 State Space Model
$$A = \begin{bmatrix} 0 & 1 \\ -3.2 & -16 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 160 \end{bmatrix}, \quad C = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

---

## Part 3: PID Control

### 3.3 Proportional Gain ($k_p$) for Saturation
If $\tau_{max} = 3.0$ and we saturate at $\theta_e = 20^\circ$ ($0.349$ rad):
$$\Delta \tau = k_p \Delta \theta \implies 3.0 - 0.245 = k_p (0.349) \implies k_p \approx 7.9$$

### 3.4 PD Design for Poles
Desired: $\Delta_{cl}(s) = s^2 + 2\zeta\omega_n s + \omega_n^2$.
Actual: $s^2 + (\frac{b+kd}{m\ell^2})s + (\frac{k_1+kp}{m\ell^2})$.
For $\zeta=0.9$ and $\omega_n$ chosen for performance:
- $\omega_n = \sqrt{(k_1+k_p)/m\ell^2}$
- $k_d = 2\zeta\omega_n m\ell^2 - b$

---

## Part 4: State Space Control

### 4.1 SSI Gains
Gains $K = [k_\theta, k_{\dot{\theta}}]$ and $k_i$ placed such that poles are at e.g., $[-5, -5.1, -10]$.
Typical values (from `ssi_controller.py`):
- $K = [0.039, 0.015]$ (Example, depends on pole locations)
- $k_i = 0.0625$

### 4.3 Observer Gains ($L$)
Poles placed 5x faster than controller poles.
$$L = \begin{bmatrix} l_1 \\ l_2 \end{bmatrix}$$

### 4.4 Disturbance Observer ($L_2$)
Poles for state and disturbance $[x, d]$ placed at e.g., $[-25, -26, -27]$.
Provides rejection of torque disturbances like the $0.5$ N-m offset in Part 4.5.

### 4.7 LQR Optimization
$Q = \text{diag}([10, 1, 100])$ (Penalize position, velocity, and integral error).
$R = [0.1]$ (Penalize torque).
Optimal $K, k_i$ derived from Riccati equation.
