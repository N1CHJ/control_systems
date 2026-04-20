# Linearization Theory & Steps

Linearization is the process of finding a linear approximation to a nonlinear system near an equilibrium point.

## 1. Identify Equilibrium ($x_e, u_e$)
The system is at equilibrium when $\dot{x} = f(x_e, u_e) = 0$.
For the Rod-Mass system, we usually choose:
- $x_e = [\theta_e, 0]^T$
- $u_e = \tau_e$ (The torque required to balance the system)

## 2. Taylor Series Expansion
We define small deviations from equilibrium:
- $\tilde{x} = x - x_e$
- $\tilde{u} = u - u_e$

The linear system is defined by the Jacobians:
$$A = \left. \frac{\partial f}{\partial x} \right|_{x_e, u_e}, \quad B = \left. \frac{\partial f}{\partial u} \right|_{x_e, u_e}$$
$$C = \left. \frac{\partial h}{\partial x} \right|_{x_e, u_e}, \quad D = \left. \frac{\partial h}{\partial u} \right|_{x_e, u_e}$$

## 3. The Linear Model
The resulting linear model is:
$$\dot{\tilde{x}} = A \tilde{x} + B \tilde{u}$$
$$\tilde{y} = C \tilde{x} + D \tilde{u}$$

## 4. Key Derivatives for Rod-Mass
If $f_2(x, u) = \frac{1}{m\ell^2} (\tau - b\dot{\theta} - mg\ell\cos\theta - k_1\theta - k_2\theta^3)$:

- $\frac{\partial f_2}{\partial \theta} = \frac{1}{m\ell^2} (mg\ell\sin\theta - k_1 - 3k_2\theta^2)$
- $\frac{\partial f_2}{\partial \dot{\theta}} = -\frac{b}{m\ell^2}$
- $\frac{\partial f_2}{\partial \tau} = \frac{1}{m\ell^2}$

## 5. Implementation Strategy
**ALWAYS** use the numerical linearization tool (`design_model_tool.py`) first. It is faster and less prone to sign errors. Use the analytical derivatives above only to "sanity check" the numerical results or if the exam explicitly asks for the symbolic form.
