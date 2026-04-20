# Lab Assignment H.9: System Type and Integrators

This document provides a theoretical analysis of the Hummingbird system type and steady-state error characteristics when using PD control.

## 1. Inner-Loop Lateral Dynamics (Roll, $\phi$)

The transfer function for the inner roll loop is $P_\phi(s) = \frac{b_\phi}{s^2}$, which is a **double integrator**.

### System Type for Tracking
With a PD controller $C_\phi(s) = k_{p\phi} + k_{d\phi}s$, the loop transfer function is:
$$L(s) = C_\phi(s) P_\phi(s) = \frac{b_\phi(k_{p\phi} + k_{d\phi}s)}{s^2}$$
Because there are two integrators (two poles at $s=0$) in $L(s)$, the system is **Type 2 for tracking**.

### Steady-State Error ($e_{ss}$) for Tracking $\phi_r$
*   **Step ($1/s$):** $e_{ss} = 0$
*   **Ramp ($1/s^2$):** $e_{ss} = 0$
*   **Parabola ($1/s^3$):** $e_{ss} = \frac{1}{K_a} = \frac{1}{b_\phi k_{p\phi}}$ (Constant error)

### System Type for Input Disturbance
An input disturbance $d_i$ enters before the double integrator. The transfer function from $d_i$ to output $\phi$ is:
$$T_{\phi d_i}(s) = \frac{P(s)}{1+L(s)} = \frac{b_\phi}{s^2 + b_\phi(k_{d\phi}s + k_{p\phi})}$$
For a **step disturbance** ($d_i = 1/s$), the steady-state error is:
$$e_{ss} = \lim_{s\to 0} s \left( - \frac{b_\phi}{s^2 + b_\phi k_{d\phi}s + b_\phi k_{p\phi}} \cdot \frac{1}{s} \right) = -\frac{1}{k_{p\phi}}$$
Since a step disturbance results in a non-zero constant error, the system is **Type 0 with respect to input disturbances**. (Note: Adding an integrator in the controller would make it Type 1).

---

## 2. Outer-Loop Lateral Dynamics (Yaw, $\psi$)

Assuming the inner roll loop is tuned to be much faster than the outer yaw loop, its closed-loop transfer function is approximated as $1$. The plant for the outer loop is then $P_\psi(s) = \frac{a_\psi}{s^2}$ (another double integrator).

### System Type for Tracking
With a PD controller $C_\psi(s) = k_{p\psi} + k_{d\psi}s$, the loop transfer function is:
$$L(s) = \frac{a_\psi(k_{p\psi} + k_{d\psi}s)}{s^2}$$
This is **Type 2 for tracking**.

### Steady-State Error ($e_{ss}$) for Tracking $\psi_r$
*   **Step:** $e_{ss} = 0$
*   **Ramp:** $e_{ss} = 0$
*   **Parabola:** $e_{ss} = \frac{1}{a_\psi k_{p\psi}}$ (Constant error)

### System Type for Input Disturbance
Similar to the inner loop, it is **Type 0 with respect to input disturbances** (errors like wind or asymmetric drag will cause constant offsets in yaw if using only PD control).

---

## 3. Longitudinal Dynamics (Pitch, $\theta$)

The longitudinal plant is $P_\theta(s) = \frac{b_\theta}{s^2}$.

### System Type for Tracking
With a PD controller $C_\theta(s) = k_{p\theta} + k_{d\theta}s$, the loop transfer function is $L(s) = \frac{b_\theta(k_{p\theta} + k_{d\theta}s)}{s^2}$.
This is **Type 2 for tracking**.

### Steady-State Error ($e_{ss}$) for Tracking $\theta_r$
*   **Step:** $e_{ss} = 0$
*   **Ramp:** $e_{ss} = 0$
*   **Parabola:** $e_{ss} = \frac{1}{b_\theta k_{p\theta}}$ (Constant error)

### System Type for Input Disturbance
The system is **Type 0 with respect to input disturbances**. 
*In the Hummingbird, this is particularly noticeable: if the physical mass ($m$) is slightly different from the model used for gravity compensation ($F_e$), it acts like a step disturbance, creating a steady-state error in pitch that PD control cannot eliminate. This is the primary motivation for adding an integrator in Lab H.10.*
