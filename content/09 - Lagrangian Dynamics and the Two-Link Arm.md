---
aliases:
  - Two-Link Robot Dynamics
  - Lagrangian Manipulator Dynamics
tags:
  - robotics
  - lagrangian-dynamics
  - two-link-arm
  - worked-example
course: Robotic Manipulator Dynamics on SE(3)
module: 9
---

# Lagrangian Dynamics and the Two-Link Arm

> [!summary]
> This note derives the full $2$R planar-arm dynamics from positions and energies. It makes every term in $M(q)\ddot q+C(q,\dot q)\dot q+g(q)=\tau$ concrete.

## 1. Model and assumptions

Consider two rigid links moving in the vertical $xy$ plane.

- $q_1$ is link 1's angle from the $+x$ axis.
- $q_2$ is link 2's angle relative to link 1.
- $l_1$ is the distance from joint 1 to joint 2.
- $r_1,r_2$ are distances from each proximal joint to that link's center of mass.
- $m_1,m_2$ are link masses.
- $I_1,I_2$ are planar rotational inertias about each center of mass.
- $y$ points upward and gravity acceleration is $g_0=[0,-g]^T$ with $g>0$.
- Joint friction is omitted initially.

Use shorthand

$$
c_1=\cos q_1,\quad s_1=\sin q_1,
\quad c_{12}=\cos(q_1+q_2),\quad s_{12}=\sin(q_1+q_2).
$$

## 2. Center-of-mass kinematics

The first center of mass is

$$
p_{C_1}=
\begin{bmatrix}
r_1c_1\\
r_1s_1
\end{bmatrix}.
$$

The second is

$$
p_{C_2}=
\begin{bmatrix}
l_1c_1+r_2c_{12}\\
l_1s_1+r_2s_{12}
\end{bmatrix}.
$$

Differentiate:

$$
\dot p_{C_1}=
\begin{bmatrix}
-r_1s_1\dot q_1\\
r_1c_1\dot q_1
\end{bmatrix},
$$

$$
\dot p_{C_2}=
\begin{bmatrix}
-l_1s_1\dot q_1-r_2s_{12}(\dot q_1+\dot q_2)\\
l_1c_1\dot q_1+r_2c_{12}(\dot q_1+\dot q_2)
\end{bmatrix}.
$$

Their squared speeds are

$$
\|\dot p_{C_1}\|^2=r_1^2\dot q_1^2,
$$

$$
\|\dot p_{C_2}\|^2
=l_1^2\dot q_1^2
+r_2^2(\dot q_1+\dot q_2)^2
+2l_1r_2\cos q_2\,\dot q_1(\dot q_1+\dot q_2).
$$

The angular velocities are

$$
\omega_1=\dot q_1,
\qquad
\omega_2=\dot q_1+\dot q_2.
$$

## 3. Kinetic energy

Link 1 contributes

$$
K_1=\frac12m_1r_1^2\dot q_1^2
+\frac12I_1\dot q_1^2.
$$

Link 2 contributes

$$
K_2=\frac12m_2\|\dot p_{C_2}\|^2
+\frac12I_2(\dot q_1+\dot q_2)^2.
$$

Collecting terms gives

$$
K=\frac12\dot q^TM(q)\dot q,
$$

with

$$
M(q)=
\begin{bmatrix}
M_{11}&M_{12}\\
M_{12}&M_{22}
\end{bmatrix},
$$

$$
M_{11}=I_1+I_2+m_1r_1^2
+m_2\left(l_1^2+r_2^2+2l_1r_2\cos q_2\right),
$$

$$
M_{12}=I_2+m_2\left(r_2^2+l_1r_2\cos q_2\right),
$$

$$
M_{22}=I_2+m_2r_2^2.
$$

### 3.1 Interpretation

- $M_{11}$ includes both links because joint 1 moves both.
- $M_{22}$ contains only link 2 because joint 2 does not move link 1.
- $\cos q_2$ changes coupling with elbow configuration.
- The matrix is symmetric because both cross-energy terms share one coefficient.

## 4. Potential energy and gravity

Since $U=mgy$,

$$
U(q)=m_1gr_1\sin q_1
+m_2g\left(l_1\sin q_1+r_2\sin(q_1+q_2)\right).
$$

Therefore

$$
g(q)=\frac{\partial U}{\partial q}
=
\begin{bmatrix}
(m_1r_1+m_2l_1)g\cos q_1+m_2r_2g\cos(q_1+q_2)\\
m_2r_2g\cos(q_1+q_2)
\end{bmatrix}.
$$

At $q_1=q_2=0$, both links are horizontal and gravity torque magnitude is largest. When both centers of mass lie vertically above or below their joints, the corresponding cosine terms vanish.

## 5. Velocity-dependent terms

Define

$$
b=m_2l_1r_2.
$$

Only the $\cos q_2$ terms of $M$ depend on configuration. Applying Euler–Lagrange equations gives

$$
c(q,\dot q)=
\begin{bmatrix}
-b\sin q_2\left(2\dot q_1\dot q_2+\dot q_2^2\right)\\
b\sin q_2\dot q_1^2
\end{bmatrix}.
$$

One valid $C$ satisfying $c=C\dot q$ is

$$
C(q,\dot q)=
\begin{bmatrix}
-b\sin q_2\dot q_2&-b\sin q_2(\dot q_1+\dot q_2)\\
b\sin q_2\dot q_1&0
\end{bmatrix}.
$$

This choice also makes $\dot M-2C$ skew-symmetric.

### 5.1 Why these terms vanish in special cases

- If $q_2=0$ or $\pi$, then $\sin q_2=0$: local change of inertial coupling with elbow angle vanishes.
- If $\dot q=0$, all Coriolis and centrifugal terms vanish.
- If $m_2=0$, link-2 coupling vanishes.
- If $l_1=0$ or $r_2=0$, the offset responsible for coupling vanishes.

## 6. Complete equations of motion

The result is

$$
\boxed{
M(q)\ddot q+c(q,\dot q)+g(q)=\tau
}.
$$

With viscous friction $B\dot q$ and an external end-effector planar force $f_e$,

$$
M\ddot q+c+g+B\dot q
=\tau+J_p(q)^Tf_e.
$$

If a planar end-effector moment $m_e$ is also applied, use the planar twist/wrench Jacobian and wrench $[m_e,f_x,f_y]^T$.

## 7. Euler–Lagrange derivation in detail

For coordinate $q_i$,

$$
\frac{\partial K}{\partial\dot q_i}
=\sum_jM_{ij}\dot q_j.
$$

Differentiate:

$$
\frac{d}{dt}\frac{\partial K}{\partial\dot q_i}
=\sum_jM_{ij}\ddot q_j
+\sum_{j,k}\frac{\partial M_{ij}}{\partial q_k}\dot q_k\dot q_j.
$$

Also,

$$
\frac{\partial K}{\partial q_i}
=\frac12\sum_{j,k}
\frac{\partial M_{jk}}{\partial q_i}\dot q_j\dot q_k.
$$

Thus the kinetic part is

$$
\sum_jM_{ij}\ddot q_j
+\sum_{j,k}\frac12
\left(
\frac{\partial M_{ij}}{\partial q_k}
+\frac{\partial M_{ik}}{\partial q_j}
-\frac{\partial M_{jk}}{\partial q_i}
\right)
\dot q_j\dot q_k.
$$

The coefficient in parentheses is twice the Christoffel symbol. Adding $\partial U/\partial q_i$ yields the manipulator equation.

## 8. Energy verification

Total mechanical energy is

$$
E=\frac12\dot q^TM(q)\dot q+U(q).
$$

Differentiate:

$$
\dot E
=\dot q^TM\ddot q
+\frac12\dot q^T\dot M\dot q
+g^T\dot q.
$$

Using $M\ddot q+C\dot q+g=\tau$,

$$
\dot E
=\dot q^T\tau
+\frac12\dot q^T(\dot M-2C)\dot q.
$$

The last term is zero because $\dot M-2C$ is skew-symmetric. Hence

$$
\boxed{\dot E=\tau^T\dot q}.
$$

With viscous friction and no external load,

$$
\dot E=\tau^T\dot q-\dot q^TB\dot q.
$$

## 9. Static torques

At equilibrium, $\dot q=0$ and $\ddot q=0$, so

$$
\tau=g(q)-J_p(q)^Tf_e.
$$

Without an external force,

$$
\tau=g(q).
$$

This is gravity compensation. It balances gravity but does not create restoring stiffness; with ideal compensation, any stationary configuration is an equilibrium.

## 10. Forward and inverse dynamics examples

### 10.1 Inverse dynamics

Given $q,\dot q,\ddot q_d$, command

$$
\tau_d=M(q)\ddot q_d+c(q,\dot q)+g(q).
$$

### 10.2 Forward dynamics

Given $q,\dot q,\tau$, solve

$$
M(q)\ddot q=\tau-c(q,\dot q)-g(q).
$$

Do not compute $M^{-1}$ explicitly in numerical code. Use a Cholesky factorization because $M\succ0$.

## 11. Connection back to $SE(3)$

The planar derivation used center-of-mass positions and scalar angular velocities. The $SE(3)$ derivation gives exactly the same result by assigning each link a body pose, body Jacobian, and spatial inertia:

$$
M(q)=\sum_{i=1}^2(J_i^b)^T\mathcal G_i^bJ_i^b.
$$

The planar algebra is shorter only because all angular velocities are parallel to $z$. In a general spatial arm, the $SE(3)$ formulation scales without inventing separate case-specific equations.

## Checkpoint exercises

1. Re-derive $\|\dot p_{C_2}\|^2$ and identify the source of $\cos q_2$.
2. Derive every entry of $M(q)$ from kinetic energy.
3. Derive $g(q)$ from potential energy and check the horizontal-arm case.
4. Verify $C\dot q=c$ for the stated $C$.
5. Verify $\dot M-2C$ is skew-symmetric.
6. Add an end-effector payload point mass $m_p$ and derive its addition to $M$ and $g$.

Solutions are in [[13 - Worked Problems and Solutions#Module 9 solutions]].

## Next

Continue to [[09A - Additional Worked Dynamics Examples]].
