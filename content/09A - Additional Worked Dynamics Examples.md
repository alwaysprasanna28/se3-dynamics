---
aliases:
  - Manipulator Dynamics Examples
  - RP Robot Dynamics
tags:
  - robotics
  - dynamics
  - worked-example
  - lagrangian-dynamics
course: Robotic Manipulator Dynamics on SE(3)
module: 9A
---

# Additional Worked Dynamics Examples

> [!summary]
> These examples isolate the origins of inertia, gravity, Coriolis, and centrifugal terms. The revolute–prismatic polar robot is especially useful because each velocity-product term has a direct physical interpretation.

## 1. A vertical prismatic joint

Let a mass $m$ move vertically with coordinate $q$, positive upward. Applied actuator force $u$ is positive upward.

Velocity and kinetic energy are

$$
v=\dot q,
\qquad
K=\frac12m\dot q^2.
$$

Potential energy is

$$
U=mgq.
$$

Euler–Lagrange gives

$$
\boxed{m\ddot q+mg=u}.
$$

Therefore

$$
M=m,
\qquad
c=0,
\qquad
g(q)=mg.
$$

Generalized effort has units of newtons because $q$ has units of meters.

### 1.1 Static and free-fall checks

At rest, holding force is $u=mg$. With $u=0$,

$$
\ddot q=-g.
$$

These two tests catch gravity-sign mistakes immediately.

## 2. A rigid single-link pendulum

Let $q$ be measured from horizontal, $r$ be the center-of-mass distance, $m$ the mass, and $I_C$ the center-of-mass rotational inertia.

The center-of-mass speed is $r\dot q$, so

$$
K=\frac12mr^2\dot q^2+\frac12I_C\dot q^2
=\frac12(I_C+mr^2)\dot q^2.
$$

The potential is

$$
U=mgr\sin q.
$$

Therefore

$$
\boxed{
(I_C+mr^2)\ddot q+mgr\cos q=\tau
}.
$$

There is no velocity bias because the scalar inertia is constant. The physical point mass still has centripetal acceleration, but its corresponding force is radial and handled by the joint constraint reaction rather than the allowed rotation coordinate.

### 2.1 Uniform rod

For a uniform slender rod of length $l$ rotating about one end,

$$
r=\frac l2,
\qquad
I_C=\frac1{12}ml^2.
$$

Thus

$$
I_O=I_C+mr^2=\frac13ml^2,
$$

and

$$
\frac13ml^2\ddot q+\frac12mgl\cos q=\tau.
$$

## 3. A Cartesian 2P point-mass robot

Let two orthogonal prismatic joints directly set a point mass position:

$$
p(q)=\begin{bmatrix}q_1\\q_2\end{bmatrix}.
$$

Then

$$
J=I,
\qquad
K=\frac12m(\dot q_1^2+\dot q_2^2).
$$

Therefore

$$
M=mI,
\qquad
c=0.
$$

If $q_2$ is vertical upward,

$$
g=\begin{bmatrix}0\\mg\end{bmatrix}.
$$

This is the simplest case: generalized coordinates are Cartesian coordinates, so constant joint velocity is constant Cartesian velocity.

## 4. Revolute–prismatic polar robot

Consider a point mass $m$ moving in a vertical plane. The revolute coordinate is $\theta$ from the $+x$ axis and the prismatic coordinate is radial distance $r>0$:

$$
q=\begin{bmatrix}\theta\\r\end{bmatrix}.
$$

The position is

$$
p(q)=
\begin{bmatrix}
r\cos\theta\\
r\sin\theta
\end{bmatrix}.
$$

### 4.1 Velocity and Jacobian

Differentiate:

$$
\dot p=
\begin{bmatrix}
-r\sin\theta\,\dot\theta+\cos\theta\,\dot r\\
r\cos\theta\,\dot\theta+\sin\theta\,\dot r
\end{bmatrix}.
$$

Thus

$$
J(q)=
\begin{bmatrix}
-r\sin\theta&\cos\theta\\
r\cos\theta&\sin\theta
\end{bmatrix}.
$$

The squared speed simplifies to

$$
\|\dot p\|^2=r^2\dot\theta^2+\dot r^2.
$$

The radial and tangential velocity directions are orthogonal, so the cross term vanishes.

### 4.2 Kinetic and potential energy

$$
K=\frac12m\left(r^2\dot\theta^2+\dot r^2\right),
$$

$$
U=mg r\sin\theta.
$$

Therefore

$$
M(q)=
\begin{bmatrix}
mr^2&0\\
0&m
\end{bmatrix},
$$

and

$$
g(q)=
\begin{bmatrix}
mgr\cos\theta\\
mg\sin\theta
\end{bmatrix}.
$$

### 4.3 Euler–Lagrange equation for $\theta$

$$
\frac{\partial K}{\partial\dot\theta}=mr^2\dot\theta.
$$

Differentiate:

$$
\frac{d}{dt}\frac{\partial K}{\partial\dot\theta}
=mr^2\ddot\theta+2mr\dot r\dot\theta.
$$

Since $\partial K/\partial\theta=0$ and

$$
\frac{\partial U}{\partial\theta}=mgr\cos\theta,
$$

the angular equation is

$$
\boxed{
mr^2\ddot\theta
+2mr\dot r\dot\theta
+mgr\cos\theta
=\tau_\theta
}.
$$

The term $2mr\dot r\dot\theta$ is Coriolis-like. If the mass slides outward while rotating, torque is needed to maintain angular rate. With zero external torque and gravity, angular momentum $mr^2\dot\theta$ remains constant.

### 4.4 Euler–Lagrange equation for $r$

$$
\frac{\partial K}{\partial\dot r}=m\dot r,
\qquad
\frac{d}{dt}\frac{\partial K}{\partial\dot r}=m\ddot r.
$$

Also,

$$
\frac{\partial K}{\partial r}=mr\dot\theta^2,
\qquad
\frac{\partial U}{\partial r}=mg\sin\theta.
$$

Thus

$$
\boxed{
m\ddot r
-mr\dot\theta^2
+mg\sin\theta
=F_r
}.
$$

The term $-mr\dot\theta^2$ appears on the left. Equivalently, $+mr\dot\theta^2$ is the outward actuator force needed to hold $r$ fixed while rotating.

### 4.5 Manipulator form

Collecting both equations:

$$
M(q)\ddot q+c(q,\dot q)+g(q)=u,
$$

where

$$
c(q,\dot q)=
\begin{bmatrix}
2mr\dot r\dot\theta\\
-mr\dot\theta^2
\end{bmatrix},
$$

and

$$
u=\begin{bmatrix}\tau_\theta\\F_r\end{bmatrix}.
$$

One valid Christoffel-compatible choice is

$$
\boxed{
C(q,\dot q)=
\begin{bmatrix}
mr\dot r&mr\dot\theta\\
-mr\dot\theta&0
\end{bmatrix}
},
$$

because

$$
C\dot q
=\begin{bmatrix}
2mr\dot r\dot\theta\\
-mr\dot\theta^2
\end{bmatrix}.
$$

The corrected $C$ also satisfies the skew property.

### 4.6 Energy check

With $u=0$, compute

$$
E=\frac12m(r^2\dot\theta^2+\dot r^2)+mgr\sin\theta.
$$

Substitution of the two equations gives

$$
\dot E=0.
$$

The Coriolis and centrifugal terms exchange energy between radial and angular motion but do not create or destroy it.

## 5. Payload added to an arbitrary tool

Suppose a payload has body spatial inertia $\mathcal G_p$ and is rigidly attached to the tool. Let $J_p(q)$ be the compatible payload body Jacobian. Its mass-matrix contribution is

$$
\boxed{\Delta M(q)=J_p(q)^T\mathcal G_pJ_p(q)}.
$$

Its gravity contribution follows from its center-of-mass potential or gravity wrench. Its velocity-bias contribution is

$$
\Delta c
=J_p^T\left(
\mathcal G_p\dot J_p\dot q
-\operatorname{ad}_{V_p}^T\mathcal G_pV_p
\right).
$$

A payload changes more than gravity. It also changes inertia, coupling, velocity bias, operational-space inertia, and collision effective mass.

## 6. Locked-joint reduction

Suppose some joints are mechanically locked. Partition

$$
q=\begin{bmatrix}q_f\\q_l\end{bmatrix},
\qquad
\dot q_l=\ddot q_l=0.
$$

The free-joint dynamics use the corresponding rows:

$$
M_{ff}\ddot q_f+c_f+g_f=\tau_f+\tau_{\mathrm{ext},f}.
$$

But locked joints generally experience reaction torque

$$
\tau_l=M_{lf}\ddot q_f+c_l+g_l-\tau_{\mathrm{ext},l}.
$$

Deleting locked coordinates changes the reduced model, while their reaction forces remain physically important for structure and hardware loads.

## 7. Comparison table

| System | $M(q)$ | $c(q,\dot q)$ | Key lesson |
|---|---|---|---|
| Vertical prismatic mass | constant scalar | zero | gravity sign and force units |
| Rigid pendulum | constant scalar | zero | constraint supplies radial force |
| Cartesian 2P mass | constant diagonal | zero | Cartesian coordinates flatten geometry |
| Polar RP mass | configuration dependent | nonzero | changing coordinate basis creates velocity bias |
| Planar 2R arm | coupled and configuration dependent | nonzero | moving link inertias couple joints |

## Checkpoint exercises

1. Derive the vertical prismatic equation using Newton's law and Lagrange's equation.
2. Explain why the pendulum has centripetal acceleration but $c=0$.
3. Derive the polar robot Jacobian and verify $M=mJ^TJ$.
4. Derive both polar equations without using memorized Christoffel formulas.
5. Verify the displayed valid $C$ satisfies both $C\dot q=c$ and skew symmetry.
6. Derive the effect of a point payload on the planar 2R mass matrix.

Solutions are in [[13 - Worked Problems and Solutions#Module 9A solutions]].

## Next

Continue to [[10 - Recursive Rigid-Body Dynamics Algorithms]].
