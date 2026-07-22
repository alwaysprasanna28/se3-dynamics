---
aliases:
  - Rigid Body Dynamics on SE3
  - Spatial Rigid Body Dynamics
tags:
  - robotics
  - rigid-body-dynamics
  - spatial-inertia
  - se3
course: Robotic Manipulator Dynamics on SE(3)
module: 7
---

# Single Rigid Body Dynamics on $SE(3)$

> [!summary]
> A rigid body's configuration is $T\in SE(3)$, velocity is a twist $V\in\mathfrak{se}(3)$, momentum is a dual vector, and applied load is a wrench. A $6\times6$ spatial inertia unifies mass, center of mass, and rotational inertia.

## 1. From particle dynamics to a rigid body

A rigid body is a continuum of particles whose pairwise distances remain constant. Its motion is completely described by one pose

$$
T(t)=\begin{bmatrix}R(t)&p(t)\\0&1\end{bmatrix}\in SE(3).
$$

Its body kinematics are

$$
\dot T=T[V]^\wedge,
\qquad
V=(T^{-1}\dot T)^\vee.
$$

Write

$$
V=\begin{bmatrix}\omega\\v_O\end{bmatrix},
$$

where all coordinates are expressed in a body-fixed frame with origin $O$. $\omega$ is angular velocity and $v_O$ is the velocity of $O$, expressed in that body frame.

## 2. Mass distribution and rotational inertia

Let $C$ be the center of mass and $r$ the body-frame position of a mass element relative to $C$. The rotational inertia about $C$ is

$$
I_C=\int_{\mathcal B}
\left(\|r\|^2I-rr^T\right)\,dm.
$$

$I_C$ is symmetric. For a three-dimensional body with nondegenerate mass distribution, it is positive definite. Its eigenvectors are principal axes and eigenvalues are principal moments.

If the inertia about a parallel point $O$ is required, let $c=p_{OC}$ be the vector from $O$ to $C$, expressed in the body frame. The parallel-axis theorem gives

$$
I_O=I_C+m\left(\|c\|^2I-cc^T\right)
=I_C+m[c]_\times^T[c]_\times.
$$

## 3. Spatial inertia

The center-of-mass velocity is

$$
v_C=v_O+\omega\times c=v_O-[c]_\times\omega.
$$

The kinetic energy is

$$
K=\frac12\omega^TI_C\omega+\frac12m v_C^Tv_C.
$$

Expanding and collecting $V=[\omega;v_O]$ gives

$$
K=\frac12V^T\mathcal G_OV,
$$

where

$$
\mathcal G_O=
\begin{bmatrix}
I_O&m[c]_\times\\
-m[c]_\times&mI
\end{bmatrix}.
$$

This $6\times6$ symmetric matrix is the **spatial inertia** about $O$, expressed in the chosen frame. If $O=C$,

$$
\mathcal G_C=
\begin{bmatrix}
I_C&0\\
0&mI
\end{bmatrix}.
$$

> [!important] “About” and “expressed in” are different
> A spatial inertia must state both the point about which moments and velocities are taken and the frame in which its numeric entries are expressed.

## 4. Spatial momentum

Define the body-coordinate spatial momentum

$$
\mu=\mathcal G V
=\begin{bmatrix}h_O\\\ell\end{bmatrix},
$$

where $h_O$ is angular momentum about $O$ and $\ell$ is linear momentum. Momentum is dual to velocity: it pairs with a twist to form an energy-like scalar.

The spatial inertia is therefore a metric-like map

$$
\mathcal G:\mathfrak{se}(3)\rightarrow\mathfrak{se}(3)^*.
$$

It converts a twist into momentum.

## 5. Newton–Euler equation in body coordinates

Let the net applied body-coordinate wrench be

$$
F=\begin{bmatrix}m_O\\f\end{bmatrix}.
$$

The compact rigid-body equation is

$$
F=\mathcal G\dot V-\operatorname{ad}_V^T\mathcal G V.
$$

Equivalently, with $\mu=\mathcal GV$,

$$
\dot\mu-\operatorname{ad}_V^T\mu=F.
$$

This is the Euler–Poincaré or body-coordinate Newton–Euler equation. The second term accounts for differentiating momentum in a rotating and translating body frame.

### 5.1 Expansion at the center of mass

At $O=C$,

$$
\mathcal G=
\begin{bmatrix}I_C&0\\0&mI\end{bmatrix}.
$$

Using

$$
\operatorname{ad}_V=
\begin{bmatrix}
[\omega]_\times&0\\
[v]_\times&[\omega]_\times
\end{bmatrix},
$$

the angular equation becomes

$$
m_C=I_C\dot\omega+\omega\times(I_C\omega),
$$

and the linear equation becomes

$$
f=m(\dot v_C+\omega\times v_C).
$$

$\dot v_C$ is the coordinate derivative in the rotating body frame, while $\dot v_C+\omega\times v_C$ is the physical acceleration expressed in that frame.

## 6. Why the coadjoint term has that sign

For a moving body frame, the derivative of a physical vector differs from the derivative of its body coordinates. The force-space cross operator is

$$
\operatorname{ad}_V^*=-\operatorname{ad}_V^T.
$$

Therefore the convective momentum term is

$$
\operatorname{ad}_V^*\mu=-\operatorname{ad}_V^T\mu.
$$

The dynamics can be written

$$
F=\mathcal G\dot V+\operatorname{ad}_V^*\mathcal GV.
$$

This notation makes the dual action explicit.

## 7. Energy and power check

For a constant body spatial inertia,

$$
K=\frac12V^T\mathcal GV,
\qquad
\dot K=V^T\mathcal G\dot V.
$$

Premultiply the dynamics by $V^T$:

$$
V^TF=V^T\mathcal G\dot V-V^T\operatorname{ad}_V^T\mathcal GV.
$$

But

$$
V^T\operatorname{ad}_V^T\mathcal GV
=(\operatorname{ad}_VV)^T\mathcal GV=0,
$$

because $\operatorname{ad}_VV=[V,V]=0$. Hence

$$
\dot K=F^TV.
$$

The geometric inertial term changes the direction of momentum but performs no net work.

## 8. Transforming spatial inertia

Suppose

$$
V_a=\operatorname{Ad}_{T_{ab}}V_b.
$$

Kinetic energy must be invariant:

$$
\frac12V_a^T\mathcal G_aV_a
=\frac12V_b^T\mathcal G_bV_b.
$$

Therefore

$$
\mathcal G_a
=\operatorname{Ad}_{T_{ab}}^{-T}
\mathcal G_b
\operatorname{Ad}_{T_{ab}}^{-1}.
$$

Momentum and wrench transform with the same dual rule:

$$
\mu_a=\operatorname{Ad}_{T_{ab}}^{-T}\mu_b,
\qquad
F_a=\operatorname{Ad}_{T_{ab}}^{-T}F_b.
$$

This congruence transformation simultaneously rotates the inertia and shifts its reference point.

## 9. Gravity as a wrench

Let space gravity acceleration be $g_0$. The force at the center of mass is $mg_0$. Expressed in body coordinates,

$$
f_g^b=mR^Tg_0.
$$

If the body frame origin is $O$ and $c=p_{OC}^b$, the gravity wrench about $O$ is

$$
F_g^b=
\begin{bmatrix}
c\times f_g^b\\
f_g^b
\end{bmatrix}.
$$

If the frame origin is the center of mass, the gravity moment is zero.

## 10. Free rigid-body equations

With no external wrench,

$$
\mathcal G\dot V-\operatorname{ad}_V^T\mathcal GV=0.
$$

At the center of mass,

$$
m\left(\dot v+\omega\times v\right)=0,
$$

$$
I\dot\omega+\omega\times I\omega=0.
$$

In inertial coordinates, linear momentum and angular momentum about the center of mass are conserved. In body coordinates, their numeric components evolve because the basis rotates.

### 10.1 Rotation about a principal axis

If $\omega$ is aligned with an eigenvector of $I$, then $I\omega=\lambda\omega$ and

$$
\omega\times I\omega=0.
$$

Thus $\dot\omega=0$ in torque-free motion. Rotation about the largest and smallest principal-moment axes is stable; rotation about the intermediate axis is unstable, a result known as the tennis-racket theorem.

## 11. Forced dynamics and pose integration

Given $T,V,F$, compute

$$
\dot V=\mathcal G^{-1}
\left(F+\operatorname{ad}_V^T\mathcal GV\right),
$$

then integrate

$$
\dot T=T[V]^\wedge.
$$

A first-order group update is

$$
V_{k+1}=V_k+h\dot V_k,
$$

$$
T_{k+1}=T_k\exp(h[V_k]^\wedge).
$$

Higher-order Lie-group integrators improve long-horizon energy and momentum behavior.

## 12. Variational viewpoint

The rigid-body Lagrangian in body variables is

$$
L(T,V)=\frac12V^T\mathcal GV-U(T).
$$

Variations of $T$ are constrained to the tangent space:

$$
\delta T=T[\eta]^\wedge.
$$

The induced variation in body twist is

$$
\delta V=\dot\eta+\operatorname{ad}_V\eta.
$$

Substitution into Hamilton's principle and integration by parts yields the Euler–Poincaré equation. This explains why the $\operatorname{ad}^*$ term is a geometric necessity, not a modeling correction.

## Checkpoint exercises

1. Derive the spatial inertia at an offset origin from the kinetic energy.
2. Expand the compact Newton–Euler equation at the center of mass.
3. Prove the coadjoint inertial term performs zero power.
4. Derive the spatial-inertia coordinate transformation from energy invariance.
5. Write the body gravity wrench about a frame origin offset from the center of mass.
6. Explain how momentum can be constant geometrically while its body coordinates change.

Solutions are in [[13 - Worked Problems and Solutions#Module 7 solutions]].

## Next

Continue to [[07A - From Particles to Constrained Mechanisms]].
