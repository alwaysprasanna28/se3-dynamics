---
aliases:
  - Robot Jacobians on SE3
tags:
  - robotics
  - jacobian
  - differential-kinematics
  - statics
course: Robotic Manipulator Dynamics on SE(3)
module: 5
---

# Differential Kinematics and Jacobians

> [!summary]
> A manipulator Jacobian is the differential of forward kinematics after the tangent space of $SE(3)$ is identified with $\mathbb R^6$. It maps joint rates to a twist. Its transpose maps a wrench to generalized force because both mappings must preserve power.

## 1. From pose to velocity

Forward kinematics is

$$
T=T(q)\in SE(3).
$$

Differentiating $T$ directly produces a tangent matrix, not a six-vector. Right- or left-trivialization converts it into a body or space twist:

$$
V^b=(T^{-1}\dot T)^\vee,
\qquad
V^s=(\dot T T^{-1})^\vee.
$$

Because $\dot T$ depends linearly on $\dot q$,

$$
V^b=J_b(q)\dot q,
\qquad
V^s=J_s(q)\dot q.
$$

$J_b,J_s\in\mathbb R^{6\times n}$ are two coordinate representations of the differential of the same forward-kinematics map.

## 2. Space Jacobian from product of exponentials

Let

$$
T=e^{[S_1]^\wedge q_1}\cdots e^{[S_n]^\wedge q_n}M.
$$

Define the upstream transform

$$
T_{<i}=e^{[S_1]^\wedge q_1}\cdots e^{[S_{i-1}]^\wedge q_{i-1}}.
$$

The current $i$th screw expressed in space is

$$
J_{s,i}(q)=\operatorname{Ad}_{T_{<i}}S_i.
$$

Therefore

$$
J_s(q)=
\begin{bmatrix}
S_1&
\operatorname{Ad}_{e^{[S_1]^\wedge q_1}}S_2&
\cdots&
\operatorname{Ad}_{T_{<n}}S_n
\end{bmatrix}.
$$

The first column is constant because the first joint axis is fixed in the space frame. Column $i$ depends only on upstream joints.

### 2.1 Derivation sketch

Differentiate the PoE product once for each joint. For the $i$th term, use

$$
\frac{d}{dt}e^{[S_i]^\wedge q_i}
=[S_i]^\wedge\dot q_i e^{[S_i]^\wedge q_i},
$$

then multiply $\dot T$ by $T^{-1}$ on the right. All downstream factors cancel. Conjugating $[S_i]^\wedge$ by the upstream product produces $\operatorname{Ad}_{T_{<i}}S_i$.

## 3. Body Jacobian

For

$$
T=M e^{[B_1]^\wedge q_1}\cdots e^{[B_n]^\wedge q_n},
$$

the $i$th body-Jacobian column is transported through the inverse downstream motion:

$$
J_{b,i}(q)
=\operatorname{Ad}_{e^{-[B_n]^\wedge q_n}\cdots e^{-[B_{i+1}]^\wedge q_{i+1}}}B_i.
$$

Thus

$$
J_b=
\begin{bmatrix}
\operatorname{Ad}_{e^{-[B_n]q_n}\cdots e^{-[B_2]q_2}}B_1&
\cdots&
B_n
\end{bmatrix},
$$

where wedges on the exponent arguments are understood. The last column is constant because the last joint axis is fixed relative to the end-effector side in the body construction.

## 4. Relation between space and body Jacobians

Since $V^s=\operatorname{Ad}_T V^b$ for every $\dot q$,

$$
J_s(q)=\operatorname{Ad}_{T(q)}J_b(q).
$$

Equivalently,

$$
J_b(q)=\operatorname{Ad}_{T(q)^{-1}}J_s(q).
$$

This is a powerful consistency test for implementations.

## 5. Extracting an end-effector-origin velocity

A space twist is $V^s=[\omega^s;v^s]$, and a point at space position $p$ moves as

$$
\dot p=\omega^s\times p+v^s.
$$

Therefore the end-effector-origin translational Jacobian is

$$
J_p(q)=
\begin{bmatrix}
-[p(q)]_\times&I
\end{bmatrix}J_s(q).
$$

The angular-velocity Jacobian is simply the top three rows of $J_s$:

$$
J_\omega(q)=
\begin{bmatrix}I&0\end{bmatrix}J_s(q).
$$

For a point fixed to the end effector but offset by $r_e$,

$$
p_P=p+Rr_e,
$$

and its Jacobian can be computed either by differentiating this expression or by transporting the twist to a frame located at $P$.

## 6. Geometric versus analytic Jacobians

A **geometric Jacobian** maps joint rates to a twist:

$$
\begin{bmatrix}\omega\\v\end{bmatrix}=J_g(q)\dot q.
$$

An **analytic Jacobian** maps joint rates to derivatives of a chosen coordinate vector such as position plus Euler angles:

$$
\begin{bmatrix}\dot p\\\dot\alpha\end{bmatrix}=J_a(q)\dot q.
$$

If angular velocity and coordinate rate are related by $\omega=E(\alpha)\dot\alpha$, then $J_g$ and $J_a$ differ by a coordinate-dependent matrix. $J_a$ inherits the singularities of the orientation chart even when the physical mechanism is not singular.

> [!important]
> Use a geometric Jacobian for mechanics, wrenches, and coordinate-free control. Use an analytic Jacobian only when a specific output coordinate rate is truly required.

## 7. The planar 2R Jacobian

For the arm from [[04 - Manipulator Configuration and Forward Kinematics#8. Worked example: planar 2R arm]],

$$
p(q)=
\begin{bmatrix}
l_1c_1+l_2c_{12}\\
l_1s_1+l_2s_{12}
\end{bmatrix},
$$

where $c_1=\cos q_1$, $s_{12}=\sin(q_1+q_2)$, and so on. Direct differentiation gives

$$
J_p(q)=
\begin{bmatrix}
-l_1s_1-l_2s_{12}&-l_2s_{12}\\
l_1c_1+l_2c_{12}&l_2c_{12}
\end{bmatrix}.
$$

Its determinant is

$$
\det J_p=l_1l_2\sin q_2.
$$

The translational task is singular when $q_2=0$ or $q_2=\pi$: the links are collinear and the end effector instantaneously loses one planar motion direction.

## 8. Singularities

A configuration is singular for a chosen task when the corresponding Jacobian loses rank.

Consequences can include:

- some task twists become unattainable;
- inverse kinematics demands unbounded joint rates near the lost direction;
- some wrenches can be supported with little or no actuator torque;
- force and velocity resolution become strongly anisotropic;
- operational-space inertia formulas involving an inverse become ill-conditioned.

Singularity is task-dependent. A $6\times n$ end-effector Jacobian and a $3\times n$ position Jacobian need not lose rank at the same configurations.

### 8.1 Manipulability

For a full-row-rank Jacobian, unit joint-rate vectors map to the velocity ellipsoid

$$
V^T(JJ^T)^{-1}V=1.
$$

Yoshikawa's scalar manipulability measure is

$$
w(q)=\sqrt{\det(JJ^T)}.
$$

It vanishes at rank loss but depends on units and coordinate scaling. Mixing angular units and linear units requires a characteristic length or task-specific weighting.

## 9. Redundancy and nullspace motion

If $n$ exceeds the task dimension and $J$ has full row rank, solutions of

$$
V_d=J\dot q
$$

are

$$
\dot q=J^+V_d+(I-J^+J)z.
$$

The first term is the minimum-Euclidean-norm solution. The second lies in $\mathcal N(J)$ and can pursue secondary objectives such as joint-limit avoidance:

$$
z=-k\nabla_qH(q).
$$

This nullspace is kinematic. [[11 - Operational-Space Dynamics]] introduces a dynamically consistent nullspace that decouples accelerations and forces under the inertia metric.

## 10. Statics and the transpose Jacobian

Let an end-effector wrench $F$ act while joint velocity is $\dot q$. Since

$$
V=J\dot q,
$$

power at the task and joints must match:

$$
F^TV=\tau^T\dot q.
$$

Substitution gives

$$
F^TJ\dot q=\tau^T\dot q.
$$

For arbitrary $\dot q$,

$$
\tau=J^TF.
$$

This is the principle of virtual work. It also explains why the transpose appears without an inverse, even for redundant or singular robots.

### 10.1 Multiple external wrenches

If wrench $F_i$ acts on link $i$ and $J_i$ maps joint rates to the compatible twist coordinates at that link,

$$
\tau_{\mathrm{ext}}=\sum_iJ_i^TF_i.
$$

All wrench and twist frames must match within each term.

## 11. Weighted pseudoinverses

The solution minimizing $\frac12\dot q^TW\dot q$ subject to $J\dot q=V_d$, with $W\succ0$, is

$$
J_W^+=W^{-1}J^T(JW^{-1}J^T)^{-1}.
$$

Choosing $W=M(q)$ leads to the inertia-weighted inverse used in operational-space dynamics:

$$
\bar J=M^{-1}J^T(JM^{-1}J^T)^{-1}.
$$

## 12. Differential-kinematics implementation recipe

1. Decide whether the desired twist and error are body- or space-expressed.
2. Build the matching Jacobian.
3. Confirm $J_s=\operatorname{Ad}_T J_b$ numerically.
4. Verify columns by finite differences on the group:

$$
J_{b,i}\approx\frac{1}{\varepsilon}
\operatorname{Log}\left(T(q)^{-1}T(q+\varepsilon e_i)\right)^\vee.
$$

5. Inspect singular values, not only determinants.
6. Scale rotation and translation consistently for mixed tasks.

## Checkpoint exercises

1. Derive the space-Jacobian columns by differentiating the PoE formula.
2. Prove $J_s=\operatorname{Ad}_T J_b$.
3. Derive the 2R position Jacobian and identify its singular configurations.
4. Use virtual power to derive $\tau=J^TF$.
5. Show that $(I-J^+J)z$ lies in $\mathcal N(J)$.
6. Explain why a geometric Jacobian can be nonsingular while an Euler-angle analytic Jacobian is singular.

Solutions are in [[13 - Worked Problems and Solutions#Module 5 solutions]].

## Next

Continue to [[06 - Acceleration Kinematics and Inverse Kinematics]].
