---
aliases:
  - SE3 Manipulator Dynamics
  - Manipulator Dynamics from Spatial Inertia
tags:
  - robotics
  - manipulator-dynamics
  - spatial-inertia
  - se3
course: Robotic Manipulator Dynamics on SE(3)
module: 8
---

# Manipulator Dynamics from Link Geometry

> [!summary]
> Each link contributes kinetic energy $\frac12V_i^T\mathcal G_iV_i$. Since $V_i=J_i(q)\dot q$, pulling these energies back to joint space gives the robot mass matrix. Forces pull back by $J_i^T$. This is the clean bridge from $SE(3)$ link mechanics to joint-space manipulator dynamics.

## 1. Kinematic setup

For a fixed-base $n$-joint robot, let

$$
q\in\mathcal Q,
\qquad
\dot q\in T_q\mathcal Q.
$$

For link $i$, choose a body-fixed frame and compute

$$
T_{0i}(q)\in SE(3).
$$

Its body twist is

$$
V_i^b=J_i^b(q)\dot q.
$$

Let $\mathcal G_i^b$ be its constant body spatial inertia about the same body-frame origin. Consistency of frame and point is essential.

## 2. Kinetic energy and the joint-space mass matrix

Link $i$ has kinetic energy

$$
K_i=\frac12(V_i^b)^T\mathcal G_i^bV_i^b.
$$

Substitute $V_i^b=J_i^b\dot q$:

$$
K_i=\frac12\dot q^T(J_i^b)^T\mathcal G_i^bJ_i^b\dot q.
$$

Sum over links:

$$
K=\sum_iK_i
=\frac12\dot q^TM(q)\dot q,
$$

with

$$
\boxed{
M(q)=\sum_i(J_i^b(q))^T\mathcal G_i^bJ_i^b(q)
}.
$$

The same result arises using space Jacobians and space-expressed inertias:

$$
M(q)=\sum_i(J_i^s)^T\mathcal G_i^sJ_i^s.
$$

The two forms agree because Jacobians and inertias transform dually.

### 2.1 Why $M(q)$ is symmetric

Each spatial inertia is symmetric, so

$$
\left(J_i^T\mathcal G_iJ_i\right)^T
=J_i^T\mathcal G_iJ_i.
$$

Therefore $M=M^T$.

### 2.2 Why $M(q)$ is positive definite

For nonzero joint velocity,

$$
\dot q^TM\dot q
=\sum_i(J_i\dot q)^T\mathcal G_i(J_i\dot q).
$$

Every term is nonnegative. For independent coordinates in a physical mechanism, a nonzero $\dot q$ moves at least one massive link, making the sum positive. Thus $M(q)\succ0$ away from degenerate parameterizations.

### 2.3 Physical meaning of off-diagonal entries

$M_{ij}$ measures kinetic coupling between joint velocities $\dot q_i$ and $\dot q_j$. If $M_{ij}\ne0$, simultaneous motion of those joints changes kinetic energy through a cross term. The mass matrix is a configuration-dependent metric on joint velocity space.

## 3. Potential energy and gravity

Let $p_{C_i}(q)$ be the space position of link $i$'s center of mass and let $g_0$ be the gravity acceleration vector. Define

$$
U(q)=-\sum_i m_i g_0^Tp_{C_i}(q).
$$

The gravity term conventionally placed on the left is

$$
\boxed{g(q)=\frac{\partial U}{\partial q}}.
$$

The actual generalized load applied by gravity is $-g(q)$. Equivalently, sum center-of-mass force Jacobians:

$$
-g(q)=\sum_iJ_{v,C_i}(q)^T m_i g_0.
$$

This is another virtual-work pullback.

## 4. External wrenches

If compatible wrench $F_i$ acts on link $i$, its joint generalized force is

$$
\tau_{\mathrm{ext},i}=J_i^TF_i.
$$

For multiple loads,

$$
\tau_{\mathrm{ext}}=\sum_iJ_i^TF_i.
$$

The complete virtual-power identity is

$$
\tau_{\mathrm{ext}}^T\dot q
=\sum_iF_i^TV_i.
$$

Be explicit about sign: some software reports the wrench exerted **by the environment on the robot**, while other interfaces report the wrench exerted **by the robot on the environment**.

## 5. Euler–Lagrange derivation of the manipulator equation

Let

$$
L(q,\dot q)=\frac12\dot q^TM(q)\dot q-U(q).
$$

Apply

$$
\frac{d}{dt}\frac{\partial L}{\partial\dot q_i}
-\frac{\partial L}{\partial q_i}
=\tau_i+\tau_{\mathrm{ext},i}-\tau_{f,i}.
$$

The result is

$$
\boxed{
M(q)\ddot q+C(q,\dot q)\dot q+g(q)+\tau_f(q,\dot q)
=\tau+\tau_{\mathrm{ext}}
}.
$$

Often the velocity-dependent term is stored as one vector

$$
c(q,\dot q)=C(q,\dot q)\dot q,
$$

because $c$ is unique for a model while the matrix $C$ is not.

## 6. Coriolis and centrifugal terms

One standard choice uses Christoffel symbols:

$$
C_{ij}(q,\dot q)
=\sum_k\Gamma_{ijk}(q)\dot q_k,
$$

where

$$
\Gamma_{ijk}
=\frac12\left(
\frac{\partial M_{ij}}{\partial q_k}
+\frac{\partial M_{ik}}{\partial q_j}
-\frac{\partial M_{jk}}{\partial q_i}
\right).
$$

Then

$$
c_i(q,\dot q)
=\sum_{j,k}\Gamma_{ijk}\dot q_j\dot q_k.
$$

These terms are quadratic in joint velocity. They arise because the joint-space inertia metric varies with configuration and because link frames and joint axes move.

### 6.1 Geometric interpretation

With no potential or applied forces,

$$
M(q)\ddot q+C(q,\dot q)\dot q=0.
$$

This is the geodesic equation for the kinetic-energy metric $M(q)$. Free mechanism motion follows the “straightest possible” curves under that metric. Christoffel symbols are the coordinate representation of the corresponding connection.

## 7. Structural energy property

For the Christoffel-compatible choice of $C$,

$$
\dot M-2C
$$

is skew-symmetric. Therefore

$$
x^T(\dot M-2C)x=0
$$

for every $x$. Taking $x=\dot q$ gives

$$
\frac{d}{dt}\left(\frac12\dot q^TM\dot q\right)
=\dot q^T\left(M\ddot q+C\dot q\right).
$$

Using the manipulator equation without friction,

$$
\frac{d}{dt}(K+U)
=\dot q^T\tau+\dot q^T\tau_{\mathrm{ext}}.
$$

This identity underlies passivity and many robot-control proofs.

## 8. Bias term and inverse dynamics

Define

$$
h(q,\dot q)=c(q,\dot q)+g(q)+\tau_f(q,\dot q).
$$

Then

$$
M(q)\ddot q+h(q,\dot q)
=\tau+\tau_{\mathrm{ext}}.
$$

Inverse dynamics computes

$$
\tau=M(q)\ddot q+h(q,\dot q)-\tau_{\mathrm{ext}}.
$$

Forward dynamics computes

$$
\ddot q=M(q)^{-1}
\left(\tau+\tau_{\mathrm{ext}}-h(q,\dot q)\right).
$$

Numerically, solve a linear system rather than explicitly constructing $M^{-1}$.

## 9. Connection to link Newton–Euler equations

The Lagrangian derivation compresses all link mechanics into $M,c,g$. A Newton–Euler derivation instead:

1. propagates link twists and accelerations outward along the tree;
2. applies each link equation

$$
F_i=\mathcal G_i\dot V_i-\operatorname{ad}_{V_i}^T\mathcal G_iV_i;
$$

3. propagates required wrenches inward;
4. projects each joint wrench onto its motion subspace.

Both methods produce the same generalized dynamics. The recursive Newton–Euler algorithm does so in $O(n)$ time for a tree.

## 10. Floating-base systems

For a floating-base robot,

$$
\mathcal Q=SE(3)\times\mathcal Q_j.
$$

Use generalized velocity

$$
\nu=\begin{bmatrix}V_B\\\dot q_j\end{bmatrix},
$$

whose dimension may differ from the number of stored configuration coordinates if the base orientation uses a quaternion. Kinematics have the form

$$
\dot q=N(q)\nu.
$$

Dynamics are

$$
M(q)\dot\nu+h(q,\nu)=S^T\tau+J_c^TF_c,
$$

where $S$ selects actuated joint directions. The six floating-base directions are unactuated; contact wrenches are essential for locomotion.

## 11. Friction, motors, and reflected inertia

A practical model may include

$$
\tau_f=B_v\dot q+\tau_c\operatorname{sgn}(\dot q)+\tau_{\mathrm{stiction}}.
$$

Gear trains reflect motor inertia approximately as $N_i^2I_{m,i}$ into the joint coordinate for gear ratio $N_i$. Flexible transmissions, backlash, and torque-controller dynamics require additional states and cannot always be absorbed into a rigid manipulator equation.

## 12. Linearity in inertial parameters

Although robot dynamics are nonlinear in $q$ and $\dot q$, they are linear in a suitable vector of inertial parameters $\pi$:

$$
\tau=Y(q,\dot q,\ddot q)\pi.
$$

$Y$ is the regressor. This property enables least-squares identification. Only combinations in the base-parameter subspace are identifiable for a given mechanism and experiment.

## 13. What has been accomplished

The bridge from $SE(3)$ to the manipulator equation is now explicit:

$$
T_i(q)\in SE(3)
\xrightarrow{\text{differentiate}}
V_i=J_i(q)\dot q
\xrightarrow{\mathcal G_i}
K_i=\frac12V_i^T\mathcal G_iV_i
\xrightarrow{\text{sum and vary}}
M\ddot q+C\dot q+g=\tau+J^TF.
$$

$SE(3)$ is not an alternative decorative notation for the final joint equation. It supplies the correct geometry from which every term is assembled.

## Checkpoint exercises

1. Starting from link kinetic energies, derive $M=\sum_iJ_i^T\mathcal G_iJ_i$.
2. Prove $M$ is symmetric and state the conditions for positive definiteness.
3. Derive generalized gravity using both potential energy and virtual work.
4. Show that the skew property implies the mechanical-energy balance.
5. Explain why $C$ is not unique but $C\dot q$ is physically meaningful.
6. Write the configuration and velocity spaces for a floating-base arm with seven revolute joints.

Solutions are in [[13 - Worked Problems and Solutions#Module 8 solutions]].

## Next

Continue to [[08A - Deriving M c and g from SE3 Link Mechanics]].
