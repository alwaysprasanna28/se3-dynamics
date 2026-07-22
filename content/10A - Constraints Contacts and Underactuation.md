---
aliases:
  - Constrained Manipulator Dynamics
  - Robot Contact Dynamics
tags:
  - robotics
  - constrained-dynamics
  - contact
  - underactuation
course: Robotic Manipulator Dynamics on SE(3)
module: 10A
---

# Constraints, Contacts, and Underactuation

> [!summary]
> Minimal joint coordinates eliminate internal tree constraints, but closed chains, environmental contact, and floating bases require explicit constraints and actuation maps. Lagrange multipliers are the generalized constraint wrenches that enforce admissible motion.

## 1. Three different kinds of restriction

Do not conflate:

1. **Kinematic mechanism constraints:** a revolute joint permits one relative motion and forbids five; minimal joint coordinates already encode these.
2. **Additional bilateral constraints:** a closed loop or rigid contact imposes equality constraints that remain continuously active.
3. **Unilateral contact constraints:** bodies may separate, so normal contact force can push but not pull.

Underactuation is different again: the system may be able to move in a direction without having a direct actuator in that direction.

## 2. Holonomic equality constraints

Let

$$
\phi(q)=0,
$$

where $\phi\in\mathbb R^k$. Differentiate:

$$
J_c(q)\dot q=0,
\qquad
J_c(q)=\frac{\partial\phi}{\partial q}.
$$

Differentiate again:

$$
\boxed{
J_c(q)\ddot q+\dot J_c(q,\dot q)\dot q=0
}.
$$

The $\dot J_c\dot q$ term is required even when the constraint value is constant.

## 3. Constraint force from virtual work

Admissible virtual displacements satisfy

$$
J_c\delta q=0.
$$

A generalized constraint force must do zero virtual work for every such displacement. The orthogonal complement of $\mathcal N(J_c)$ is $\mathcal R(J_c^T)$, so

$$
\boxed{
\tau_c=J_c(q)^T\lambda
}.
$$

$\lambda\in\mathbb R^k$ is a Lagrange multiplier. Its physical units depend on how $\phi$ is defined. If $J_c$ maps to a Cartesian contact velocity, $\lambda$ has force or wrench units.

## 4. Constrained equations of motion

Let $B(q)u$ be actuation. The dynamics are

$$
M\ddot q+h=B u+\tau_{\mathrm{ext}}+J_c^T\lambda.
$$

Together with acceleration consistency,

$$
J_c\ddot q=-\dot J_c\dot q,
$$

this gives the saddle-point system

$$
\boxed{
\begin{bmatrix}
M&-J_c^T\\
J_c&0
\end{bmatrix}
\begin{bmatrix}
\ddot q\\\lambda
\end{bmatrix}
=
\begin{bmatrix}
B u+\tau_{\mathrm{ext}}-h\\
-\dot J_c\dot q
\end{bmatrix}
}.
$$

Solving it produces both admissible acceleration and the constraint wrench needed to create it.

## 5. Schur-complement solution

From the dynamics,

$$
\ddot q=M^{-1}
\left(Bu+\tau_{\mathrm{ext}}-h+J_c^T\lambda\right).
$$

Insert into the acceleration constraint:

$$
J_cM^{-1}J_c^T\lambda
=J_cM^{-1}(h-Bu-\tau_{\mathrm{ext}})
-\dot J_c\dot q.
$$

If $J_c$ has independent rows, define

$$
\Lambda_c=(J_cM^{-1}J_c^T)^{-1}.
$$

Then

$$
\boxed{
\lambda
=\Lambda_c\left[
J_cM^{-1}(h-Bu-\tau_{\mathrm{ext}})
-\dot J_c\dot q
\right]
}.
$$

This $\Lambda_c$ is the inertia seen in the constraint directions.

## 6. Constraint-consistent projection

Define the dynamically consistent inverse

$$
\bar J_c=M^{-1}J_c^T\Lambda_c.
$$

The velocity-space projector into the constraint nullspace is

$$
N_c=I-\bar J_cJ_c.
$$

It satisfies

$$
J_cN_c=0.
$$

This projection helps describe motion compatible with contact, but the bias term $\dot J_c\dot q$ must still be included at acceleration level.

## 7. Ideal constraints exchange no power

If the velocity constraint holds,

$$
J_c\dot q=0,
$$

then constraint power is

$$
P_c=(J_c^T\lambda)^T\dot q
=\lambda^TJ_c\dot q=0.
$$

An ideal stationary bilateral constraint can redirect motion and exchange forces without changing the mechanism's mechanical energy. A moving constraint or a nonideal contact can perform work.

## 8. Example: end effector pinned to a fixed point

For a planar arm with end-effector position $p_e(q)$, impose

$$
\phi(q)=p_e(q)-p_*=0.
$$

Then

$$
J_c(q)=J_p(q).
$$

If $J_p$ has rank two, the contact removes two instantaneous degrees of freedom. The multiplier $\lambda\in\mathbb R^2$ is the planar force exerted by the fixed point on the robot under the chosen sign convention.

At a singular configuration, the constraint rows may become dependent. The multiplier may not be unique even though the total generalized constraint force is meaningful.

## 9. Closed-chain mechanisms

A loop-closure condition equates two poses or points reached through different kinematic paths. A pose-level condition can be written locally as

$$
\phi(q)=\operatorname{Log}
\left(T_A(q)^{-1}T_B(q)T_{AB,d}^{-1}\right)^\vee=0.
$$

Its differential produces a relative-twist constraint Jacobian. The log coordinates are local; implementations often construct the relative Jacobian directly to avoid differentiating a large-error log formula.

Options for closed chains are:

- choose minimal loop coordinates, eliminating constraint forces;
- retain redundant coordinates and solve for multipliers;
- cut one joint to form a tree and enforce closure numerically.

## 10. Unilateral normal contact

Let $\phi_n(q)$ be a signed gap, positive when separated. Nonpenetration is

$$
\phi_n(q)\ge0.
$$

The normal force can push but not pull:

$$
\lambda_n\ge0.
$$

Ideal rigid contact uses complementarity:

$$
\boxed{
0\le\lambda_n\ \perp\ \phi_n(q)\ge0
},
$$

meaning

$$
\lambda_n\phi_n=0.
$$

Either the bodies are separated with zero contact force, or touching with a possibly positive normal force. At acceleration level, active-contact complementarity is formulated using normal acceleration and must account for velocity and impact state.

## 11. Friction

For Coulomb friction, tangential force lies in a friction cone:

$$
\|f_t\|\le\mu f_n.
$$

During sticking, tangential relative velocity is zero and friction takes whatever value inside the cone is needed. During sliding,

$$
f_t=-\mu f_n\frac{v_t}{\|v_t\|}.
$$

In computation, the cone is often approximated by a friction pyramid so contact dynamics can be written as a linear or quadratic complementarity/optimization problem.

## 12. Compliant contact as an alternative

A penalty model replaces the rigid constraint with a stiff force law, for example

$$
f_n=
\begin{cases}
k\delta+d\dot\delta,&\delta>0,\\
0,&\delta\le0,
\end{cases}
$$

where $\delta$ is penetration depth. This removes complementarity but introduces artificial penetration, stiffness, and small numerical time scales. Rigid and compliant contact models answer slightly different physical questions.

## 13. Impacts and impulses

An impact changes velocity over a very short interval. Configuration is approximately constant while finite impulse $P$ acts:

$$
M(q)(\dot q^+-\dot q^-)=J_c(q)^TP.
$$

For a frictionless contact with coefficient of restitution $e\in[0,1]$,

$$
J_c\dot q^+=-eJ_c\dot q^-.
$$

Combining them gives

$$
\boxed{
P=-(1+e)
\left(J_cM^{-1}J_c^T\right)^{-1}
J_c\dot q^-
}.
$$

$e=0$ gives a perfectly inelastic normal impact; $e=1$ gives an ideal elastic normal impact. Multiple frictional impacts are substantially more complex and may not have a unique solution under naive rigid models.

## 14. Underactuation

Let $u\in\mathbb R^m$ with $m<n$. Actuation enters as

$$
B(q)u,
\qquad
B\in\mathbb R^{n\times m}.
$$

The dynamics are

$$
M\ddot q+h=B u+J_c^T\lambda.
$$

The unactuated generalized directions lie in the left nullspace of $B$:

$$
B_\perp^TB=0.
$$

Projecting gives feasibility conditions that actuator commands cannot directly change:

$$
B_\perp^T(M\ddot q+h-J_c^T\lambda)=0.
$$

For a floating-base robot, joint motors cannot directly apply a net wrench to the free base. Contact forces make many whole-body accelerations feasible.

## 15. Floating-base manipulator form

Use generalized velocity

$$
\nu=\begin{bmatrix}V_B\\\dot q_j\end{bmatrix}.
$$

Dynamics are

$$
M(q)\dot\nu+h(q,\nu)
=S^T\tau_j+J_c(q)^TF_c,
$$

where

$$
S=\begin{bmatrix}0_{n_j\times6}&I_{n_j}\end{bmatrix}.
$$

The first six rows are the unactuated base momentum balance. The remaining rows are joint dynamics. Contact forces couple the two.

## 16. Inverse dynamics with constraints

Given desired acceleration $\ddot q_d$, solve

$$
M\ddot q_d+h=B u+J_c^T\lambda
$$

subject to

$$
J_c\ddot q_d+\dot J_c\dot q=0.
$$

For fully actuated systems, many pairs $(u,\lambda)$ can satisfy the same motion; force objectives select among them. For underactuated systems, the desired acceleration may be infeasible. Optimization-based inverse dynamics commonly adds:

- torque bounds;
- friction cones;
- unilateral-force constraints;
- acceleration tracking costs;
- contact-force costs;
- joint-limit constraints.

## 17. Constraint stabilization

Numerical integration can drift from $\phi(q)=0$. Baumgarte stabilization replaces pure acceleration consistency by

$$
J_c\ddot q+\dot J_c\dot q
+2\alpha J_c\dot q+\beta^2\phi(q)=0.
$$

This feeds position and velocity error back into constraint acceleration. Large gains create stiffness. Projection methods instead map positions and velocities back onto the constraint manifold.

## 18. Common mistakes

- Omitting $\dot J_c\dot q$.
- Treating unilateral contact as an always-active equality.
- Giving a normal contact force the ability to pull.
- Forgetting the friction cone.
- Assuming $\lambda$ is unique when constraint rows are dependent.
- Confusing a constraint wrench with an actuator torque.
- Ignoring impact and integrating through a discontinuous collision.
- Asking an underactuated system to realize an arbitrary $\ddot q_d$.
- Using a stiff penalty model without reducing the integrator step.

## Checkpoint exercises

1. Derive $J_c^T\lambda$ from the zero-virtual-work property.
2. Derive the saddle-point and Schur-complement solutions.
3. Prove an ideal stationary bilateral constraint performs zero power.
4. Derive the frictionless impact impulse formula.
5. Explain why a floating-base robot cannot accelerate its base arbitrarily without contact.
6. Formulate constrained inverse dynamics with torque bounds and a friction cone.

Solutions are in [[13 - Worked Problems and Solutions#Module 10A solutions]].

## Next

Continue to [[11 - Operational-Space Dynamics]].

