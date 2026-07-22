---
aliases:
  - Task-Space Dynamics
  - Operational Space Formulation
tags:
  - robotics
  - operational-space
  - task-space
  - dynamics
course: Robotic Manipulator Dynamics on SE(3)
module: 11
---

# Operational-Space Dynamics

> [!summary]
> Operational-space dynamics describe the inertia and bias forces seen in a task such as the end-effector twist. The central object is $\Lambda=(JM^{-1}J^T)^{-1}$, not $JMJ^T$.

## 1. Why move dynamics to task space?

Joint-space dynamics are

$$
M(q)\ddot q+h(q,\dot q)
=\tau+J(q)^TF_{\mathrm{ext}}.
$$

But a manipulation task is usually stated in terms of end-effector pose, twist, acceleration, or contact wrench. Let the task twist be

$$
V=J(q)\dot q.
$$

Then

$$
\dot V=J\ddot q+\dot J\dot q.
$$

Operational-space dynamics answer: what task acceleration results from a task wrench, after accounting for the entire robot configuration and inertia?

## 2. Deriving operational-space inertia

Assume $J\in\mathbb R^{m\times n}$ has full row rank and the actuator torque dedicated to the task is

$$
\tau=J^TF.
$$

Joint acceleration is

$$
\ddot q=M^{-1}(J^T(F+F_{\mathrm{ext}})-h).
$$

Substitute into task acceleration:

$$
\dot V
=JM^{-1}J^T(F+F_{\mathrm{ext}})
-JM^{-1}h+\dot J\dot q.
$$

Define

$$
\boxed{
\Lambda(q)=\left(JM^{-1}J^T\right)^{-1}
}.
$$

Multiplying by $\Lambda$ gives

$$
\Lambda\dot V+\eta=F+F_{\mathrm{ext}},
$$

where

$$
\boxed{
\eta(q,\dot q)=
\Lambda\left(JM^{-1}h-\dot J\dot q\right)
}.
$$

If $h=c+g$, split

$$
\mu=\Lambda\left(JM^{-1}c-\dot J\dot q\right),
$$

$$
p=\Lambda JM^{-1}g,
$$

so

$$
\Lambda\dot V+\mu+p=F+F_{\mathrm{ext}}.
$$

## 3. Why $\Lambda\ne JMJ^T$

$J$ maps joint velocity to task velocity. Inertia maps acceleration to force and therefore transforms through the inverse kinematic relation, not by simply sandwiching $M$ with $J$.

The formula follows from the task acceleration response:

$$
\dot V=JM^{-1}J^TF
$$

when biases vanish. Thus task inverse inertia is

$$
\Lambda^{-1}=JM^{-1}J^T.
$$

Only after inverting do we obtain task inertia.

## 4. Minimum kinetic-energy interpretation

Among all joint velocities satisfying $J\dot q=V$, minimize

$$
K=\frac12\dot q^TM\dot q.
$$

The constrained minimizer is

$$
\dot q=\bar JV,
$$

where

$$
\boxed{
\bar J=M^{-1}J^T\Lambda
}
$$

is the dynamically consistent generalized inverse. Substitution gives

$$
K_{\min}=\frac12V^T\Lambda V.
$$

Thus $\Lambda$ is the inertia metric induced on task velocity space by the robot's minimum-energy motion.

## 5. Properties of the dynamically consistent inverse

For full-row-rank $J$,

$$
J\bar J=JM^{-1}J^T\Lambda=I.
$$

The joint-velocity nullspace projector is

$$
N=I-\bar JJ.
$$

It satisfies

$$
JN=0.
$$

For torques, use the dual projector $N^T$:

$$
\tau=J^TF+N^T\tau_0.
$$

Then

$$
JM^{-1}N^T=0,
$$

so the nullspace torque creates no instantaneous task acceleration in the ideal model.

> [!warning]
> The Euclidean projector $I-J^+J$ is kinematically valid, but its transpose does not generally decouple task acceleration under a nonidentity mass matrix. Dynamic consistency uses the $M$ metric.

## 6. Effective mass in a direction

Let $u$ be a unit direction in a translational task and apply force $f u$. Ignoring bias,

$$
a=\Lambda^{-1}u f.
$$

The acceleration component along $u$ is

$$
u^Ta=f\,u^T\Lambda^{-1}u.
$$

Therefore directional effective mass is

$$
\boxed{
m_{\mathrm{eff}}(u)=
\frac{1}{u^T\Lambda^{-1}u}
}.
$$

This matters in collision and human–robot interaction: the same robot can feel light in one direction and heavy in another.

## 7. $SE(3)$ task coordinates

For a full end-effector task, $V$ is a body or space twist. If coordinates change by

$$
V_a=\operatorname{Ad}_{T_{ab}}V_b,
$$

then task kinetic energy invariance requires

$$
\Lambda_a
=\operatorname{Ad}_{T_{ab}}^{-T}
\Lambda_b
\operatorname{Ad}_{T_{ab}}^{-1}.
$$

Task wrenches transform by

$$
F_a=\operatorname{Ad}_{T_{ab}}^{-T}F_b.
$$

Thus operational-space inertia transforms exactly like a spatial inertia.

### 7.1 Frame consistency

If $J_b$ maps to a body twist, then $\Lambda_b$, $F_b$, $\eta_b$, and $\dot V_b$ must all use compatible body coordinates. Mixing a body Jacobian with a space wrench can silently produce incorrect torque.

## 8. Multiple tasks and hierarchy

For a primary task $J_1$ and a secondary task $J_2$, a simple hierarchy is

$$
\tau=J_1^TF_1+N_1^TJ_2^TF_2.
$$

However, the effective secondary Jacobian is $J_2N_1$, and rigorous dynamic decoupling requires recomputing its inertia within the primary nullspace. Multiple priorities, inequality limits, and contact constraints are often handled by hierarchical quadratic programs.

## 9. Constraints as an operational space

For a contact constraint Jacobian $J_c$, the constraint-space inertia is

$$
\Lambda_c=(J_cM^{-1}J_c^T)^{-1}.
$$

Solving

$$
J_c\ddot q+\dot J_c\dot q=0
$$

with the joint dynamics gives the required constraint wrench. This is the same mathematics as operational-space dynamics, but the task acceleration is constrained rather than commanded.

## 10. Singular and underactuated cases

If $J$ loses row rank, $JM^{-1}J^T$ is singular. Options include:

- restrict the task to the attainable subspace;
- use an eigenvalue or singular-value decomposition and a pseudoinverse;
- use damping, accepting approximate dynamics;
- formulate a constrained optimization directly in joint acceleration and wrench variables.

A damped quantity such as

$$
\Lambda_\lambda=(JM^{-1}J^T+\lambda^2I)^{-1}
$$

is numerically finite but is no longer the exact physical task inertia.

For underactuated systems, not every generalized torque $J^TF$ is realizable because actuator inputs enter through a selection matrix. Operational-space commands must respect actuation and contact constraints.

## 11. Operational-space inverse dynamics control

Suppose a desired task acceleration $\dot V_r$ has already been constructed from reference feedforward and feedback. Choose

$$
F=\Lambda\dot V_r+\eta.
$$

Then, in the exact full-rank model and without external wrench,

$$
\dot V=\dot V_r.
$$

The joint torque is

$$
\tau=J^TF+N^T\tau_0.
$$

This cancellation-based result is powerful but sensitive to model error, torque limits, time delay, and unmodeled contact.

## 12. Common mistakes

- Using $JMJ^T$ as task inertia.
- Omitting the $\dot J\dot q$ term from task bias.
- Using a pseudoinverse where a dynamically consistent inverse is required.
- Applying $N\tau_0$ instead of the appropriate dual $N^T\tau_0$.
- Mixing body and space Jacobians, wrenches, or errors.
- Inverting near a singularity without rank handling.
- Assuming nullspace decoupling remains exact under saturation or model error.

## Checkpoint exercises

1. Derive $\Lambda$ and $\eta$ from joint dynamics.
2. Derive $\bar J$ by minimizing kinetic energy subject to $J\dot q=V$.
3. Prove $JM^{-1}N^T=0$.
4. Derive directional effective mass.
5. Show how $\Lambda$ transforms between body and space task coordinates.
6. Explain why a damped $\Lambda_\lambda$ is useful but not the exact task inertia.

Solutions are in [[13 - Worked Problems and Solutions#Module 11 solutions]].

## Next

Continue to [[12 - Control on Joint Space and SE3]].

