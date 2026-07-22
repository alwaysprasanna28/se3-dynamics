---
aliases:
  - SE3 Inverse Kinematics
  - Acceleration Kinematics
tags:
  - robotics
  - inverse-kinematics
  - acceleration
  - trajectory
course: Robotic Manipulator Dynamics on SE(3)
module: 6
---

# Acceleration Kinematics and Inverse Kinematics

> [!summary]
> Differentiating $V=J(q)\dot q$ gives commanded acceleration plus a configuration-velocity bias. Inverse kinematics on $SE(3)$ uses a logarithmic pose error and the matching body or space Jacobian.

## 1. Acceleration kinematics

For a chosen twist representation,

$$
V=J(q)\dot q.
$$

Differentiating its coordinates gives

$$
\dot V=J(q)\ddot q+\dot J(q,\dot q)\dot q.
$$

The two terms have distinct meanings:

- $J\ddot q$ is acceleration directly caused by joint accelerations;
- $\dot J\dot q$ is bias acceleration caused by moving joint axes and curvilinear kinematics even when $\ddot q=0$.

For a planar point moving at constant angular speed on a circle, $\ddot q=0$ but the point still has centripetal acceleration. That acceleration lives in $\dot J\dot q$.

> [!warning] Spatial-acceleration conventions
> Different libraries define a “spatial acceleration” using slightly different transported derivatives and point conventions. Always inspect the library definition. The coordinate derivative $\dot V$ is sufficient for the local manipulator formulas here, while recursive algorithms will explicitly include $\operatorname{ad}$ transport terms.

## 2. Computing $\dot J\dot q$ without forming $\dot J$

For the space Jacobian,

$$
J_{s,i}=\operatorname{Ad}_{T_{<i}}S_i.
$$

Let the twist produced by joints upstream of $i$ be

$$
V_{<i}^s=\sum_{j<i}J_{s,j}\dot q_j.
$$

Then

$$
\dot J_{s,i}=\operatorname{ad}_{V_{<i}^s}J_{s,i}.
$$

Hence

$$
\dot J_s\dot q
=\sum_i\operatorname{ad}_{V_{<i}^s}J_{s,i}\dot q_i.
$$

This recursive calculation avoids building a third-order derivative tensor.

For point-position tasks, direct analytic differentiation may be simpler. For the planar 2R arm, differentiate the position Jacobian and multiply by $\dot q$.

## 3. Acceleration-level inverse kinematics

To request task acceleration $\dot V_d$, solve

$$
J\ddot q=\dot V_d-\dot J\dot q.
$$

A minimum-norm solution is

$$
\ddot q=J^+(\dot V_d-\dot J\dot q)+(I-J^+J)z.
$$

Near singularities use damping or a constrained optimizer. Joint-acceleration, velocity, and position bounds are naturally expressed as quadratic-program constraints.

## 4. The inverse-kinematics problem

Given a desired pose $T_d\in SE(3)$, find $q$ such that

$$
T(q)=T_d.
$$

There may be:

- no solution because the target is outside the workspace;
- finitely many solutions, such as elbow-up and elbow-down;
- infinitely many solutions for a redundant arm;
- solutions near singularities with poor robustness.

Closed-form inverse kinematics is valuable for special structures. General robots use iterative numerical methods.

## 5. Newton iteration on $SE(3)$

### 5.1 Body-error version

At iterate $q_k$, compute

$$
T_k=T(q_k),
$$

$$
e_k^b=\operatorname{Log}(T_k^{-1}T_d)^\vee.
$$

A small joint update changes the pose approximately by

$$
T(q_k+\Delta q)\approx T_k\exp([J_b(q_k)\Delta q]^\wedge).
$$

To first order, choose

$$
J_b(q_k)\Delta q\approx e_k^b.
$$

Thus

$$
\Delta q=J_b(q_k)^+e_k^b,
\qquad
q_{k+1}=q_k+\alpha\Delta q,
$$

where $0<\alpha\le1$ is a step size.

### 5.2 Space-error version

Use

$$
e_k^s=\operatorname{Log}(T_dT_k^{-1})^\vee
$$

with the space Jacobian:

$$
\Delta q=J_s(q_k)^+e_k^s.
$$

Do not pair a body error with an untransformed space Jacobian.

## 6. Weighted pose error

Angular error has units of radians; translational error has units of meters. A raw Euclidean norm on a six-vector silently equates one radian with one meter. Choose a characteristic length $\ell$ or explicit task weights:

$$
\|e\|_W^2=e^TWe,
$$

$$
W=
\begin{bmatrix}
w_RI&0\\
0&w_pI
\end{bmatrix}.
$$

A characteristic-length scaling can use $w_R=\ell^2$ and $w_p=1$, converting angular error to a comparable linear displacement.

The weighted damped update solves

$$
\min_{\Delta q}
\frac12\|J\Delta q-e\|_W^2
+\frac{\lambda^2}{2}\|\Delta q\|^2.
$$

Its solution is

$$
\Delta q=(J^TWJ+\lambda^2I)^{-1}J^TWe.
$$

## 7. Closed-loop differential inverse kinematics

For a moving desired pose with desired compatible twist $V_d$, command

$$
\dot q=J^+\left(V_d+Ke\right)+(I-J^+J)z.
$$

Here $e$ and $V_d$ must be expressed in the same body or space convention as $J$. For finite errors, the exact derivative of $e=\operatorname{Log}(E)^\vee$ contains an inverse Lie Jacobian. The proportional form is a local feedback law and works best when errors remain in the logarithm's well-behaved region.

## 8. Secondary objectives and joint limits

For redundancy, set

$$
z=-k\nabla H(q),
$$

where, for example,

$$
H(q)=\frac12\sum_i
\left(\frac{q_i-q_{i,\mathrm{mid}}}{q_{i,\max}-q_{i,\min}}\right)^2.
$$

Then nullspace motion pulls joints toward their midranges. A simple projector can still cause priority leakage when damping is used or constraints activate. Hierarchical quadratic programming handles hard priorities more rigorously.

## 9. Pose trajectories on $SE(3)$

### 9.1 Screw interpolation

For endpoints $T_0,T_1$, define

$$
T(s)=T_0\exp\left(s\operatorname{Log}(T_0^{-1}T_1)\right),
\qquad s\in[0,1].
$$

This traces one constant body-screw motion with respect to path parameter $s$.

### 9.2 Separate translation and rotation interpolation

Sometimes a task requires a straight Cartesian path plus orientation interpolation:

$$
p(s)=(1-s)p_0+sp_1,
$$

$$
R(s)=R_0\exp\left(s\operatorname{Log}(R_0^TR_1)\right).
$$

This is not generally the same curve as a single $SE(3)$ screw interpolation. Neither is universally superior; the task decides.

### 9.3 Time scaling

Let $s=s(t)$ with

$$
s(0)=0,\quad s(T_f)=1,
$$

and usually

$$
\dot s(0)=\dot s(T_f)=0.
$$

A cubic scaling is

$$
s(t)=3\left(\frac{t}{T_f}\right)^2
-2\left(\frac{t}{T_f}\right)^3.
$$

A quintic scaling also sets endpoint accelerations to zero:

$$
s(t)=10r^3-15r^4+6r^5,
\qquad r=\frac{t}{T_f}.
$$

Twist scales with $\dot s$ and acceleration contains both $\ddot s$ and geometric transport terms.

## 10. Numerical robustness

### 10.1 Adaptive damping

If $\sigma_{\min}(J)$ becomes small, increase damping:

$$
\lambda=\lambda(\sigma_{\min}).
$$

This bounds velocities but introduces tracking error.

### 10.2 Step control

A full Newton step may leave the local linear region. Use line search or cap $\|\Delta q\|$. Recompute the group error after every accepted step.

### 10.3 Multiple initial guesses

Iterative IK is local. Different seeds can find different branches or reveal infeasibility. Use seeds based on continuity from the previous command when tracking.

### 10.4 Termination tests

Check separately:

$$
\|\phi\|<\varepsilon_R,
\qquad
\|\rho\|<\varepsilon_p,
$$

as well as step norm and iteration limit. A weighted combined norm alone can hide a large error in one component.

## 11. Minimal pseudocode

```text
given q, target T_d
repeat:
    T = forward_kinematics(q)
    e = vee(Log(inv(T) * T_d))       # body error
    if rotation_error(e) and translation_error(e) are small:
        return q
    J = body_jacobian(q)
    dq = solve((J^T W J + lambda^2 I), J^T W e)
    q = project_to_joint_limits(q + alpha * dq)
return failure_or_best_iterate
```

Projection to joint limits can disrupt convergence; a bound-constrained least-squares solver is better when limits are active.

## Checkpoint exercises

1. Derive $\ddot p=J_p\ddot q+\dot J_p\dot q$ for the planar 2R arm.
2. With $\dot q$ constant and $q_2=0$, interpret the nonzero parts of $\dot J\dot q$.
3. Derive the first-order body-error Newton update.
4. Explain why body error must be paired with the body Jacobian.
5. Compare screw interpolation with straight-position-plus-SLERP interpolation.
6. Design a weighted IK objective for a robot whose tool point must be precise but whose yaw may vary.

Solutions are in [[13 - Worked Problems and Solutions#Module 6 solutions]].

## Next

Continue to [[07 - Single Rigid Body Dynamics on SE3]].

