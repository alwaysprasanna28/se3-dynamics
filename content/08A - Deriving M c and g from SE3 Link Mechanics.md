---
aliases:
  - Deriving Manipulator Dynamics with SE3
  - Spatial Derivation of M c g
tags:
  - robotics
  - manipulator-dynamics
  - newton-euler
  - se3
  - first-principles
course: Robotic Manipulator Dynamics on SE(3)
module: 8A
---

# Deriving $M$, $c$, and $g$ from $SE(3)$ Link Mechanics

> [!summary]
> This note derives every term of the joint-space manipulator equation directly from link twists, spatial accelerations, spatial inertias, and virtual power. It is the force-based counterpart of the energy derivation.

## 1. Setup and declared convention

For link $i$, choose a body-fixed frame and define

$$
T_i(q)=T_{0i}(q)\in SE(3).
$$

Use its body twist

$$
V_i=(T_i^{-1}\dot T_i)^\vee
=J_i(q)\dot q.
$$

The body spatial inertia $\mathcal G_i$ is constant, expressed in that same body frame and taken about its origin.

The body-coordinate Newton–Euler equation is

$$
F_i^{\mathrm{net}}
=\mathcal G_i\dot V_i
-\operatorname{ad}_{V_i}^T\mathcal G_iV_i.
$$

$F_i^{\mathrm{net}}$ is the net physical wrench on link $i$, expressed in its body frame.

## 2. Link acceleration from joint acceleration

Differentiate the twist relation:

$$
\dot V_i=J_i(q)\ddot q+\dot J_i(q,\dot q)\dot q.
$$

Substitute into Newton–Euler:

$$
F_i^{\mathrm{net}}
=\mathcal G_iJ_i\ddot q
+\mathcal G_i\dot J_i\dot q
-\operatorname{ad}_{V_i}^T\mathcal G_iV_i.
$$

The first term is linear in $\ddot q$. The last two are velocity-dependent inertial wrenches.

## 3. Projecting a link wrench into joint space

An admissible virtual joint displacement $\delta q$ produces the link's virtual body displacement

$$
\eta_i=J_i(q)\delta q.
$$

Virtual work of a body wrench is

$$
\delta W_i=F_i^T\eta_i
=F_i^TJ_i\delta q
=(J_i^TF_i)^T\delta q.
$$

Therefore its generalized force is $J_i^TF_i$. Summing links yields the generalized inertial force required for the complete robot:

$$
Q_{\mathrm{inertia}}
=\sum_iJ_i^T
\left(
\mathcal G_iJ_i\ddot q
+\mathcal G_i\dot J_i\dot q
-\operatorname{ad}_{V_i}^T\mathcal G_iV_i
\right).
$$

## 4. Identifying the mass matrix

Collect the coefficients multiplying $\ddot q$:

$$
Q_{\mathrm{inertia}}
=\left(\sum_iJ_i^T\mathcal G_iJ_i\right)\ddot q
+\text{velocity terms}.
$$

Thus

$$
\boxed{
M(q)=\sum_iJ_i(q)^T\mathcal G_iJ_i(q)
}.
$$

This matches the kinetic-energy derivation exactly.

## 5. Identifying the velocity bias

The remaining inertial terms define

$$
\boxed{
c(q,\dot q)
=\sum_iJ_i^T
\left(
\mathcal G_i\dot J_i\dot q
-\operatorname{ad}_{V_i}^T\mathcal G_iV_i
\right)
}.
$$

This expression uses body twists and body spatial inertias. If space quantities or a different spatial-acceleration convention are used, the individual terms change, but the final generalized vector $c(q,\dot q)$ does not.

### 5.1 Meaning of its two contributions

For each link:

- $\mathcal G_i\dot J_i\dot q$ is caused by the changing mapping from joint rates to link twist;
- $-\operatorname{ad}_{V_i}^T\mathcal G_iV_i$ is caused by expressing momentum change in a moving body frame.

Both are required. Omitting either generally produces incorrect Coriolis and centrifugal effects.

> [!warning]
> The formula is convention sensitive. It assumes $V_i=(T_i^{-1}\dot T_i)^\vee$, a constant body inertia, and the body Newton–Euler equation from [[07 - Single Rigid Body Dynamics on SE3]]. Do not combine it with a library's differently defined spatial acceleration without translating conventions.

## 6. Where internal joint forces went

Every joint transmits a wrench from parent to child and the equal opposite wrench from child to parent. When virtual work is summed over all links:

- components orthogonal to allowed joint motion are ideal constraint reactions and cancel;
- the component along an actuated joint's motion subspace becomes actuator generalized force;
- external wrenches remain because they act at the system boundary.

This is the rigid-body version of the constraint-force cancellation in [[07A - From Particles to Constrained Mechanisms]]. Recursive Newton–Euler makes the internal wrenches explicit during computation, then projects them to joint torques.

## 7. Gravity from link wrenches

Let the gravity acceleration expressed in the space frame be $g_0$. If link $i$ has rotation $R_i$ and center of mass at body offset $c_i$, then gravity force in body coordinates is

$$
f_{g,i}^b=m_iR_i^Tg_0.
$$

The gravity wrench about the body-frame origin is

$$
F_{g,i}^b=
\begin{bmatrix}
[c_i]_\times f_{g,i}^b\\
f_{g,i}^b
\end{bmatrix}.
$$

Its physical generalized load is

$$
\tau_g(q)=\sum_iJ_i(q)^TF_{g,i}^b.
$$

The standard left-side gravity convention is

$$
\boxed{g(q)=-\tau_g(q)}.
$$

Therefore

$$
g(q)=-\sum_iJ_i^TF_{g,i}^b.
$$

## 8. Gravity from potential energy

Define

$$
U(q)=-\sum_im_ig_0^Tp_{C_i}(q).
$$

Then

$$
g(q)=\frac{\partial U}{\partial q}.
$$

To prove equivalence, vary $U$:

$$
\delta U
=-\sum_im_ig_0^T\delta p_{C_i}
=-\sum_i(m_ig_0)^TJ_{v,C_i}\delta q.
$$

Thus

$$
\frac{\partial U}{\partial q}
=-\sum_iJ_{v,C_i}^Tm_ig_0
=-\tau_g.
$$

The wrench and potential derivations are the same work relation in two languages.

## 9. External wrenches and actuator inputs

If an external body wrench $F_i^{\mathrm{ext}}$ acts on link $i$,

$$
\tau_{\mathrm{ext}}=\sum_iJ_i^TF_i^{\mathrm{ext}}.
$$

Let $u$ be actuator effort and $B(q)$ the actuation map. The complete equation is

$$
\boxed{
M(q)\ddot q+c(q,\dot q)+g(q)+\tau_f(q,\dot q)
=B(q)u+\tau_{\mathrm{ext}}
}.
$$

For a fully actuated fixed-base arm with one torque-controlled motor per joint, $B=I$ and $u=\tau$.

## 10. Independent Euler–Lagrange derivation

From

$$
K=\frac12\dot q^TM(q)\dot q,
$$

the $i$th momentum coordinate is

$$
\frac{\partial K}{\partial\dot q_i}
=\sum_jM_{ij}\dot q_j.
$$

Differentiate:

$$
\frac{d}{dt}\frac{\partial K}{\partial\dot q_i}
=\sum_jM_{ij}\ddot q_j
+\sum_{j,k}\frac{\partial M_{ij}}{\partial q_k}
\dot q_k\dot q_j.
$$

The configuration derivative is

$$
\frac{\partial K}{\partial q_i}
=\frac12\sum_{j,k}
\frac{\partial M_{jk}}{\partial q_i}
\dot q_j\dot q_k.
$$

Subtracting produces

$$
\sum_jM_{ij}\ddot q_j
+\sum_{j,k}\Gamma_{ijk}\dot q_j\dot q_k,
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

Hence

$$
c_i(q,\dot q)=\sum_{j,k}\Gamma_{ijk}\dot q_j\dot q_k.
$$

The force-based spatial derivation and the energy-based Lagrange derivation must yield the same $M$, $c$, and $g$.

## 11. Why the two derivations agree

They begin from the same particle mechanics:

- Newton–Euler sums linear and angular momentum balance over each rigid link;
- Euler–Lagrange sums kinetic energy and applies constrained variations;
- virtual work projects physical wrenches into generalized forces.

Newton–Euler is usually computationally efficient and exposes internal wrenches. Euler–Lagrange is often shorter for hand derivations and exposes energy structure. Agreement between independently implemented versions is a strong model test.

## 12. Example: one rigid pendulum link

Let a rigid link rotate in a vertical plane. Its center of mass is distance $r$ from the joint, its mass is $m$, and its center-of-mass rotational inertia about the out-of-plane axis is $I_C$. Let $q$ be measured from horizontal.

The angular speed is $\dot q$ and center-of-mass speed is $r\dot q$. Therefore

$$
K=\frac12(I_C+mr^2)\dot q^2.
$$

Thus

$$
M=I_C+mr^2.
$$

$M$ is constant, so

$$
c(q,\dot q)=0.
$$

Potential energy is

$$
U=mg r\sin q,
$$

so

$$
g(q)=mgr\cos q.
$$

The equation is

$$
\boxed{
(I_C+mr^2)\ddot q+mgr\cos q=\tau
}.
$$

Adding motor inertia $I_m$ through gear ratio $N$ gives approximately

$$
M_{\mathrm{effective}}=I_C+mr^2+N^2I_m.
$$

## 13. Extracting dynamics by inverse-dynamics calls

Suppose a trusted inverse-dynamics routine returns

$$
\operatorname{ID}(q,\dot q,\ddot q)
=M(q)\ddot q+c(q,\dot q)+g(q).
$$

Then

$$
g(q)=\operatorname{ID}(q,0,0),
$$

$$
c(q,\dot q)
=\operatorname{ID}(q,\dot q,0)-g(q),
$$

and column $j$ of $M$ is

$$
M(q)e_j
=\operatorname{ID}(q,0,e_j)-g(q).
$$

This is useful for testing, though CRBA computes the full mass matrix more efficiently.

## 14. A derivation checklist

For every link, verify:

1. $T_i(q)$ has the declared transform direction;
2. $J_i$ produces the declared body or space twist;
3. $\mathcal G_i$ is taken about the same point and expressed in the same frame as $V_i$;
4. external wrenches use the dual convention compatible with $J_i$;
5. gravity sign matches the chosen manipulator equation;
6. $M$ from link Jacobians agrees with inverse-dynamics columns;
7. $c$ vanishes at $\dot q=0$;
8. energy and power identities hold.

## Checkpoint exercises

1. Starting from the body Newton–Euler equation, derive the displayed formula for $c(q,\dot q)$.
2. Explain the separate roles of $\mathcal G\dot J\dot q$ and $-\operatorname{ad}_V^T\mathcal GV$.
3. Prove the gravity-wrench and potential-energy formulas agree.
4. Derive the rigid pendulum equation using both spatial inertia and scalar energy.
5. Show how to recover $M$, $c$, and $g$ from inverse-dynamics evaluations.
6. Explain why internal joint reaction wrenches cancel in generalized virtual work.

Solutions are in [[13 - Worked Problems and Solutions#Module 8A solutions]].

## Next

Continue to [[08B - Structure and Physical Meaning of Manipulator Dynamics]].

