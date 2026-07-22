---
aliases:
  - RNEA CRBA ABA
  - Computational Robot Dynamics
tags:
  - robotics
  - rnea
  - crba
  - aba
  - algorithms
course: Robotic Manipulator Dynamics on SE(3)
module: 10
---

# Recursive Rigid-Body Dynamics Algorithms

> [!summary]
> Spatial algebra turns multibody dynamics into passes over a kinematic tree. RNEA solves inverse dynamics in $O(n)$, CRBA constructs the mass matrix in $O(n^2)$, and ABA solves forward dynamics in $O(n)$ for tree systems.

## 1. Why algorithms matter

The symbolic equation

$$
M(q)\ddot q+h(q,\dot q)=\tau
$$

does not prescribe how to compute its terms. Expanding symbolic expressions becomes unwieldy for large robots. Recursive algorithms exploit the fact that each link interacts directly only with its parent, children, joint, and external loads.

## 2. Spatial-algebra setup for a tree

For link $i$, define:

- parent index $\lambda(i)$;
- motion transform $X_i=\operatorname{Ad}_{T_{i,\lambda(i)}}$, mapping a parent-expressed twist to link-$i$ coordinates;
- constant joint motion subspace $S_i\in\mathbb R^{6\times d_i}$ expressed in link coordinates;
- joint velocity $\dot q_i\in\mathbb R^{d_i}$ and acceleration $\ddot q_i$;
- body spatial inertia $\mathcal G_i$;
- external wrench $F_i^{\mathrm{ext}}$ acting on the link, expressed in link coordinates.

For one-DoF joints, $S_i$ is a six-vector. A spherical or free joint has multiple motion-subspace columns.

The cross-motion and cross-force operators are

$$
V\times W=\operatorname{ad}_VW,
$$

$$
V\times^*F=-\operatorname{ad}_V^TF.
$$

## 3. Recursive Newton–Euler algorithm

RNEA computes the joint forces required for known $q,\dot q,\ddot q$.

### 3.1 Outward velocity pass

For each link from base to leaves,

$$
V_i=X_iV_{\lambda(i)}+S_i\dot q_i.
$$

The joint's relative twist is

$$
V_{J_i}=S_i\dot q_i.
$$

### 3.2 Outward acceleration pass

$$
A_i=X_iA_{\lambda(i)}+S_i\ddot q_i
+\operatorname{ad}_{V_i}S_i\dot q_i.
$$

The last term is the velocity-product bias caused by the moving joint direction and frame transport.

For a fixed base, gravity can be included without per-link gravity wrenches by initializing

$$
V_0=0,
\qquad
A_0=\begin{bmatrix}0\\-g_0\end{bmatrix},
$$

where $g_0$ is gravity acceleration expressed in the base frame. This is a computational device: the base is not physically accelerating.

### 3.3 Link net wrench

The inertial wrench required by link $i$ is

$$
F_i=\mathcal G_iA_i
-\operatorname{ad}_{V_i}^T\mathcal G_iV_i
-F_i^{\mathrm{ext}}.
$$

The external wrench is subtracted because $F_i$ represents what the parent/joint must supply after environmental assistance is accounted for.

### 3.4 Inward force pass

Traverse from leaves to base. Project onto the joint motion subspace:

$$
\tau_i=S_i^TF_i.
$$

Then add the link wrench to its parent:

$$
F_{\lambda(i)}\mathrel{+}=X_i^TF_i.
$$

$X_i^T$ is the dual force transform corresponding to the parent-to-child motion transform $X_i$.

### 3.5 Pseudocode

```text
V[0] = 0
A[0] = [0; -gravity]

for i in topological_order:
    X[i] = joint_motion_transform(q[i]) * fixed_transform[i]
    VJ = S[i] * qd[i]
    V[i] = X[i] * V[parent[i]] + VJ
    A[i] = X[i] * A[parent[i]] + S[i] * qdd[i] + ad(V[i]) * VJ
    F[i] = G[i] * A[i] - ad(V[i])^T * G[i] * V[i] - Fext[i]

for i in reverse_topological_order:
    tau[i] = S[i]^T * F[i]
    F[parent[i]] += X[i]^T * F[i]
```

The exact multiplication order used to build $X_i$ depends on how fixed joint frames are stored. Unit-test transforms separately.

## 4. What RNEA returns

RNEA evaluates

$$
\operatorname{ID}(q,\dot q,\ddot q,F_{\mathrm{ext}})
=M(q)\ddot q+c(q,\dot q)+g(q)-\tau_{\mathrm{ext}}.
$$

Useful calls include:

$$
h(q,\dot q)=\operatorname{ID}(q,\dot q,0,0),
$$

and, with gravity disabled and $\dot q=0$,

$$
M(q)e_j=\operatorname{ID}(q,0,e_j,0).
$$

Calling RNEA once per basis acceleration constructs $M$ simply, though CRBA is more efficient.

## 5. Composite rigid-body algorithm

CRBA computes $M(q)$ by accumulating the inertia of each subtree.

Initialize composite inertias:

$$
\mathcal G_i^C\leftarrow\mathcal G_i.
$$

Traverse inward:

$$
\mathcal G_{\lambda(i)}^C
\mathrel{+}=X_i^T\mathcal G_i^CX_i.
$$

The diagonal mass entry for a one-DoF joint is

$$
M_{ii}=S_i^T\mathcal G_i^CS_i.
$$

Off-diagonal entries are obtained by transporting the force-like vector

$$
F=\mathcal G_i^CS_i
$$

up the ancestor chain and projecting it onto ancestor motion subspaces. Symmetry fills the other triangle.

CRBA is $O(n^2)$ for a chain because the dense $n\times n$ output itself contains $O(n^2)$ entries.

## 6. Articulated-body algorithm

Forward dynamics asks for $\ddot q$ given $\tau$. Forming $M$ and factoring it costs roughly $O(n^3)$ for a dense generic solve, though robot structure and optimized factorizations help. ABA eliminates accelerations recursively and computes tree forward dynamics in $O(n)$.

Conceptually ABA performs:

1. an outward pass to compute transforms, velocities, and bias accelerations;
2. an inward pass to build each subtree's articulated inertia and bias wrench after accounting for joint freedom;
3. an outward pass to recover joint and link accelerations.

For a one-DoF joint, the inward elimination uses

$$
U_i=\mathcal I_i^AS_i,
$$

$$
d_i=S_i^TU_i,
$$

$$
u_i=\tau_i-S_i^Tp_i^A,
$$

where $\mathcal I_i^A$ is articulated inertia and $p_i^A$ is articulated bias force. The joint's free direction is eliminated through a rank-one correction involving $U_id_i^{-1}U_i^T$. On the final pass,

$$
\ddot q_i=d_i^{-1}
\left(u_i-U_i^TA_i^{\mathrm{parent,bias}}\right),
$$

with the parent acceleration transported into link coordinates.

The full index-level ABA is best learned from Featherstone or a tested library; the important insight is that articulated inertia is the effective inertia of an entire subtree as seen through its movable joints.

## 7. Algorithm comparison

| Problem | Inputs | Output | Standard algorithm | Tree complexity |
|---|---|---|---|---:|
| Inverse dynamics | $q,\dot q,\ddot q$ | $\tau$ | RNEA | $O(n)$ |
| Bias forces | $q,\dot q$ | $h$ | RNEA with $\ddot q=0$ | $O(n)$ |
| Mass matrix | $q$ | $M$ | CRBA | $O(n^2)$ |
| Forward dynamics | $q,\dot q,\tau$ | $\ddot q$ | ABA | $O(n)$ |
| Simple forward dynamics | $M,h,\tau$ | $\ddot q$ | factor $M$ | typically $O(n^3)$ dense |

For modest fixed-base arms, a well-optimized mass-matrix factorization is often fast enough and easy to inspect. ABA matters for large trees, repeated simulation, and whole-body systems.

## 8. Constraints and contacts

With holonomic or contact constraints

$$
J_c(q)\dot q=0,
$$

acceleration consistency is

$$
J_c\ddot q+\dot J_c\dot q=0.
$$

Dynamics become

$$
M\ddot q+h=\tau+J_c^T\lambda,
$$

where $\lambda$ is the constraint wrench coordinate. Combining the equations gives the saddle-point system

$$
\begin{bmatrix}
M&-J_c^T\\
J_c&0
\end{bmatrix}
\begin{bmatrix}
\ddot q\\\lambda
\end{bmatrix}
=
\begin{bmatrix}
\tau-h\\
-\dot J_c\dot q
\end{bmatrix}.
$$

Unilateral contact also needs nonpenetration, normal-force positivity, and friction constraints; impacts require an impulse-level model.

## 9. Numerical integration

Given forward dynamics $\ddot q=f(q,\dot q,\tau)$, a semi-implicit Euler step is

$$
\dot q_{k+1}=\dot q_k+h\ddot q_k,
$$

$$
q_{k+1}=q_k+h\dot q_{k+1}.
$$

It is often more stable for mechanical systems than fully explicit Euler. Higher-order Runge–Kutta improves local accuracy but does not automatically preserve energy or constraints.

For an $SE(3)$ floating base, update its pose with an exponential:

$$
T_{k+1}=T_k\exp(h[V_{B,k+1}]^\wedge)
$$

for a body-twist convention.

## 10. Verification strategy

### 10.1 Dynamics identities

- Check $M=M^T$ numerically.
- Check eigenvalues of $M$ are positive.
- Check inverse and forward dynamics round-trip:

$$
\tau=\operatorname{ID}(q,\dot q,\ddot q)
\quad\Rightarrow\quad
\operatorname{FD}(q,\dot q,\tau)\approx\ddot q.
$$

- Compare RNEA mass columns with CRBA.
- Compare gravity from RNEA with $\partial U/\partial q$ by finite differences.

### 10.2 Power identity

For a link wrench and Jacobian,

$$
F^TJ\dot q\approx(J^TF)^T\dot q.
$$

This trivial-looking equality catches frame and ordering mistakes.

### 10.3 Finite-difference energy checks

Approximate $\dot M$ along $\dot q$ and verify

$$
\dot q^T(\dot M-2C)\dot q\approx0.
$$

In passive, frictionless simulation with gravity potential included, monitor total energy drift.

## Checkpoint exercises

1. Explain every term in the RNEA acceleration recurrence.
2. Prove that $X_i^T$ is the correct child-to-parent wrench accumulation map using power.
3. Show how to compute one column of $M$ using RNEA.
4. Explain why CRBA cannot generally return a dense mass matrix in less than $O(n^2)$ work.
5. Derive the constrained saddle-point dynamics system.
6. Design three numerical tests that would expose a reversed transform in RNEA.

Solutions are in [[13 - Worked Problems and Solutions#Module 10 solutions]].

## Next

Continue to [[10A - Constraints Contacts and Underactuation]].
