---
aliases:
  - Properties of the Manipulator Equation
  - Physical Meaning of M C g
tags:
  - robotics
  - manipulator-dynamics
  - energy
  - passivity
  - first-principles
course: Robotic Manipulator Dynamics on SE(3)
module: 8B
---

# Structure and Physical Meaning of Manipulator Dynamics

> [!summary]
> The manipulator equation is more than a collection of terms. $M$ is a kinetic-energy metric, $c$ is the coordinate expression of inertial curvature, $g$ is a potential gradient, and $J^TF$ is the work-preserving pullback of external load. Their structural identities are essential for analysis, control, and debugging.

## 1. Canonical equation and sign convention

We use

$$
M(q)\ddot q+c(q,\dot q)+g(q)+\tau_f(q,\dot q)
=B(q)u+\tau_{\mathrm{ext}}.
$$

Here:

- $M(q)\in\mathbb R^{n\times n}$ is the generalized mass matrix;
- $c(q,\dot q)$ contains Coriolis, centrifugal, and other velocity-product inertial terms;
- $g(q)=\partial U/\partial q$ is the left-side gravity vector;
- $\tau_f$ contains friction or dissipation;
- $B(q)u$ is actuator generalized force;
- $\tau_{\mathrm{ext}}=\sum_iJ_i^TF_i^{\mathrm{ext}}$ is external generalized load.

It is common to define

$$
h(q,\dot q)=c(q,\dot q)+g(q)+\tau_f(q,\dot q),
$$

giving

$$
M(q)\ddot q+h(q,\dot q)=B(q)u+\tau_{\mathrm{ext}}.
$$

Always inspect what a software library includes in its “nonlinear effects” or “bias” vector.

## 2. $M(q)$ is a kinetic-energy metric

Kinetic energy is

$$
K(q,\dot q)=\frac12\dot q^TM(q)\dot q.
$$

This gives several facts immediately.

### 2.1 Symmetry

Only the symmetric part contributes to a quadratic form. Physical construction gives

$$
M(q)=M(q)^T.
$$

An appreciably nonsymmetric numerical mass matrix indicates an implementation or storage error.

### 2.2 Positive definiteness

For independent coordinates and positive physical inertias,

$$
\dot q^TM(q)\dot q>0
\qquad\text{for all }\dot q\ne0.
$$

Thus $M(q)\succ0$. It is invertible, admits a Cholesky factorization, and defines an inner product on $T_q\mathcal Q$:

$$
\langle u,v\rangle_{M(q)}=u^TM(q)v.
$$

### 2.3 Configuration dependence

Link masses do not change, but their velocities under a unit joint rate do. Therefore $M$ changes with geometry. In the planar 2R arm, the relative elbow angle changes whether link velocities reinforce or oppose one another, producing $\cos q_2$ terms.

### 2.4 Coupling

The entry $M_{ij}$ is the bilinear kinetic coupling between joint directions $i$ and $j$:

$$
M_{ij}=\left\langle e_i,e_j\right\rangle_M.
$$

If $M_{ij}\ne0$, accelerating one joint generally requires torque at the other. Diagonal inertia is a coordinate- and configuration-specific special case.

## 3. Inertia ellipsoid in joint space

The set of generalized velocities with fixed kinetic energy $K_0$ is

$$
\dot q^TM(q)\dot q=2K_0.
$$

This is an ellipsoid in joint-velocity coordinates. If

$$
M=Q\Lambda Q^T,
$$

its principal directions are columns of $Q$ and semi-axis lengths scale as $1/\sqrt{\lambda_i}$. A large eigenvalue means that velocity direction carries high kinetic energy.

This ellipsoid is not the same as the kinematic manipulability ellipsoid based on $JJ^T$. Operational-space inertia combines both geometry and mass:

$$
\Lambda=(JM^{-1}J^T)^{-1}.
$$

## 4. What $c(q,\dot q)$ really means

The vector $c$ is the generalized inertial force required even when $\ddot q=0$. It appears because constant coordinate rates do not generally correspond to constant Cartesian or spatial velocities.

With Christoffel symbols,

$$
c_i(q,\dot q)=\sum_{j,k}\Gamma_{ijk}(q)\dot q_j\dot q_k.
$$

Terms proportional to $\dot q_j^2$ are often called centrifugal; terms proportional to $\dot q_j\dot q_k$ with $j\ne k$ are often called Coriolis. This separation is descriptive, not fundamental:

- it changes under coordinate transformations;
- algebraic rearrangements can move terms between categories;
- the full vector $c$ is the meaningful object.

### 4.1 Why $c$ is quadratic in velocity

Kinetic energy is quadratic in $\dot q$, and its configuration derivative introduces products of two velocity components. For rigid-body mechanisms without velocity-dependent fields,

$$
c(q,\alpha\dot q)=\alpha^2c(q,\dot q).
$$

This homogeneity is a useful numerical check.

### 4.2 Why $C$ is not unique

Writing

$$
c(q,\dot q)=C(q,\dot q)\dot q
$$

does not uniquely determine $C$. Any $\Delta C$ satisfying

$$
\Delta C(q,\dot q)\dot q=0
$$

can be added. A Christoffel-compatible $C$ is preferred because it satisfies an energy identity.

## 5. Deriving the skew-symmetry identity

Choose

$$
C_{ij}=\sum_k\Gamma_{ijk}\dot q_k,
$$

with

$$
\Gamma_{ijk}
=\frac12\left(
\frac{\partial M_{ij}}{\partial q_k}
+\frac{\partial M_{ik}}{\partial q_j}
-\frac{\partial M_{jk}}{\partial q_i}
\right).
$$

The time derivative of $M$ is

$$
\dot M_{ij}=\sum_k\frac{\partial M_{ij}}{\partial q_k}\dot q_k.
$$

Define $N=\dot M-2C$. Then

$$
N_{ij}
=\sum_k\left(
-\frac{\partial M_{ik}}{\partial q_j}
+\frac{\partial M_{jk}}{\partial q_i}
\right)\dot q_k.
$$

Swap $i$ and $j$:

$$
N_{ji}
=\sum_k\left(
-\frac{\partial M_{jk}}{\partial q_i}
+\frac{\partial M_{ik}}{\partial q_j}
\right)\dot q_k
=-N_{ij}.
$$

Therefore

$$
\boxed{\dot M-2C\text{ is skew-symmetric}}.
$$

Consequently,

$$
x^T(\dot M-2C)x=0
$$

for every $x$.

## 6. Energy balance from the manipulator equation

Differentiate kinetic energy:

$$
\dot K
=\dot q^TM\ddot q
+\frac12\dot q^T\dot M\dot q.
$$

The skew identity implies

$$
\frac12\dot q^T\dot M\dot q
=\dot q^TC\dot q
=\dot q^Tc.
$$

Therefore

$$
\dot K=\dot q^T(M\ddot q+c).
$$

Since $g=\partial U/\partial q$,

$$
\dot U=g^T\dot q.
$$

Premultiply the full dynamics by $\dot q^T$:

$$
\boxed{
\frac{d}{dt}(K+U)
=u^TB^T\dot q
+\tau_{\mathrm{ext}}^T\dot q
-\tau_f^T\dot q
}.
$$

For external link wrenches,

$$
\tau_{\mathrm{ext}}^T\dot q
=\sum_i(F_i^{\mathrm{ext}})^TV_i.
$$

The energy rate equals input power minus dissipated power.

## 7. Passivity

Let the input port be generalized actuator force $\tau$ and output be $\dot q$. With storage

$$
E=K+U-U_{\min}\ge0,
$$

and passive friction satisfying

$$
\tau_f^T\dot q\ge0,
$$

the robot satisfies

$$
\dot E\le\tau^T\dot q+\sum_iF_i^TV_i.
$$

The mechanical plant is passive at its force–velocity ports. This is why damping injection and energy-based control are powerful in physical interaction.

## 8. Conservation laws

If no actuator, external, or dissipative power acts and the potential is time invariant,

$$
\dot E=0.
$$

Total mechanical energy is conserved.

Momentum conservation requires a corresponding symmetry. A free rigid body in empty space is invariant to global translation and rotation, producing linear and angular momentum conservation. A fixed-base manipulator exchanges momentum with the base, so link momentum alone is not generally conserved.

## 9. Coordinate changes

Suppose another local coordinate $s$ parameterizes the same configurations:

$$
q=\varphi(s).
$$

Let

$$
E(s)=\frac{\partial\varphi}{\partial s},
\qquad
\dot q=E(s)\dot s.
$$

Then

$$
\ddot q=E\ddot s+\dot E\dot s.
$$

Power invariance gives transformed generalized force

$$
Q_s=E^TQ_q.
$$

Kinetic energy gives

$$
\boxed{M_s=E^TM_qE}.
$$

Substituting into the dynamics gives

$$
c_s=E^T\left(M_q\dot E\dot s+c_q\right),
$$

$$
g_s=E^Tg_q.
$$

Thus $M,c,g$ coordinate values change, while physical predictions do not. The extra $M\dot E\dot s$ term shows why generalized acceleration does not transform like an ordinary vector under nonlinear coordinates.

## 10. State-space form

Let

$$
x=\begin{bmatrix}q\\v\end{bmatrix},
\qquad v=\dot q.
$$

Then

$$
\dot x=
\begin{bmatrix}
v\\
M(q)^{-1}\left(B(q)u+\tau_{\mathrm{ext}}-h(q,v)\right)
\end{bmatrix}.
$$

This is nonlinear but control affine if $B$ does not depend on $u$:

$$
\dot x=f(x)+G(x)u.
$$

Again, numerical code should solve with $M$ rather than form $M^{-1}$ explicitly.

## 11. Small-motion linearization about equilibrium

Consider a fully actuated equilibrium

$$
q=q_*,\qquad\dot q=0,\qquad\tau_*=g(q_*).
$$

Let $\delta q=q-q_*$ and $\delta\tau=\tau-\tau_*$. Since rigid-body $c$ is quadratic in velocity, its first derivative with respect to velocity vanishes at rest. With viscous friction matrix $D_f$, the first-order model is

$$
M_*\delta\ddot q+D_f\delta\dot q+K_g\delta q
=\delta\tau,
$$

where

$$
M_*=M(q_*),
\qquad
K_g=\left.\frac{\partial g}{\partial q}\right|_{q_*}.
$$

$K_g$ is gravitational stiffness. At a hanging pendulum equilibrium it contributes restoring stiffness; at an inverted equilibrium it has a destabilizing sign.

## 12. Inertial parameters and identification

A rigid body's inertial description has ten conventional parameters:

- one mass;
- three first moments $mc_x,mc_y,mc_z$;
- six independent entries of a symmetric rotational inertia.

Robot inverse dynamics are linear in a stacked inertial parameter vector:

$$
\tau=Y(q,\dot q,\ddot q)\pi.
$$

But not every parameter combination is independently observable. Mechanism constraints create a lower-dimensional base-parameter set.

Physical consistency requires more than arbitrary regression coefficients:

- mass must be positive;
- center of mass must be meaningful;
- rotational inertia must be positive definite;
- principal moments must satisfy triangle inequalities;
- all parameters must correspond to one nonnegative mass distribution.

## 13. Friction and actuator dynamics are separate modeling choices

Rigid-body dynamics do not automatically include:

- viscous friction;
- Coulomb friction and stiction;
- gearbox backlash;
- motor electrical dynamics;
- elastic transmissions;
- torque-loop delay;
- cable or hydraulic dynamics.

A common rigid-joint approximation is

$$
\tau_f=B_v\dot q+\tau_c\operatorname{sgn}(\dot q).
$$

It is nonsmooth at zero velocity and often insufficient for precision identification. Do not attribute every model residual to rigid-body inertia.

## 14. Numerical and physical validation

### 14.1 Mass matrix

- Check $\|M-M^T\|$ is near numerical precision.
- Check Cholesky factorization succeeds.
- Check random quadratic forms $z^TMz>0$.
- Compare against inverse-dynamics acceleration columns.

### 14.2 Velocity bias

- Check $c(q,0)=0$.
- Check $c(q,\alpha\dot q)\approx\alpha^2c(q,\dot q)$.
- Compare Christoffel, Newton–Euler, and inverse-dynamics versions.

### 14.3 Gravity

- Check $g=\partial U/\partial q$ by central finite differences.
- Rotate or reflect simple configurations and inspect signs.
- Check static torque balance.

### 14.4 Whole dynamics

- Verify inverse/forward dynamics round trips.
- Verify power equality for every external wrench.
- Simulate without input or friction and monitor energy.
- Test limiting cases: zero gravity, zero velocity, zero payload, locked joints.

## 15. Common conceptual mistakes

- Calling every velocity-dependent term “Coriolis” without including centrifugal effects.
- Assuming a diagonal $M$ because joints have separate motors.
- Treating $M$ as constant because physical masses are constant.
- Comparing $C$ matrices from different conventions instead of comparing $C\dot q$.
- Mixing left-side $g$ with the physical gravity load $\tau_g=-g$.
- Believing generalized force always has units of torque; a prismatic coordinate's generalized force is a force.
- Ignoring actuator mapping $B$ in floating-base or underactuated systems.
- Explicitly inverting $M$ in numerical code.

## Checkpoint exercises

1. Derive the skew-symmetry of $\dot M-2C$ component by component.
2. Derive the complete energy balance including external wrenches and friction.
3. Under $q=\varphi(s)$, derive $M_s$, $c_s$, and $Q_s$.
4. Explain why “centrifugal” and “Coriolis” are not coordinate-invariant categories.
5. Linearize a single pendulum about its hanging and inverted equilibria.
6. Design a validation suite for a dynamics implementation without relying on one reference library.

Solutions are in [[13 - Worked Problems and Solutions#Module 8B solutions]].

## Next

Continue to [[09 - Lagrangian Dynamics and the Two-Link Arm]].

