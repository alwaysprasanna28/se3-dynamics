---
aliases:
  - Geometric Robot Control
  - Manipulator Control on SE3
tags:
  - robotics
  - control
  - impedance-control
  - se3
course: Robotic Manipulator Dynamics on SE(3)
module: 12
---

# Control on Joint Space and $SE(3)$

> [!summary]
> Joint controllers regulate $q$. Task controllers regulate an end-effector pose or wrench. Geometric controllers define pose error through group composition and logarithms rather than subtracting Euler angles.

## 1. Control objectives and levels

A real robot often has nested loops:

1. motor current loop;
2. joint torque or velocity loop;
3. joint-space or task-space motion controller;
4. trajectory generator, planner, or policy.

The analysis below treats $\tau$ as the commanded generalized joint torque. Hardware may expose only position or velocity commands, in which case the low-level proprietary dynamics alter what can be implemented.

## 2. Joint-space PD with gravity compensation

Let

$$
e=q-q_d,
$$

with constant $q_d$. Choose

$$
\tau=g(q)-K_pe-K_d\dot q,
$$

where $K_p,K_d\succ0$. Closed-loop dynamics are

$$
M\ddot q+C\dot q+K_d\dot q+K_pe=0.
$$

Use Lyapunov candidate

$$
\mathcal V=\frac12\dot q^TM(q)\dot q
+\frac12e^TK_pe.
$$

Using the skew property,

$$
\dot{\mathcal V}=-\dot q^TK_d\dot q\le0.
$$

Under standard conditions, the desired equilibrium is asymptotically stable.

> [!note]
> Gravity compensation alone, $\tau=g(q)$, makes stationary configurations equilibria but does not select $q_d$. The stiffness term selects the configuration.

## 3. Computed-torque control

For a trajectory $q_d(t)$, define

$$
e=q-q_d,
\qquad
\dot e=\dot q-\dot q_d.
$$

Choose reference acceleration

$$
\ddot q_r=\ddot q_d-K_d\dot e-K_pe.
$$

Command

$$
\tau=M(q)\ddot q_r+c(q,\dot q)+g(q).
$$

With an exact model,

$$
\ddot e+K_d\dot e+K_pe=0.
$$

This feedback linearization produces simple nominal error dynamics. In practice, model mismatch, friction, delays, and torque saturation prevent exact cancellation. Robust feedback and bounded commands remain necessary.

## 4. Task-space kinematic control

Let the current and desired poses be $T,T_d\in SE(3)$. Define body error

$$
e_b=\operatorname{Log}(T^{-1}T_d)^\vee.
$$

For a fixed target and small error, a velocity command is

$$
\dot q=J_b^+K e_b.
$$

This moves along the local pose-error direction. Add desired body twist, damping, nullspace objectives, and constraints as developed in [[06 - Acceleration Kinematics and Inverse Kinematics]].

## 5. Orientation potential and topology

For current $R$ and desired $R_d$, a smooth potential is

$$
\Psi(R,R_d)=\frac12\operatorname{tr}(I-R_d^TR).
$$

A corresponding current-minus-desired body error is

$$
e_R=\frac12(R_d^TR-R^TR_d)^\vee.
$$

For $R=R_d\exp([\delta\phi]^\wedge)$ with small $\delta\phi$,

$$
e_R\approx\delta\phi.
$$

A restoring body moment uses $-K_Re_R$.

No smooth time-invariant feedback can give one globally asymptotically stable equilibrium on $SO(3)$ without additional undesired equilibria or discontinuity. Smooth geometric attitude controllers are typically **almost globally** stable; the problematic set includes $180^\circ$ attitude errors.

The log error

$$
\phi=\operatorname{Log}(R^TR_d)^\vee
$$

is locally direct but has branch ambiguity at $\pi$.

## 6. Pose impedance on $SE(3)$

Choose body error pointing from current to desired:

$$
e=\operatorname{Log}(T^{-1}T_d)^\vee.
$$

Let $V$ be current body twist. Transport the desired twist into the same body coordinates and define the current-minus-desired velocity error

$$
e_V=V-\widetilde V_d.
$$

Locally, a desired body wrench is

$$
F_{mathrm{cmd}}=K e-D e_V+F_{mathrm{ff}},
$$

with signs chosen so that $K e$ restores toward $T_d$ and $-De_V$ opposes relative motion.

Map it to joint torque:

$$
\tau=J_b(q)^TF_{mathrm{cmd}}+g(q)+N^T\tau_0.
$$

This is a Jacobian-transpose impedance law. Adding full operational-space cancellation gives

$$
F=\Lambda\dot V_r+\eta,
$$

where $\dot V_r$ contains feedforward and pose/twist feedback.

### 6.1 A subtle point about logarithmic stiffness

For finite error, the gradient of

$$
U_e=\frac12e^TKe
$$

with $e=\operatorname{Log}(T^{-1}T_d)^\vee$ is not simply $Ke$; the differential of the logarithm introduces a Lie-Jacobian transpose. The simple wrench $Ke$ is a local approximation unless the controller is derived with the exact differential.

For small tracking errors this approximation is common. For a formal global or large-error proof, derive the wrench from a declared group potential and its exact variation.

## 7. Passivity viewpoint

For the robot,

$$
\dot E=\tau^T\dot q+F_{\mathrm{ext}}^TV
$$

in the ideal frictionless case. A damping wrench $F_d=-DV$ dissipates

$$
P_d=F_d^TV=-V^TDV\le0.
$$

If a stiffness wrench is the negative gradient of a bounded-below potential, it stores rather than creates energy. This energy view is especially important in physical interaction, uncertain environments, and delayed control.

Model-canceling operational-space control may have excellent tracking but can be less robustly passive than a lower-gain impedance law.

## 8. Impedance versus admittance

An impedance controller maps motion error to commanded wrench:

$$
F_{mathrm{cmd}}=K e-D e_V.
$$

It is natural for torque-controlled, low-friction robots.

An admittance controller maps measured external wrench to a desired motion through a virtual dynamic system:

$$
M_d\dot V_d+D_dV_d+K_de_d=F_{\mathrm{ext}}.
$$

It is often used when a reliable position/velocity inner loop is available. Sensor noise, delay, and saturation affect their safety differently.

## 9. Force and hybrid motion–force control

During constrained contact, some task directions should regulate motion and others force. Let selection matrices $S_m$ and $S_f$ describe complementary task subspaces. A schematic hybrid command is

$$
F_{mathrm{cmd}}
=S_mF_{mathrm{motion}}+S_fF_{mathrm{force}}.
$$

The selections must be defined in a meaningful contact frame and account for the environment constraint geometry. Abrupt contact changes, friction, and uncertain surface normals make a fixed selection fragile; compliant transition and wrench limits are important.

## 10. Redundancy and posture control

For an operational task,

$$
\tau=J^TF+N^T\tau_{mathrm{posture}}.
$$

A posture torque may be

$$
\tau_{mathrm{posture}}
=-K_{p0}(q-q_0)-K_{d0}\dot q.
$$

Use the dynamically consistent torque projector from [[11 - Operational-Space Dynamics]] when task decoupling matters. Joint limits are safety constraints, not merely soft posture preferences; enforce them with constraint-aware control where possible.

## 11. Gain selection

For a scalar target model

$$
m\ddot e+d\dot e+ke=0,
$$

natural frequency and damping ratio are

$$
\omega_n=\sqrt{\frac{k}{m}},
\qquad
\zeta=\frac{d}{2\sqrt{mk}}.
$$

Critical damping is $\zeta=1$, or

$$
d=2\sqrt{mk}.
$$

For decoupled desired operational dynamics with diagonal desired inertia and stiffness,

$$
D=2Z(KM_d)^{1/2}
$$

under commuting/diagonal assumptions, with damping-ratio matrix $Z$. For general matrices, “take elementwise critical damping” is not coordinate invariant; modal or matrix methods are safer.

## 12. Saturation and anti-windup

Physical robots have limits:

$$
\tau_{\min}\le\tau\le\tau_{\max},
$$

as well as velocity, acceleration, power, temperature, and workspace constraints. Once torque saturates, computed-torque cancellation and nullspace decoupling no longer hold.

Good practice includes:

- bounded pose-error and wrench maps;
- rate limits on desired pose, twist, and gains;
- torque and power-aware trajectory scaling;
- anti-windup for integral action;
- explicit collision and joint-limit constraints;
- watchdogs and tested stop behavior.

## 13. Discrete-time considerations

Continuous-time stability does not automatically survive arbitrary sample time. Practical checks include:

- controller update rate relative to desired bandwidth;
- latency and jitter;
- filtered velocity and wrench measurements;
- discrete-time energy injection by delay;
- manifold-preserving pose updates;
- numerical conditioning near singularity.

Increase gains only after measuring the complete closed-loop response, not from the continuous model alone.

## 14. A complete geometric-control workflow

1. Declare $T_{0e}$ semantics and body/space convention.
2. Compute $T(q)$ and matching $J(q)$.
3. Construct $E=T^{-1}T_d$ or $T_dT^{-1}$ consistently.
4. Compute $e=\operatorname{Log}(E)^\vee$ with robust branches.
5. Transport desired twists and wrenches into the same frame.
6. Choose kinematic, impedance, or inverse-dynamics control according to hardware and contact needs.
7. Add nullspace behavior with the correct projector.
8. enforce torque, velocity, joint, and workspace constraints;
9. verify energy and sign conventions in simulation;
10. validate at low gains and low speed before increasing performance.

## Checkpoint exercises

1. Prove stability of joint PD plus gravity compensation using the stated Lyapunov function.
2. Derive the computed-torque error equation.
3. Compare the trace orientation error with the logarithmic error near $180^\circ$.
4. Explain why $Ke$ is only locally the exact gradient wrench for $e=\operatorname{Log}(T^{-1}T_d)^\vee$.
5. Derive the power dissipated by a body damping wrench.
6. Explain why task and nullspace decoupling can fail under torque saturation.

Solutions are in [[13 - Worked Problems and Solutions#Module 12 solutions]].

## Next

Use [[13 - Worked Problems and Solutions]] for practice, [[14 - Notation and Formula Sheet]] for review, and [[15 - References and Further Study]] to go deeper.
