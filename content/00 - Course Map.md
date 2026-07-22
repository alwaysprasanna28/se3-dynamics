---
aliases:
  - Robotic Manipulator Dynamics on SE(3)
  - SE3 Robot Dynamics Course
tags:
  - robotics
  - kinematics
  - dynamics
  - lie-groups
  - course
status: complete
---

# Robotic Manipulator Dynamics on $SE(3)$

> [!abstract] What this course does
> This course develops rigid-body motion, manipulator kinematics, and manipulator dynamics from first principles. Its central language is the Lie group $SE(3)$, which represents a three-dimensional rigid pose without singular orientation coordinates. The course ends with computational dynamics, operational-space dynamics, and geometric control.

> [!map] Interactive visual roadmap
> Open [[00A - Interactive Course Map.excalidraw|Interactive Excalidraw Course Map]] to navigate the complete course by clicking its color-coded lesson cards.

## The central idea

A pose is not merely six unrelated numbers. Position and orientation together form a geometric object:

$$
T =
\begin{bmatrix}
R & p\\
0 & 1
\end{bmatrix}
\in SE(3).
$$

Here $R\in SO(3)$ is an orientation and $p\in\mathbb R^3$ is a position. The set $SE(3)$ is simultaneously:

- a set of rigid transformations;
- a group, because transformations can be composed and inverted;
- a smooth six-dimensional manifold;
- a configuration space for one free rigid body.

For a fixed-base $n$-joint manipulator, however, the complete configuration is generally $q\in\mathcal Q$, not a single element of $SE(3)$. The forward-kinematics map sends joint configurations to link poses:

$$
T_{0i}:\mathcal Q\rightarrow SE(3),
\qquad
q\mapsto T_{0i}(q).
$$

The manipulator dynamics evolve on the tangent bundle $T\mathcal Q$. The geometry of $SE(3)$ enters because every link has an $SE(3)$ pose, every link velocity is a twist in $\mathfrak{se}(3)$, every applied load is a wrench in $\mathfrak{se}(3)^*$, and the joint-space inertia is assembled by pulling link kinetic energies back through link Jacobians.

> [!important] One-sentence mental model
> $SE(3)$ describes **where each rigid body is**; $\mathfrak{se}(3)$ describes **how it is moving instantaneously**; joint coordinates describe **which poses the mechanism is allowed to have**; dynamics describe **how forces and torques change that motion**.

## Learning outcomes

After completing the course, you should be able to:

1. interpret rotation and homogeneous-transformation matrices with correct frame semantics;
2. derive and use the hat, vee, exponential, logarithm, adjoint, and small-adjoint maps;
3. distinguish space and body twists, Jacobians, accelerations, and wrenches;
4. derive forward kinematics using the product of exponentials;
5. derive velocity and acceleration kinematics, including $\dot J\dot q$;
6. derive a rigid body's Newton–Euler equation in spatial-vector form;
7. derive the manipulator equation from Newton's laws, D'Alembert's principle, link spatial mechanics, and Euler–Lagrange equations;
8. interpret and derive $M(q)$, $c(q,\dot q)$, $g(q)$, constraint forces, and actuator mappings;
9. understand RNEA, CRBA, and ABA and their computational roles;
10. formulate bilateral constraints, unilateral contact, impact, and underactuated dynamics;
11. transform joint-space dynamics into operational-space dynamics;
12. formulate pose error and impedance control intrinsically on $SE(3)$;
13. check equations using units, power invariance, energy, and limiting cases.

## Course map

| Order | Note | Main question |
|---:|---|---|
| 0 | This note | How should I study the material? |
| 0A | [[00A - Interactive Course Map.excalidraw\|Interactive Course Map]] | How do the topics, prerequisites, and study stages fit together? |
| 1 | [[01 - Mathematical and Mechanical Foundations]] | What mathematical objects and mechanics do we need? |
| 2 | [[02 - Rotations and SO3]] | What is a three-dimensional orientation? |
| 3 | [[03 - Rigid Motions and SE3]] | What does $SE(3)$ mean, and why is it useful? |
| 4 | [[04 - Manipulator Configuration and Forward Kinematics]] | How do joint coordinates produce link poses? |
| 5 | [[05 - Differential Kinematics and Jacobians]] | How do joint rates produce twists and wrenches? |
| 6 | [[06 - Acceleration Kinematics and Inverse Kinematics]] | How do poses, twists, and accelerations track targets? |
| 7 | [[07 - Single Rigid Body Dynamics on SE3]] | How are momentum and Newton–Euler equations written geometrically? |
| 7A | [[07A - From Particles to Constrained Mechanisms]] | How does Newton's law become a generalized-coordinate equation? |
| 8 | [[08 - Manipulator Dynamics from Link Geometry]] | How do link inertias combine into robot dynamics? |
| 8A | [[08A - Deriving M c and g from SE3 Link Mechanics]] | How does link Newton–Euler produce $M$, $c$, and $g$ explicitly? |
| 8B | [[08B - Structure and Physical Meaning of Manipulator Dynamics]] | What do the dynamics terms mean, and what identities must they satisfy? |
| 9 | [[09 - Lagrangian Dynamics and the Two-Link Arm]] | How does the familiar manipulator equation arise from energy? |
| 9A | [[09A - Additional Worked Dynamics Examples]] | How do the terms appear in prismatic, pendulum, and polar robots? |
| 10 | [[10 - Recursive Rigid-Body Dynamics Algorithms]] | How are inverse and forward dynamics computed efficiently? |
| 10A | [[10A - Constraints Contacts and Underactuation]] | How do contacts, loops, impacts, and missing actuators change the equations? |
| 11 | [[11 - Operational-Space Dynamics]] | What inertia and forces does the end effector feel? |
| 12 | [[12 - Control on Joint Space and SE3]] | How do we control pose and interaction? |
| 13 | [[13 - Worked Problems and Solutions]] | Can I perform the calculations myself? |
| 14 | [[14 - Notation and Formula Sheet]] | What are the essential formulas and conventions? |
| 15 | [[15 - References and Further Study]] | What should I read or watch next? |

## Suggested twelve-week schedule

| Week | Material | Deliverable |
|---:|---|---|
| 1 | Notes 1–2 | Derive Rodrigues' formula and implement $\operatorname{Exp}_{SO(3)}$ |
| 2 | Note 3 | Transform points, twists, and wrenches between frames |
| 3 | Note 4 | Derive product-of-exponentials kinematics for a 2R and a 3R arm |
| 4 | Notes 5–6 | Derive a Jacobian and solve inverse kinematics numerically |
| 5 | Notes 7–7A | Derive rigid-body Newton–Euler and the particle D'Alembert equation |
| 6 | Notes 8–8A | Derive $M$, $c$, and $g$ from link Jacobians and spatial inertias |
| 7 | Notes 8B–9 | Prove the energy identities and derive complete planar 2R dynamics |
| 8 | Notes 9A–10 | Derive polar-robot dynamics and hand-execute an RNEA pass |
| 9 | Note 10A | Solve a constrained forward-dynamics and impact problem |
| 10 | Note 11 | Compute $\Lambda$, $\bar J$, and a dynamically consistent nullspace |
| 11 | Note 12 | Design and analyze a pose controller on $SE(3)$ |
| 12 | Notes 13–15 | Complete a capstone derivation and compare with reference software |

## How to use these notes

1. Read definitions slowly and reproduce every derivation with pen and paper.
2. Treat frame subscripts as part of the mathematics, not decoration.
3. After each module, solve its checkpoint before opening [[13 - Worked Problems and Solutions]].
4. Implement the hat, exponential, logarithm, adjoint, and Jacobian operations yourself once.
5. Then compare with the educational [Modern Robotics software](https://github.com/NxRLab/ModernRobotics) or a production library such as Pinocchio or Drake.
6. Use [[14 - Notation and Formula Sheet]] while working problems.

> [!tip] Active recall prompts
> Close the note and answer: What space does this quantity belong to? What frame is it expressed in? What point is a spatial quantity taken about? How does it transform? Does the equation preserve units and power?

## Prerequisites

The notes rebuild the required foundations, but the following are useful:

- matrix multiplication, transpose, inverse, rank, eigenvalues;
- derivatives and ordinary differential equations;
- Newton's laws and basic rotational mechanics;
- basic Python, MATLAB, Julia, or C++ for optional implementations.

## Conventions used throughout

This course deliberately fixes one convention because robotics sources often differ.

- $T_{ab}$ is the pose of frame $\{b\}$ relative to frame $\{a\}$ and maps coordinates from $b$ to $a$: $p_a=T_{ab}p_b$ in homogeneous coordinates.
- Twists are ordered angular first: $V=[\omega^T\;v^T]^T$.
- Wrenches are ordered moment first: $F=[m^T\;f^T]^T$.
- The hat map sends $V$ to $[V]^\wedge\in\mathfrak{se}(3)$.
- A body twist is $[V^b]^\wedge=T^{-1}\dot T$.
- A space twist is $[V^s]^\wedge=\dot T T^{-1}$.
- $\operatorname{Ad}_T$ is the $6\times6$ group adjoint; $\operatorname{ad}_V$ is the $6\times6$ Lie-algebra adjoint.
- Gravity acceleration is denoted $g_0\in\mathbb R^3$; the generalized gravity term on the left side of the manipulator equation is $g(q)=\partial U/\partial q$.

When reading another source, never copy a formula until you have reconciled its twist ordering, transform direction, and frame convention.

## Dependency graph

```mermaid
flowchart LR
    A[Linear algebra and mechanics] --> B[SO(3)]
    B --> C[SE(3)]
    C --> D[Forward kinematics]
    D --> E[Jacobians]
    E --> F[Acceleration and IK]
    C --> G[Single-body spatial dynamics]
    A --> P[Particles and virtual work]
    E --> H[Manipulator dynamics]
    G --> H
    P --> H
    H --> I[Recursive algorithms]
    H --> Q[Constraints and contact]
    H --> J[Operational-space dynamics]
    Q --> J
    F --> K[Geometric control]
    J --> K
```

## Mastery checklist

- [ ] I can explain why $SO(3)$ is not a vector space.
- [ ] I can explain the semidirect-product structure of $SE(3)$.
- [ ] I can convert body twists to space twists using $\operatorname{Ad}_T$.
- [ ] I can explain why a wrench transforms by the inverse transpose.
- [ ] I can derive the product-of-exponentials formula.
- [ ] I can construct a space or body Jacobian column by column.
- [ ] I can explain why $J^T F$ is a joint torque using virtual power.
- [ ] I can derive generalized dynamics from D'Alembert's principle.
- [ ] I can derive $M(q)=\sum_iJ_i^T\mathcal G_iJ_i$.
- [ ] I can derive $c(q,\dot q)$ from link Newton–Euler equations.
- [ ] I can explain the physical meaning of $C(q,\dot q)\dot q$.
- [ ] I can state and use the skew-symmetry property of $\dot M-2C$.
- [ ] I can derive the saddle-point equations for a constrained mechanism.
- [ ] I can distinguish bilateral constraints, unilateral contact, and impact.
- [ ] I can distinguish inverse dynamics, forward dynamics, and simulation.
- [ ] I can compute operational-space inertia and interpret it physically.
- [ ] I can define pose error with $\operatorname{Log}(T^{-1}T_d)$.

## Start here

Continue to [[01 - Mathematical and Mechanical Foundations]].
