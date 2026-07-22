---
aliases:
  - Robot Dynamics References
  - SE3 Further Reading
tags:
  - robotics
  - references
  - bibliography
  - se3
course: Robotic Manipulator Dynamics on SE(3)
module: 15
---

# References and Further Study

> [!summary]
> Start with *Modern Robotics* for a coherent first pass, use Murray–Li–Sastry for deeper screw and geometric structure, Featherstone for computational dynamics, and the MIT notes for intuition and control connections.

## Essential reading path

### 1. Lynch and Park — the primary companion

Kevin M. Lynch and Frank C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017.

- [Official book home, videos, exercises, errata, and software](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- [Free updated first-edition preprint](https://hades.mech.northwestern.edu/images/2/25/MR-v2.pdf)
- [Official video supplements](https://modernrobotics.northwestern.edu/)
- [Educational Python, MATLAB, and Mathematica library](https://github.com/NxRLab/ModernRobotics)

Best chapters for this course:

| Course topic | *Modern Robotics* chapter |
|---|---:|
| Configuration spaces | 2 |
| $SO(3)$, $SE(3)$, twists, wrenches | 3 |
| Product of exponentials | 4 |
| Jacobians, statics, singularities | 5 |
| Numerical inverse kinematics | 6 |
| Open-chain dynamics | 8 |
| Trajectories | 9 |
| Motion, force, and impedance control | 11 |

This is the closest match to the notation in these notes. It uses angular-first twists and develops body/space formulations explicitly.

### 2. Murray, Li, and Sastry — geometric foundations

Richard M. Murray, Zexiang Li, and S. Shankar Sastry, *A Mathematical Introduction to Robotic Manipulation*, CRC Press, 1994.

- [Caltech course page and syllabus](https://robotics.caltech.edu/~jwb/courses/ME115/ME115.2014.html)
- [Official book wiki](https://www.cds.caltech.edu/~murray/mlswiki/index.php?title=Main_Page)
- [Publisher page](https://www.routledge.com/A-Mathematical-Introduction-to-Robotic-Manipulation/Murray-Li-Sastry/p/book/9781138440166)

The publisher no longer permits the complete PDF to be posted on the official wiki, although some older course pages still describe it as freely available. Read Chapters 2–4 and Appendix A for rigid motion, manipulator kinematics, dynamics, control, and Lie-group foundations; Chapters 5–6 extend the treatment to multifingered hands. This text is more mathematical than *Modern Robotics* and is excellent for understanding why Lie groups and screw theory are natural rather than fashionable notation.

### 3. Featherstone — spatial vectors and fast dynamics

Roy Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008.

- [Publisher page and DOI](https://link.springer.com/book/10.1007/978-1-4899-7560-7)
- [Author's teaching materials](https://royfeatherstone.org/teaching/index.html)
- [Short-course notes on spatial vector algebra](https://royfeatherstone.org/teaching/2008/notes.pdf)
- [Spatial dynamics software](https://royfeatherstone.org/spatial/)

This is the definitive source for spatial-vector conventions, RNEA, CRBA, ABA, closed loops, and computational complexity. Read it after the $SE(3)$ and single-rigid-body notes; its notation differs from *Modern Robotics*, so translate motion and force transforms carefully.

### 4. MIT Underactuated Robotics — dynamics and control intuition

Russ Tedrake, *Underactuated Robotics* course notes.

- [Multi-body dynamics chapter](https://underactuated.mit.edu/multibody.html)
- [Course home](https://underactuated.mit.edu/)

The multibody chapter derives Lagrangian dynamics, the manipulator equation, constraints, and structural properties with strong physical and control intuition. The broader notes connect robot dynamics to nonlinear control, trajectory optimization, and contact.

## Lie groups and $SE(3)$

### Solà, Deray, and Atchuthan

Joan Solà, Jérémie Deray, and Dinesh Atchuthan, “A micro Lie theory for state estimation in robotics,” 2018.

- [arXiv paper](https://arxiv.org/abs/1812.01537)

Although written for estimation, this is one of the clearest compact references for group actions, left/right perturbations, exponential and logarithm maps, adjoints, and Lie Jacobians. Pay close attention to vector ordering and perturbation convention.

### Barfoot

Timothy D. Barfoot, *State Estimation for Robotics*, second edition, Cambridge University Press, 2024.

- [Author publication page](https://asrl.utias.utoronto.ca/~tdb/pubs.html)
- [Publisher DOI](https://doi.org/10.1017/9781009299909)

Its early chapters give a detailed matrix-Lie-group treatment with careful differentials and uncertainty. It goes beyond manipulator dynamics but is especially valuable if you need exact $SE(3)$ Jacobians for optimization, estimation, or large-error control.

## Classical manipulator modeling and control

### Siciliano, Sciavicco, Villani, and Oriolo

Bruno Siciliano, Lorenzo Sciavicco, Luigi Villani, and Giuseppe Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009.

- [Publisher page](https://link.springer.com/book/10.1007/978-1-84628-642-1)

This is an excellent conventional treatment of manipulator kinematics, Lagrangian dynamics, motion control, force control, and interaction. It complements the Lie-group presentation with standard engineering notation.

### Spong, Hutchinson, and Vidyasagar

Mark W. Spong, Seth Hutchinson, and M. Vidyasagar, *Robot Modeling and Control*, second edition, Wiley, 2020.

- [Publisher page](https://www.wiley-vch.de/en/areas-interest/engineering/robot-modeling-and-control-978-1-119-52399-4)

Use this for detailed classical robot control, Lyapunov arguments, computed torque, adaptive control, and underactuation. Verify the exact edition and accompanying resources through a library or publisher.

### Springer Handbook of Robotics

Bruno Siciliano and Oussama Khatib, editors, *Springer Handbook of Robotics*.

- [Publisher page](https://link.springer.com/book/10.1007/978-3-540-30301-5)

The kinematics, dynamics, motion-control, force-control, and redundant-manipulator chapters are strong survey references with extensive bibliographies.

## Operational space and interaction

### Khatib's operational-space formulation

Oussama Khatib, “A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation,” *IEEE Journal of Robotics and Automation*, 1987.

- [Author-hosted paper PDF](https://khatib.stanford.edu/publications/pdfs/Khatib_1987_RA.pdf)
- [Stanford publication record](https://khatib.stanford.edu/publications.html)

This is the foundational paper for end-effector dynamics, operational-space inertia, dynamic consistency, redundancy, and motion/force control.

### Hogan's impedance control papers

Neville Hogan, “Impedance Control: An Approach to Manipulation,” Parts I–III, *ASME Journal of Dynamic Systems, Measurement, and Control*, 1985.

- [MIT laboratory publication list](https://newmanlab.mit.edu/publications/)
- [Part I: Theory](https://newmanlab.mit.edu/wp-content/uploads/2017/09/1985-impedance-control-an-approach-to-manipulation-part-I-theory.pdf)
- [Part II: Implementation](https://newmanlab.mit.edu/wp-content/uploads/2017/09/1985-impedance-control-an-approach-to-manipulation-part-II-implementation.pdf)
- [Part III: Applications](https://newmanlab.mit.edu/wp-content/uploads/2017/09/1985-impedance-control-an-approach-to-manipulation-part-III-applications.pdf)

These papers established the idea of controlling the dynamic relation between motion and force rather than commanding one in isolation.

## Software documentation as a reference

### Pinocchio

Pinocchio is a high-performance rigid-body dynamics library with Lie-group-aware joint integration.

- [Dynamics algorithms: RNEA, CRBA, and ABA](https://docs.ros.org/en/ros2_packages/rolling/api/pinocchio/doc/a-features/g-dynamic.html)
- [Forward kinematics and Jacobians](https://docs.ros.org/en/jazzy/p/pinocchio/doc/a-features/f-kinematic.html)
- [Practical dynamics exercise](https://docs.ros.org/en/humble/p/pinocchio/doc/d-practical-exercises/4-dyn.html)

Use Pinocchio after implementing the core formulas once. Its configuration dimension $n_q$ can differ from velocity dimension $n_v$, especially for free-flyer and quaternion joints.

### Drake

Drake provides unusually careful multibody notation and robust dynamics tools.

- [Multibody terminology and notation](https://drake.mit.edu/doxygen_cxx/group__multibody__notation.html)
- [Spatial vectors](https://drake.mit.edu/doxygen_cxx/group__multibody__spatial__vectors.html)
- [Spatial inertia](https://drake.mit.edu/doxygen_cxx/group__multibody__spatial__inertia.html)
- [MultibodyPlant documentation](https://drake.mit.edu/pydrake/pydrake.multibody.plant.html)

The notation pages are worth reading even if you never use Drake. They insist that spatial quantities identify the body, point, measured-in frame, and expressed-in frame.

### Modern Robotics code

The [Modern Robotics repository](https://github.com/NxRLab/ModernRobotics) prioritizes readability and correspondence with textbook formulas. It is ideal for checking a first implementation, though it is not intended as a production-grade high-performance dynamics library.

## Recommended order by goal

### I want a strong first course

1. This note set.
2. *Modern Robotics*, Chapters 2–6 and 8.
3. Its video supplements and exercises.
4. MIT Underactuated multi-body dynamics chapter.

### I want to implement fast dynamics

1. [[07 - Single Rigid Body Dynamics on SE3]].
2. [[08A - Deriving M c and g from SE3 Link Mechanics]].
3. [[10 - Recursive Rigid-Body Dynamics Algorithms]].
4. Featherstone's spatial-vector short course.
5. Featherstone's book, Chapters 2–7.
6. Compare against Pinocchio.

### I want to understand manipulator dynamics from first principles

1. [[07A - From Particles to Constrained Mechanisms]].
2. [[08 - Manipulator Dynamics from Link Geometry]].
3. [[08A - Deriving M c and g from SE3 Link Mechanics]].
4. [[08B - Structure and Physical Meaning of Manipulator Dynamics]].
5. [[09 - Lagrangian Dynamics and the Two-Link Arm]].
6. [[09A - Additional Worked Dynamics Examples]].

### I want geometric control or estimation

1. [[02 - Rotations and SO3]] and [[03 - Rigid Motions and SE3]].
2. Solà et al.
3. Murray–Li–Sastry.
4. Barfoot for exact differentials and uncertainty.
5. Research papers whose left/right error convention matches your application.

### I want manipulation and contact control

1. [[10A - Constraints Contacts and Underactuation]].
2. [[11 - Operational-Space Dynamics]].
3. [[12 - Control on Joint Space and SE3]].
4. Khatib's operational-space paper.
5. Hogan's impedance-control series.
6. *Springer Handbook of Robotics* chapters on force control and redundant manipulators.

## Convention crosswalk

Before using any source, record:

| Question | This course |
|---|---|
| Transform meaning | $T_{ab}$ maps $b$ coordinates to $a$ |
| Twist order | angular, then linear |
| Wrench order | moment, then force |
| Body twist | $(T^{-1}\dot T)^\vee$ |
| Space twist | $(\dot TT^{-1})^\vee$ |
| Joint gravity convention | $M\ddot q+C\dot q+g=\tau$ |
| External wrench sign | wrench on robot is added as $J^TF_{\mathrm{ext}}$ |

Then write the source's convention beside it. Many apparent contradictions disappear immediately.

## Research-level extensions

After mastering the course, good next topics are:

- variational integrators on Lie groups;
- centroidal momentum dynamics for humanoids;
- constrained and contact dynamics with complementarity;
- flexible-joint and series-elastic actuator dynamics;
- inertial-parameter identification and base parameters;
- passivity, energy tanks, and variable impedance;
- geometric mechanics and reduction;
- whole-body optimization and hierarchical inverse dynamics;
- differentiable rigid-body dynamics;
- trajectory optimization directly on manifolds.

## Citation note

The equations in this course are standard results synthesized and re-derived from the references above. When using them in academic work, cite the original or authoritative source appropriate to the claim: Lynch and Park or Murray–Li–Sastry for $SE(3)$ manipulator kinematics, Featherstone for recursive spatial algorithms, Khatib for operational space, and Hogan for impedance control.
