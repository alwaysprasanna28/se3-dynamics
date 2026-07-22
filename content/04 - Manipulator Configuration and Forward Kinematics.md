---
aliases:
  - Forward Kinematics with SE3
tags:
  - robotics
  - forward-kinematics
  - product-of-exponentials
  - se3
course: Robotic Manipulator Dynamics on SE(3)
module: 4
---

# Manipulator Configuration and Forward Kinematics

> [!summary]
> Forward kinematics maps joint configuration $q\in\mathcal Q$ to each link pose $T_{0i}(q)\in SE(3)$. The product-of-exponentials formula expresses this map using physically meaningful screw axes.

## 1. A manipulator is a constrained multibody system

A serial manipulator consists of rigid links connected by joints. If the base is fixed and every joint has one degree of freedom, the joint configuration is

$$
q=\begin{bmatrix}q_1&\cdots&q_n\end{bmatrix}^T.
$$

Locally $q\in\mathbb R^n$; globally revolute coordinates live on circles and may be subject to limits. Each link pose is a function

$$
T_{0i}:\mathcal Q\rightarrow SE(3).
$$

The joint coordinates enforce the mechanism constraints automatically. We do not independently integrate all link poses and then try to constrain them.

### 1.1 Modeling ingredients

For each link or joint, specify:

- a parent link;
- a joint type and motion subspace;
- a transform locating the joint relative to its parent;
- a link-fixed body frame;
- inertial parameters: mass, center of mass, and rotational inertia;
- optional collision and visual geometry.

Kinematics uses the topology and transforms. Dynamics additionally uses inertial parameters.

## 2. One-degree-of-freedom joints as exponentials

A one-DoF joint follows a one-parameter subgroup of $SE(3)$:

$$
T_J(q_i)=\exp([S_i]^\wedge q_i).
$$

Here $S_i$ is the joint's screw axis in a declared frame at a declared reference configuration.

### 2.1 Revolute joint

Let the axis direction be unit vector $\omega$ and let it pass through point $r$. A zero-pitch revolute screw is

$$
S=
\begin{bmatrix}
\omega\\
-\omega\times r
\end{bmatrix}.
$$

The scalar coordinate $q_i$ has units of radians.

### 2.2 Prismatic joint

For unit translation direction $u$,

$$
S=\begin{bmatrix}0\\u\end{bmatrix},
$$

and $q_i$ has units of length.

### 2.3 Helical joint

For pitch $h$,

$$
S=
\begin{bmatrix}
\omega\\
-\omega\times r+h\omega
\end{bmatrix}.
$$

One radian of joint displacement advances the body by $h$ units along the axis.

## 3. Product of exponentials in the space form

Choose a home configuration $q=0$. Let

$$
M=T_{0e}(0)
$$

be the end-effector home pose, and express every screw axis $S_i$ in the fixed space frame at home. Then

$$
T_{0e}(q)
=e^{[S_1]^\wedge q_1}
e^{[S_2]^\wedge q_2}
\cdots
e^{[S_n]^\wedge q_n}M.
$$

This is the **space product-of-exponentials formula**.

### 3.1 Why the order is left to right

At home, every $S_i$ is expressed in the space frame. Joint 1 moves everything downstream. Once joint 1 moves, the physical axis of joint 2 moves too; left multiplication by joint 1's exponential transports all subsequent geometry. Repeating this reasoning produces the ordered product.

### 3.2 Derivation by induction

For one joint,

$$
T(q_1)=e^{[S_1]^\wedge q_1}M.
$$

Assume the first $k-1$ joints produce

$$
T_{k-1}=e^{[S_1]^\wedge q_1}\cdots e^{[S_{k-1}]^\wedge q_{k-1}}M_{k-1}.
$$

The $k$th joint's home screw is carried by the upstream motion. Using

$$
Te^{[V]^\wedge\theta}T^{-1}
=e^{[\operatorname{Ad}_T V]^\wedge\theta},
$$

its moved-axis motion can be commuted back to the home-axis exponential, yielding one more factor on the left of the remaining home transform. Induction gives the complete product.

## 4. Product of exponentials in the body form

Express joint screws in the end-effector body frame at home. Define

$$
B_i=\operatorname{Ad}_{M^{-1}}S_i.
$$

Then

$$
T_{0e}(q)
=M e^{[B_1]^\wedge q_1}
e^{[B_2]^\wedge q_2}
\cdots
e^{[B_n]^\wedge q_n}.
$$

Both formulas represent the same map. The space form anchors motions to the base side; the body form anchors them to the tool side.

> [!warning] “Body screw” does not mean a changing numeric vector
> $B_i$ is a constant screw expressed in the end-effector frame at the home configuration. The current body Jacobian columns later depend on $q$ because downstream joints transport these home screws.

## 5. Forward kinematics for every link

For link $i$, let $M_i=T_{0i}(0)$. Only upstream joints affect it:

$$
T_{0i}(q)
=e^{[S_1]^\wedge q_1}\cdots e^{[S_i]^\wedge q_i}M_i.
$$

Its center-of-mass frame can be included through a fixed transform $T_{iC_i}$:

$$
T_{0C_i}(q)=T_{0i}(q)T_{iC_i}.
$$

These center-of-mass poses are what dynamics needs.

## 6. Recursive transform formulation

Robot-description formats usually store a fixed parent-to-joint transform and a joint motion. If $p(i)$ is the parent of link $i$, write

$$
T_{0i}=T_{0p(i)}T_{p(i),i}(q_i),
$$

with

$$
T_{p(i),i}(q_i)=T_{p(i),J_i}\exp([S_i^{J_i}]^\wedge q_i)T_{J_i,i}.
$$

This recursive tree representation is equivalent to product of exponentials and is convenient in software.

## 7. Denavit–Hartenberg versus product of exponentials

The Denavit–Hartenberg (DH) convention represents each adjacent-link transform using four parameters. It remains widespread and is compact for serial arms. Product of exponentials uses arbitrary frames and explicit screw axes.

| Feature | DH | Product of exponentials |
|---|---|---|
| Primitive | Four-parameter adjacent transform | Screw exponential and home pose |
| Frame placement | Constrained by convention | Arbitrary but declared |
| Geometric meaning | Encoded in parameter table | Axes visible directly |
| Branching trees | Awkward | Natural recursively |
| Lie-group Jacobians | Requires further derivation | Follows directly |
| Common pitfall | Mixing standard and modified DH | Expressing screws in wrong frame |

Neither method changes the physics. A correct DH model and a correct PoE model produce the same $T(q)$.

## 8. Worked example: planar 2R arm

Let link lengths be $l_1,l_2$. At home, both links point along $+x$ and both joints rotate about $+z$.

Joint 1 passes through the origin:

$$
S_1=
\begin{bmatrix}
0\\0\\1\\0\\0\\0
\end{bmatrix}.
$$

Joint 2 passes through $r_2=[l_1,0,0]^T$:

$$
S_2=
\begin{bmatrix}
0\\0\\1\\0\\-l_1\\0
\end{bmatrix}.
$$

The home tool pose is

$$
M=
\begin{bmatrix}
I&[l_1+l_2,0,0]^T\\
0&1
\end{bmatrix}.
$$

Therefore

$$
T_{0e}(q)=e^{[S_1]^\wedge q_1}e^{[S_2]^\wedge q_2}M.
$$

Expanding gives

$$
R_{0e}=R_z(q_1+q_2),
$$

$$
p_{0e}=
\begin{bmatrix}
l_1\cos q_1+l_2\cos(q_1+q_2)\\
l_1\sin q_1+l_2\sin(q_1+q_2)\\
0
\end{bmatrix}.
$$

The familiar trigonometric formula is therefore a special case of $SE(3)$ composition.

## 9. Worked example: locating a screw axis

Suppose a revolute joint axis points along

$$
\omega=\frac{1}{\sqrt2}\begin{bmatrix}1\\1\\0\end{bmatrix}
$$

and passes through

$$
r=\begin{bmatrix}0\\0\\d\end{bmatrix}.
$$

Then

$$
v=-\omega\times r
=-\frac{1}{\sqrt2}
\begin{bmatrix}
d\\-d\\0
\end{bmatrix}
=
\frac{1}{\sqrt2}
\begin{bmatrix}
-d\\d\\0
\end{bmatrix}.
$$

Check the axis: the velocity of any point $x$ under the twist is $\omega\times x+v$. At $x=r$ it is zero, so the axis point is instantaneously stationary.

## 10. Forward-kinematics implementation recipe

1. Draw the home configuration and label frames.
2. For each revolute joint, record an axis direction $\omega_i$ and any point $r_i$ on that axis, all expressed in the same home space frame.
3. Form $S_i=[\omega_i;-\omega_i\times r_i]$.
4. For a prismatic joint, form $S_i=[0;u_i]$.
5. Record every required home pose $M_i$.
6. Evaluate exponentials in joint order.
7. Test $q=0$: the result must equal $M_i$.
8. Test one joint at a time against geometric intuition.
9. Verify every rotation block satisfies $R^TR\approx I$ and $\det R\approx1$.

## 11. Common mistakes

- Using a point $r_i$ expressed in a different frame from $\omega_i$.
- Using $v=+\omega\times r$ instead of $v=-\omega\times r$.
- Reversing exponential order.
- Treating degrees as radians.
- Multiplying homogeneous transforms without frame cancellation.
- Forgetting that the home pose $M$ remains after all joint exponentials.
- Confusing a transform's translation block with a displacement expressed in the child frame.

## Checkpoint exercises

1. Derive the space screw for a revolute axis parallel to $y$ through $[a,0,c]^T$.
2. Derive the space PoE model of a planar revolute-prismatic arm.
3. For the 2R arm, verify the expanded position from the exponential formula.
4. Compute $B_1$ and $B_2$ for the planar 2R arm using $B_i=\operatorname{Ad}_{M^{-1}}S_i$.
5. Explain why only joints $1$ through $i$ appear in the pose of link $i$.

Solutions are in [[13 - Worked Problems and Solutions#Module 4 solutions]].

## Next

Continue to [[05 - Differential Kinematics and Jacobians]].
