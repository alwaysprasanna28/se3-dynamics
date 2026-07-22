---
aliases:
  - SE3
  - Rigid Motions and SE(3)
tags:
  - robotics
  - rigid-body-motion
  - lie-groups
  - se3
course: Robotic Manipulator Dynamics on SE(3)
module: 3
---

# Rigid Motions and $SE(3)$

> [!summary]
> $SE(3)$ is the group of orientation-preserving rigid motions in three dimensions. A pose $T\in SE(3)$ combines orientation and position; a twist $V\in\mathbb R^6$ represents an instantaneous rigid motion; a wrench $F\in\mathbb R^6$ represents a generalized rigid-body load.

## 1. Why position plus orientation is not just $\mathbb R^6$

A rigid pose has three translational and three rotational degrees of freedom, but “six degrees of freedom” does not imply that the space is the vector space $\mathbb R^6$.

Translations add, while rotations compose by multiplication. Translating and rotating do not commute: rotating about one origin and then translating is generally different from translating and then rotating. The correct structure is the special Euclidean group

$$
SE(3)=SO(3)\ltimes\mathbb R^3,
$$

a semidirect product. “Semi” signals that rotations act on translations.

## 2. Homogeneous transformation matrices

A pose of frame $\{b\}$ relative to frame $\{a\}$ is

$$
T_{ab}=
\begin{bmatrix}
R_{ab}&p_{ab}^a\\
0_{1\times3}&1
\end{bmatrix}\in SE(3).
$$

$R_{ab}$ gives the orientation of $b$ relative to $a$. The vector $p_{ab}^a$ points from the origin of $a$ to the origin of $b$, expressed in $a$.

For a physical point $P$ with coordinates $p_b$ in frame $b$, define its homogeneous coordinates $\bar p_b=[p_b^T\;1]^T$. Then

$$
\bar p_a=T_{ab}\bar p_b,
$$

or

$$
p_a=R_{ab}p_b+p_{ab}^a.
$$

The last homogeneous coordinate converts an affine map into matrix multiplication.

## 3. Definition and group operations

$$
SE(3)=\left\{
\begin{bmatrix}R&p\\0&1\end{bmatrix}
\middle|R\in SO(3),\ p\in\mathbb R^3
\right\}.
$$

Composition is

$$
T_1T_2=
\begin{bmatrix}
R_1R_2&p_1+R_1p_2\\
0&1
\end{bmatrix}.
$$

The inverse is

$$
T^{-1}=
\begin{bmatrix}
R^T&-R^Tp\\
0&1
\end{bmatrix}.
$$

Frame composition follows

$$
T_{ac}=T_{ab}T_{bc},
\qquad
T_{ba}=T_{ab}^{-1}.
$$

> [!example] Reading the translation in a product
> In $T_1T_2$, the displacement $p_2$ is expressed in frame 2's parent coordinates, so $R_1p_2$ first expresses it in frame 1's parent coordinates. That is why the translation block is $p_1+R_1p_2$, not merely $p_1+p_2$.

## 4. The Lie algebra $\mathfrak{se}(3)$

The tangent space at the identity is

$$
\mathfrak{se}(3)=\left\{
\begin{bmatrix}
[\omega]_\times&v\\
0&0
\end{bmatrix}
\middle|\omega,v\in\mathbb R^3
\right\}.
$$

For the six-vector

$$
V=
\begin{bmatrix}\omega\\v\end{bmatrix},
$$

define

$$
[V]^\wedge=
\begin{bmatrix}
[\omega]_\times&v\\
0&0
\end{bmatrix},
\qquad
([V]^\wedge)^\vee=V.
$$

$V$ is called a **twist**. It packages angular and linear velocity, but the precise meaning of $v$ depends on whether the twist is body- or space-trivialized and which origin it refers to.

## 5. Body and space twists

Let $T=T_{ab}(t)$. The body twist and space twist are

$$
[V^b]^\wedge=T^{-1}\dot T,
$$

$$
[V^s]^\wedge=\dot T T^{-1}.
$$

Therefore

$$
\dot T=T[V^b]^\wedge=[V^s]^\wedge T.
$$

With $T=[R,p;0,1]$,

$$
V^b=
\begin{bmatrix}
\omega^b\\
R^T\dot p
\end{bmatrix},
$$

whereas

$$
V^s=
\begin{bmatrix}
\omega^s\\
\dot p-\omega^s\times p
\end{bmatrix}.
$$

The lower part of a space twist is not generally the velocity of the body-frame origin. The actual velocity of a point at space position $x$ under a space twist is

$$
\dot x=\omega^s\times x+v^s.
$$

At the body origin $x=p$,

$$
\dot p=\omega^s\times p+v^s.
$$

By contrast, the lower part of the body twist is the origin's velocity expressed in the body frame: $v^b=R^T\dot p$.

## 6. The adjoint representation

The group adjoint changes twist coordinates between related frames. From

$$
[V^s]^\wedge=T[V^b]^\wedge T^{-1},
$$

we obtain

$$
V^s=\operatorname{Ad}_T V^b,
$$

where, for angular-first ordering,

$$
\operatorname{Ad}_T=
\begin{bmatrix}
R&0\\
[p]_\times R&R
\end{bmatrix}.
$$

Useful properties are

$$
\operatorname{Ad}_{T_1T_2}
=\operatorname{Ad}_{T_1}\operatorname{Ad}_{T_2},
$$

$$
\operatorname{Ad}_{T^{-1}}=\operatorname{Ad}_T^{-1}.
$$

The off-diagonal term $[p]_\times R$ accounts for the change of reference point. Pure angular motion about one origin can include a linear component when represented about another origin.

## 7. The small adjoint and twist cross product

For $V=[\omega^T\;v^T]^T$, define

$$
\operatorname{ad}_V=
\begin{bmatrix}
[\omega]_\times&0\\
[v]_\times&[\omega]_\times
\end{bmatrix}.
$$

The Lie bracket is

$$
[V_1,V_2]_{\mathfrak{se}(3)}
=\operatorname{ad}_{V_1}V_2.
$$

At the matrix level,

$$
\left[ [V_1]^\wedge, [V_2]^\wedge\right]
=\left[\operatorname{ad}_{V_1}V_2\right]^\wedge.
$$

The identity

$$
\operatorname{Ad}_{\exp([V]^\wedge)}
=\exp(\operatorname{ad}_V)
$$

connects the group and algebra adjoints.

## 8. Exponential map and screw motion

Let $\xi=[\omega^T\;v^T]^T$ be a constant twist and $\theta$ a scalar displacement. Then

$$
T(\theta)=\exp([\xi]^\wedge\theta)\in SE(3).
$$

### 8.1 Prismatic motion

If $\omega=0$ and $\|v\|=1$,

$$
\exp([\xi]^\wedge\theta)
=
\begin{bmatrix}
I&v\theta\\
0&1
\end{bmatrix}.
$$

### 8.2 Revolute or helical motion

For $\|\omega\|=1$,

$$
\exp([\xi]^\wedge\theta)
=
\begin{bmatrix}
\exp([\omega]_\times\theta)&G(\theta)v\\
0&1
\end{bmatrix},
$$

where

$$
G(\theta)
=I\theta+(1-\cos\theta)[\omega]_\times
+(\theta-\sin\theta)[\omega]_\times^2.
$$

For a zero-pitch rotation about an axis through point $q$ with direction $\omega$,

$$
v=-\omega\times q.
$$

For a screw of pitch $h$,

$$
v=-\omega\times q+h\omega.
$$

Thus every one-parameter rigid motion is a screw: rotation about an axis combined with translation along that axis. Pure translation is the infinite-pitch limiting case.

## 9. Logarithm map and pose coordinates

For

$$
T=\begin{bmatrix}R&p\\0&1\end{bmatrix},
$$

compute $[\phi]_\times=\operatorname{Log}(R)$ with $\phi=\theta\omega$. In this course's angular-first ordering, write the exponential coordinate as

$$
\eta=\begin{bmatrix}\phi\\\rho\end{bmatrix},
\qquad
T=\exp([\eta]^\wedge).
$$

Translation and rotation are coupled through the $SO(3)$ left Jacobian:

$$
p=J_l(\phi)\rho,
\qquad
\rho=J_l(\phi)^{-1}p.
$$

Therefore

$$
\operatorname{Log}_{SE(3)}(T)^\vee
=
\begin{bmatrix}
\phi\\
J_l(\phi)^{-1}p
\end{bmatrix}.
$$

The lower logarithm coordinate is generally not equal to raw translation $p$ unless rotation is zero or small.

> [!warning] Notation varies
> Some authors order twists as $[v;\omega]$, call the translational exponential coordinate $\rho$, or attach $\theta$ to the screw axis differently. Translate conventions before using formulas.

## 10. Pose error on $SE(3)$

For current pose $T$ and desired pose $T_d$, define the right/body error

$$
E_b=T^{-1}T_d,
\qquad
e_b=\operatorname{Log}(E_b)^\vee,
$$

or the left/space error

$$
E_s=T_dT^{-1},
\qquad
e_s=\operatorname{Log}(E_s)^\vee.
$$

Since $E_s=TE_bT^{-1}$,

$$
e_s=\operatorname{Ad}_T e_b
$$

on a consistent logarithm branch.

The logarithm error respects rigid-motion geometry. Simply stacking $p_d-p$ with Euler-angle subtraction does not.

### 10.1 Error velocity

The body twist of $E_b=T^{-1}T_d$ is

$$
V_{E_b}^b
=V_d^b-\operatorname{Ad}_{E_b^{-1}}V^b.
$$

If $e_b=\operatorname{Log}(E_b)^\vee$, its derivative is not generally equal to $V_{E_b}^b$. It is related through the right Jacobian of $SE(3)$:

$$
V_{E_b}^b=J_r^{SE(3)}(e_b)\dot e_b.
$$

Near zero error, $J_r^{SE(3)}(e_b)\approx I$, so $\dot e_b\approx V_{E_b}^b$.

## 11. Wrenches and duality

A wrench is

$$
F=\begin{bmatrix}m\\f\end{bmatrix}\in\mathbb R^6,
$$

where $m$ is moment and $f$ is force, expressed in the same frame and with the moment taken about the frame origin.

Twist–wrench pairing gives instantaneous power:

$$
P=F^TV=m^T\omega+f^Tv.
$$

If twists transform as

$$
V_a=\operatorname{Ad}_{T_{ab}}V_b,
$$

power invariance requires

$$
F_a=\operatorname{Ad}_{T_{ab}}^{-T}F_b.
$$

Equivalently,

$$
F_b=\operatorname{Ad}_{T_{ab}}^TF_a.
$$

This inverse-transpose rule is not arbitrary. It is forced by

$$
F_a^TV_a=F_b^TV_b.
$$

The same duality later gives $\tau=J^TF$.

## 12. Acceleration on $SE(3)$

If $V^b=(T^{-1}\dot T)^\vee$, its coordinate derivative $\dot V^b$ is commonly used as body spatial acceleration when the same body frame and origin convention are maintained. Transport between moving frames introduces $\operatorname{ad}$ terms.

For a twist transported by a time-varying transform,

$$
V_a=\operatorname{Ad}_{T_{ab}}V_b,
$$

differentiation yields a transport term. Schematically,

$$
\dot V_a
=\operatorname{Ad}_{T_{ab}}\dot V_b
+\operatorname{ad}_{V_{ab}^a}V_a,
$$

with the exact sign depending on which relative twist and transform direction are defined. In practice, derive it once from $\dot T=T[V^b]^\wedge$ under your declared convention rather than memorizing an unlabelled equation.

## 13. Integrating a pose

For body twist input,

$$
T_{k+1}=T_k\exp(h[V_k^b]^\wedge).
$$

For space twist input,

$$
T_{k+1}=\exp(h[V_k^s]^\wedge)T_k.
$$

These updates remain on $SE(3)$ exactly, up to numerical evaluation of the exponential. Adding $h\dot T$ directly causes rotation drift.

## 14. What “kinematics and dynamics on $SE(3)$” means

For one free rigid body:

$$
T\in SE(3),
\qquad
V=(T^{-1}\dot T)^\vee\in\mathbb R^6,
$$

so kinematics are

$$
\dot T=T[V]^\wedge.
$$

Dynamics relate spatial momentum and wrench, developed in [[07 - Single Rigid Body Dynamics on SE3]].

For a fixed-base manipulator:

$$
q\in\mathcal Q,
\qquad
T_{0i}=T_{0i}(q)\in SE(3),
$$

$$
V_i=J_i(q)\dot q\in\mathfrak{se}(3)\simeq\mathbb R^6,
$$

$$
M(q)\ddot q+C(q,\dot q)\dot q+g(q)
=\tau+\sum_iJ_i(q)^TF_i.
$$

$SE(3)$ supplies the link-pose and link-motion geometry; the final equations of motion are usually expressed in joint coordinates because joints impose the mechanism's constraints automatically.

For a floating-base robot, the configuration can be

$$
\mathcal Q=SE(3)\times\mathcal Q_j,
$$

and the generalized velocity combines a six-dimensional base twist with joint rates. In that case the system configuration itself contains an $SE(3)$ factor.

## Checkpoint exercises

1. Multiply two generic elements of $SE(3)$ and interpret the translation block.
2. Derive the inverse $T^{-1}$ by enforcing $TT^{-1}=I$.
3. For a revolute axis through $q=[1,0,0]^T$ in direction $\omega=[0,0,1]^T$, compute $v$.
4. Derive $\operatorname{Ad}_T$ from $T[V]^\wedge T^{-1}$.
5. Derive the wrench transformation using invariance of power.
6. Explain why the translational component of $\operatorname{Log}(T)^\vee$ need not equal $p$.
7. Explain precisely which parts of manipulator modeling live in $\mathcal Q$, $SE(3)$, $\mathfrak{se}(3)$, and $\mathfrak{se}(3)^*$.

Solutions are in [[13 - Worked Problems and Solutions#Module 3 solutions]].

## Next

Continue to [[04 - Manipulator Configuration and Forward Kinematics]].
