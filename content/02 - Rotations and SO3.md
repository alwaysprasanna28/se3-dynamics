---
aliases:
  - SO3
  - Rotations and SO(3)
tags:
  - robotics
  - rotations
  - lie-groups
  - so3
course: Robotic Manipulator Dynamics on SE(3)
module: 2
---

# Rotations and $SO(3)$

> [!summary]
> $SO(3)$ is the set of proper three-dimensional rotation matrices. It is a nonlinear manifold and a group. Its tangent directions are skew-symmetric matrices in $\mathfrak{so}(3)$, which can be represented by ordinary three-vectors.

## 1. What is an orientation?

Attach a right-handed orthonormal frame $\{b\}$ to a rigid body and compare it with a reference frame $\{a\}$. The orientation of $b$ relative to $a$ is represented by

$$
R_{ab}=
\begin{bmatrix}
\vert&\vert&\vert\\
x_b^a&y_b^a&z_b^a\\
\vert&\vert&\vert
\end{bmatrix}.
$$

The columns are the axes of frame $b$, expressed in frame $a$. Therefore, if the same geometric vector has coordinates $v_b$ and $v_a$,

$$
v_a=R_{ab}v_b.
$$

This is a passive coordinate transformation. The same matrix can also be read actively as rotating a vector in a fixed frame. Both readings are valid, but silently switching between them causes sign and ordering errors.

## 2. Definition of $SO(3)$

The special orthogonal group is

$$
SO(3)=\left\{R\in\mathbb R^{3\times3}\mid R^TR=I,\ \det R=+1\right\}.
$$

Orthogonality means lengths and angles are preserved. The determinant condition excludes reflections.

### 2.1 Group operations

For $R_1,R_2\in SO(3)$:

- closure: $R_1R_2\in SO(3)$;
- identity: $I\in SO(3)$;
- inverse: $R^{-1}=R^T$;
- associativity: $(R_1R_2)R_3=R_1(R_2R_3)$.

Composition follows frame cancellation:

$$
R_{ac}=R_{ab}R_{bc}.
$$

The order matters because rotations generally do not commute:

$$
R_1R_2\ne R_2R_1.
$$

## 3. Why $SO(3)$ is not $\mathbb R^3$

$SO(3)$ is a three-dimensional manifold embedded in $\mathbb R^9$ by six independent constraints. It is not a vector space:

- $R_1+R_2$ is usually not a rotation;
- scalar multiplication does not preserve rotations;
- globally unique nonsingular three-number coordinates do not exist;
- orientation differences should be composed as $R_1^{-1}R_2$, not subtracted as matrices or Euler angles.

Euler angles are useful for interfaces, but they are charts with singularities. Unit quaternions avoid local kinematic singularity but double-cover $SO(3)$: $q$ and $-q$ represent the same orientation. Rotation matrices are redundant but globally smooth as embedded objects.

## 4. Tangent space and $\mathfrak{so}(3)$

Let $R(t)\in SO(3)$. Differentiating $R^TR=I$ gives

$$
R^T\dot R+(R^T\dot R)^T=0.
$$

Thus $R^T\dot R$ is skew-symmetric. The Lie algebra is

$$
\mathfrak{so}(3)=\left\{\Omega\in\mathbb R^{3\times3}\mid\Omega^T=-\Omega\right\}.
$$

Every element can be written $\Omega=[\omega]_\times$. The tangent space at the identity is $T_I SO(3)=\mathfrak{so}(3)$. Tangent spaces elsewhere are translations of it:

$$
T_RSO(3)=\{R\Omega\mid\Omega\in\mathfrak{so}(3)\}
=\{\Omega R\mid\Omega\in\mathfrak{so}(3)\}.
$$

## 5. Body and space angular velocity

Two equally valid trivializations are

$$
[\omega^b]_\times=R^T\dot R,
\qquad
[\omega^s]_\times=\dot R R^T.
$$

Therefore

$$
\dot R=R[\omega^b]_\times=[\omega^s]_\times R,
$$

and

$$
\omega^s=R\omega^b.
$$

$\omega^b$ is expressed in the rotating body frame; $\omega^s$ is expressed in the fixed space frame. They are coordinates of the same angular velocity.

For a body-fixed point with constant coordinates $r_b$, its space position relative to the body origin is $r_s=Rr_b$. Then

$$
\dot r_s=\dot Rr_b=[\omega^s]_\times Rr_b=\omega^s\times r_s.
$$

This recovers the familiar rigid-body velocity law.

## 6. The Lie bracket

The matrix commutator defines the Lie bracket:

$$
[\Omega_1,\Omega_2]=\Omega_1\Omega_2-\Omega_2\Omega_1.
$$

Under the hat map,

$$
\left[ [\omega_1]_\times, [\omega_2]_\times\right]
=[\omega_1\times\omega_2]_\times.
$$

The bracket measures noncommutativity. Infinitesimal rotations about distinct axes fail to commute by a second-order amount governed by the cross product.

## 7. Exponential map: angular displacement to rotation

If angular velocity is constant in body or space coordinates over an interval, the matrix differential equation integrates to a matrix exponential. For $\phi=\theta u$, where $\|u\|=1$,

$$
R=\operatorname{Exp}_{SO(3)}(\phi)
=\exp([\phi]_\times).
$$

Using $[u]_\times^3=-[u]_\times$, the power series reduces to Rodrigues' formula:

$$
\exp([\phi]_\times)
=I+\frac{\sin\theta}{\theta}[\phi]_\times
+\frac{1-\cos\theta}{\theta^2}[\phi]_\times^2.
$$

Equivalently,

$$
\exp(\theta[u]_\times)
=I+\sin\theta[u]_\times+(1-\cos\theta)[u]_\times^2.
$$

At $\theta\approx0$, use series expansions to avoid cancellation:

$$
\frac{\sin\theta}{\theta}\approx1-\frac{\theta^2}{6},
\qquad
\frac{1-\cos\theta}{\theta^2}\approx\frac12-\frac{\theta^2}{24}.
$$

## 8. Logarithm map: rotation to angular displacement

For $R\in SO(3)$,

$$
\theta=\cos^{-1}\left(\frac{\operatorname{tr}(R)-1}{2}\right),
$$

and, away from $\theta=0$ and $\theta=\pi$,

$$
[\phi]_\times=\operatorname{Log}(R)
=\frac{\theta}{2\sin\theta}(R-R^T).
$$

Then $\phi=\operatorname{Log}(R)^\vee$. The principal logarithm uses $\theta\in[0,\pi]$.

> [!warning] The $\pi$ ambiguity
> At a $180^\circ$ rotation, the axis sign is ambiguous and the formula divides by $\sin\theta=0$. Robust implementations extract the axis from the diagonal or use a quaternion-based branch. Near identity, use the skew part and series expansions.

## 9. Orientation error

Given current orientation $R$ and desired orientation $R_d$, useful errors include

$$
R_e^b=R^TR_d,
\qquad
e_R^b=\operatorname{Log}(R^TR_d)^\vee,
$$

and

$$
R_e^s=R_dR^T,
\qquad
e_R^s=\operatorname{Log}(R_dR^T)^\vee.
$$

They are related by

$$
e_R^s=Re_R^b
$$

on the same logarithm branch. These errors say “which infinitesimal rotation, expressed in which frame, moves the current attitude toward the desired attitude?”

Matrix subtraction $R_d-R$ does not belong to $SO(3)$ and does not directly represent a rotation error.

## 10. Left and right perturbations

A small perturbation may act on either side:

$$
R'=\exp([\delta\phi^s]_\times)R
$$

or

$$
R'=R\exp([\delta\phi^b]_\times).
$$

The perturbations satisfy

$$
\delta\phi^s=R\delta\phi^b.
$$

Left perturbations are expressed in the reference/space frame. Right perturbations are expressed in the current/body frame. Optimization and estimation derivations must declare which convention they use.

## 11. Jacobian of the exponential coordinates

Let $R=\exp([\phi]_\times)$. Since rotations do not commute, $\omega$ is not generally equal to $\dot\phi$. Instead,

$$
\omega^s=J_l(\phi)\dot\phi,
\qquad
\omega^b=J_r(\phi)\dot\phi,
$$

where

$$
J_l(\phi)
=I+\frac{1-\cos\theta}{\theta^2}[\phi]_\times
+\frac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2,
$$

$$
J_r(\phi)=J_l(-\phi).
$$

Their small-angle expansions are

$$
J_l(\phi)\approx I+\frac12[\phi]_\times+\frac16[\phi]_\times^2,
$$

$$
J_r(\phi)\approx I-\frac12[\phi]_\times+\frac16[\phi]_\times^2.
$$

This distinction matters when differentiating logarithmic pose errors.

## 12. Numerically integrating orientation

Naive Euler integration,

$$
R_{k+1}=R_k+hR_k[\omega_k^b]_\times,
$$

does not preserve $R^TR=I$ exactly. A Lie-group update does:

$$
R_{k+1}=R_k\exp(h[\omega_k^b]_\times),
$$

or equivalently

$$
R_{k+1}=\exp(h[\omega_k^s]_\times)R_k.
$$

For varying angular velocity, this is first order. Midpoint or Runge–Kutta–Munthe-Kaas methods improve accuracy while respecting the manifold.

## 13. Interpolating orientations

The constant-body-angular-velocity path from $R_0$ to $R_1$ is

$$
R(s)=R_0\exp\left(s\operatorname{Log}(R_0^TR_1)\right),
\qquad s\in[0,1].
$$

This is the rotation-matrix equivalent of quaternion SLERP on the principal branch.

## Checkpoint exercises

1. Verify that $R_z(\alpha)R_x(\beta)\ne R_x(\beta)R_z(\alpha)$ with $\alpha=\beta=\pi/2$.
2. Starting from $R^TR=I$, prove $R^T\dot R\in\mathfrak{so}(3)$.
3. Derive $\omega^s=R\omega^b$.
4. Use the exponential map to compute a $90^\circ$ rotation about $z$.
5. Given $R=R_z(30^\circ)$ and $R_d=R_z(80^\circ)$, compute $e_R^b$.
6. Explain why adding two rotation matrices is not a valid way to average orientations.

Solutions are in [[13 - Worked Problems and Solutions#Module 2 solutions]].

## Next

Continue to [[03 - Rigid Motions and SE3]].
