---
aliases:
  - SE3 Formula Sheet
  - Robot Dynamics Cheat Sheet
tags:
  - robotics
  - reference
  - formulas
  - se3
course: Robotic Manipulator Dynamics on SE(3)
module: 14
---

# Notation and Formula Sheet

> [!important] Course convention
> $T_{ab}$ maps coordinates from frame $b$ to frame $a$. Twists use angular-first ordering $V=[\omega;v]$ and wrenches use moment-first ordering $F=[m;f]$.

## Frame transforms

$$
T_{ab}=\begin{bmatrix}R_{ab}&p_{ab}^a\\0&1\end{bmatrix},
\qquad
p_a=R_{ab}p_b+p_{ab}^a.
$$

$$
T_{ac}=T_{ab}T_{bc}.
$$

$$
T^{-1}=\begin{bmatrix}R^T&-R^Tp\\0&1\end{bmatrix}.
$$

## Cross-product matrix

$$
[a]_\times=
\begin{bmatrix}
0&-a_3&a_2\\a_3&0&-a_1\\-a_2&a_1&0
\end{bmatrix}.
$$

$$
[a]_\times b=a\times b,
\qquad
[Ra]_\times=R[a]_\times R^T.
$$

$$
[a]_\times^2=aa^T-\|a\|^2I.
$$

## $SO(3)$

$$
SO(3)=\{R\mid R^TR=I,\det R=1\}.
$$

$$
[\omega^b]_\times=R^T\dot R,
\qquad
[\omega^s]_\times=\dot RR^T.
$$

$$
\dot R=R[\omega^b]_\times=[\omega^s]_\times R,
\qquad
\omega^s=R\omega^b.
$$

### Exponential

For $\phi=\theta u$,

$$
\operatorname{Exp}(\phi)
=I+\frac{\sin\theta}{\theta}[\phi]_\times
+\frac{1-\cos\theta}{\theta^2}[\phi]_\times^2.
$$

### Logarithm

$$
\theta=\cos^{-1}\left(\frac{\operatorname{tr}R-1}{2}\right).
$$

Away from $0$ and $\pi$,

$$
\operatorname{Log}(R)=\frac{\theta}{2\sin\theta}(R-R^T).
$$

### Left and right Jacobians

$$
J_l(\phi)=I+rac{1-\cos\theta}{\theta^2}[\phi]_\times
+\frac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2.
$$

$$
J_r(\phi)=J_l(-\phi).
$$

$$
\omega^s=J_l(\phi)\dot\phi,
\qquad
\omega^b=J_r(\phi)\dot\phi.
$$

## $SE(3)$ and $\mathfrak{se}(3)$

$$
SE(3)=\left\{\begin{bmatrix}R&p\\0&1\end{bmatrix}\right\}.
$$

$$
V=\begin{bmatrix}\omega\\v\end{bmatrix},
\qquad
[V]^\wedge=
\begin{bmatrix}[\omega]_\times&v\\0&0\end{bmatrix}.
$$

$$
[V^b]^\wedge=T^{-1}\dot T,
\qquad
[V^s]^\wedge=\dot TT^{-1}.
$$

$$
\dot T=T[V^b]^\wedge=[V^s]^\wedge T.
$$

### Group adjoint

$$
\operatorname{Ad}_T=
\begin{bmatrix}R&0\\{}[p]_\times R&R\end{bmatrix}.
$$

$$
V^s=\operatorname{Ad}_TV^b.
$$

$$
\operatorname{Ad}_{T_1T_2}=\operatorname{Ad}_{T_1}\operatorname{Ad}_{T_2}.
$$

### Algebra adjoint

$$
\operatorname{ad}_V=
\begin{bmatrix}
[\omega]_\times&0\\
[v]_\times&[\omega]_\times
\end{bmatrix}.
$$

$$
\left[ [V_1]^\wedge, [V_2]^\wedge\right]
=\left[\operatorname{ad}_{V_1}V_2\right]^\wedge.
$$

### $SE(3)$ exponential

For $\xi=[\omega;v]$, $\|\omega\|=1$,

$$
e^{[\xi]^\wedge\theta}
=\begin{bmatrix}
e^{[\omega]_\times\theta}&G(\theta)v\\0&1
\end{bmatrix},
$$

$$
G(\theta)=I\theta+(1-\cos\theta)[\omega]_\times
+(\theta-\sin\theta)[\omega]_\times^2.
$$

If $\omega=0$,

$$
e^{[\xi]^\wedge\theta}=\begin{bmatrix}I&v\theta\\0&1\end{bmatrix}.
$$

### Screw axes

Revolute through point $r$:

$$
S=\begin{bmatrix}\omega\\-\omega\times r\end{bmatrix}.
$$

Helical with pitch $h$:

$$
S=\begin{bmatrix}\omega\\-\omega\times r+h\omega\end{bmatrix}.
$$

Prismatic along $u$:

$$
S=\begin{bmatrix}0\\u\end{bmatrix}.
$$

## Wrenches and power

$$
F=\begin{bmatrix}m\\f\end{bmatrix},
\qquad
P=F^TV.
$$

If $V_a=\operatorname{Ad}_{T_{ab}}V_b$,

$$
F_a=\operatorname{Ad}_{T_{ab}}^{-T}F_b.
$$

## Product of exponentials

Space form:

$$
T(q)=e^{[S_1]^\wedge q_1}\cdots e^{[S_n]^\wedge q_n}M.
$$

Body form:

$$
T(q)=M e^{[B_1]^\wedge q_1}\cdots e^{[B_n]^\wedge q_n}.
$$

$$
B_i=\operatorname{Ad}_{M^{-1}}S_i.
$$

## Jacobians

$$
V^s=J_s(q)\dot q,
\qquad
V^b=J_b(q)\dot q.
$$

$$
J_{s,1}=S_1,
\qquad
J_{s,i}=\operatorname{Ad}_{e^{[S_1]q_1}\cdots e^{[S_{i-1}]q_{i-1}}}S_i.
$$

$$
J_s=\operatorname{Ad}_TJ_b.
$$

For tool origin position $p$,

$$
J_p=\begin{bmatrix}-[p]_\times&I\end{bmatrix}J_s.
$$

Statics:

$$
\tau=J^TF.
$$

Acceleration:

$$
\dot V=J\ddot q+\dot J\dot q.
$$

## Pose errors

Body/right error:

$$
E_b=T^{-1}T_d,
\qquad
e_b=\operatorname{Log}(E_b)^\vee.
$$

Space/left error:

$$
E_s=T_dT^{-1},
\qquad
e_s=\operatorname{Log}(E_s)^\vee.
$$

$$
e_s=\operatorname{Ad}_Te_b
$$

on a consistent branch.

## Spatial inertia and rigid-body dynamics

For body origin $O$ and $c=p_{OC}$,

$$
I_O=I_C+m[c]_\times^T[c]_\times.
$$

$$
\mathcal G_O=
\begin{bmatrix}
I_O&m[c]_\times\\
-m[c]_\times&mI
\end{bmatrix}.
$$

$$
K=\frac12V^T\mathcal GV,
\qquad
\mu=\mathcal GV.
$$

$$
F=\mathcal G\dot V-\operatorname{ad}_V^T\mathcal GV.
$$

$$
\mathcal G_a
=\operatorname{Ad}_T^{-T}\mathcal G_b\operatorname{Ad}_T^{-1}.
$$

## Manipulator dynamics

### Particle and virtual-work origin

$$
r_a=r_a(q),
\qquad
\dot r_a=J_a\dot q,
\qquad
\ddot r_a=J_a\ddot q+\dot J_a\dot q.
$$

$$
\delta r_a=J_a\delta q,
\qquad
Q=\sum_aJ_a^Tf_a.
$$

$$
M(q)=\sum_am_aJ_a^TJ_a,
\qquad
b(q,\dot q)=\sum_am_aJ_a^T\dot J_a\dot q.
$$

### Rigid-link form

$$
M(q)=\sum_iJ_i(q)^T\mathcal G_iJ_i(q).
$$

For compatible body twists and inertias,

$$
c(q,\dot q)
=\sum_iJ_i^T\left(
\mathcal G_i\dot J_i\dot q
-\operatorname{ad}_{V_i}^T\mathcal G_iV_i
\right).
$$

$$
U(q)=-\sum_im_ig_0^Tp_{C_i}(q),
\qquad
g(q)=\frac{\partial U}{\partial q}.
$$

$$
M(q)\ddot q+c(q,\dot q)+g(q)+\tau_f
=B(q)u+\sum_iJ_i^TF_i.
$$

Christoffel symbols:

$$
\Gamma_{ijk}=\frac12\left(
\frac{\partial M_{ij}}{\partial q_k}
+\frac{\partial M_{ik}}{\partial q_j}
-\frac{\partial M_{jk}}{\partial q_i}
\right).
$$

$$
C_{ij}=\sum_k\Gamma_{ijk}\dot q_k.
$$

Structural property:

$$
\dot M-2C\text{ is skew-symmetric}.
$$

Energy balance:

$$
\frac{d}{dt}(K+U)
=u^TB^T\dot q+\sum_iF_i^TV_i-\tau_f^T\dot q.
$$

### Coordinate change $q=\varphi(s)$

With $E=\partial q/\partial s$,

$$
M_s=E^TM_qE,
$$

$$
c_s=E^T(M_q\dot E\dot s+c_q),
\qquad
g_s=E^Tg_q,
\qquad
Q_s=E^TQ_q.
$$

## Constraints and contact

$$
\phi(q)=0,
\qquad
J_c\dot q=0,
\qquad
J_c\ddot q+\dot J_c\dot q=0.
$$

$$
M\ddot q+h=Bu+\tau_{\mathrm{ext}}+J_c^T\lambda.
$$

$$
\begin{bmatrix}
M&-J_c^T\\
J_c&0
\end{bmatrix}
\begin{bmatrix}\ddot q\\\lambda\end{bmatrix}
=
\begin{bmatrix}
Bu+\tau_{\mathrm{ext}}-h\\
-\dot J_c\dot q
\end{bmatrix}.
$$

$$
\Lambda_c=(J_cM^{-1}J_c^T)^{-1}.
$$

Unilateral normal contact:

$$
0\le\lambda_n\ \perp\ \phi_n(q)\ge0.
$$

Coulomb cone:

$$
\|f_t\|\le\mu f_n.
$$

Frictionless impact:

$$
M(\dot q^+-\dot q^-)=J_c^TP,
$$

$$
P=-(1+e)(J_cM^{-1}J_c^T)^{-1}J_c\dot q^-.
$$

## Operational-space dynamics

$$
\Lambda=(JM^{-1}J^T)^{-1}.
$$

$$
\bar J=M^{-1}J^T\Lambda.
$$

$$
N=I-\bar JJ.
$$

$$
\eta=\Lambda(JM^{-1}h-\dot J\dot q).
$$

$$
\Lambda\dot V+\eta=F+F_{\mathrm{ext}}.
$$

$$
m_{\mathrm{eff}}(u)=\frac{1}{u^T\Lambda^{-1}u}.
$$

## Controllers

Joint PD plus gravity, with $e=q-q_d$:

$$
\tau=g-K_pe-K_d\dot q.
$$

Computed torque:

$$
\tau=M(\ddot q_d-K_d\dot e-K_pe)+c+g.
$$

Local body pose impedance, with $e=\operatorname{Log}(T^{-1}T_d)^\vee$:

$$
F=Ke-De_V+F_{\mathrm{ff}},
\qquad
\tau=J_b^TF+g+N^T\tau_0.
$$

Operational inverse dynamics:

$$
F=\Lambda\dot V_r+\eta,
\qquad
\tau=J^TF+N^T\tau_0.
$$

## Units

| Quantity | Angular part | Linear part |
|---|---:|---:|
| Twist $V$ | $\mathrm{rad/s}$ | $\mathrm{m/s}$ |
| Twist acceleration | $\mathrm{rad/s^2}$ | $\mathrm{m/s^2}$ |
| Wrench $F$ | $\mathrm{N\,m}$ | $\mathrm N$ |
| Power $F^TV$ | $\mathrm W$ | $\mathrm W$ |
| Revolute joint torque | $\mathrm{N\,m}$ | — |
| Prismatic joint force | — | $\mathrm N$ |

Radians are dimensionless in SI, but angular and linear components still require meaningful scaling in norms and optimizers.

## Five debugging questions

1. What frame is each quantity expressed in?
2. What point is each twist, wrench, or inertia taken about?
3. Which way does each transform map coordinates?
4. Does the equation preserve power or kinetic energy under a frame change?
5. Does it reduce correctly when velocity, gravity, an offset, or one mass is set to zero?
