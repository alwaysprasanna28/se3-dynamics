---
aliases:
  - SE3 Robotics Exercises
tags:
  - robotics
  - exercises
  - solutions
  - se3
course: Robotic Manipulator Dynamics on SE(3)
module: 13
---

# Worked Problems and Solutions

> [!tip]
> Attempt each checkpoint on paper before reading its solution. Frame labels, units, and sign explanations count as part of an answer.

## Module 1 solutions

### 1. Rotation of a cross-product matrix

Rotations preserve cross products:

$$
R(a\times b)=(Ra)\times(Rb).
$$

Using matrices,

$$
R[a]_\times b=[Ra]_\times Rb.
$$

Let $x=Rb$, so $b=R^Tx$. Then

$$
R[a]_\times R^Tx=[Ra]_\times x
$$

for every $x$, hence

$$
[Ra]_\times=R[a]_\times R^T.
$$

### 2. Derivative of an inverse

Differentiate $A^{-1}A=I$:

$$
\dot A^{-1}A+A^{-1}\dot A=0.
$$

Right-multiply by $A^{-1}$:

$$
\dot A^{-1}=-A^{-1}\dot A A^{-1}.
$$

### 3. Generalized force from point force

Point power is

$$
P=f^Tv=f^TJ\dot q.
$$

Joint power is $P=\tau^T\dot q$. Equality for arbitrary $\dot q$ gives

$$
\tau=J^Tf.
$$

### 4. Positive definiteness of inertia

Kinetic energy is $K=\frac12\dot q^TM\dot q$. For independent generalized coordinates, any $\dot q\ne0$ causes nonzero motion of some nonzero mass, so $K>0$. Thus $\dot q^TM\dot q>0$ and $M\succ0$. A zero-energy nonzero velocity would signal a redundant coordinate or a massless unmodeled direction.

### 5. Spaces for a planar 2R arm

Globally, two continuous revolute joints give

$$
\mathcal Q=S^1\times S^1.
$$

With joint limits one uses a subset. The planar tool pose is naturally in $SE(2)$, or in the planar-motion subgroup embedded in $SE(3)$. Thus

$$
f:\mathcal Q\rightarrow SE(2)\subset SE(3).
$$

## Module 2 solutions

### 1. Noncommuting rotations

Using $90^\circ$ rotations,

$$
R_zR_x=
\begin{bmatrix}
0&0&1\\
1&0&0\\
0&1&0
\end{bmatrix},
$$

while

$$
R_xR_z=
\begin{bmatrix}
0&-1&0\\
0&0&-1\\
1&0&0
\end{bmatrix}.
$$

They are unequal because the second rotation acts about an axis whose interpretation depends on multiplication order.

### 2. Skew symmetry of angular velocity

Differentiate $R^TR=I$:

$$
\dot R^TR+R^T\dot R=0.
$$

Since $(R^T\dot R)^T=\dot R^TR$, this says

$$
(R^T\dot R)^T+R^T\dot R=0.
$$

Therefore $R^T\dot R\in\mathfrak{so}(3)$.

### 3. Relating angular-velocity coordinates

From

$$
\dot R=R[\omega^b]_\times=[\omega^s]_\times R,
$$

right-multiply the first equality by $R^T$:

$$
[\omega^s]_\times=R[\omega^b]_\times R^T=[R\omega^b]_\times.
$$

Apply vee:

$$
\omega^s=R\omega^b.
$$

### 4. $90^\circ$ about $z$

With $u=e_z$ and $\theta=\pi/2$, Rodrigues' formula gives

$$
R_z(\pi/2)=
\begin{bmatrix}
0&-1&0\\
1&0&0\\
0&0&1
\end{bmatrix}.
$$

### 5. Logarithmic orientation error

$$
R^TR_d=R_z(-30^\circ)R_z(80^\circ)=R_z(50^\circ).
$$

Thus

$$
e_R^b=\frac{5\pi}{18}
\begin{bmatrix}0\\0\\1\end{bmatrix}.
$$

### 6. Why matrix addition is not orientation averaging

$R_1+R_2$ generally has nonunit columns and determinant different from one, so it is not in $SO(3)$. A valid mean needs a projection back to $SO(3)$ or an intrinsic optimization such as minimizing summed squared geodesic errors.

## Module 3 solutions

### 1. Composition in $SE(3)$

$$
\begin{bmatrix}R_1&p_1\\0&1\end{bmatrix}
\begin{bmatrix}R_2&p_2\\0&1\end{bmatrix}
=
\begin{bmatrix}
R_1R_2&p_1+R_1p_2\\
0&1
\end{bmatrix}.
$$

$p_2$ is first rotated into the coordinates of $p_1$, then added.

### 2. Inverse transform

Let $T^{-1}=[A,b;0,1]$. From $TT^{-1}=I$,

$$
RA=I\Rightarrow A=R^T,
$$

$$
Rb+p=0\Rightarrow b=-R^Tp.
$$

### 3. Revolute screw through $[1,0,0]^T$

With $\omega=[0,0,1]^T$,

$$
v=-\omega\times r
=-
\begin{bmatrix}0\\1\\0\end{bmatrix}
=
\begin{bmatrix}0\\-1\\0\end{bmatrix}.
$$

### 4. Adjoint derivation

Direct multiplication gives

$$
T[V]^\wedge T^{-1}
=
\begin{bmatrix}
R[\omega]_\times R^T&-R[\omega]_\times R^Tp+Rv\\
0&0
\end{bmatrix}.
$$

Since $R[\omega]_\times R^T=[R\omega]_\times$ and

$$
-R[\omega]_\times R^Tp=[p]_\times R\omega,
$$

the vector form is

$$
\begin{bmatrix}R\omega\\{}[p]_\times R\omega+Rv\end{bmatrix}
=
\begin{bmatrix}R&0\\{}[p]_\times R&R\end{bmatrix}
\begin{bmatrix}\omega\\v\end{bmatrix}.
$$

### 5. Wrench transformation

With $V_a=\operatorname{Ad}_TV_b$, power invariance requires

$$
F_a^T\operatorname{Ad}_TV_b=F_b^TV_b
$$

for all $V_b$. Hence

$$
\operatorname{Ad}_T^TF_a=F_b,
$$

or

$$
F_a=\operatorname{Ad}_T^{-T}F_b.
$$

### 6. Translation inside the $SE(3)$ logarithm

Rotation and translation do not commute. If $T=\exp([\phi;\rho]^\wedge)$, then

$$
p=J_l(\phi)\rho.
$$

Therefore $\rho=J_l(\phi)^{-1}p$, which equals $p$ only when $J_l=I$, such as at zero rotation to first order.

### 7. Where the objects live

- Joint configuration: $q\in\mathcal Q$.
- Link pose: $T_i(q)\in SE(3)$.
- Link velocity: $V_i\in\mathfrak{se}(3)\simeq\mathbb R^6$.
- Link load: $F_i\in\mathfrak{se}(3)^*$.
- System state: $(q,\dot q)\in T\mathcal Q$.
- Joint generalized force: $\tau\in T_q^*\mathcal Q$.

## Module 4 solutions

### 1. Revolute axis parallel to $y$

For $\omega=[0,1,0]^T$ and $r=[a,0,c]^T$,

$$
\omega\times r=
\begin{bmatrix}c\\0\\-a\end{bmatrix},
$$

so

$$
S=\begin{bmatrix}0\\1\\0\\-c\\0\\a\end{bmatrix}.
$$

### 2. Planar revolute–prismatic arm

Let the revolute joint be at the origin about $z$, and at home let the prismatic joint translate along $+x$. Then

$$
S_1=\begin{bmatrix}0\\0\\1\\0\\0\\0\end{bmatrix},
\qquad
S_2=\begin{bmatrix}0\\0\\0\\1\\0\\0\end{bmatrix}.
$$

With home tool pose $M$,

$$
T(q)=e^{[S_1]^\wedge q_1}e^{[S_2]^\wedge q_2}M.
$$

The first exponential rotates the prismatic direction with the arm.

### 3. Expanding the 2R position

The second revolute exponential rotates about the line through $[l_1,0,0]^T$, while the first rotates the whole downstream chain. Applying both to the home tool point gives

$$
p=
\begin{bmatrix}
l_1\cos q_1+l_2\cos(q_1+q_2)\\
l_1\sin q_1+l_2\sin(q_1+q_2)\\
0
\end{bmatrix}.
$$

At $q=0$, $p=[l_1+l_2,0,0]^T$ as required.

### 4. Body screws of the 2R arm

Let $L=l_1+l_2$ and $M=[I,[L,0,0]^T;0,1]$. Since

$$
\operatorname{Ad}_{M^{-1}}
\begin{bmatrix}\omega\\v\end{bmatrix}
=\begin{bmatrix}\omega\\v-[p]_\times\omega\end{bmatrix},
$$

we obtain

$$
B_1=\begin{bmatrix}0\\0\\1\\0\\L\\0\end{bmatrix},
\qquad
B_2=\begin{bmatrix}0\\0\\1\\0\\l_2\\0\end{bmatrix}.
$$

### 5. Why downstream joints do not move link $i$

Joint $j>i$ lies in the subtree beyond link $i$. Its relative motion changes descendants but not ancestors. In a tree's dependency graph, $T_{0i}$ depends only on coordinates along the unique base-to-$i$ path.

## Module 5 solutions

### 1. Space Jacobian derivation

Differentiate the PoE product one factor at a time and right-trivialize with $T^{-1}$. For joint $i$, downstream factors cancel and the result is

$$
[V_i^s]^\wedge
=T_{<i}[S_i]^\wedge T_{<i}^{-1}\dot q_i.
$$

Applying vee gives

$$
V_i^s=\operatorname{Ad}_{T_{<i}}S_i\dot q_i.
$$

Summing contributions gives the stated columns.

### 2. Space/body Jacobian relation

For every $\dot q$,

$$
J_s\dot q=V^s=\operatorname{Ad}_TV^b
=\operatorname{Ad}_TJ_b\dot q.
$$

Therefore

$$
J_s=\operatorname{Ad}_TJ_b.
$$

### 3. 2R singularity

Differentiation gives

$$
J_p=
\begin{bmatrix}
-l_1s_1-l_2s_{12}&-l_2s_{12}\\
l_1c_1+l_2c_{12}&l_2c_{12}
\end{bmatrix}.
$$

Its determinant is $l_1l_2\sin q_2$, so rank is lost for $q_2=0$ or $\pi$.

### 4. Transpose statics

$$
F^TV=F^TJ\dot q=(J^TF)^T\dot q.
$$

Comparing with $\tau^T\dot q$ gives $\tau=J^TF$.

### 5. Nullspace projector

Using $JJ^+J=J$,

$$
J(I-J^+J)z=Jz-JJ^+Jz=0.
$$

### 6. Analytic versus mechanism singularity

An Euler-rate map $\omega=E(\alpha)\dot\alpha$ can lose rank at gimbal lock. Then $J_a=E^{-1}J_\omega$ becomes singular or unbounded even if the physical geometric Jacobian still spans the needed angular velocities.

## Module 6 solutions

### 1. 2R point acceleration

With $J_p$ from Module 5,

$$
\ddot p=J_p\ddot q+\dot J_p\dot q.
$$

Directly,

$$
\dot J_p\dot q=
\begin{bmatrix}
-l_1c_1\dot q_1^2-l_2c_{12}(\dot q_1+\dot q_2)^2\\
-l_1s_1\dot q_1^2-l_2s_{12}(\dot q_1+\dot q_2)^2
\end{bmatrix}.
$$

These are centripetal components.

### 2. Constant rates at $q_2=0$

Although $\ddot q=0$, each link point follows a curved path. At $q_2=0$, the bias acceleration points along the common link direction toward the instantaneous rotation centers. It is nonzero unless both relevant angular rates vanish.

### 3. Body-error Newton update

For small $\Delta q$,

$$
T(q+\Delta q)\approx T(q)e^{[J_b\Delta q]^\wedge}.
$$

To move from $T$ toward $T_d=Te^{[e_b]^\wedge}$, match the first-order increments:

$$
J_b\Delta q=e_b.
$$

The least-squares minimum-norm update is $\Delta q=J_b^+e_b$.

### 4. Matching error and Jacobian

$e_b$ is a tangent coordinate expressed on the body's side, whereas columns of $J_s$ are space-expressed. Adding them is a frame mismatch. Convert one with $\operatorname{Ad}_T$ or use $J_b$ directly.

### 5. Comparing pose paths

Screw interpolation holds one body screw constant and generally curves the origin's Cartesian path. Separate interpolation makes $p(s)$ a straight line and $R(s)$ a shortest principal rotation, but the combined motion is generally not a constant screw.

### 6. Position-priority weighted IK

Choose

$$
W=\operatorname{diag}(w_r,w_r,w_{\mathrm{yaw}},w_p,w_p,w_p),
$$

with large $w_p$, moderate roll/pitch weights $w_r$, and small $w_{\mathrm{yaw}}$. If yaw is entirely free, remove it from the task rather than merely assigning an extremely small weight.

## Module 7 solutions

### 1. Offset spatial inertia

Use $v_C=v_O-[c]_\times\omega$ in

$$
K=\frac12\omega^TI_C\omega+\frac12m v_C^Tv_C.
$$

Expansion produces

$$
K=\frac12
\begin{bmatrix}\omega\\v_O\end{bmatrix}^T
\begin{bmatrix}
I_C+m[c]_\times^T[c]_\times&m[c]_\times\\
-m[c]_\times&mI
\end{bmatrix}
\begin{bmatrix}\omega\\v_O\end{bmatrix}.
$$

The upper-left block is $I_O$ by the parallel-axis theorem.

### 2. Expanded Newton–Euler equation

At the center of mass, $\mathcal G=\operatorname{diag}(I_C,mI)$. Substitution into

$$
F=\mathcal G\dot V-\operatorname{ad}_V^T\mathcal GV
$$

gives

$$
m_C=I_C\dot\omega+\omega\times I_C\omega,
$$

$$
f=m(\dot v+\omega\times v).
$$

### 3. Zero power of the convective term

$$
V^T\operatorname{ad}_V^T\mathcal GV
=(\operatorname{ad}_VV)^T\mathcal GV.
$$

But $\operatorname{ad}_VV=[V,V]=0$, so the scalar is zero.

### 4. Inertia transformation

Set $V_a=\operatorname{Ad}_TV_b$ and equate energies. For all $V_b$,

$$
V_b^T\operatorname{Ad}_T^T\mathcal G_a\operatorname{Ad}_TV_b
=V_b^T\mathcal G_bV_b.
$$

Thus $\operatorname{Ad}_T^T\mathcal G_a\operatorname{Ad}_T=\mathcal G_b$, yielding

$$
\mathcal G_a=\operatorname{Ad}_T^{-T}\mathcal G_b\operatorname{Ad}_T^{-1}.
$$

### 5. Gravity wrench

Express force in body coordinates, $f_g^b=mR^Tg_0$. About origin $O$,

$$
F_g^b=\begin{bmatrix}[c]_\times f_g^b\\f_g^b\end{bmatrix}.
$$

### 6. Constant geometric momentum, changing coordinates

If an inertial-frame momentum vector is constant and the body basis rotates, its body coordinate column is multiplied by $R^T(t)$ and therefore changes. Conservation concerns the geometric vector, not a particular moving coordinate column.

## Module 8 solutions

### 1. Pullback of link energies

With $V_i=J_i\dot q$,

$$
K=\sum_i\frac12V_i^T\mathcal G_iV_i
=\frac12\dot q^T\left(\sum_iJ_i^T\mathcal G_iJ_i\right)\dot q.
$$

Identify the coefficient with $M(q)$.

### 2. Symmetry and definiteness

$J_i^T\mathcal G_iJ_i$ is symmetric because $\mathcal G_i$ is symmetric. Its quadratic form is nonnegative. The sum is positive definite if no nonzero generalized velocity leaves every massive link stationary.

### 3. Generalized gravity in two ways

Potential gives

$$
g=\frac{\partial}{\partial q}left(-\sum_im_ig_0^Tp_{C_i}\right).
$$

Virtual power of physical gravity forces gives

$$
\tau_g=\sum_iJ_{v,C_i}^Tm_ig_0.
$$

Since conservative generalized force is $-\partial U/\partial q$, $\tau_g=-g$.

### 4. Energy balance

Premultiply $M\ddot q+C\dot q+g=\tau+\tau_{\mathrm{ext}}$ by $\dot q^T$, use

$$
\dot K=\dot q^TM\ddot q+\frac12\dot q^T\dot M\dot q
$$

and the skew property to get

$$
\dot K+\dot U=\dot q^T(\tau+\tau_{\mathrm{ext}}).
$$

### 5. Nonuniqueness of $C$

Any matrix $\Delta C$ satisfying $\Delta C\dot q=0$ can be added without changing the dynamics. The vector $c=C\dot q$ is fixed by the model, while many matrix factorizations represent it.

### 6. Floating-base seven-joint arm

$$
\mathcal Q=SE(3)\times(S^1)^7
$$

before joint limits. A minimal generalized velocity is

$$
\nu=\begin{bmatrix}V_B\\\dot q_j\end{bmatrix}\in\mathbb R^{13}.
$$

If a quaternion stores base orientation, the configuration array has $3+4+7=14$ numbers but only $13$ velocity coordinates.

## Module 9 solutions

### 1. Origin of the $\cos q_2$ term

The dot product between the velocity caused by rotation of link 1 and the velocity about joint 2 contains the angle between the two link directions. Expanding the square gives

$$
2l_1r_2\cos q_2\,\dot q_1(\dot q_1+\dot q_2).
$$

### 2. Mass entries

Collect coefficients in

$$
K=\frac12M_{11}\dot q_1^2+M_{12}\dot q_1\dot q_2+\frac12M_{22}\dot q_2^2.
$$

This yields exactly the $M_{11},M_{12},M_{22}$ stated in [[09 - Lagrangian Dynamics and the Two-Link Arm#3. Kinetic energy]].

### 3. Gravity check

Differentiate $U$ to obtain the stated $g(q)$. At $q_1=q_2=0$,

$$
g_1=g\left(m_1r_1+m_2l_1+m_2r_2\right),
$$

$$
g_2=gm_2r_2.
$$

Both are nonzero because horizontal links have maximum gravitational moment arms.

### 4. Checking $C\dot q$

Matrix multiplication gives

$$
(C\dot q)_1
=-b\sin q_2\dot q_2\dot q_1
-b\sin q_2(\dot q_1+\dot q_2)\dot q_2,
$$

which simplifies to the stated first component. The second is $b\sin q_2\dot q_1^2$.

### 5. Skew check

Only $q_2$ changes $M$. Compute

$$
\dot M=
\begin{bmatrix}
-2b\sin q_2\dot q_2&-b\sin q_2\dot q_2\\
-b\sin q_2\dot q_2&0
\end{bmatrix}.
$$

Subtracting $2C$ produces a matrix with zero diagonal and opposite off-diagonal entries, hence it is skew-symmetric.

### 6. End-effector point payload

For a point payload $m_p$ at the tool,

$$
\Delta M=m_pJ_p^TJ_p.
$$

Its gravity addition is

$$
\Delta g=m_pg
\begin{bmatrix}
l_1\cos q_1+l_2\cos(q_1+q_2)\\
l_2\cos(q_1+q_2)
\end{bmatrix}.
$$

If the payload has rotational inertia or an offset center of mass, use a full spatial inertia and body Jacobian.

## Module 10 solutions

### 1. RNEA acceleration terms

In

$$
A_i=X_iA_p+S_i\ddot q_i+\operatorname{ad}_{V_i}S_i\dot q_i,
$$

the first term transports parent acceleration, the second is direct relative joint acceleration, and the third is the velocity-product acceleration caused by composing a moving parent with relative joint motion.

### 2. Child wrench to parent

The child twist induced by parent motion is $V_i=X_iV_p$. Equality of power demands

$$
F_i^TV_i=F_p^TV_p.
$$

Thus $F_i^TX_iV_p=F_p^TV_p$ for all $V_p$, giving

$$
F_p=X_i^TF_i.
$$

### 3. One mass-matrix column

Disable gravity and external forces, set $\dot q=0$, and set $\ddot q=e_j$. Then RNEA returns

$$
\tau=M(q)e_j,
$$

the $j$th column of $M$.

### 4. CRBA output-size lower bound

A dense symmetric $n\times n$ matrix has $n(n+1)/2=O(n^2)$ independent entries. Explicitly writing them already requires $O(n^2)$ storage operations.

### 5. Constrained dynamics

Combine

$$
M\ddot q+h=\tau+J_c^T\lambda
$$

and

$$
J_c\ddot q=-\dot J_c\dot q.
$$

Moving terms to the two block rows yields

$$
\begin{bmatrix}M&-J_c^T\\J_c&0\end{bmatrix}
\begin{bmatrix}\ddot q\\\lambda\end{bmatrix}
=\begin{bmatrix}\tau-h\\-\dot J_c\dot q\end{bmatrix}.
$$

### 6. Tests for a reversed transform

Useful tests are:

1. compare RNEA gravity torque with a finite-difference gradient of potential;
2. compare RNEA mass columns with $\sum J_i^T\mathcal G_iJ_i$;
3. apply a link wrench and verify link power equals $\tau^T\dot q$;
4. compare inverse/forward dynamics round-trip at random states;
5. rotate the entire model in space and confirm coordinate-consistent physical results.

## Module 11 solutions

### 1. Deriving task dynamics

Substitute

$$
\ddot q=M^{-1}(J^T(F+F_{\mathrm{ext}})-h)
$$

into $\dot V=J\ddot q+\dot J\dot q$, then premultiply by

$$
\Lambda=(JM^{-1}J^T)^{-1}.
$$

Collecting bias terms gives

$$
\Lambda\dot V+Lambda(JM^{-1}h-\dot J\dot q)=F+F_{\mathrm{ext}}.
$$

### 2. Minimum-energy inverse

Minimize

$$
\frac12\dot q^TM\dot q+\lambda^T(V-J\dot q).
$$

Stationarity gives $M\dot q-J^T\lambda=0$, so $\dot q=M^{-1}J^T\lambda$. Enforce the constraint:

$$
V=JM^{-1}J^T\lambda,
$$

thus $\lambda=\Lambda V$ and

$$
\dot q=M^{-1}J^T\Lambda V=\bar JV.
$$

### 3. Dynamic nullspace decoupling

With $N=I-\bar JJ$,

$$
N^T=I-J^T\bar J^T.
$$

Then

$$
JM^{-1}N^T
=JM^{-1}-JM^{-1}J^T\bar J^T.
$$

Since $JM^{-1}J^T=\Lambda^{-1}$ and $\bar J^T=\Lambda JM^{-1}$ for symmetric $M,\Lambda$,

$$
JM^{-1}N^T=JM^{-1}-\Lambda^{-1}\Lambda JM^{-1}=0.
$$

### 4. Directional effective mass

Apply $F=fu$. Since $a=\Lambda^{-1}fu$, the component is $u^Ta=f,u^T\Lambda^{-1}u$. Matching $f=m_{\mathrm{eff}}u^Ta$ gives

$$
m_{\mathrm{eff}}=(u^T\Lambda^{-1}u)^{-1}.
$$

### 5. Task-inertia transformation

Equate task kinetic energies under $V_a=\operatorname{Ad}_TV_b$:

$$
V_a^T\Lambda_aV_a=V_b^T\Lambda_bV_b.
$$

The same congruence derivation as spatial inertia gives

$$
\Lambda_a=\operatorname{Ad}_T^{-T}\Lambda_b\operatorname{Ad}_T^{-1}.
$$

### 6. Meaning of damping

Adding $\lambda^2I$ makes inversion bounded at rank loss. It also changes the force–acceleration relationship, so the result is a regularized controller metric rather than the exact inertia felt by the task.

## Module 12 solutions

### 1. PD plus gravity stability

For

$$
M\ddot q+C\dot q+K_d\dot q+K_pe=0,
$$

differentiate

$$
\mathcal V=\frac12\dot q^TM\dot q+\frac12e^TK_pe.
$$

Substitution gives

$$
\dot{\mathcal V}
=\frac12\dot q^T(\dot M-2C)\dot q-\dot q^TK_d\dot q
=-\dot q^TK_d\dot q\le0.
$$

LaSalle's invariance principle gives convergence under the usual regularity and isolated-equilibrium conditions.

### 2. Computed-torque error dynamics

Insert

$$
\tau=M(\ddot q_d-K_d\dot e-K_pe)+c+g
$$

into $M\ddot q+c+g=\tau$. Cancel $c,g$ and multiply by $M^{-1}$:

$$
\ddot q=\ddot q_d-K_d\dot e-K_pe.
$$

Since $\ddot e=\ddot q-\ddot q_d$,

$$
\ddot e+K_d\dot e+K_pe=0.
$$

### 3. Trace versus logarithmic error near $\pi$

For relative angle $\theta$, the trace-error vector magnitude scales as $|\sin\theta|$ and therefore becomes zero at $\theta=\pi$, an undesired critical point. The principal log magnitude approaches $\pi$ but its axis becomes ambiguous there. One is smooth with undesired equilibria; the other is direct but nonsmooth/ambiguous at the cut locus.

### 4. Exact gradient of a log potential

A pose variation $\delta T=T[\eta]^\wedge$ changes $e=\operatorname{Log}(T^{-1}T_d)^\vee$ through an inverse left/right Lie Jacobian. Thus

$$
\delta\left(\frac12e^TKe\right)
=e^TK\,J_{\log}(e)\eta,
$$

so the gradient wrench contains $J_{\log}(e)^TKe$, not merely $Ke$, except locally where $J_{\log}\approx I$.

### 5. Damping power

For $F_d=-DV$ with $D\succeq0$,

$$
P_d=F_d^TV=-V^TDV\le0.
$$

Thus ideal damping cannot add mechanical energy.

### 6. Saturation breaks decoupling

The commanded structure $J^TF+N^T\tau_0$ is altered componentwise by actuator limits. The saturated torque generally cannot be decomposed with the same exact task and nullspace projections, so $JM^{-1}N^T\tau_0=0$ no longer describes the applied torque.

## Module 7A solutions

### 1. D'Alembert derivation

For particle $a$,

$$
r_a=r_a(q),
\qquad
\delta r_a=J_a\delta q,
\qquad
\ddot r_a=J_a\ddot q+\dot J_a\dot q.
$$

D'Alembert's principle is

$$
\sum_a(f_a-m_a\ddot r_a)^T\delta r_a=0.
$$

Substitute the admissible displacement and factor arbitrary $\delta q$:

$$
\sum_aJ_a^Tm_a
\left(J_a\ddot q+\dot J_a\dot q\right)
=\sum_aJ_a^Tf_a.
$$

Therefore

$$
M=\sum_am_aJ_a^TJ_a,
\qquad
b=\sum_am_aJ_a^T\dot J_a\dot q,
\qquad
Q=\sum_aJ_a^Tf_a.
$$

### 2. Ideal pin-joint reaction

At an ideal pin, the connected material points have the same admissible virtual translation. If the reaction on one link is $R$ and on the other is $-R$, their total virtual work is

$$
R^T\delta r+(-R)^T\delta r=0.
$$

The ideal pin transmits no reaction moment along its allowed rotation. Reactions in forbidden directions therefore do no work on admissible virtual motion.

### 3. Pendulum radial reaction

Let

$$
e_r=\begin{bmatrix}\cos q\\\sin q\end{bmatrix}.
$$

Radial acceleration is $-l\dot q^2e_r$. Let $F_r$ be the rod force on the mass, positive outward. Gravity's radial component is $-mg\sin q$. Radial Newton balance gives

$$
-ml\dot q^2=F_r-mg\sin q.
$$

Thus

$$
F_r=mg\sin q-ml\dot q^2.
$$

The inward tension is

$$
T=ml\dot q^2-mg\sin q.
$$

It is absent from the generalized angular equation because radial force is orthogonal to admissible tangential virtual displacement.

### 4. Symmetry and positive semidefiniteness

Each $m_aJ_a^TJ_a$ is symmetric. For any $z$,

$$
z^TMz=\sum_am_a\|J_az\|^2\ge0.
$$

It is strictly positive if no nonzero generalized velocity $z$ leaves every positive-mass particle stationary.

### 5. From particle integral to spatial inertia

For body point $\rho$, velocity is a linear function of twist:

$$
v(\rho)=\begin{bmatrix}-[\rho]_\times&I\end{bmatrix}V.
$$

Integrating

$$
\frac12\int\|v(\rho)\|^2dm
$$

collects the mass, first moments, and second moments into

$$
K=\frac12V^T\mathcal GV.
$$

$\mathcal G$ is therefore the integrated particle mass matrix expressed in rigid-body twist coordinates.

### 6. When redundant coordinates help

They are useful when minimal coordinates are difficult, discontinuous, or topology dependent—for example closed chains, multiple contacts, or maximal-coordinate simulation. They preserve simple body coordinates at the cost of explicit constraints and Lagrange multipliers.

## Module 8A solutions

### 1. Force-based derivation of $c$

For link $i$,

$$
V_i=J_i\dot q,
\qquad
\dot V_i=J_i\ddot q+\dot J_i\dot q.
$$

Its required net wrench is

$$
F_i^I=\mathcal G_i\dot V_i
-\operatorname{ad}_{V_i}^T\mathcal G_iV_i.
$$

Project and sum:

$$
\sum_iJ_i^TF_i^I
=\sum_iJ_i^T\mathcal G_iJ_i\ddot q
+\sum_iJ_i^T
\left(\mathcal G_i\dot J_i\dot q
-\operatorname{ad}_{V_i}^T\mathcal G_iV_i\right).
$$

The first coefficient is $M$ and the remaining vector is $c$.

### 2. The two velocity terms

$\mathcal G_i\dot J_i\dot q$ reflects changing link-twist directions as the mechanism moves. The coadjoint term reflects momentum differentiation in the moving link frame. One is mechanism-level transport and the other is body-frame momentum transport; both contribute to the same generalized velocity bias.

### 3. Gravity equivalence

Gravity virtual work is

$$
\delta W_g=\sum_i(m_ig_0)^T\delta p_{C_i}
=\left(\sum_iJ_{v,C_i}^Tm_ig_0\right)^T\delta q.
$$

Thus $\tau_g=\sum_iJ_{v,C_i}^Tm_ig_0$. Since

$$
U=-\sum_im_ig_0^Tp_{C_i},
$$

its gradient is $g=\partial U/\partial q=-\tau_g$.

### 4. Rigid pendulum two ways

Energy gives

$$
K=\frac12(I_C+mr^2)\dot q^2,
\qquad
U=mgr\sin q.
$$

Spatially, the link inertia about the joint has rotational entry $I_O=I_C+mr^2$, and gravity moment about the joint is $-mgr\cos q$. Both yield

$$
(I_C+mr^2)\ddot q+mgr\cos q=\tau.
$$

### 5. Extracting terms from inverse dynamics

For $\operatorname{ID}=M\ddot q+c+g$,

$$
g=\operatorname{ID}(q,0,0),
$$

$$
c=\operatorname{ID}(q,\dot q,0)-g,
$$

$$
M e_j=\operatorname{ID}(q,0,e_j)-g.
$$

Repeating the last call for all basis vectors constructs $M$.

### 6. Internal wrench cancellation

At a joint, parent and child carry equal opposite interface wrenches. Constraint-compatible virtual velocities coincide in forbidden directions, so the pair's virtual powers cancel. Only the wrench component paired with permitted relative joint motion survives as generalized joint effort.

## Module 8B solutions

### 1. Skew-symmetry

With the Christoffel-compatible $C$,

$$
(\dot M-2C)_{ij}
=\sum_k\left(
-\frac{\partial M_{ik}}{\partial q_j}
+\frac{\partial M_{jk}}{\partial q_i}
\right)\dot q_k.
$$

Exchanging $i,j$ negates the expression, so

$$
(\dot M-2C)_{ji}=-(\dot M-2C)_{ij}.
$$

### 2. Full energy balance

Using the skew identity,

$$
\dot K=\dot q^T(M\ddot q+c),
\qquad
\dot U=\dot q^Tg.
$$

Premultiplying the dynamics gives

$$
\dot K+\dot U
=\dot q^TBu+\dot q^T\tau_{\mathrm{ext}}
-\dot q^T\tau_f.
$$

Since $\tau_{\mathrm{ext}}=\sum_iJ_i^TF_i$,

$$
\dot q^T\tau_{\mathrm{ext}}=\sum_iV_i^TF_i.
$$

### 3. Coordinate transformation

For $q=\varphi(s)$ and $E=\partial q/\partial s$,

$$
\dot q=E\dot s,
\qquad
\ddot q=E\ddot s+\dot E\dot s.
$$

Premultiply the $q$ dynamics by $E^T$ and identify coefficients:

$$
M_s=E^TM_qE,
$$

$$
c_s=E^T(M_q\dot E\dot s+c_q),
$$

$$
g_s=E^Tg_q,
\qquad
Q_s=E^TQ_q.
$$

### 4. Coriolis versus centrifugal labels

A nonlinear coordinate transformation mixes squared and cross velocity products through $\dot E\dot s$. Therefore a term called centrifugal in one chart can become a combination of terms in another. The total connection-induced vector $c$ is the covariant physical object.

### 5. Pendulum linearizations

Let $I_O=I_C+mr^2$ and

$$
I_O\ddot q+mgr\cos q=\tau.
$$

At the hanging equilibrium $q_*=-\pi/2$,

$$
\cos(-\pi/2+\delta q)\approx\delta q,
$$

so

$$
I_O\delta\ddot q+mgr\delta q=\delta\tau,
$$

which is restoring. At the inverted equilibrium $q_*=\pi/2$,

$$
I_O\delta\ddot q-mgr\delta q=\delta\tau,
$$

which is unstable without feedback.

### 6. Independent validation suite

Use analytical identities rather than only cross-library equality:

- symmetry and Cholesky tests for $M$;
- finite-difference potential gradient for $g$;
- velocity scaling $c(q,\alpha v)=\alpha^2c(q,v)$;
- finite-difference $\dot M$ and skew-power test;
- inverse/forward dynamics round trip;
- external wrench power equality;
- zero-input energy conservation;
- limiting configurations with hand-computable answers.

## Module 9A solutions

### 1. Vertical prismatic mass

Newton gives $m\ddot q=u-mg$. Lagrange uses

$$
K=\frac12m\dot q^2,
\qquad
U=mgq,
$$

and yields the same result:

$$
m\ddot q+mg=u.
$$

### 2. Pendulum centripetal acceleration with $c=0$

The tangential generalized direction is orthogonal to radial centripetal acceleration. The joint constraint supplies radial force. Since the scalar generalized inertia $I_C+mr^2$ is constant, no velocity-dependent generalized force remains and $c=0$.

### 3. Polar Jacobian and mass matrix

The columns of

$$
J=
\begin{bmatrix}
-r\sin\theta&\cos\theta\\
r\cos\theta&\sin\theta
\end{bmatrix}
$$

are orthogonal with squared norms $r^2$ and $1$. Therefore

$$
mJ^TJ=\begin{bmatrix}mr^2&0\\0&m\end{bmatrix}=M.
$$

### 4. Polar equations from energy

Using

$$
K=\frac12m(r^2\dot\theta^2+\dot r^2),
\qquad
U=mgr\sin\theta,
$$

Euler–Lagrange gives

$$
mr^2\ddot\theta+2mr\dot r\dot\theta+mgr\cos\theta=\tau_\theta,
$$

$$
m\ddot r-mr\dot\theta^2+mg\sin\theta=F_r.
$$

### 5. Polar $C$ verification

For

$$
C=
\begin{bmatrix}
mr\dot r&mr\dot\theta\\
-mr\dot\theta&0
\end{bmatrix},
$$

matrix multiplication gives the stated $c$. Also,

$$
\dot M-2C
=\begin{bmatrix}
0&-2mr\dot\theta\\
2mr\dot\theta&0
\end{bmatrix},
$$

which is skew-symmetric.

### 6. Point payload on a planar 2R arm

For a point payload at the tool,

$$
\Delta M=m_pJ_p^TJ_p.
$$

Expanding gives

$$
\Delta M_{11}
=m_p(l_1^2+l_2^2+2l_1l_2\cos q_2),
$$

$$
\Delta M_{12}
=m_p(l_2^2+l_1l_2\cos q_2),
$$

$$
\Delta M_{22}=m_pl_2^2.
$$

Symmetry gives $\Delta M_{21}=\Delta M_{12}$.

## Module 10A solutions

### 1. Constraint generalized force

Admissible virtual displacement satisfies $J_c\delta q=0$. A generalized force orthogonal to every vector in $\mathcal N(J_c)$ lies in $\mathcal R(J_c^T)$. Hence

$$
\tau_c=J_c^T\lambda,
$$

and its admissible virtual work is $\lambda^TJ_c\delta q=0$.

### 2. Saddle point and Schur complement

Combine

$$
M\ddot q+h=Bu+\tau_{\mathrm{ext}}+J_c^T\lambda
$$

with

$$
J_c\ddot q=-\dot J_c\dot q
$$

to obtain the block system in [[10A - Constraints Contacts and Underactuation#4. Constrained equations of motion]]. Eliminating $\ddot q$ gives

$$
\lambda=(J_cM^{-1}J_c^T)^{-1}
\left[J_cM^{-1}(h-Bu-\tau_{\mathrm{ext}})-\dot J_c\dot q\right].
$$

### 3. Zero constraint power

$$
P_c=(J_c^T\lambda)^T\dot q
=\lambda^TJ_c\dot q=0.
$$

The result relies on a stationary ideal bilateral constraint and a velocity satisfying the constraint.

### 4. Impact impulse

Use

$$
\dot q^+=\dot q^-+M^{-1}J_c^TP
$$

in the restitution condition

$$
J_c\dot q^+=-eJ_c\dot q^-.
$$

Solving gives

$$
P=-(1+e)(J_cM^{-1}J_c^T)^{-1}J_c\dot q^-.
$$

### 5. Floating-base actuation limit

Joint motors create equal and opposite internal forces between links and cannot create net system spatial momentum. In

$$
M\dot\nu+h=S^T\tau_j+J_c^TF_c,
$$

the first six columns of $S^T$ provide no direct base wrench. External contact or another external wrench is needed to arbitrarily shape base acceleration.

### 6. Constrained inverse-dynamics program

One formulation uses decision variables $(\ddot q,u,\lambda)$:

$$
\min\ \frac12\|J_t\ddot q+\dot J_t\dot q-a_d\|_W^2
+\frac\epsilon2(\|u\|^2+\|\lambda\|^2)
$$

subject to

$$
M\ddot q+h=Bu+J_c^T\lambda,
$$

$$
J_c\ddot q+\dot J_c\dot q=0,
$$

$$
u_{\min}\le u\le u_{\max},
$$

$$
\lambda_n\ge0,
\qquad
\|\lambda_t\|\le\mu\lambda_n.
$$

Use a polyhedral friction cone if a quadratic program is required.

## Capstone problems

### Capstone A: model a 3R arm

Choose a spatial 3R arm, draw its home pose, and:

1. compute $S_1,S_2,S_3$;
2. compute $T(q)$ with PoE;
3. derive $J_s$ and $J_b$;
4. verify them by group finite differences;
5. attach link spatial inertias and compute $M=\sum J_i^T\mathcal G_iJ_i$;
6. compare one inverse-dynamics evaluation with an RNEA implementation.

### Capstone B: operational-space impedance

For a redundant arm:

1. define $e_b=\operatorname{Log}(T^{-1}T_d)^\vee$;
2. compute $J_b,\Lambda_b,\eta_b$;
3. form a bounded pose/twist feedback wrench;
4. add a dynamically consistent posture torque;
5. test frame covariance by changing the tool coordinate frame;
6. test singularity handling and torque saturation;
7. plot task error, joint torque, energy, and smallest Jacobian singular value.
