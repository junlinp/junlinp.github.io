---
layout: post
title: "Equivariant Filter (EqF): A General Filter Design for Systems on Homogeneous Spaces"
date: 2026-09-06 00:00:00 -0000
categories: technical
---

Reading notes on van Goor, Hamel, and Mahony, ["Equivariant Filter (EqF): A General Filter Design for Systems on Homogeneous Spaces"](https://arxiv.org/abs/2107.05193) (IEEE CDC 2020 / arXiv:2107.05193).

The paper's motivation: a lot of estimation problems in robotics live on curved state spaces — attitude on $SO(3)$, pose on $SE(3)$, bearing directions on the circle or sphere. A plain EKF linearizes the estimation error as if it were a vector in $\mathbb{R}^n$, which silently assumes the error space is flat. The Invariant EKF (IEKF) fixes this by requiring the whole system to be *left-invariant* under a group action, which gives clean linear error dynamics — but that's a strong structural requirement that many real systems (e.g. a pose estimator coupled to landmark measurements) simply don't satisfy. The EqF relaxes the requirement to mere *equivariance*, which is enough to still get a valid linearization, and works on a strictly larger class of systems.

Below is the derivation, followed end to end rather than just quoting results — the main chain runs from the lift's defining property through the global error, the (trajectory-independent) error dynamics, fixed-origin linearization, and finally the Kalman–Bucy correction. Then the same machinery is applied to the paper's own worked example.

## Setup and notation

The state lives on a smooth manifold $\xi\in\mathcal{M}$, and the input lives in a vector space $V$, with dynamics

$$
\dot\xi = f_v(\xi)
$$

A Lie group $\mathbf{G}$ acts on $\mathcal{M}$ through a **right action** $\phi:\mathbf{G}\times\mathcal{M}\to\mathcal{M}$, with the convention

$$
\phi(Y,\phi(X,\xi)) = \phi(XY,\xi)
$$

This convention matters: the order of products like $X\hat X^{-1}$ below follows directly from it. For a fixed $\xi$, write $\phi\_\xi(X):=\phi(X,\xi)$, so its differential at the identity is a linear map $D\phi\_\xi(\mathrm{id}):\mathfrak{g}\to T\_\xi\mathcal{M}$, where $\mathfrak{g}=T\_{\mathrm{id}}\mathbf{G}$ is the Lie algebra.

## Why a lift is needed, and where its defining property comes from

$f\_v(\xi)$ is a tangent vector on the (possibly curved) manifold $\mathcal{M}$. The EqF instead places the observer's state on the group, $\hat X\in\mathbf{G}$, so it needs a way to turn a manifold velocity into a Lie-algebra velocity. That's what a **lift** $\Lambda:\mathcal{M}\times V\to\mathfrak{g}$ is for.

Fix a reference state $\xi^\circ\in\mathcal{M}$ and drive a group-valued trajectory by

$$
\dot X = dL_X\Lambda(\phi(X,\xi^\circ),v)
$$

If $\xi:=\phi(X,\xi^\circ)$, differentiating gives $\dot\xi = D\phi\_{\xi^\circ}(X)[\dot X]$. For this to reproduce the real dynamics $\dot\xi=f\_v(\xi)$, $\Lambda$ has to satisfy

$$
D\phi_\xi(\mathrm{id})[\Lambda(\xi,v)] = f_v(\xi)
$$

That's the lift's defining property — it's exactly the condition under which "drive $X$ on the group, then project through $\phi$" reproduces the true dynamics on $\mathcal{M}$. Once it holds, $\phi(X(t),\xi^\circ)=\xi(t)$ for all $t$: the group trajectory really is a lift of the manifold trajectory.

This condition alone doesn't pin $\Lambda$ down uniquely: $D\phi\_\xi(\mathrm{id})$ generally isn't injective, since $\dim\mathbf{G}\geq\dim\mathcal{M}$. The extra constraint placed on that remaining freedom is **equivariance**.

## Equivariance of the lift

$\Lambda$ is required to satisfy

$$
\mathrm{Ad}_X\,\Lambda(\phi(X,\xi),\psi(X,v)) = \Lambda(\xi,v)
$$

where $\psi$ is the group's action on the input space. Equivalently, $\Lambda(\phi(X,\xi),\psi(X,v)) = \mathrm{Ad}\_{X^{-1}}\Lambda(\xi,v)$: transform the state and input through $\phi$ and $\psi$, and the corresponding Lie-algebra velocity transforms through the adjoint representation. This single identity is what later removes all explicit dependence on the observer's own trajectory from the error dynamics — used twice, below.

## Building the observer

The true lifted system is $\dot X = dL\_X\Lambda(\xi,v)$ with $\xi=\phi(X,\xi^\circ)$. A natural observer copies this propagation and adds a correction $\Delta\in\mathfrak{g}$ still to be designed:

$$
\dot{\hat X} = dL_{\hat X}\Lambda(\hat\xi,v) + dR_{\hat X}\Delta, \qquad \hat\xi := \phi(\hat X,\xi^\circ)
$$

The correction enters through *right* translation, $dR\_{\hat X}\Delta$, rather than left — this specific choice is what makes the correction show up as a clean, unadorned $-\Delta$ term once the error dynamics are worked out below, rather than something conjugated by $\hat X$.

## The global error

Rather than a Euclidean difference $\xi-\hat\xi$ (which doesn't make sense on a curved $\mathcal{M}$), define

$$
e = \phi(\hat X^{-1},\xi)
$$

If the estimate is exact ($\hat\xi=\xi$), then $e=\phi(\hat X^{-1},\hat\xi)=\phi(\hat X^{-1},\phi(\hat X,\xi^\circ))=\phi(\hat X\hat X^{-1},\xi^\circ)=\xi^\circ$. So $\hat\xi=\xi \iff e=\xi^\circ$: unlike an ordinary EKF, where zero error means an exact estimate, here the fixed reference point $\xi^\circ$ plays that role, no matter where the physical trajectory has wandered off to. The transformed input $v^\circ := \psi(\hat X^{-1},v)$ is computable directly, since both $v(t)$ and $\hat X(t)$ are known to the filter.

## Deriving the error dynamics

Define the group error $E := X\hat X^{-1}$. Using the right-action convention and $\xi=\phi(X,\xi^\circ)$:

$$
e = \phi(\hat X^{-1},\phi(X,\xi^\circ)) = \phi(X\hat X^{-1},\xi^\circ) = \phi(E,\xi^\circ)
$$

For concreteness, if $\mathbf{G}$ is a matrix group, $dL\_X(\xi)=X\xi$ and $dR\_X(\xi)=\xi X$ are just matrix products, so $E=X\hat X^{-1}$ gives $\dot E = \dot X\hat X^{-1} - X\hat X^{-1}\dot{\hat X}\hat X^{-1}$. Substituting the two ODEs and rearranging:

$$
\dot E = dL_E\Big[\mathrm{Ad}_{\hat X}\Lambda(\xi,v) - \mathrm{Ad}_{\hat X}\Lambda(\hat\xi,v) - \Delta\Big]
$$

$\hat X$ still appears explicitly here — this is exactly where equivariance is needed to make progress. Since $E=X\hat X^{-1}$, we have $X=E\hat X$, so

$$
\xi = \phi(E\hat X,\xi^\circ) = \phi(\hat X,\phi(E,\xi^\circ)) = \phi(\hat X,e)
$$

and likewise, inverting $v^\circ=\psi(\hat X^{-1},v)$ gives $v=\psi(\hat X,v^\circ)$. Now apply the equivariance identity with $X=\hat X$, state $e$, input $v^\circ$:

$$
\mathrm{Ad}_{\hat X}\Lambda(\phi(\hat X,e),\psi(\hat X,v^\circ)) = \Lambda(e,v^\circ)
$$

Since $\phi(\hat X,e)=\xi$ and $\psi(\hat X,v^\circ)=v$, this reads $\mathrm{Ad}\_{\hat X}\Lambda(\xi,v) = \Lambda(e,v^\circ)$. The same move applied to $\hat\xi=\phi(\hat X,\xi^\circ)$ gives $\mathrm{Ad}\_{\hat X}\Lambda(\hat\xi,v) = \Lambda(\xi^\circ,v^\circ)$. Substituting both:

$$
\dot E = dL_E\Big[\Lambda(e,v^\circ) - \Lambda(\xi^\circ,v^\circ) - \Delta\Big]
$$

The dependence on $\hat X$ has disappeared completely — this cancellation is the central mechanical step of the whole method, and it's exactly what equivariance was imposed to guarantee.

Since $e=\phi(E,\xi^\circ)$ and $\dot E = dL\_E U$ for $U:=\Lambda(e,v^\circ)-\Lambda(\xi^\circ,v^\circ)-\Delta$, the chain rule gives $\dot e = D\phi\_{\xi^\circ}(E)[\dot E] = D\phi\_{\xi^\circ}(E)\,dL\_E[U]$. The identity $D\phi\_{\xi^\circ}(E)\,dL\_E = D\phi\_e(\mathrm{id})$ — proved next — lets this be written as $d\phi\_e[U]$:

$$
\dot e = d\phi_e\Big(\Lambda(e,v^\circ) - \Lambda(\xi^\circ,v^\circ) - \Delta\Big)
$$

This has the form $\dot e = F(e,v^\circ,\Delta)$ rather than $F(e,\hat\xi,v,\Delta)$: it depends only on the current error, the reference-frame velocity, and the filter's own correction — never on the unmeasurable true trajectory. And setting $\Delta=0$, $e=\xi^\circ$ gives $\dot e=0$ for every $v^\circ$: the reference point is an equilibrium of the noise-free error dynamics, exactly as it should be.

## Why $D\phi\_{\xi^\circ}(E)\,dL\_E = D\phi\_e(\mathrm{id})$

This is worth actually proving rather than waving through, since it's used to project the group-level result back onto $\mathcal{M}$. Let $Y\in\mathbf{G}$ be near the identity. Since $e=\phi(E,\xi^\circ)$, the right-action convention gives

$$
\phi(Y,e) = \phi(Y,\phi(E,\xi^\circ)) = \phi(EY,\xi^\circ)
$$

i.e. $\phi\_e(Y) = \phi\_{\xi^\circ}(L\_E(Y))$, or $\phi\_e = \phi\_{\xi^\circ}\circ L\_E$ as functions of $Y$. Differentiating at $Y=\mathrm{id}$ by the chain rule, using $L\_E(\mathrm{id})=E$ and $DL\_E(\mathrm{id})=dL\_E$:

$$
D\phi_e(\mathrm{id}) = D\phi_{\xi^\circ}(E)\,dL_E
$$

A second way to see it, more concretely: for any $\delta\in\mathfrak{g}$, let $Y(t)=\exp(t\delta)$. Then

$$
D\phi_{\xi^\circ}(E)\,dL_E[\delta] = \left.\frac{d}{dt}\right|_{t=0}\phi(E\exp(t\delta),\xi^\circ) = \left.\frac{d}{dt}\right|_{t=0}\phi(\exp(t\delta),\phi(E,\xi^\circ)) = \left.\frac{d}{dt}\right|_{t=0}\phi(\exp(t\delta),e) = D\phi_e(\mathrm{id})[\delta]
$$

Since this holds for every $\delta$, the two linear maps agree. Geometrically: moving $\delta$ from the identity to $E$ via $dL\_E$ and then projecting through $D\phi\_{\xi^\circ}(E)$ produces the same tangent vector at $e$ as just applying the infinitesimal action directly at $e$ — which makes sense, since $e$ *is* the image of $E$ under $\phi\_{\xi^\circ}$.

## Linearizing on the manifold

$e\in\mathcal{M}$ isn't generally a vector, so pick a local chart $\varepsilon:\mathcal{U}\to\mathbb{R}^M$ around $\xi^\circ$ with $\varepsilon(\xi^\circ)=0$, and set $\varepsilon:=\varepsilon(e)$. Define the drift relative to the origin,

$$
\tilde\Lambda_{\xi^\circ}(e,v^\circ) := \Lambda(e,v^\circ) - \Lambda(\xi^\circ,v^\circ)
$$

so that $\dot e = d\phi\_e\tilde\Lambda\_{\xi^\circ}(e,v^\circ) - d\phi\_e\Delta$, and in local coordinates $\dot\varepsilon = d\varepsilon\,d\phi\_e\tilde\Lambda\_{\xi^\circ}(e,v^\circ) - d\varepsilon\,d\phi\_e\Delta$. Write $F(e):=d\phi\_e\tilde\Lambda\_{\xi^\circ}(e,v^\circ)$. Since $\tilde\Lambda\_{\xi^\circ}(\xi^\circ,v^\circ)\equiv 0$ by construction, $F(\xi^\circ)=0$ exactly — not approximately. Applying the product rule to $F$ near $e=\xi^\circ$ produces two terms, one from differentiating $d\phi\_e$ and one from differentiating $\tilde\Lambda\_{\xi^\circ}$; the first vanishes at $e=\xi^\circ$ precisely because $\tilde\Lambda\_{\xi^\circ}(\xi^\circ,v^\circ)=0$, leaving only

$$
DF(\xi^\circ) = D\phi_{\xi^\circ}(\mathrm{id})\,D_{\xi^\circ}\Lambda(\xi^\circ,v^\circ)
$$

Substituting $\delta e = d\varepsilon^{-1}\varepsilon$ gives the linear(ized) model

$$
\dot\varepsilon \approx A^\circ_t\varepsilon - d\varepsilon\,D\phi_{\xi^\circ}(\mathrm{id})[\Delta], \qquad A^\circ_t := d\varepsilon\,D\phi_{\xi^\circ}(\mathrm{id})\,D_{\xi^\circ}\Lambda(\xi^\circ,v^\circ)\,d\varepsilon^{-1}
$$

## What's actually different from an EKF

In an ordinary EKF, $A\_t=\partial f/\partial x$ evaluated at the *moving* estimate $\hat x(t)$, so $A\_t=A(\hat x(t),u(t))$. Here, $A^\circ\_t$ is always evaluated at the *fixed* $\xi^\circ$; only the transformed input $v^\circ(t)$ changes with time:

$$
A^\circ_t = A(v^\circ(t)), \quad \text{not} \quad A(\hat\xi(t),v(t))
$$

This is the paper's fixed-origin linearization, and it's the whole point: the linearization point never moves, so $A^\circ\_t$ doesn't have to be recomputed around a drifting estimate the way an EKF's Jacobian does.

## Linearizing the measurement

Inverting the error relation gives $\xi=\phi(\hat X,e)$, so $h(\xi)=h(\phi(\hat X,\varepsilon^{-1}(\varepsilon)))$. At $\varepsilon=0$ (i.e. $e=\xi^\circ$), this is $h(\phi(\hat X,\xi^\circ))=h(\hat\xi)$, so the residual $h(\xi)-h(\hat\xi)$ vanishes at $\varepsilon=0$ and, to first order,

$$
h(\xi)-h(\hat\xi) \approx C_t\varepsilon, \qquad C_t := Dh(\hat\xi)\,D\phi_{\hat X}(\xi^\circ)\,d\varepsilon^{-1}
$$

Notably, $A^\circ\_t$ never depends on $\hat\xi$, but $C\_t$ generally still does (through $\hat X$) — the fixed-origin trick only applies to the dynamics matrix, not the output matrix.

## From the linear error system to a Kalman correction

The linearized system is $\dot\varepsilon = A^\circ\_t\varepsilon - B\Delta$ with $B:=d\varepsilon\,D\phi\_{\xi^\circ}(\mathrm{id})$, and the measurement model is $r:=h(\xi)-h(\hat\xi)\approx C\_t\varepsilon$. For a standard continuous-time (Kalman–Bucy) observer the gain is $K=\Sigma C\_t^\top Q^{-1}$, so the EqF chooses $\Delta$ to make $B\Delta = \Sigma C\_t^\top Q^{-1}r$.

Recovering $\Delta\in\mathfrak{g}$ from this means inverting $D\phi\_{\xi^\circ}(\mathrm{id}):\mathfrak{g}\to T\_{\xi^\circ}\mathcal{M}$, which in general isn't square since $\dim\mathbf{G}\geq\dim\mathcal{M}$. Choosing a right inverse $D\phi\_{\xi^\circ}(\mathrm{id})^\dagger$ satisfying $D\phi\_{\xi^\circ}(\mathrm{id})\,D\phi\_{\xi^\circ}(\mathrm{id})^\dagger = I$, the correction is

$$
\Delta = D\phi_{\xi^\circ}(\mathrm{id})^\dagger\,d\varepsilon^{-1}\,\Sigma C_t^\top Q^{-1}\big(h(\xi)-h(\hat\xi)\big)
$$

Substituting back, $d\varepsilon\,D\phi\_{\xi^\circ}(\mathrm{id})[\Delta]$ collapses (the right-inverse property cancels $D\phi\_{\xi^\circ}(\mathrm{id})\,D\phi\_{\xi^\circ}(\mathrm{id})^\dagger$ down to the identity) to exactly $\Sigma C\_t^\top Q^{-1}r$, leaving

$$
\dot\varepsilon \approx \big(A^\circ_t - \Sigma C_t^\top Q^{-1}C_t\big)\varepsilon
$$

— precisely the Kalman–Bucy observer structure.

## The Riccati equation

For the standard continuous-time linear-Gaussian system $\dot x=Ax+w$, $y=Cx+n$ with $\mathbb{E}[ww^\top]=P$, $\mathbb{E}[nn^\top]=Q$, the Kalman–Bucy covariance satisfies $\dot\Sigma = A\Sigma+\Sigma A^\top+P-\Sigma C^\top Q^{-1}C\Sigma$. Substituting the EqF's own $A^\circ\_t$ and $C\_t$:

$$
\dot\Sigma = A^\circ_t\Sigma + \Sigma(A^\circ_t)^\top + P - \Sigma C_t^\top Q^{-1}C_t\Sigma
$$

with $P$, $Q$ playing the role of process/measurement noise weights (or just tuning matrices, in practice).

## The complete EqF equations

Propagation: $\dot{\hat X} = dL\_{\hat X}\Lambda(\hat\xi,v) + dR\_{\hat X}\Delta$, with $\hat\xi=\phi(\hat X,\xi^\circ)$.

Correction: $\Delta = D\phi\_{\xi^\circ}(\mathrm{id})^\dagger\,d\varepsilon^{-1}\,\Sigma C\_t^\top Q^{-1}\big[h(\xi)-h(\hat\xi)\big]$.

Covariance: $\dot\Sigma = A^\circ\_t\Sigma + \Sigma(A^\circ\_t)^\top + P - \Sigma C\_t^\top Q^{-1}C\_t\Sigma$.

with $A^\circ\_t = d\varepsilon\,D\phi\_{\xi^\circ}(\mathrm{id})\,D\_{\xi^\circ}\Lambda(\xi^\circ,v^\circ)\,d\varepsilon^{-1}$, $C\_t = Dh(\hat\xi)\,D\phi\_{\hat X}(\xi^\circ)\,d\varepsilon^{-1}$, and $v^\circ=\psi(\hat X^{-1},v)$.

Zooming out, the whole construction is: symmetry-based nonlinear error (the main innovation) $+$ fixed-origin linearization (the key benefit) $+$ a Kalman–Bucy gain (the classical component). And the one identity that makes it all work is $\mathrm{Ad}\_{\hat X}\Lambda(\xi,v) = \Lambda(e,v^\circ)$ — it's what removes the moving estimate from the nonlinear error dynamics in the first place, and everything downstream (fixed-origin $A^\circ\_t$, the whole Kalman-style correction) depends on that cancellation having happened.

## Worked example: bearing-only landmark tracking

Section 6 of the paper works through a small concrete example: a robot tracking $n$ stationary landmarks using only their bearing (direction), expressed in its own body frame. The design principle at work: choose the symmetry group to match the *geometry of the measurement*, rather than forcing a manifold-valued state into Euclidean EKF coordinates.

**State, group, dynamics, output.** Each landmark's position in the body frame is $x\_i\in\mathbb{R}^2\setminus\lbrace 0\rbrace$, so $\mathcal{M} = (\mathbb{R}^2\setminus\lbrace 0\rbrace)^n$. As the robot moves, $\dot x\_i = -v$ for a shared inertial velocity $v$ — but this raw system isn't equivariant under the natural symmetry group (rotating/rescaling one landmark's coordinate doesn't commute cleanly with the shared $v$). The fix is to extend the velocity per-landmark: $\dot x\_i = f\_{v\_i}(x\_i) := -v\_i$, with $v\_i$ all sharing the true value $v$ but treated as independent lift-level quantities — extend the input space until the system is equivariant.

The symmetry group is $\mathbf{G} = (S^1\times\mathbb{R}\_{>0})^n$ (an independent rotation and positive scale per landmark), acting as

$$
\phi((\theta_i,a_i), x_i) := a_i^{-1}R(\theta_i)^\top x_i
$$

and the measurement is the bearing $y\_i = h(x\_i) := x\_i/\lVert x\_i\rVert \in S^1$.

**The lift.**

$$
\Lambda(x_i,v_i) = \left(\frac{x_i^\top S v_i}{x_i^\top x_i},\; -\frac{x_i^\top v_i}{x_i^\top x_i}\right), \qquad S=\begin{pmatrix}0&-1\\1&0\end{pmatrix}
$$

Decomposing $v\_i$ in the orthogonal basis $\lbrace x\_i, Sx\_i\rbrace$ as $v\_i = \alpha x\_i + \beta Sx\_i$, the first component of $\Lambda$ picks out $-\beta$ (the rotation rate needed to explain the part of $v\_i$ perpendicular to $x\_i$) and the second picks out $-\alpha$ (the scale rate needed to explain the part parallel to $x\_i$): the rotational component describes angular motion, the scale component describes radial motion.

**Deriving $C\_i$ (the output matrix), fully.** Three ingredients:

1. The normalization map has Jacobian $Dh(x) = \frac{1}{\lVert x\rVert}\left(I\_2 - \frac{xx^\top}{\lVert x\rVert^2}\right)$ — the projector onto the tangent of the unit circle at $x/\lVert x\rVert$.
2. $\phi((\hat\theta\_i,\hat a\_i),\xi) = \hat a\_i^{-1}R(\hat\theta\_i)^\top\xi$ is *linear* in $\xi$, so its derivative with respect to $\xi$ is just itself: $D\phi\_{\hat X}(\xi^\circ) = \hat a\_i^{-1}R(\hat\theta\_i)^\top$.
3. $h$ has a clean equivariance property: $h(\hat a\_i^{-1}R(\hat\theta\_i)^\top x) = R(\hat\theta\_i)^\top h(x)$ (rotating and positively rescaling a vector rotates its direction the same way; a positive scale never flips it). Differentiating both sides with respect to $x$ and evaluating at $x\_i^\circ$:

$$
Dh(\hat a_i^{-1}R(\hat\theta_i)^\top x_i^\circ)\cdot\hat a_i^{-1}R(\hat\theta_i)^\top = R(\hat\theta_i)^\top\cdot Dh(x_i^\circ)
$$

The left side is exactly $Dh(\hat\xi\_i)\cdot D\phi\_{\hat X}(\xi\_i^\circ)$ from the general formula, so with $d\varepsilon=\mathrm{id}$ (the chart here is just the ambient $\mathbb{R}^2$):

$$
C_i = R(\theta_i)^\top\,\frac{1}{\lVert x_i^\circ\rVert}\left(I_2 - \frac{x_i^\circ x_i^{\circ\top}}{x_i^{\circ\top}x_i^\circ}\right)
$$

which matches the paper's stated result exactly.

**Deriving $A\_i$ (the state matrix).** Two ingredients composed together:

1. $D\_{x\_i}\Lambda(x\_i,v\_i)$, via the quotient rule. Writing $n(x):=x^\top x$ (so $\nabla n = 2x$), the gradient of the first component of $\Lambda$ is $\frac{1}{n^2}\big(nSv - 2(x^\top Sv)x\big)$ and the gradient of the second is $\frac{1}{n^2}\big(-nv + 2(x^\top v)x\big)$. Stacked as rows, that's the $2\times 2$ Jacobian $D\_x\Lambda(x,v)$.
2. $D\phi\_x(\mathrm{id})$, the derivative of the action at the identity. Rather than differentiate the action formula directly (which risks a sign convention mismatch depending on exactly how the exponential coordinates are set up), it's cleaner to pin it down from its defining property: $D\phi\_x(\mathrm{id})[\Lambda(x,v)] = f\_v(x) = -v$ must hold for *every* $v$. Using the same $\lbrace x,Sx\rbrace$ decomposition as above (with $x^\top Sx=0$ and $S^2=-I$), solving this requirement shows $D\phi\_x(\mathrm{id})[(\omega,\rho)] = \omega Sx + \rho x$ is the map consistent with it.

Composing the two (again $d\varepsilon=\mathrm{id}$), $A\_i = D\phi\_x(\mathrm{id})\circ D\_x\Lambda(x,v)$ works out to

$$
A_i = \frac{1}{\lVert x^\circ\rVert^4}\Big[\lVert x^\circ\rVert^2\big(Sx^\circ(Sv^\circ)^\top - x^\circ v^{\circ\top}\big) - 2(x^{\circ\top}Sv^\circ)(Sx^\circ)x^{\circ\top} + 2(x^{\circ\top}v^\circ)x^\circ x^{\circ\top}\Big]
$$

This has exactly the same term-for-term structure as the paper's stated $A\_i$ (the same four outer-product terms), differing only by an overall sign. Worth being upfront about that: the sign traces back to the exact convention used for the group action / lift, and since I'm working partly from an HTML-rendered transcription of the paper rather than the original typeset PDF, I can't fully rule out a sign flip somewhere in the source. The derivation mechanism and the fact that it reproduces the paper's algebraic structure term-for-term are what matter here — a reasonable reminder that sign conventions in Lie-group papers are exactly the kind of detail worth double-checking against the primary source before using in production code.

$A^\circ\_t = \mathrm{diag}(A\_i)$ is block-diagonal — the landmarks decouple completely, since neither the dynamics nor the per-landmark group action have any cross-landmark terms.

**Observability and parallax.** The paper shows the linearized error system is uniformly observable when the camera motion provides enough excitation, roughly of the form

$$
\frac{1}{\tau}\int_t^{t+\tau}\big(v(s)^\top Sx_i(s)\big)^2\,ds \geq \delta
$$

The intuition: if the camera moves straight along the line of sight to a landmark ($v\parallel x\_i$), the bearing barely changes and depth stays unobservable; lateral motion ($v^\top Sx\_i\neq 0$) changes the bearing and makes depth recoverable. This is exactly the classical monocular-SLAM parallax condition, and it's a direct, checkable consequence of the $A\_i$/$C\_i$ matrices derived above — the same $v^\top Sx\_i$ term that shows up in $A\_i$ is what governs observability.

**Simulation and result.** Four landmarks with initial positions drawn uniformly from $[-0.5,0.5]\times[1.0,2.0]$, robot velocity $v=(2\cos(2t),0)$, a random origin offset in $[-1,1]\times[-1,1]$, filter gains $P\_i=0.02^2I\_2$, $Q\_i=0.01^2I\_2$, $\Sigma\_i(0)=4^2I\_2$, Euler integration at $dt=0.01$. No comparison against a plain EKF or IEKF is run — the paper states the goal is to demonstrate the design procedure rather than benchmark performance. The reported result (Fig. 1) is convergence of each landmark's local Lyapunov function, with the convergence rate speeding up and slowing down periodically (period $\pi$ seconds) — tracking the system's observability, which scales with the square of the velocity, matching the parallax condition above.

**Why this needed the EqF specifically.** In the paper's own words, the lifted system here "was shown to be equivariant, but it is not invariant nor group affine, precluding the use of existing observer design methodologies." IEKF-style designs need the stronger invariance/group-affine property, which this system doesn't have, but the EqF's weaker equivariance requirement is satisfied — exactly the class of problem the paper is arguing this framework opens up.

## Significance for VIO, and closing thoughts

A typical VIO state might be $x=(R,p,v,b\_g,b\_a,\ldots)$, and an error-state Kalman filter commonly defines $R=\hat R\exp(\delta\theta)$, $p=\hat p+\delta p$, $v=\hat v+\delta v$ — already geometrically better than a naive Euclidean EKF, since at least the attitude error lives on the right manifold. The EqF pushes the same idea further: ask whether the *entire* system (pose, velocity, landmarks, biases) admits a symmetry under which all of their error dynamics simultaneously become closer to autonomous — not just individually well-behaved, but jointly decoupled from the moving estimate the way the fixed-origin $A^\circ\_t$ is above. That question underlies invariant EKFs, equivariant VIO, and symmetry-based SLAM more broadly: when a filter degrades under large rotations or large initial errors, the fix isn't always a faster update rate — sometimes it's redesigning the *error coordinates* so the linearization matches the system's actual geometry.

The mental model, compressed: an ordinary EKF repeatedly linearizes around the moving estimate $\hat x(t)$; the EqF uses symmetry to transport the error back to a fixed origin first, then runs an ordinary Kalman filter on the resulting (now well-behaved) global error system. It's primarily a linearization-quality improvement, not a different Riccati equation — which is exactly why it matters most in the SO(3)/SE(3)/S²-flavored problems (SLAM, VIO) where naive linearization is the thing that actually breaks.

The remaining caveats are the usual ones for any Kalman-family filter on a manifold: the linearization is a small-error approximation (large initial errors aren't covered by this analysis), and the choice of lift $\Lambda$ for a given group/action pair isn't unique — different valid lifts give different (though presumably comparably effective) filters, and picking a good one is still something of an art.

<script>
  window.MathJax = {
    tex: {
      inlineMath: [['$', '$'], ['\\(', '\\)']],
      displayMath: [['$$', '$$'], ['\\[', '\\]']]
    }
  };
</script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
