---
layout: post
title: "Equivariant Filter (EqF): A General Filter Design for Systems on Homogeneous Spaces"
date: 2026-09-06 00:00:00 -0000
categories: technical
---

Reading notes on van Goor, Hamel, and Mahony, ["Equivariant Filter (EqF)"](https://arxiv.org/abs/2010.14666) (arXiv:2010.14666; published as *IEEE Transactions on Automatic Control*, vol. 68, no. 6, 2023). This is the full journal version — it supersedes the earlier CDC 2020 conference paper (arXiv:2107.05193) that a first draft of this post was based on, and it adds a genuinely useful refinement, **EqF⋆**, that the conference version didn't have.

The paper's motivation: a lot of estimation problems in robotics live on curved state spaces — attitude on $SO(3)$, pose on $SE(3)$, bearing directions on the circle or sphere. A plain EKF linearizes the estimation error as if it were a vector in $\mathbb{R}^n$, which silently assumes the error space is flat. The Invariant EKF (IEKF) fixes this by requiring the whole system to be *left-invariant* under a group action, which gives clean linear error dynamics — but that's a strong structural requirement that many real systems (e.g. a pose estimator coupled to landmark measurements) simply don't satisfy. The EqF relaxes the requirement to mere *equivariance*, which is enough to still get a valid linearization, and works on a strictly larger class of systems. (The paper proves this precisely in an appendix: EqF specializes exactly to the IEKF whenever the system happens to be posed directly on a Lie group with group-affine dynamics, the origin is the identity, and the chart is the exponential map — IEKF is the special case, not a separate design.)

Below is the derivation, followed end to end rather than just quoting results — the main chain runs from the lift's defining property through the global error, the (trajectory-independent) error dynamics, fixed-origin linearization, and finally the Kalman–Bucy correction. Along the way there's also **EqF⋆**: a second, sharper output linearization the paper introduces that costs nothing extra to compute but removes an entire order of linearization error. Then the same machinery (both EqF and EqF⋆) is applied to the paper's own worked example.

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

## EqF⋆: a sharper output linearization for free

$C\_t$ above comes from a first-order Taylor expansion of $h$ around a single point ($\hat\xi$), so the residual it drops is $O(\lVert\varepsilon\rVert^2)$ — same as an ordinary EKF's output linearization error. The paper's Lemma 5.3 shows that when the output itself is equivariant (i.e. there's an action $\rho$ on the measurement space with $\rho(X,h(\xi))=h(\phi(X,\xi))$ — exactly the condition already needed to define $C\_t$ in the first place, nothing extra to assume), you can do strictly better *for free*: evaluate the same linearization at **two** points and average them, and the leading error term cancels, leaving $O(\lVert\varepsilon\rVert^3)$.

It's worth seeing why averaging buys a whole extra order, since it's a completely generic calculus fact and not specific to manifolds at all. For smooth $g$ and small $\Delta x$, expand $g(x\_0+\Delta x)$ to third order: $g(x\_0+\Delta x) = g(x\_0) + g'(x\_0)\Delta x + \tfrac{1}{2}g''(x\_0)\Delta x^2 + \tfrac{1}{6}g'''(x\_0)\Delta x^3 + O(\Delta x^4)$.

- **Single-point derivative**: approximating $g(x\_0+\Delta x)-g(x\_0)$ by $g'(x\_0)\Delta x$ leaves an error of $\tfrac{1}{2}g''(x\_0)\Delta x^2 + O(\Delta x^3)$ — quadratic error, exactly the $O(\lVert\varepsilon\rVert^2)$ that $C\_t$ leaves.
- **Averaged derivative**: also expand $g'(x\_0+\Delta x) = g'(x\_0) + g''(x\_0)\Delta x + O(\Delta x^2)$, so $\tfrac{1}{2}(g'(x\_0)+g'(x\_0+\Delta x)) = g'(x\_0) + \tfrac{1}{2}g''(x\_0)\Delta x + O(\Delta x^2)$. Multiplying by $\Delta x$: $g'(x\_0)\Delta x + \tfrac{1}{2}g''(x\_0)\Delta x^2 + O(\Delta x^3)$ — which now matches $g(x\_0+\Delta x)-g(x\_0)$ through the *quadratic* term exactly. The mismatch has shrunk from $O(\Delta x^2)$ to $O(\Delta x^3)$.

That's the entire mechanism (it's the same reason the trapezoidal rule beats the rectangle rule by an order): the quadratic term in the Taylor series is exactly what a single-point derivative can't see, and averaging the derivative at both endpoints is precisely what's needed to pick it up.

Mapped onto the EqF: the "two points" are the estimated output $\hat y = h(\hat\xi)$ (known to the filter) and the actual measurement $y$ (which arrives with the update — using it here is legitimate, since it doesn't require anything the filter doesn't already have at correction time). The paper's exact construction (its eq. 35) replaces $C\_t$ with

$$
C^\star_t\,\varepsilon := \tfrac{1}{2}\Big(D_E\big|_{\mathrm{id}}\,\rho(E,y) + D_E\big|_{\mathrm{id}}\,\rho(E,\hat y)\Big)\,\mathrm{Ad}_{\hat X^{-1}}\varepsilon^\wedge
$$

— averaging the derivative of the output-equivariance action $\rho$ evaluated at $y$ and at $\hat y$, in exactly the same spirit as the scalar calculus argument above. Using $C^\star\_t$ in place of $C\_t$ anywhere below — in the correction $\Delta$, the Riccati equation, all of it — is a drop-in replacement; nothing else about the filter changes. The resulting algorithm is what the paper calls **EqF⋆**, and its output residual is $O(\lVert\varepsilon\rVert^3)$ instead of $O(\lVert\varepsilon\rVert^2)$ — a strictly better local approximation, obtained by reusing information (the actual measurement) the filter already has at update time, at essentially no extra cost.

## From the linear error system to a Kalman correction

The linearized system is $\dot\varepsilon = A^\circ\_t\varepsilon - B\Delta$ with $B:=d\varepsilon\,D\phi\_{\xi^\circ}(\mathrm{id})$, and the measurement model is $r:=h(\xi)-h(\hat\xi)\approx C\_t\varepsilon$ (or $\approx C^\star\_t\varepsilon$, using the sharper linearization from above — everything from here on reads identically either way, so I'll just write $C\_t$). For a standard continuous-time (Kalman–Bucy) observer the gain is $K=\Sigma C\_t^\top Q^{-1}$, so the EqF chooses $\Delta$ to make $B\Delta = \Sigma C\_t^\top Q^{-1}r$.

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

with $A^\circ\_t = d\varepsilon\,D\phi\_{\xi^\circ}(\mathrm{id})\,D\_{\xi^\circ}\Lambda(\xi^\circ,v^\circ)\,d\varepsilon^{-1}$, $C\_t = Dh(\hat\xi)\,D\phi\_{\hat X}(\xi^\circ)\,d\varepsilon^{-1}$, and $v^\circ=\psi(\hat X^{-1},v)$. Swap in $C^\star\_t$ everywhere $C\_t$ appears here (correction, Riccati, both) and this is the EqF⋆ instead — same equations, better linearization.

Zooming out, the whole construction is: symmetry-based nonlinear error (the main innovation) $+$ fixed-origin linearization (the key benefit) $+$ a Kalman–Bucy gain (the classical component). And the one identity that makes it all work is $\mathrm{Ad}\_{\hat X}\Lambda(\xi,v) = \Lambda(e,v^\circ)$ — it's what removes the moving estimate from the nonlinear error dynamics in the first place, and everything downstream (fixed-origin $A^\circ\_t$, the whole Kalman-style correction) depends on that cancellation having happened.

## Worked example: single-bearing direction estimation on $S^2$

The paper's own worked example is deliberately minimal: estimate the direction (unit vector) of a fixed reference — say, the local magnetic field — as seen from a rotating platform whose angular velocity is measured. It's small enough to carry every derivation through by hand, and it's a genuine homogeneous-space problem that plain IEKF *cannot* handle at all: $S^2$ isn't a Lie group, and IEKF (per the specialization result mentioned in the intro) only applies to group-affine systems posed directly on a group. This is exactly the kind of problem EqF exists for.

**State, group, dynamics, output.** The state is a unit vector $\eta\in S^2\subset\mathbb{R}^3$ — the bearing, expressed in the rotating body frame. The symmetry group is $\mathbf{G}=SO(3)$, acting by $\phi(R,\eta):=R^\top\eta$. A vector fixed in the inertial frame but expressed in a frame rotating with (known, measured) angular velocity $\Omega$ evolves as $\dot\eta = -\Omega^\wedge\eta$. The sensor is a magnetometer reading $h(\eta) := c\_m\eta$ for a known field strength $c\_m$ (i.e. the sensor sees a scaled copy of the bearing direction itself).

**The lift.** $\Lambda(\eta,\Omega) := \Omega^\wedge$ (identifying $\mathfrak{so}(3)$ with $\mathbb{R}^3$, this is just $\Omega$ itself). Checking the defining property directly: $D\phi\_\eta(\mathrm{id})$, the derivative of $R\mapsto R^\top\eta$ at $R=\mathrm{id}$ in the direction $\omega^\wedge$, evaluated at $t=0$, is

$$
\left.\frac{d}{dt}\right|_{t=0}\exp(-t\omega^\wedge)\eta = -\omega^\wedge\eta = -\omega\times\eta
$$

So $D\phi\_\eta(\mathrm{id})[\Lambda(\eta,\Omega)] = -\Omega\times\eta = -\Omega^\wedge\eta = \dot\eta$ — exactly the required dynamics, with no ambiguity (the lift here is forced, just like $G\_O$ in the [INS post](/technical/2026/09/06/equivariant-symmetries-ins.html)'s MEKF example, since there's no leftover freedom once you've matched the one dynamics equation).

**Output equivariance.** Since $h$ is just a linear rescaling of $\eta$, equivariance is immediate: $h(\phi(R,\eta)) = c\_m R^\top\eta = R^\top(c\_m\eta) = R^\top h(\eta)$, so $\rho(R,y):=R^\top y$ satisfies $h(\phi(R,\eta))=\rho(R,h(\eta))$ exactly, with no approximation.

**Deriving $C\_t$ from scratch.** Pick a reference $\eta^\circ$ and, in some fixed orthonormal basis of its tangent plane, parameterize nearby points on $S^2$ by exponential coordinates: $e = \exp(\varepsilon^\wedge)\eta^\circ$ for small $\varepsilon\perp\eta^\circ$. Expanding the matrix exponential, $\exp(\varepsilon^\wedge)\eta^\circ = \eta^\circ + \varepsilon^\wedge\eta^\circ + O(\lVert\varepsilon\rVert^2) = \eta^\circ + \varepsilon\times\eta^\circ + O(\lVert\varepsilon\rVert^2)$ — a small rotation by $\varepsilon$ moves $\eta^\circ$ by $\varepsilon\times\eta^\circ$ to first order. Using $\xi=\phi(\hat X,e)=\hat R^\top e$:

$$
h(\xi) = c_m\hat R^\top e \approx c_m\hat R^\top\eta^\circ + c_m\hat R^\top(\varepsilon\times\eta^\circ) = h(\hat\xi) + c_m\hat R^\top(\varepsilon\times\eta^\circ)
$$

Rewriting $\varepsilon\times\eta^\circ = -\eta^{\circ\wedge}\varepsilon$ gives $h(\xi)-h(\hat\xi)\approx -c\_m\hat R^\top\eta^{\circ\wedge}\varepsilon =: C\_t\varepsilon$. Substituting $\eta^\circ = \frac{1}{c\_m}\hat R\hat y$ (from $\hat y := h(\hat\xi) = c\_m\hat R^\top\eta^\circ$) and using the standard $SO(3)$ identity $(Ra)^\wedge = Ra^\wedge R^\top$:

$$
C_t = -c_m\hat R^\top\Big(\tfrac{1}{c_m}\hat R\hat y\Big)^{\!\wedge} = -c_m\hat R^\top\cdot\tfrac{1}{c_m}\hat R\hat y^\wedge\hat R^\top = -\hat y^\wedge\hat R^\top
$$

This is exactly the structure the paper reports ($C\_t = \hat y^\wedge\hat R^\top$, composed with the tangent-plane embedding), up to the same kind of overall sign convention noted in other worked examples in this series.

**Deriving $C^\star\_t$.** This is where it gets satisfying: the averaging argument from the EqF⋆ section above applies immediately, since we now have both $\hat y$ (used above) and the true measurement $y$ available. Redo the same linearization but evaluate the cross-product term at the *actual* measurement direction instead of the predicted one, then average the two:

$$
C^\star_t = -\tfrac{1}{2}\big(y^\wedge + \hat y^\wedge\big)\hat R^\top
$$

— literally the same $-\hat y^\wedge\hat R^\top$ structure as $C\_t$, just with $\hat y^\wedge$ replaced by the average of $y^\wedge$ and $\hat y^\wedge$, matching the paper's reported $C^\star\_t = \tfrac{1}{2}(y^\wedge+\hat y^\wedge)\hat R^\top$ term-for-term.

**Simulation and result.** The paper drives $\Omega(t) = (0.1\cos(2t),\,0.2\sin(t),\,0)$ rad/s, with initial-state noise $\mathcal{N}(0,0.5^2I\_3)$, gyro noise $\mathcal{N}(0,0.01^2I\_3)$, and magnetometer noise $\mathcal{N}(0,0.05^2I\_3)$, integrated over 5 seconds at $dt=0.01$ across 500 Monte Carlo trials, comparing EqF⋆ against plain EqF and an EKF run in local coordinates. Reported results: even noiseless, EqF⋆ converges visibly faster than EqF, which in turn beats the EKF; under noise, EqF⋆ has both lower angle error and a better Lyapunov value throughout; and a heatmap of output-linearization error across all of $S^2$ shows EqF⋆ "clearly superior" everywhere — exactly consistent with it being a strictly higher-order approximation, not just a better fit near one particular point.

## Significance for VIO, and closing thoughts

A typical VIO state might be $x=(R,p,v,b\_g,b\_a,\ldots)$, and an error-state Kalman filter commonly defines $R=\hat R\exp(\delta\theta)$, $p=\hat p+\delta p$, $v=\hat v+\delta v$ — already geometrically better than a naive Euclidean EKF, since at least the attitude error lives on the right manifold. The EqF pushes the same idea further: ask whether the *entire* system (pose, velocity, landmarks, biases) admits a symmetry under which all of their error dynamics simultaneously become closer to autonomous — not just individually well-behaved, but jointly decoupled from the moving estimate the way the fixed-origin $A^\circ\_t$ is above. That question underlies invariant EKFs, equivariant VIO, and symmetry-based SLAM more broadly: when a filter degrades under large rotations or large initial errors, the fix isn't always a faster update rate — sometimes it's redesigning the *error coordinates* so the linearization matches the system's actual geometry. (I followed up on exactly this question for a real IMU+GNSS system in a [later post](/technical/2026/09/06/equivariant-symmetries-ins.html): it turns out even among equivariant designs, the specific choice of symmetry group still matters a lot in practice.)

The mental model, compressed: an ordinary EKF repeatedly linearizes around the moving estimate $\hat x(t)$; the EqF uses symmetry to transport the error back to a fixed origin first, then runs an ordinary Kalman filter on the resulting (now well-behaved) global error system. It's primarily a linearization-quality improvement, not a different Riccati equation — which is exactly why it matters most in the SO(3)/SE(3)/S²-flavored problems (SLAM, VIO) where naive linearization is the thing that actually breaks. EqF⋆ pushes the same idea one step further still: even after fixing the linearization *point* (the whole contribution of the equivariant-error trick), you can improve the linearization *order* essentially for free, just by reusing the actual measurement instead of only the predicted one when building $C\_t$.

The remaining caveats are the usual ones for any Kalman-family filter on a manifold: the linearization is a small-error approximation (large initial errors aren't covered by this analysis), and the choice of lift $\Lambda$ for a given group/action pair isn't unique — different valid lifts give different (though presumably comparably effective) filters, and picking a good one is still something of an art. EqF⋆ also isn't free of assumptions either — it needs the output map to be equivariant in the first place (true here, and true whenever $C\_t$ was going to be usable at all, but not automatic for an arbitrary measurement model).

<script>
  window.MathJax = {
    tex: {
      inlineMath: [['$', '$'], ['\\(', '\\)']],
      displayMath: [['$$', '$$'], ['\\[', '\\]']]
    }
  };
</script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
