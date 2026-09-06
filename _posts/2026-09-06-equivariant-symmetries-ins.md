---
layout: post
title: "Equivariant Symmetries for Inertial Navigation Systems"
date: 2026-09-06 00:00:00 -0000
categories: technical
---

Reading notes on Fornasier, Ge, van Goor, Mahony, and Weiss, ["Equivariant Symmetries for Inertial Navigation Systems"](https://arxiv.org/abs/2309.03765) (arXiv:2309.03765, submitted to Automatica). Follow-up to [my previous post](/technical/2026/09/06/equivariant-filter.html) on the general Equivariant Filter (EqF) theory — that post built the machinery; this paper applies it to a real, practically important system: an IMU+GNSS inertial navigation system (INS).

The paper's hook is a nice one: it argues that essentially every well-known INS EKF variant — the classical multiplicative EKF (MEKF), the Invariant EKF (IEKF), and others — is just the *same* EqF recipe applied to the *same* state space, but with a different choice of symmetry group. The state space itself is not a Lie group; the group only enters through the action you choose to put on it, and different choices genuinely produce different (but all individually valid) filters with different error-dynamics structure. The paper catalogs six such choices — including two new ones — and compares how they perform in practice.

## The state and its dynamics

The INS state is standard strapdown mechanization: attitude, velocity, position, and the two IMU biases,

$$
\xi = (R, v, p, b_\omega, b_a) \in \mathcal{M} = SO(3)\times\mathbb{R}^3\times\mathbb{R}^3\times\mathbb{R}^3\times\mathbb{R}^3
$$

driven by gyroscope and accelerometer measurements $\omega, a$:

$$
\dot R = R(\omega-b_\omega)^\wedge, \qquad \dot v = R(a-b_a)+g, \qquad \dot p = v, \qquad \dot b_\omega = \tau_\omega, \qquad \dot b_a = \tau_a
$$

($(\cdot)^\wedge$ is the usual skew-symmetric map $\mathbb{R}^3\to\mathfrak{so}(3)$; $\tau\_\omega,\tau\_a$ are the bias-noise inputs, zero under a pure random-walk bias model.) None of this is paper-specific — it's the same mechanization every INS filter starts from. What differs between filters is entirely the choice of symmetry group used to linearize the error around it.

## The menu of six symmetry groups

Each choice below is a group $\mathbf{G}$, a right action $\phi(X,\xi)$ of $\mathbf{G}$ on $\mathcal{M}$, and — once you run it through the general EqF machinery from the previous post — a specific classical filter that falls out as a special case.

1. **$G\_O := SO(3)\times\mathbb{R}^{12}$.** The simplest possible choice: rotate the attitude, translate everything else, independently. $\phi(X,\xi) := (RA,\, v+a,\, p+b,\, b\_\omega+\alpha,\, b\_a+\beta)$ for $X=(A,a,b,\alpha,\beta)$. This is exactly the classical **multiplicative EKF (MEKF)** — worked out in full below.
2. **$G\_{ES} := SE\_2(3)\times\mathbb{R}^6$.** Bundles attitude, velocity, and position together into the "extended pose" group $SE\_2(3)$ (the group whose elements act like $SE(3)$ but carry both a translation and a velocity column), leaving the biases as a separate translation-only block. Recovers the (imperfect) **Invariant EKF (IEKF)**.
3. **$G\_{TF} := SO(3)\ltimes(\mathbb{R}^6\oplus\mathbb{R}^6)$.** A "two-frames" group: biases are rotated along with attitude but not coupled into the $SE\_2(3)$ velocity/position structure the way $G\_{ES}$ couples them. Recovers the **two-frames-group invariant EKF (TFG-IEKF)**.
4. **$G\_{TG} := SE\_2(3)\ltimes\mathfrak{se}\_2(3)$** (prior work). Here the biases are folded into the *Lie algebra* of $SE\_2(3)$ itself via a semi-direct product — which requires introducing a "virtual" extra bias state $b\_\nu$ (so the bias block grows from 6 to 9 dimensions) purely to make the group structure work out. The payoff is real: this is the *only* one of the six choices with **exactly linear** error dynamics in the navigation states $(R,v,p)$ — every bit of nonlinearity gets pushed into the (now 9-dimensional) bias error.
5. **$G\_{DP} := \mathbf{HG}(3)\ltimes\mathfrak{hg}(3)\times\mathbb{R}^3$ — novel.** "Direct Position." Keeps $G\_{TG}$'s semi-direct bias-coupling trick (still buys some of that nice bias-error structure), but avoids its extra virtual state by pulling position *out* of the geometric group entirely — position sits as a plain Euclidean factor, and only attitude+velocity are bundled into a smaller "Galilean" group $\mathbf{HG}(3)$.
6. **$G\_{SD} := SE\_2(3)\ltimes\mathfrak{se}(3)$ — novel.** "Semi-Direct bias." The mirror-image trade-off to $G\_{DP}$: keeps position inside the full $SE\_2(3)$ structure (unlike $G\_{DP}$), but uses the smaller 6-dimensional algebra $\mathfrak{se}(3)$ for the bias coupling instead of $G\_{TG}$'s 9-dimensional $\mathfrak{se}\_2(3)$ — again sidestepping the extra virtual bias state.

The two novel ones ($G\_{DP}$, $G\_{SD}$) are both attempts to get most of $G\_{TG}$'s good bias-error behavior without paying for the extra virtual state — they just disagree about which piece (position, or the bias algebra) to shrink back down to get there.

## Worked derivation: $G\_O$ really is the MEKF

$G\_O$ is the simplest case in the table, and it's simple enough to carry all the way through by hand — a good sanity check that the general theory actually produces the filter everyone already knows.

**The differential of the action.** $\mathfrak{g}=\mathfrak{so}(3)\times\mathbb{R}^{12}$, with a tangent vector written $(\omega,a,b,\alpha,\beta)$. Differentiating $\phi((\exp(t\omega^\wedge),ta,tb,t\alpha,t\beta),\xi)=(R\exp(t\omega^\wedge),\,v+ta,\,p+tb,\,b\_\omega+t\alpha,\,b\_a+t\beta)$ at $t=0$:

$$
D\phi_\xi(\mathrm{id})[(\omega,a,b,\alpha,\beta)] = (R\omega^\wedge,\, a,\, b,\, \alpha,\, \beta)
$$

Since $\dim\mathbf{G}\_O = 3+12=15$ exactly matches $\dim\mathcal{M}=3+3+3+3+3=15$, and $R$ is invertible, this map is a genuine bijection — no ambiguity anywhere, unlike the landmark example in the previous post where $\dim\mathbf{G}>\dim\mathcal{M}$ left real freedom in choosing the lift.

**The lift.** Solving $D\phi\_\xi(\mathrm{id})[\Lambda(\xi,u)] = f\_u(\xi)$ term by term against the IMU dynamics above — matching $R\omega\_\Lambda^\wedge$ against $\dot R = R(\omega-b\_\omega)^\wedge$, and the rest directly against $\dot v,\dot p,\dot b\_\omega,\dot b\_a$ — gives, uniquely (no design choice involved, since the map above is a bijection):

$$
\Lambda(\xi,u) = \big(\omega-b_\omega,\; R(a-b_a)+g,\; v,\; \tau_\omega,\; \tau_a\big)
$$

**The global error.** Take the reference point $\xi^\circ$ to be the identity, so that a group element $X=(A,a,b,\alpha,\beta)$ and a state $\xi=(A,a,b,\alpha,\beta)$ literally coincide as tuples (this identification is exactly what "$\dim\mathbf{G}=\dim\mathcal{M}$ and the action is simply transitive" buys you). Then the observer's lifted state $\hat X$ is just $(\hat R,\hat v,\hat p,\hat b\_\omega,\hat b\_a)$ itself, its inverse is $\hat X^{-1}=(\hat R^\top,-\hat v,-\hat p,-\hat b\_\omega,-\hat b\_a)$, and the global error is

$$
e = \phi(\hat X^{-1},\xi) = \big(R\hat R^\top,\; v-\hat v,\; p-\hat p,\; b_\omega-\hat b_\omega,\; b_a-\hat b_a\big)
$$

That's exactly the classical multiplicative-EKF error definition: a *multiplicative* attitude error $R\hat R^\top \approx \exp(\delta\theta^\wedge)$ and *additive* Euclidean errors everywhere else (up to which side of the rotation the inverse lands on — $R\hat R^\top$ vs. $\hat R^\top R$ is a left/right-invariant-error convention choice, well documented in the invariant-filtering literature, and fixed here by the right-action convention used throughout). Running this $\Lambda$ and this error definition through the general $A^\circ\_t$/$C\_t$/Riccati machinery from the previous post reproduces the MEKF's actual filter equations — the point isn't that this is a new result, it's that it falls straight out of the general recipe with zero extra assumptions.

## The other five, briefly

$G\_{ES}$ folds attitude, velocity, and position into one $SE\_2(3)$-valued block instead of treating them as three separate pieces the way $G\_O$ does. Concretely, $SE\_2(3)$ elements look like $5\times 5$ matrices

$$
\begin{pmatrix}A&a&b\\0&1&0\\0&0&1\end{pmatrix}
$$

and the group multiplication couples $a$ and $b$ through $A$ — which is exactly what makes the resulting error dynamics couple attitude, velocity, and position errors together rather than treating them independently the way $G\_O$'s error does. That coupling is the IEKF's whole reason for existing: it captures correlations between attitude and velocity/position error that $G\_O$'s simpler product structure misses.

$G\_{TF}$ sits between the two: it still rotates the bias states along with attitude (unlike $G\_O$, where biases are pure translations untouched by attitude), but it doesn't fold them into the full $SE\_2(3)$ coupling the way $G\_{ES}$ does for velocity/position. It's a "some coupling, not all of it" middle ground, and it reproduces the TFG-IEKF.

$G\_{TG}$, $G\_{DP}$, and $G\_{SD}$ all chase the same idea from different angles: put the *bias* error inside a semi-direct-product structure (rather than a flat translation), so that the bias error dynamics pick up the same kind of beneficial coupling that $G\_{ES}$ gave the navigation states. The cost of doing this exactly (as $G\_{TG}$ does, using the full 9-dimensional $\mathfrak{se}\_2(3)$ algebra) is an extra virtual bias state that doesn't correspond to anything physical. $G\_{DP}$ and $G\_{SD}$ are two different ways of trimming that cost back down — by shrinking position out of the geometric group, or by shrinking the bias algebra itself — while keeping most of the benefit.

## The GNSS output problem

Feeding GNSS position fixes into any of the larger symmetry groups runs into a subtlety worth flagging even without re-deriving it in full: the naive output map $h(\xi)=p$ is **not equivariant** once position is bundled into an $SE\_2(3)$-type action, because those actions move $p$ through a combination of rotation and translation, not the pure translation a plain $h(\xi)=p$ can absorb on its own. The paper's fix (its Lemma 15) is to measure position as a body-frame residual instead,

$$
h(\xi) := R^\top(\pi - p)
$$

where $\pi$ is the raw GNSS fix, paired with a matching action on the output space, $\rho\_X(y) := A^\top(y-b)$. This restores equivariance and lets the general $C\_t$-linearization from the previous post apply directly — and as a reported side benefit, the paper shows this reformulation actually improves the *order* of the output linearization error (cubic rather than quadratic). I'm reporting this rather than re-deriving it here: the exact semi-direct/adjoint bookkeeping behind it is intricate enough, and the source material dense enough, that I'd rather flag the boundary of what I've independently checked than fake precision on it.

## How the choices actually compare

The paper backs this up with simulation and real UAV flight data. A few of the headline numbers, comparing transient-phase RMSE against the plain MEKF baseline over the first 30 seconds:

- **TG-EqF**: roughly 85% lower orientation RMSE, 82% lower velocity RMSE, 87% lower accelerometer-bias RMSE.
- **DP-EqF** and **SD-EqF** track closely behind TG-EqF on all three.
- Orientation convergence time (to within 10% of peak RMSE): TG-EqF reaches it in about 21.3s vs. 32.1s for MEKF — roughly 66% faster.
- Filter consistency (ANEES, where 1 is ideal): TG-EqF sits around 1.20–1.22 throughout; MEKF spikes to around 3.11 during the transient — badly overconfident in exactly the period where it matters most.

The paper's own explanation ties directly back to the derivation above: TG-EqF is the one symmetry choice with *exactly* linear navigation-state error dynamics, and it attributes essentially all of the performance and consistency advantage to that fact. Their practical recommendation, for INS problems where the trajectory stays fully observable, is TG-EqF as the leading choice — with DP-EqF and SD-EqF as lighter-weight alternatives that recover most of the same benefit without the extra virtual bias state.

## Closing thought

This paper is really the concrete answer to the question the previous post's closing paragraph raised in the abstract: *which* symmetry should you actually pick for a real IMU-driven system? The answer here isn't "any equivariant choice works equally well" — it's "they're all valid, but the choice that makes the navigation-state error dynamics *exactly* linear (not just approximately, to first order) measurably wins," both in RMSE and, maybe more importantly for a real deployed filter, in how honestly it reports its own uncertainty.

<script>
  window.MathJax = {
    tex: {
      inlineMath: [['$', '$'], ['\\(', '\\)']],
      displayMath: [['$$', '$$'], ['\\[', '\\]']]
    }
  };
</script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
