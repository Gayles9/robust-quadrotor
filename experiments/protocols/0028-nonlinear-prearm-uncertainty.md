# ADR 0028: nonlinear joint uncertainty for supported pre-arm alignment

Status: design and acceptance frozen before candidate execution, 2026-09-28.
Scope: one offline uncertainty model and a go/no-go for a subsequent standalone
component. No production initializer, flight integration or hardware arming.

## Audit and operating boundary

Audit merged PR 26, main `2f92c9ece0d88c4fdff95bca2de1f10158d6d2ba`.
Fresh warning-strict pre-arm/documentation checks pass 33 tests in 0.28 s;
all five evidence payloads authenticate. Independent reconstruction reproduces
the retained Gaussian maximum whitened variance 1.1121636092293619, versus
1.10 acceptance, and linearized control 1.0621711504966704. The full nine-state
check includes rejected trials. Main matches GitHub and the worktree is clean.

Retain [ADR 0027](0027-stationary-prearm-alignment-design.md)'s external supported,
motors-off stationary guarantee through release, 400 Hz IMU, 201 samples/0.5 s,
sensor noise and bias walks, original bias/heading priors, support/motion/data
rejection, disjoint fresh release sample and complete-flight scoring boundary.
All original production source, controller defaults and reserved flight seeds
remain unchanged. Support is not inferred from quiet measurements.

## One mathematically specified candidate

At the measured mean s, propagate the four-dimensional Gaussian latent input
[eta_a, heading_error] through the exact inverse gravity-direction map and SO(3)
error coordinates. Eta_a has covariance V_a from ADR 0027. Conditional terminal
accelerometer bias has mean C_a V_a^-1 eta_a and independent residual covariance
P_aT-C_a V_a^-1 C_a^T. This retains all attitude/bias cross-covariance; terminal
gyro-bias covariance remains its independent exact mean-to-terminal expression.

Use fixed positive tensor Gauss-Hermite quadrature of order five (625 nodes),
scaled for a standard normal distribution. Choose the SO(3) mean by right-local
log/exp iterations from the original inclination estimate, tolerance 1e-13 rad,
at most eight corrections; reject if unresolved. Recompute central moments
and cross-moments about that mean, adding the independent conditional bias
residual only in the bias block. No fitted factors, Q/R changes, truncated
correlations, empirical bias correction or alternate candidate is permitted.

The nonlinear rotation mean is a representation correction, not a heading
observation. The latent heading prior remains unchanged. This local Gaussian
pushforward is an uncertainty approximation at the measured mean; it is not
claimed to be an exact Bayesian posterior conditioned on the nonlinear gravity
norm constraint. That distinction is tested empirically, not hidden.

The leading mixed term follows from
Log(Exp(u*psi) Exp(t))=t+u*psi+0.5*(u*psi cross t)+..., with
t approximately -J*eta_a. It couples heading and inclination even when their
latent inputs are independent. Audit this term and the covariance evaluation
point against finite differences and the retained raw trial errors before
interpreting calibration changes.

## Frozen independent tests and acceptance

- Positive quadrature weights; exact standard-normal moments through degree
  nine in each dimension; independent latent Gaussian conditioning algebra;
  SO(3) frame/log/exp and mixed-derivative checks; full PSD covariance with all
  cross blocks; zero modeled centered rotation mean to 1e-13 rad.
- Numerical integration error check only: order five versus order seven at
  (roll,pitch)=(0,0),(1,-1),(-2,2),(5,-3) degrees, each measured gravity norm
  g-0.06, g and g+0.06 m/s². Mean difference <=1e-8 rad and maximum eigenvalue
  magnitude of the covariance difference whitened by order-seven covariance
  <=0.001. Order seven is a fixed accuracy reference, not another candidate.
- Fresh PCG64 streams SeedSequence([0x50524541,2,partition,trial]), 5,000 nominal
  and 5,000 Gaussian trials in partitions 0 and 1, with exactly the distributions
  and sample noise/walk construction from ADR 0027. Do not use reserved flight
  seeds. Retain all outcomes and paired first-order results from the same data.
- Maximum centered empirical variance of the nine nonlinear errors whitened by
  each candidate covariance <=1.10 in the fresh Gaussian population. Also retain
  the full whitened mean and require its maximum absolute component <=0.05.
  Retain the original 0.75-degree local axis-uncertainty budget, <=1% nominal
  rejection and unchanged required motion/invalid-data fixtures.
- Reconstruct the original 5,000 Gaussian errors and covariances exactly before
  applying the one candidate to that already-seen set. Its candidate maximum
  whitened variance must also be <=1.10; this is a regression check, not the
  independent acceptance population. Never select a passing subset.
- Keep the support counterexample and fresh-sample ownership checks. A passing
  uncertainty study authorizes only the next standalone component, not flight.

One execution of this frozen study is allowed, with exact reproduction after
mechanical software corrections if necessary. Stop on a failed acceptance;
do not change quadrature order, seeds, duration, noise or thresholds after
observing candidate results. Preserve artifacts and report the precise next step.
