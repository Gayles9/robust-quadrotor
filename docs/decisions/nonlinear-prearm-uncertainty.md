# Nonlinear uncertainty in pre-arm alignment

A rotation estimate can have coupled heading and inclination errors even when
its input uncertainties are independent. This matters when combining attitude
with accelerometer-bias uncertainty. The nonlinear model carries those effects
through the rotation geometry instead of relying only on a first derivative.

The first-order feasibility calculation gave maximum whitened variance
1.1121636092293619 against a 1.10 limit, while its linearized control gave
1.0621711504966704. The discrepancy motivated this nonlinear moment calculation.
All 5,000 outcomes, including rejected trials, were included.

The operating conditions remain those in
[stationary alignment](stationary-prearm-alignment-design.md): external support,
motors off, 400 Hz IMU, 201 samples over 0.5 s, the original noise, bias walks
and heading/bias priors, and a disjoint fresh release sample. Support is never
inferred from quiet measurements.

## Joint moment calculation

At measured accelerometer mean `s`, the four-dimensional Gaussian latent input
`[eta_a, heading_error]` passes through the exact inverse gravity-direction map
and SO(3) error coordinates. The covariance of `eta_a` is `V_a` from the
stationary-alignment model. Conditional terminal accelerometer bias has mean
`C_a V_a^-1 eta_a` and independent residual covariance
`P_aT-C_a V_a^-1 C_a^T`. This retains attitude/bias cross-covariance. Terminal
gyro-bias covariance uses the independent exact mean-to-terminal expression.

The model uses positive tensor Gauss-Hermite quadrature of order five:
625 weighted nodes scaled for a standard normal distribution. Quadrature
approximates expectations by evaluating the nonlinear map at fixed weighted
inputs. Right-local log/exp iterations find the SO(3) mean, starting from the
inclination estimate, with tolerance 1e-13 rad and at most eight corrections.
An unresolved mean is rejected. Central moments and cross-moments are evaluated
about the final mean; the independent conditional bias residual is added only
to the bias block.

The leading mixed term is
`Log(Exp(u*psi) Exp(t))=t+u*psi+0.5*(u*psi cross t)+...`, with
`t approximately -J*eta_a`. The cross product couples heading and inclination.
Independent finite differences and raw trial errors check this term and the
covariance's evaluation point.

The nonlinear mean is a representation correction, not a new heading
observation. The heading prior is unchanged. This is a local Gaussian
pushforward at the measured mean, not an exact Bayesian posterior conditioned
on the nonlinear gravity-norm constraint. It uses no fitted inflation, empirical
bias correction, changed Q/R or deleted correlations.

## Independent accuracy and calibration criteria

- Positive quadrature weights integrate standard-normal moments through degree
  nine in each dimension. Independent checks cover Gaussian conditioning,
  SO(3) log/exp and mixed derivatives, positive-semidefinite full covariance,
  and modeled centered rotation mean within 1e-13 rad.
- Order five is compared with fixed order-seven integration at
  (roll,pitch)=(0,0),(1,-1),(-2,2),(5,-3) degrees and gravity norms
  g-0.06, g and g+0.06 m/s². Mean differences must be at most 1e-8 rad; the
  largest absolute eigenvalue of covariance discrepancy whitened by the
  order-seven covariance must be at most 0.001. Order seven is an accuracy
  reference, not another tuned candidate.
- Fresh PCG64 streams use `SeedSequence([0x50524541,2,partition,trial])`, with
  5,000 nominal and 5,000 Gaussian trials in partitions 0 and 1. Distributions
  and noise/walk generation match the alignment design, and paired first-order
  results use the same samples.
- In the fresh Gaussian population, maximum centered empirical variance of the
  nine whitened nonlinear errors must be at most 1.10; the maximum absolute
  whitened mean must be at most 0.05. Nominal rejection remains at most 1%,
  with the same 0.75-degree axis budget and motion/invalid-data fixtures.
- Exact reconstruction of the original 5,000 Gaussian outcomes precedes the
  paired nonlinear comparison. Its maximum whitened variance must also be at
  most 1.10. This already-seen population is a regression check, not independent
  acceptance evidence. Neither comparison selects only passing trials.

The false-stationarity counterexample and fresh-sample ownership checks remain
part of the contract. Passing these criteria supports the
[standalone component](standalone-prearm-alignment.md); it does not by itself
qualify flight performance or hardware use.
