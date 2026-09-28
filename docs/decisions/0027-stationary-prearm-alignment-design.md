# ADR 0027: stationary pre-arm alignment contract and feasibility

Status: scope and acceptance frozen before feasibility execution, 2026-09-28.
This step defines and checks a design; it does not add a production initializer
or integrate a new flight mode.

## Preceding audit

Audit main `cdc27707ce40d50eb3be8066e53f15ea46b5b258` and the merged PR 25
startup explanation. The repository is clean and matches GitHub. Fresh
warning-strict startup/documentation tests pass 32 tests in 0.29 s. Both
preserved evidence packages authenticate all 27 payloads. Source, guide, prior,
endpoint continuation and the current plan agree: no demonstrated estimator
defect, no qualified performance gain, and no free-flight gravity observation.

The next scope is explicitly authorized as a conditional supported-stationary
pre-arm stage. This permits its design, not a claim that physical support exists
in the current free-flight plant or on hardware. Retain the Python 3.12.14,
NumPy 2.5.2 and locked dependencies; no production source or pin changes.

## Operating and information boundaries

Require an external, contemporaneous assertion of mechanical support, motors
off, constant attitude, zero world acceleration and zero angular velocity over
the whole acquisition interval through release. A false assertion invalidates
the result. Quiet IMU data alone cannot establish this condition. No true
attitude, true bias, commanded attitude or flight score enters alignment.

Use only existing 400 Hz paired IMU samples, g=9.81 m/s², sample deviations
0.04 m/s² and 0.002 rad/s, and bias-walk deviations 0.0002 m/s²/sqrt(s) and
0.00002 rad/s/sqrt(s). Preserve the original initial accelerometer-bias sigma
0.03 m/s² and heading sigma 3 degrees; do not fit either to a realized seed.
Gravity constrains inclination; heading remains supplied by the prior. A single
orientation cannot separate inclination from transverse accelerometer bias.

Analyze one closed-form design: gravity direction from the accelerometer mean
minus its declared prior bias, retained prior heading, and the gyro mean as
the supported-stationary gyro-bias estimate. Do not estimate the accelerometer
bias from that same direction. Carry its uncertainty, bias walks and all induced
attitude/bias cross-covariances. Define local approximations and their domain.

## Predeclared budgets and rejection design

Choose duration from analytic noise budgets, not a flight score: accelerometer
white-noise contribution <=0.02 degree per tangent axis (one sigma), and terminal
gyro-bias uncertainty <=0.03 degree/s per axis (three sigma), including walk.
Round the minimum required duration upward to the next 0.5 s acquisition block;
use one block only if it meets both requirements. No duration sweep is allowed.

Require the reported local 99% world-thrust-axis ellipse radius <=0.75 degree;
retain heading uncertainty and its coupling when the vehicle is tilted. This
is an initialization design budget, not a proof of the 8 cm flight requirement.
Reject invalid clocks/data/support, an ill-conditioned gravity direction,
excessive inferred inclination or failed uncertainty budgets.

Use fixed model-based upper-tail compatibility checks for mean gyro, gravity
magnitude and demeaned IMU variation, including temporal correlation from bias
walks. Allocate alpha=0.001 to each of these three tests. State exactly the
Gaussian assumptions under which the union false-rejection bound applies.
Document undetectable steady acceleration and rotation/bias ambiguities.
No gate can override the external supported-stationary requirement.

End sample reuse explicitly: consume alignment samples once, propagate only
bias-walk uncertainty while support remains asserted, and use a fresh disjoint
paired IMU sample for the first flight endpoint. Do not zero cross-covariances
between attitude and bias or erase uncertainty at release. Keep position and
velocity initialization separate, with an explicit independence requirement.

## Independent checks and stopping rule

1. Derive inclination signs/Jacobians, heading ambiguity, random-walk mean and
   terminal covariances, and correlated handoff. Check against independent
   finite differences, latent-increment sums and Gaussian calculations.
2. Freeze one deterministic 5,000-trial supported-static feasibility population
   (near-level inclinations, original bias/noise scales) before execution; retain
   all outcomes. Use a separate Gaussian covariance check with the same number
   of trials and distinct deterministic randomness, not reserved flight seeds.
3. Check missing support, malformed timing, one impulse, vibration, a changing
   gravity direction and a large mean rate. Retain a known IMU-indistinguishable
   false-stationarity case as a required limitation, never an accepted flight.
4. State a go/no-go for a bounded standalone implementation. Require analytic
   identities and PSD covariance, <=1% nominal gate rejection, and no measured
   covariance underprediction beyond 10% in the independent Gaussian check.
   These finite-study criteria are not hardware or flight qualification.
5. Freeze the future production interface, independent test matrix and later
   full-flight regression acceptance in the contract. Stop if infeasible;
   do not tune budgets/noise/duration after results or start another controller
   candidate. Complete software checks and preserve numerical evidence.

Cascade, endpoint ESKF, original free-flight cases, all thresholds and reserved
flight seeds remain unchanged. A later pre-arm scenario must disclose its added
time and different operating assumptions; it cannot relabel the old failure.

## Detailed protocol frozen before numerical execution

Use PCG64 with SeedSequence([0x50524541, 1, partition, trial]); partitions 0 and
1 respectively identify nominal and Gaussian populations. Each has 5,000 trials.
Nominal truth uses the original independent rotation-vector components uniform
in +/-2 degrees, accelerometer bias in +/-0.02 m/s² and gyro bias in
+/-0.003 rad/s. These describe a new supported interval, not a replay of a flight.
Gaussian truth uses fixed roll +1 degree, pitch -1 degree, heading N(0, 3 degrees),
accelerometer bias N(0, 0.03 m/s²), and gyro bias N(0, 0.005 rad/s), independent.
Both draw every per-sample white noise and discrete bias increment explicitly.
Use the stated prior means zero, retained heading zero, and no truth input.

Evaluate actual nonlinear right-local attitude errors and terminal bias errors
together. For every Gaussian trial, whiten the nine-dimensional error by the
reported covariance's Cholesky factor at the estimated orientation. The largest
eigenvalue of their centered empirical covariance must be <=1.10. Retain all
trials, including rejected ones, to avoid selection masking underprediction.
Also retain a separate analytic linearized-error calculation from the same
latent draws to distinguish nonlinear limitations from covariance algebra.
The linear calculation cannot substitute for the nonlinear acceptance test.

Reject inferred inclination above 15 degrees (an additional small-angle domain
limit), and reject the tighter 0.75-degree world-axis uncertainty budget whenever
it fails. Fixed stress fixtures use a 1 m/s² single accelerometer impulse,
0.2 m/s² 40 Hz vibration, a roll ramp from 0 to 3 degrees, and 0.05 rad/s mean
gyro rate. The indistinguishable fixture uses a 1-degree pitch-equivalent
constant acceleration. Statistical thresholds use
c(d, alpha)=d+2*sqrt(d*ln(1/alpha))+2*ln(1/alpha).
Each stress fixture adds its signal to the same level, zero-bias white-noise
record from partition 2, trial 0 (no bias walk in these illustrative fixtures).

Clock tolerance is 1e-12 s for the 400 Hz grid. Handoff is exactly one fresh
sample interval after the alignment window, with support continuing through it.
Do not retry this study with alternative seeds, durations, covariance inflation
or post-result model changes if the frozen acceptance fails.
