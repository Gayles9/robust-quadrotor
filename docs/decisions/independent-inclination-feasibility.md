# What an independent inclination measurement would require

Independent inclination information could reduce an ambiguity between tilt and
accelerometer bias. This feasibility study defines the necessary measurement
contract and checks its mathematics. It does not assume that a calibrated
physical source exists.

The implemented sensor boundary contains an IMU, local position and barometric
altitude. It has no external orientation model or calibrated orientation sensor.
A simulator compatibility test does not establish such a measurement, and
simulation truth is only a scoring reference.

## Conditional measurement model

The model defines body-down direction from external rigid-body orientation and
explicit world, target and body calibrations. A two-coordinate tangent residual
and right-local Jacobian connect that direction to the 21-state estimator
endpoint. Tangent coordinates describe the two local directions perpendicular
to the measured axis.

An axis direction leaves twist about that body axis unobserved. Away from level,
that twist is not the same as world-vertical Euler yaw. Antipodal and invalid
hemisphere residuals are rejected because the local residual cannot represent
them reliably.

Angular uncertainty is propagated into the tangent plane. The model retains
position/orientation correlation and state/measurement correlation, including
correlation caused by reused IMU data. Joint conditioning and conditional
decorrelation are checked algebraically; this study adds no estimator update.

A usable source would need defined calibration and bias, acquisition/delivery
ownership, latency, age, rate, outages and gating. A deterministic age bound and
an optional finite-sequence Gaussian union bound express distinct assumptions:
a standard deviation is not a hard error bound. Measurement noise is not the
same as posterior estimation error, and command clipping does not establish a
bound on the true angular rate.

Local information calculations consider both calibrated inclination and two
unknown constant inclination-bias coordinates. Bias ambiguity and correlations
remain explicit. Existing position/altitude gates are unchanged; any derived
two-dimensional threshold is conditional mathematics, not a configured gate.

## Verification requirements

The input error-budget report has SHA-256
`b93b1fb050bab01b06b790b9bac7d795469cb99942572ea476ecfe47775798cb`;
its execution fingerprint is
`d83889b7189ab898aadb87bfa258259ec30e381b14f0d550d207343835216f95`.
Saved flight inputs are reused without changing the 8 cm budget or optimizing
thresholds.

Independent deterministic fixtures check:

- Jacobians at identity and nonlevel rotations against finite differences with
  absolute tolerance 1e-8; rank two, exact twist invariance, frame registration,
  tangent-basis covariance and hemisphere rejection.
- Agreement within 1e-12 between full joint Gaussian conditioning and correctly
  decorrelated sequential conditioning. A deliberately false independence
  assumption demonstrates its effect; invalid covariance is rejected.
- Age bounds under analytic constant-axis motion, Gaussian radial-tail algebra
  and existing error-budget coefficients, without fitted noise or a favorable
  random seed.
- Local nullspaces with calibrated and unknown-bias inclination. Rank changes
  are not interpreted as nonlinear observability or improved flight performance.

## Feasibility boundary

A defensible implementation requires both a justified measurement source and a
simultaneous allocation of the existing tracking budget. Missing bias, timing
or calibration quantities cannot be treated as zero. Without those inputs,
implementation is unsupported even if the conditional algebra is complete.
This study adds no scientific flight, synthetic sensor campaign, hardware
selection, middleware or production behavior. Existing mass-offset,
estimated-feedback and integration limitations remain separate.
