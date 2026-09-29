# ADR 0040: independent-inclination contract and feasibility

Status: scope and acceptance frozen before derivation code or results, 2026-09-29.

## Preceding audit and source boundary

Audit merged PR 38 at `0129d658e905a644f06da1c51731ffe85c2655a9`, tree
`17fa3b7f92ff80804972bfc7aeddd3329756cf01`. Remote main and the clean checkout
agree. Fresh warning-strict checks pass 57 error-budget, attitude-audit and
sampled-response tests in 1.54 s. The independent saved-artifact verifier passes
all 27 histories, 144 Gram matrices and 13,608 gate records. The preceding
execution fingerprint is
`d83889b7189ab898aadb87bfa258259ec30e381b14f0d550d207343835216f95`;
the error-budget report is
`b93b1fb050bab01b06b790b9bac7d795469cb99942572ea476ecfe47775798cb`.

The existing Python sensor boundary contains IMU, local position and barometric
altitude, with no external orientation model or calibrated source. The project
is simulation-first; the historical PX4/Gazebo compatibility spike does not
establish an independent orientation measurement. Vision and physical hardware
are deferred scope. Simulation truth is available for scoring, not a deployable
measurement. Do not invent a new source/noise model to pass this feasibility gate.

## Bounded work

Complete the conditional mathematical contract, using deterministic algebraic
fixtures only. No scientific flight, synthetic sensor campaign, source hardware
selection, middleware, new measurement update component or production behavior
is in scope. Reuse the preceding authenticated error budget; do not regenerate
its flight inputs or optimize thresholds.

1. Define body-down direction from external rigid-body orientation and explicit
   world/target/body calibrations. Derive its two tangent-coordinate residual
   and right-local ESKF Jacobian, including the 21-state endpoint interface.
   Distinguish unobserved body-axis twist from world-vertical Euler yaw away
   from level. Reject antipodal/hemisphere-invalid local residuals explicitly.
2. Propagate angular uncertainty into the tangent plane. Retain possible
   position/orientation correlation and state/measurement correlation, including
   reused IMU data. Verify joint conditioning and conditional decorrelation as
   algebra, without inserting an update into the ESKF.
3. Specify calibration and measurement bias, acquisition/delivery ownership,
   latency, age, rate, outages and gating. Derive a deterministic age bound and
   an optional finite-sequence Gaussian union bound; standard deviations are
   not hard error bounds. Do not substitute measurement noise for posterior
   estimator error or infer a true-rate bound from command clipping.
4. Check the local information benefit with calibrated inclination and with
   two unknown constant inclination-bias coordinates. Keep bias ambiguity and
   correlations explicit. Preserve original position/altitude gates; any
   two-dimensional threshold is a conditional derivation, not a configured gate.
5. Evaluate source/model availability and whether a defensible simultaneous
   allocation of the existing 8 cm budget can be established. Mark missing
   quantities unknown, never zero. If no justified source and bounds are
   available, conclude no-go for a component and close this extension study.

## Acceptance

- Bind preceding execution files and report. Freeze and retain this protocol.
- Check Jacobians at identity and nonlevel rotations against independent finite
  differences (absolute tolerance 1e-8); verify rank two, exact twist invariance,
  frame registration, tangent-basis covariance and hemisphere rejection.
- Verify full joint Gaussian conditioning and properly decorrelated sequential
  conditioning agree within 1e-12 on fixed correlated fixtures. Demonstrate
  that falsely independent fusion changes the result; reject invalid covariance.
- Verify age bounds with analytic constant-axis motion, Gaussian radial-tail
  algebra and the original error-budget coefficients. Derive all bounds without
  a favorable seed, fitted noise or an assumed physical sensor.
- Verify calibrated and unknown-bias local nullspaces; do not reinterpret local
  rank as nonlinear observability or flight improvement.
- Preserve every fixture and result, pass tests/static checks and hosted CI,
  and publish the contract and an explicit go/no-go. A complete feasibility
  study may reject implementation because its assumptions are unsupported.
- Keep startup tuning closed, the failed no-regression gate preserved, and
  estimated-feedback, mass-offset, geometric and integration limitations open.
  Choose a finite next deliverable within the existing project scope rather
  than recursively expanding the sensor study.
