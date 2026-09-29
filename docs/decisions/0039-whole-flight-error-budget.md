# ADR 0039: whole-flight estimation/control error-budget review

Status: scope and acceptance frozen before implementation or results, 2026-09-29.

## Audit and fixed evidence

Audit merged PR 37 at `b4fc5821fe41474bc69edf6d92d9a97286da8824`, tree
`7ab1b0b87c106063e30cb83299d2d98ace6904e2`. Remote main and the clean local
checkout agree. Fresh warning-strict checks pass all 51 combined-prior,
attitude-audit and response-model tests in 3.47 s. The archived independent
NumPy verifier authenticates the diagnosis and recovers all 27 saved algebra
records. No implementation defect is demonstrated. The startup study remains
closed and its failed no-regression comparison is retained.

Use the complete ADR 0038 report with SHA-256
`09c53c8e86f94f16b685fa6f297e54ecd4bcc3a7e78cb7c806217ed020910276`
and all its 27 authenticated response/update records. Its execution fingerprint
is `01d663368ed67174c9b300fdd2ef7450781da5f6ec3947e7f9a9b2d8473f2b0e`.
Preserve all nine jobs, all three arms, full durations and failed outcomes.

## Scope and derivation

1. Audit the current sensor assumptions, clocks, gains, limits and all remaining
   flight requirements. Keep the original 8 cm hover peak, 15 cm full-flight
   RMSE/final-position, 15 cm/s final-speed and strict paired no-regression rules.
2. Compute the full uncentred response Gram matrix for every saved flight and
   mission window. Its diagonal and signed cross terms must recover tracking
   MSE. Report triangle bounds separately from actual scores; correlated
   channels are not independent variance sources or removable interventions.
3. Lift the existing near-level horizontal cascade over one original outer
   period, preserving inner/outer update order and intersample outputs. Derive
   frequency sensitivities to position, velocity, inclination and inclination
   rate errors, and finite-horizon worst-case input bounds for the 5..11 s
   hover window. Use existing gains/motor lag only. Frequency grid is DC plus
   241 logarithmic points from 0.01 to 25 Hz. Do not optimize a gain or cutoff.
4. Check DC sensitivities analytically and lifted evolution against the existing
   time-domain model. Bounds must use absolute impulse coefficients and retain
   the two horizontal axes. Report single-input ceilings against 8 cm only as
   screening values: they are not a simultaneous allocation, include no vertical
   or physical residual allowance, and do not qualify a flight.
5. Keep accelerometer and gyro bias effects inside their dependent navigation,
   inclination and rate errors. Derive the level-hover nullspace with the
   current position/altitude observations; compare adding direct velocity and
   adding independent two-axis inclination. Make no nonlinear observability
   claim. Retain every original innovation rejection and paired branch change;
   a linear sensitivity cannot predict a changed gate sequence.

No new scientific flights, startup variant, empirical covariance, new sensor
assumption, controller change, integration, seed/gain search or dependency is
authorized by this protocol. A prospective measurement contract may be justified
by the algebra; its availability, noise/bias, timing and correlation assumptions
must be explicit and unverified until separately established.

## Acceptance and stopping rule

- Authenticate the fixed report and every referenced diagnosis payload before
  analysis; bind the preceding execution source without editing its runners.
- Recover every full-flight score and Gram identity within 1e-12 m², response
  sum within 1e-10 m, and retain all cross terms and original gate decisions.
- Match lifted/time-domain trajectories within 1e-10 for deterministic fixtures;
  DC gains within 1e-10; demonstrate impulse-bound attainment with adversarial
  signs, and verify frequency phasors using direct sampled time evolution.
- Check analytic null vectors and the rank changes independently, with unit
  tests for signs, clocks, correlated cancellation, invalid inputs and evidence
  tampering. Pass relevant static checks and full warning-strict CI.
- Document the mathematical budget, numerical results, physical limitations and
  a go/no-go for one specific next measurement/feedback hypothesis. Do not claim
  improved flight performance from saved-input linear calculations. If existing
  measurements cannot justify a new design, say so and identify the missing
  physical information rather than inventing an implementable sensor.
- Publish the completed review and preserve its generated evidence. Fresh
  estimated-feedback qualification, mass-offset, geometric and external
  integration requirements remain open regardless of this review's outcome.
