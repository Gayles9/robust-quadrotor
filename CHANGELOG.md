# Changelog

All notable changes to this project will be documented in this file following a simple Keep a Changelog-style structure.

## [Unreleased]

### 2026-09-28 nonlinear pre-arm uncertainty

- Derive the mixed heading/inclination error and propagate one local Gaussian
  model through nonlinear rotations with full terminal-bias correlations.
- Pass the original and fresh covariance gates using fixed positive quadrature,
  retaining all 15,000 outcomes and unchanged priors, duration and thresholds.
- Scope the standalone production alignment component next; no flight behavior
  or controller default changes and no flight-performance gain is claimed.

### 2026-09-28 stationary pre-arm alignment design

- Define an externally supported, motors-off 0.5 s acquisition stage, retained
  heading and accelerometer-bias uncertainty, motion rejection and fresh-sample
  handoff. No production initializer or flight behavior changes.
- Preserve all 10,000 feasibility trials: nominal alignment is accurate, but the
  nonlinear joint covariance narrowly exceeds the frozen underprediction limit.
- Stop before production implementation and scope a bounded nonlinear uncertainty
  derivation; retain the cascade default and original flight failures.

### 2026-09-28 attitude-estimation startup audit

- Recover a missing historical evidence payload by exact pinned-case reproduction
  and mandatory original-digest equality; preserve original report provenance.
- Reconstruct both full estimator histories and independently check joint
  conditioning/reset, startup correction timing and local information limits.
- Explain the first-position tilt increase without a demonstrated filter defect;
  retain production behavior and scope an explicit pre-arm alignment contract.

### 2026-09-28 causal early-flight diagnosis

- Reauthenticate and independently rescore the original and vertical campaigns;
  reconstruct saved mass/hover estimation, commands, force and motor response.
- Explain the mass transient with an unfitted scalar model and test one
  attitude-only hover oracle with exact channel isolation and noise pairing.
- Retain all flight failures and controller defaults; identify attitude-estimation
  startup as the next bounded audit, with no production candidate in this step.

### 2026-09-28 bounded vertical compensation

- Diagnose the heavier-mass offset against the PD equilibrium and saved
  estimator, thrust and motor histories before freezing one candidate.
- Add an opt-in, bounded vertical integral with explicit health gating, reset,
  conditional anti-windup and a reconstructable applied/next-state trace.
- Compare all twelve original cases and both supervision modes against the
  authenticated saved baseline, retaining all original limits and failures.
- Keep the cascade default and geometric implementation unchanged; reject
  candidate promotion when any frozen comparison condition fails.

### 2026-09-28 integrated robustness evaluation

- Add an optional causal live observation-fault boundary with exhaustive source
  accounting and exact reconciliation against the offline injector.
- Add twelve frozen maneuver/fault/mismatch cases, each executed with supervision
  off and on; keep response correctness separate from flight performance.
- Authenticate both full histories, reconstruct estimator/health/mission/control
  behavior, reject altered evidence and retain all failures.
- Preserve controller and estimator laws, original gains, response budgets and
  acceptance limits; keep the cascade default and geometric qualification open.

### 2026-09-28 observation loss supervision

- Add an opt-in supervisor with independent position/altitude unhealthy-time
  budgets, explicit optional altitude, strict epoch order and latched abort.
- Enforce recovery before expiry, preserve timers across mission phases, and
  stop before terminal commands while retaining existing safety guard priority.
- Check real ESKF dropout/rejection/delay responses, healthy mission parity,
  independent stream failures and authenticated decision reconstruction.
- Preserve controllers, estimator, evidence formats and open noisy-flight
  requirements; define broader integrated fault/mismatch evaluation next.

### 2026-09-28 observation health monitoring

- Add independent position/altitude availability states from causal estimator
  events, explicit schedule-based ages, rejection runs and recovery counts.
- Add an optional passive mission observer with immutable history, strict
  ordering/identity checks and reset; preserve flight results and archive formats.
- Verify complete payload and saved-byte equality with monitoring on/off,
  deterministic event replay, timing boundaries and damaged-evidence rejection.
- Document the monitoring boundary and define supervisor policy as the next task.

### 2026-09-26 geometric startup/hover closeout

- Reproduce the known spline and hover failures and reconstruct their complete
  force, moment, estimator-correction and actuator histories.
- Check a sampled geometric/filter/motor model against nonlinear perturbations
  in both horizontal directions, and test one frozen coherent-force experiment.
- Reject that candidate: lower spline effort does not compensate for both failed
  full hovers. Preserve production defaults and all original requirements;
  close this tuning study and define observation health monitoring as the next task.

### 2026-09-26 documentation and project review

- Simplify the README and organize guides around system design, implementation,
  controller tradeoffs, current status and a bounded next-step plan.
- Correct outdated estimated-trajectory support claims, complete the decision
  and verification indexes, and remove personal names and machine-specific paths.
- Preserve historical numerical results while distinguishing them from current
  qualification. Add a project review and explain why noisy geometric feedback
  still misses its combined hover and effort requirements.
- Add documentation checks to the regular quality gate for local links, section
  anchors, code fences and Python/JSON example syntax.

### 2026-09-26 geometric audit and bounded tuning

- Add explicit experimental estimator-correction rebasing and measured physical
  acceleration/jerk derivatives, preserving default controller behavior and
  sensor-only feedback. Test immutable filter memory, physical kinematics,
  correction accumulation, measured command reconstruction and exact ESKF replay.
- Add a bounded, reproducible development/qualification runner with saved-payload
  verification. Retain all six failed development profiles; no noisy-flight
  performance promotion or threshold change is made.
- Audit the complete project, repeat the original true-state regression,
  document completed capabilities and remaining boundaries, and publish tested
  source checkpoints to prevent loss of uncommitted implementation work.

### 2026-09-25 trajectory missions

- Add conservative Bernstein whole-curve geometry, speed, acceleration, nominal
  thrust, tilt and fixed-yaw reference-rate bounds with explicit numerical padding.
- Add bounded uniform retiming with retained attempts and explicit failure.
- Integrate owned minimum-snap segments into the true-state mission runner with
  preflight checks, preserving the historical mission serialization and protocol.
- Add five fixed complete-flight cases and independent reference/retiming checks;
  controller gains and original estimated-feedback failures remain unchanged.

### 2026-09-25 audit and planning

- Add a fixed-duration seventh-degree minimum-snap position solver with C3
  continuity, prescribed endpoint derivatives, checked scaled optimization,
  exact integrated cost and a fixed-yaw position-reference adapter.
- Retain estimated-feedback version 2 as an explicitly limited numerical
  reference after a same-case full-hover comparison. Original 8 cm qualification
  failures and both historical profiles remain unchanged.
- Condense the front page, preserve foundational mathematics in its own guide,
  and add a current documentation map and status.
- Remove the unused initial `vectors.squared_norm` scaffold and its sole test;
  no active production caller existed.

### Earlier milestones (partial historical list)

### Added

- Pre-update ESKF innovation whitening and normalized innovation squared diagnostics;
  immutable derived results with explicit finite and covariance-domain validation.
- Opt-in per-sensor fixed outlier gating and a rounded 99% chi-square preset. Rejection
  precedes correction and preserves state/covariance; events retain diagnostics and thresholds.
- Legacy unscored and diagnostics-only replay modes, explicit nominal-policy forwarding,
  and 236 analytic, numerical, gate, ownership and integration regression cases. ADR 0007
  defines the bounded contract; no persisted schema or correction-core change is introduced.

- Measurement-only ESKF sensor replay with explicit first-sample initialization, left-held
  paired IMU timing, canonical correction order, and stale/pending/disabled event outcomes.
- Immutable replay configuration, state/covariance histories and full correction diagnostics;
  strict recorded-run and nominal-parameter adapters without truth-payload access.
- 246 replay unit/integration cases, including save/load equality, truth isolation, seeded
  stationary/moving runs and vertical-bias correction; ADR 0006 defines the bounded contract.

- Same-epoch 15-state ESKF local-position and positive-up altitude measurement models and
  updates, scaled Cholesky gain solves, Joseph covariance, right-local injection/reset, and
  immutable posterior diagnostics.
- Analytic, finite-difference, numerical-edge, seeded, and known-motion regression evidence
  for the measurement-update core; ADR 0005 records the contracts and scope.

- Initial repository and reproducible workflow setup.
- Reproducible Python 3.12 workflow managed by uv 0.12.3 and `uv.lock`.
- Src-layout `quadrotor_math` package.
- Initial deterministic `squared_norm` vector function and unit test.
- Ruff linting and formatting, strict mypy checks, and pytest quality checks.

### Fixed

- Truth-execution preflight now catches configuration values or initial attitudes that the
  current numerical plant cannot consume before allocating histories or creating RNGs.
- Manifest decoding validates redundant sensor stride/effective-period values.
- Run artifacts reject nonfinite floating payloads and use scheduler-consistent timestamps.
- Rotor arithmetic and quaternion operations guard audited overflow/underflow cases.
- ESKF nominal quaternion validation now matches its downstream rotation boundary.
