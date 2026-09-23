# Changelog

All notable changes to this project will be documented in this file following a simple Keep a Changelog-style structure.

## [Unreleased]

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
