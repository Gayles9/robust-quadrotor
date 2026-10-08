# 0013: Causal Online ESKF and Estimated-State Mission Feedback

## Scope

Estimated feedback closes the loop through the sensors: the endpoint ESKF
estimates position, velocity, attitude and sensor biases, and the cascade uses
those estimates to control the simulated vehicle. This decision describes that
causal connection and the original noisy-flight evaluation.

Online and offline fusion share the correction sequence, preventing two
independent implementations of the same equations. A private feedback hook
shares the mission loop while preserving the true-state default. Plant physics,
controller gains and filter equations are the same in the paired comparison.

## Information boundary and initialization

The online estimator accepts only an explicit independent prior/configuration,
paired instantaneous IMU samples, epoch time, and delivered position/altitude
observations. It has no truth, physics, control-command, reference, scheduler or
RNG access. The caller supplies the prior at exactly t=0; there is no truth-derived
initialization or stationary alignment. Each successful call consumes exactly one
strictly later epoch (the first call initializes at t=0). Invalid input or failed
arithmetic must not partially advance a stream.

The endpoint's full 21-coordinate covariance and six conditional sample-noise
means persist between calls, including after both same-epoch observations. The
15 physical states retain ADR 0010's right-local NED/FRD convention. Both existing
propagation modes may be streamed; the mission requires the endpoint option.
Online estimates must agree with offline replay of the recorded measurements.

At each epoch the rate feedback is current gyro measurement minus the posterior
gyro-bias estimate and endpoint gyro-noise conditional mean. It is an estimated
instantaneous FRD rate, not true omega and not a new low-pass rate filter.
Position/velocity feedback and attitude feedback use the same posterior epoch.

## Fixed-grid order and sensor semantics

Plant and paired IMU run at 400 Hz; inner controller at 100 Hz and outer at 50 Hz.
Every plant epoch, including zero and termination, has an IMU sample. At t=0 the
prior is initialized, not predicted. At later epochs:

1. Arrive at the state produced by the previous held rotor command; advance
   sensor bias walks over that completed interval.
2. Acquire instantaneous specific force from actual motor force plus drag and
   true mass, and gyro from actual body rate; add true biases and sample noise.
3. Acquire due slow measurements, then deliver all due observations. Reuse the
   existing fixed-rate scheduler: slow acquisitions start at their first positive
   period, position 5 Hz and barometer 25 Hz. Fixed delays may be off-grid.
4. Predict to the current epoch with the two IMU endpoints; fuse fresh position
   before fresh altitude. Gate using the existing policy. Delivered older
   acquisitions are STALE and do not correct the current state; observations
   still queued at termination are PENDING. Neither is silently fused late.
5. Evaluate estimated-state mission guards/completion, then the existing outer
   and inner controller if due. Hold commands between ticks. No terminal command.
6. Advance the unchanged stage-resolved motor/drag/RK4 plant one interval.

The simulation separately retains its true-state geofence/tilt monitor as an
explicit numerical safety oracle; its abort reason is distinguished from an
estimated-state guard. It never feeds position, velocity, attitude or rate into
the controllers or estimated completion decision. Such a truth monitor is not
available to a real vehicle and is not a deployable safety system.

Truth sensor parameters, estimator noise/prior assumptions and controller nominal
parameters are separately supplied and copied. Existing named PCG64 streams supply
the four measurement noises and two bias walks; changing slow delivery timing must
not change acquired IMU values. Core stream calls contain no randomness.

## Evidence contracts

The evidence contains complete truth/reference/control histories, true biases,
raw paired IMU, all slow acquisition/delivery identities, post-update nominal
states and physical covariances, actual rate-feedback vectors, all observation
dispositions and innovation diagnostics. All public numeric arrays are finite, independently
owned and read-only. No historical RunArtifactData schema is repurposed.

An experimental bundle binds complete per-trial NPZ data by exact-byte SHA-256,
records protocol/source hashes and software provenance, retains every planned
success/failure, and refuses overwrite. Loading validates measured clocks,
controller correspondence and metrics. Offline replay comparison is an explicit
audit, not a substitute for online execution. Truth is allowed only in generation,
independent safety monitoring and post-run scoring.

## Verification and campaign design

- Reproduce all five fixed true-state mission histories. Test online/offline equality for first-order and endpoint
  modes, same-epoch order, nonzero shared noise memory, gates, stale/disabled
  events, pending ledgers, invalid input, failed-call atomicity and ownership.
- Independently reconstruct commanded feedback from recorded estimates. Change
  hidden truth/true bias without changing an initial prior/measurement and prove
  it cannot directly change the controller input. Test future-data causality,
  exact hold clocks, no terminal command and guard precedence.
- Noiseless square versus true-state numerical baseline: common-grid position
  difference <.005 m and attitude difference <.05 degree. This is integration
  accuracy evidence, not a noise/robustness claim.
- Noisy fixed hover (60-second hold), square, vertical step and mild wind, plus
  development square seeds 8100..8102. Use the established illustrative 1 kg model,
  gains, bounds, reference durations and completion tolerances unchanged.
- Validation seeds 90000..90009 use the fixed code and protocol. All ten estimated
  square missions must complete with no guard or numerical failure, full-mission
  true position RMSE <.15 m, final true position/speed error <.15 m/<.15 m/s,
  estimated position RMSE against truth <.10 m, peak attitude estimation error
  <15 degrees, longest actuator limiting <=.5 s and none in the final second.
  Every trial also runs the unchanged true-state cascade with identical truth
  initialization, plant and mission; report the degradation without demanding
  that noisy feedback outperform truth feedback. Keep all failures visible.
- Test delayed slow measurements as stale/pending and explicit fusion disabling;
  do not claim successful delayed-measurement compensation or broad fault tolerance.
- Formatting, lint, typing and warnings-as-errors tests complement independent
  physical/dataflow reconstruction and plot inspection.

This campaign establishes a bounded estimated-feedback numerical baseline, not
global stability, statistical recalibration of the filter, hardware readiness,
or autonomous arming/touchdown. Subsequent robustness and geometric-control work
requires its own scope and audit.

## Verification qualification

The implementation and unit/static gates pass; the first noisy hover exceeds
the inherited .08 m hold-peak target (.107563 m). That fixed case remains a failed
acceptance condition even though it completes without limiting. The code, gains,
seeds and criteria were not changed to turn it into a pass. The complete
[verification record (ZIP)](../../evidence/development-records.zip) separates
algorithm/interface correctness, previous-path compatibility and measured performance.

The bounded gain/prior design is recorded separately in
[ADR 0014](0014-estimated-feedback-bandwidth.md) and its
[qualification record (ZIP)](../../evidence/development-records.zip).
It preserves this original profile and its measured miss.
