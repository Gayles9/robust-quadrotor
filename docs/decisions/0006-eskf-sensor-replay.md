# 0006: Measurement-Only ESKF Sensor Replay

- Extension: [ADR 0007](0007-eskf-innovation-gating.md) adds optional pre-update scoring,
  fixed gates and `REJECTED` events while preserving this epoch and default execution policy.
- Extension: [ADR 0010](0010-eskf-endpoint-propagation.md) adds explicit endpoint IMU
  propagation with sample-noise memory; the original default remains unchanged.
- Extends: [ADR 0004](0004-eskf-error-state-conventions.md) and
  [ADR 0005](0005-eskf-measurement-updates.md)
- Implementation: `src/quadrotor_math/eskf_replay.py`,
  `src/quadrotor_math/eskf_run_replay.py`
- Evidence: `tests/unit/test_eskf_replay.py`, `tests/unit/test_eskf_run_replay.py`,
  [verification record (ZIP)](../../evidence/development-records.zip)

## Context and selected scope

The filter's prediction and same-epoch measurement correction are implemented mathematical
primitives. A recorded observation also has acquisition time, scheduled delivery time and
the clock row at which delivery actually occurred. Passing every delivered value directly
to a timestamp-free correction would mistake an older physical observation for a current
one. The runner must therefore define its time and data-access contracts before composing
the existing equations.

This decision implements a deterministic, in-memory replay runner, a restricted adapter
for generated or loaded `RunArtifactData`, and a nominal-parameter configuration adapter.
The mathematical runner receives only IMU measurements, position/altitude observations,
timing indices and explicit estimator assumptions. It cannot inspect simulator truth.
The run adapter reads only the clock and sensor/delivery fields. It does not read true
position, velocity, attitude, angular velocity, IMU bias histories or rotor commands.

Replay is a deterministic way to evaluate recorded measurements. Online control
uses the separate [streaming interface](0013-estimated-state-mission-feedback.md).

## Clock, initialization and IMU interval

Let the run clock have `N+1` rows $`T_j=j\Delta t`$, $`j=0,\ldots,N`$, where $`\Delta t>0`$
is the truth time step in seconds. The producer samples only completed nonzero truth rows;
there is no IMU sample at $`T_0`$. The supported run adapter requires both IMU streams at
every $`T_j`$, $`j=1,\ldots,N`$, with zero delay and actual delivery at that same row.

The replay clock is

```math
t_i=T_{i+1},\qquad i=0,\ldots,N-1.
```

Consequently, the caller's initial prior $`(\hat x_0^-,P_0^-)`$ belongs at $`t_0=\Delta t`$,
not at the simulator's zero epoch. `initial_time_s` must exactly equal `time_s[0]`.
The caller supplies this prior independently; the adapter never copies a truth row or
silently replaces its IMU bias estimates. No prediction is performed before the first row.
If the desired prior is at time zero, obtaining a time-zero IMU sample or defining another
initialization method requires a separate contract; future samples are not backfilled.

For each subsequent row $`i>0`$, the previous paired sample is held over the interval:

```math
(\hat x_i^-,P_i^-)=\operatorname{Predict}_{\text{ADR 0004}}
\left(\hat x_{i-1}^+,P_{i-1}^+,f^m_{B,i-1},\omega^m_{B,i-1},
g,Q_c,t_i-t_{i-1}\right).
```

Here $`f^m_B\in\mathbb R^3`$ is measured FRD specific force in m/s², not gravitational
acceleration; $`\omega^m_B\in\mathbb R^3`$ is measured FRD rigid-body angular velocity
in rad/s, not rotor speed. `Predict` subtracts estimated biases, applies the body-to-world
rotation and positive-down NED gravity, and propagates the same 15-state error covariance
as before. No prediction or covariance equation is changed.

This left-endpoint hold is causal: a measurement acquired at $`t_i`$ cannot affect the
state reported at an earlier time. The final IMU row has no following interval and is
validated but not used for an extra extrapolation. The mathematical input supports
strictly increasing nonuniform times and uses the actual interval difference; the run
adapter deliberately requires the repository's fixed clock.

Requiring full-rate paired IMU avoids inventing asynchronous pairing or interpolation,
and avoids repeatedly treating one held noisy sample as independent process noise across
multiple truth-grid predictions. It does not remove the existing high-rate approximation:
prediction uses start-of-step world acceleration and first-order covariance discretization.
The plant's projected RK4 and the estimator's numerical order are not the same.

## Observation epoch and event order

`EskfReplayObservation` records a sensor kind, source `observation_index`,
`acquisition_index`, `delivery_index`, and a measured value. Epoch indices refer to the
replay clock, not the artifact's truth-clock rows. The adapter converts a delivered
truth index `j` to replay index `j-1`; `-1` remains pending beyond the horizon.

The stable sensor kinds retain artifact IDs: `LOCAL_POSITION=3` and
`BAROMETRIC_ALTITUDE=4`. Each kind has consecutive source row IDs from zero, and its
acquisition indices increase strictly. Streams may omit epochs. The input is copied and
canonicalized by `(delivery_index, sensor ID, source observation index)`, with pending
records placed last. Delivery may be out of acquisition order in direct replay input;
the same explicit stale policy still applies.

At each epoch the runner first predicts, except at initialization, then processes the
delivered observations. Fresh local position precedes fresh altitude. Each correction
uses the posterior state and covariance from the preceding correction at that epoch,
including its attitude-error reset. A measurement is used at most once.

| Status | Condition | Effect on the filter |
| --- | --- | --- |
| `FUSED` | Fresh enabled observation, with no gate or a passing gate under ADR 0007 | Apply the existing model, Joseph correction, injection and reset |
| `REJECTED` | Fresh enabled observation whose finite NIS exceeds its explicit threshold (ADR 0007) | Retain diagnostics; skip correction and preserve the current state/covariance |
| `STALE` | Delivery index is later than acquisition index | Log and skip; never reinterpret it as a current measurement |
| `DISABLED` | Fresh measurement with its fusion flag disabled | Log and skip; useful for an identical-input dead-reckoning comparator |
| `PENDING` | `delivery_index == -1` | Log at the end of the result; never fuse before actual delivery |

Pending and stale classification take precedence over enable flags. A delivery preceding
acquisition is invalid, not a status. All supplied values must remain finite even when
their observation will not be fused. A direct synthetic dropout is represented by absence
of a measurement at an epoch, not by a `NaN` value. This does not add a dropout generator to
the existing sensor model.

The chosen delay policy rejects every observation acquired at an earlier replay epoch.
No age threshold is tuned, no rewind is attempted, and no arrival-time approximation is
hidden in the filter. Tiny delivery-time differences that the scheduler resolves at the
same clock row remain same-epoch observations. This is a clock-resolution convention,
not compensation for a meaningful sensor delay. Delayed IMU is rejected by the adapter,
rather than treated as an ordinary skipped position observation.

## Run-adapter validation

`eskf_replay_input_from_run_artifact(data)` validates the subset that execution consumes:

1. At least one completed row, a zero-origin strictly increasing clock, and exact equality
   to `arange(N+1)*dt`.
2. `dt > 2*tol(T_N,T_N)` so adjacent clock rows resolve the scheduler comparison tolerance.
3. Both IMU streams exactly at every completed clock row, exactly zero represented delay,
   and matching actual delivery indices. Sparse, asynchronous or delayed IMU is unsupported.
4. Finite sensor payloads, valid shapes, consecutive source IDs, strictly increasing
   position/altitude acquisition times and exact membership in the replay clock.
5. Scheduled delivery no earlier than acquisition, and actual delivery at the first
   eligible clock row, or `-1` only when no eligible row exists.
6. Exact agreement of the global sensor/sequence/observation delivery table with the
   canonical deliveries reconstructed from the streams.

The eligibility test reuses the scheduler's comparison:

```math
t_{\mathrm{scheduled}}\le t_{\mathrm{clock}}+
16\epsilon_{64}\max(1,|t_{\mathrm{scheduled}}|,|t_{\mathrm{clock}}|),
```

where $`\epsilon_{64}`$ is float64 machine epsilon. A binary search finds the first
eligible row. Acquisition membership is exact; the adapter does not snap an off-grid
acquisition time or resample an IMU vector. These execution restrictions are intentionally
narrower than generic artifact storage. Valid older artifacts with incompatible sensor
schedules remain valid stored artifacts, but cannot run through this adapter.

This boundary is not a substitute for `load_run_directory`: the loader still authenticates
the saved bytes and validates their relationship to the manifest. The adapter verifies
timing/payload consistency for replay; it does not verify a trajectory's physical dynamics,
recover omitted acquisitions from a manifest, or establish data authenticity on its own.

## Configuration, noise units and ownership

`EskfReplayConfiguration` contains an explicit initial epoch, `EskfNominalState`, prior
`initial_covariance (15,15)`, positive scalar gravity, `continuous_noise_covariance (12,12)`,
nominal position/altitude model parameters and two strict Boolean fusion flags.
ADR 0007 appends optional `innovation_policy=None`; this default retains unscored
execution. A supplied policy enables pre-update diagnostics and optional rejection.

The 12-component continuous noise order and units remain ADR 0004. For example, an
accelerometer white-noise diagonal in $`Q_c`$ has units $`(\mathrm{m/s^2})^2\,\mathrm{s}`$,
while an accelerometer-bias random-walk diagonal has units
$`(\mathrm{m/s^2})^2/\mathrm{s}`$. Their roles are different. The caller supplies $`Q_c`$
explicitly. The nominal configuration's per-sample IMU standard deviations are not silently
treated as continuous-time densities.

Position and altitude noise are discrete per-observation covariance in m². The optional
`eskf_replay_configuration_from_nominal` helper uses only `nominal.world` and
`nominal.position_sensors`: gravity, assumed sensor biases, altitude reference and squared
observation standard deviations. Position axes are independent in that configuration, so
the resulting position covariance is diagonal. Direct configuration also accepts a
correlated `(3,3)` covariance. Positive standard deviations whose square overflows or
underflows to zero are rejected; exact zero and representable subnormal variances are not
silently inflated. The explicit initial state, covariance and $`Q_c`$ are preserved.

All covariance matrices must be finite, symmetric and PSD under the existing locally
scaled validation. Every scored or fused innovation covariance must additionally be numerically
positive definite. A singular innovation is an error, not a `STALE` event or hidden gate.
There is no jitter, pseudoinverse, clipping or auto-tuning. Independent observation noise
and the existing local-Gaussian error assumptions remain requirements of the caller.

Public containers are frozen, slotted and identity-equal. Stored arrays are independently
owned C-contiguous read-only float64 copies; real numeric arrays are accepted with explicit
shapes, without implicit reshaping. Index fields require non-Boolean Python integers.
Result state, event diagnostics and caller buffers do not share memory. Read-only flags
protect ordinary use, not deliberate hostile mutation. No RNG is owned or advanced.

## Interfaces and output meaning

| Identifier | Responsibility |
| --- | --- |
| `EskfReplayObservation` | One source-indexed position `(3,)` or altitude `(1,)` value and acquisition/delivery epoch indices |
| `EskfReplayInput` | Measurement-only clock `(n,)`, paired IMU arrays `(n,3)`, canonical owned observations |
| `EskfReplayConfiguration` | Explicit prior and noise/model assumptions, independent of truth |
| `replay_eskf` | Predict/correct sequence; returns one complete result or raises without caller mutation |
| `EskfReplayEvent` | Observation, exhaustive status, optional full `EskfMeasurementUpdate`, and optional pre-update score/threshold under ADR 0007 |
| `EskfReplayResult` | Time vector, tuple of `n` nominal states, `(n,15,15)` covariance history and all event outcomes |
| `eskf_replay_input_from_run_artifact` | Strict clock/sensor adapter; no truth payload access |
| `eskf_replay_configuration_from_nominal` | Explicit nominal world/sensor mapping; caller still owns prior and $`Q_c`$ |

History row $`i`$ is the posterior after *all* fused observations at $`t_i`$. A fused event's
update retains the state and covariance immediately after that *individual* correction,
plus innovation, innovation covariance, gain and injected error. It may therefore differ
from the final history row if altitude follows position. The correction remains diagnostic
and must not be applied a second time. A history row with no update is simply the predicted
state/covariance, or the initial prior at row zero.

The input and configuration validate before execution. If later arithmetic fails, including
after an earlier successful correction, no partial public result is returned and no caller
state, file or RNG has been changed. The whole operation can be repeated deterministically.
Diagnostics are in memory only: neither the 35-array run schema nor its manifest is extended.
This batch replay API has no checkpoint or continuation state; the streaming
interface owns its own continuation memory.

## Verification and limits of evidence

The replay tests include independent analytic altitude and kinematic references, exact
composition against the public prediction/correction boundaries, left/right endpoint traps,
an arbitrary-axis Rodrigues reference with nonzero IMU bias, quaternion-sign equivalence,
first/final/one-row/nonuniform epochs, canonical ordering, future-prefix causality,
dropout/recovery, stale/pending/disabled behavior, singular and overflow failures,
ownership, invalid covariance and contradictory source/delivery metadata.

Integration tests compose generated and saved/loaded sensor runs with the adapter and
runner. Saved/loaded replay produces identical tested state, covariance and diagnostic
bytes. A read-trap rejects any attempt to read truth trajectories, true biases or commands.
Twenty-four fixed-seed stationary/constant-velocity regression cases meet their declared
position-RMSE comparison, and a separate vertical-bias fixture verifies correction of
dead-reckoning drift. Their precise durations and assertions are in the verification record.

These are regression and integration results, not held-out consistency evidence,
general IMU-bias observability or flight certification.
[ADR 0007](0007-eskf-innovation-gating.md) describes opt-in outlier gating, and
[ADR 0009](0009-eskf-completion-validation.md) describes the separate consistency
and fault campaign. Replay does not implement delayed-state rewind, asynchronous
IMU treatment or middleware transport.
Revisit this contract when one of those capabilities requires new timing or data semantics;
do not silently relax the adapter or treat stale data as current.
