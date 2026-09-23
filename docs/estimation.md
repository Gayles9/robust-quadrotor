# Measurement-Driven Error-State Kalman Filter

The estimator is a known-prior, fixed-gravity, 15-error-state inertial navigation filter.
It estimates NED position and velocity, body-to-world attitude and FRD accelerometer and
gyroscope biases from paired IMU samples, local Cartesian position and positive-up
altitude. Its runtime mathematics and measurement replay use NumPy only. The completion
boundary and predeclared acceptance protocol are in [ADR 0009](decisions/0009-eskf-completion-validation.md).
Measured outcomes belong to the completion verification record; execution success alone
does not establish statistical consistency or observability.

The [completion record](progress/2026-09-23-eskf-completion.md) documents 100 held-out
30-second nominal trajectories with no divergence and the completed fault/bias/sensitivity
evaluation. Full-state NEES coverage is 88.84%, below the declared investigation band.
That calibration finding remains explicit; the basic implementation and engineering
evidence are complete, while an unqualified statistical-consistency claim is not supported.

## State, prediction and correction

The nominal state is `(position_W, velocity_W, q_WB, accelerometer_bias_B,
gyroscope_bias_B)`. The Hamilton scalar-first unit quaternion `q_WB` maps FRD body
coordinates to NED world coordinates. There are 16 stored scalars but only 15 local
degrees of freedom. The error order is position, velocity, body-local rotation vector,
accelerometer bias and gyro bias, each with three components. Units are m, m/s, rad,
m/s² and rad/s. Position, velocity and biases use true-minus-nominal additive errors;
attitude uses `R_true_WB = R_nominal_WB Exp(skew(delta_theta_B))`.

For measured specific force `f_m` and angular rate `omega_m`, prediction subtracts the
estimated biases, forms `a_W = [0,0,g] + R_WB @ (f_m-b_a)`, and integrates position,
velocity and the quaternion over a left-held IMU interval. Specific force excludes
gravity: a stationary level accelerometer observes `[0,0,-g]`, not zero or `[0,0,g]`.
Quaternion normalization maintains its constraint. The continuous error Jacobians,
noise signs, first-order transition and covariance equations are specified in
[ADR 0004](decisions/0004-eskf-error-state-conventions.md). Bias random walks increase
uncertainty even though the nominal biases remain constant during prediction.

The position prediction is `p_W + assumed_position_bias_W`; the altitude prediction is
`reference_altitude - p_W[2] + assumed_altitude_bias`. Thus a positive altitude innovation
corrects the NED-down coordinate negatively. Sensor biases here are explicit assumptions,
not additional estimated states. For observation `z`, prediction `h`, Jacobian `H`, prior
covariance `P` and independent per-observation covariance `R`, the innovation is `r=z-h`
and its covariance is `S=H P H.T+R`. A diagonally scaled Cholesky solve computes the gain
without forming an inverse. The correction is injected once, Joseph form updates the
covariance, and the SO(3) right Jacobian resets its attitude coordinates and every
attitude cross-covariance. See [ADR 0005](decisions/0005-eskf-measurement-updates.md).

The caller must supply a meaningful initial pose/velocity/bias estimate and covariance
at the first IMU timestamp. There is no automatic stationary alignment, magnetometer
heading, Earth-rate/geodetic correction or unknown-pose bootstrap. Small local attitude
errors and meaningful uncertainty are part of the filter model; passing array validation
does not make arbitrary large-error initialization reliable.

## Interfaces, timing and failure contracts

| Boundary | Responsibility and contract |
| --- | --- |
| `EskfNominalState`, `predict_eskf` | Typed state and one deterministic IMU prediction |
| `update_eskf_local_position`, `update_eskf_barometric_altitude` | Same-epoch corrections; owned posterior/gain/residual/reset diagnostics |
| `EskfReplayInput` | Increasing `time_s (n,)`, paired specific force and rate `(n,3)`, typed observation tuple; no truth fields |
| `EskfReplayConfiguration` | Explicit epoch/state/P, gravity, Q, nominal observation models/R and optional gate policy |
| `replay_eskf` | State/covariance histories and an exhaustive observation event ledger |
| `eskf_replay_input_from_run_artifact` | Validated adaptation of full-rate, simultaneous, zero-delay recorded IMU; no truth-state access |
| `evaluate_eskf_replay` | Separate truth-based error/NEES calculation, requiring exact epoch equality |
| `inject_eskf_observation_faults` | Copy measured observations with explicit faults and retain a separate source-label ledger |
| `make_eskf_synthetic_case` | Independent analytic kinematics, randomized measurement generation and known-prior verification fixture |

At `t[k+1]`, replay predicts with IMU row `k`, then corrects with fresh position followed
by fresh altitude. No prediction precedes `t[0]`; the last IMU row has no following interval.
Each observation has an acquisition index and actual delivery index. Delivery `-1` means
pending beyond the horizon. A delivered observation acquired at an older epoch is `STALE`
and cannot affect state or covariance. Disabled fresh observations are `DISABLED`.

With `innovation_policy=None`, legacy replay remains unscored. An explicit
`EskfInnovationPolicy()` records NIS without gating. The explicit 99% preset uses
thresholds 11.345 for 3D position and 6.635 for altitude. Only NIS **strictly greater** than
its threshold is `REJECTED`; equality is accepted. Rejection precedes correction and the
next sensor sees the actual current posterior. Invalid arithmetic or singular innovation
covariance raises an error; it is never disguised as statistical rejection.

Arrays are validated, independently copied and exposed read-only. Covariances must be
finite, symmetric and positive semidefinite under the locally scaled checks. Innovation
covariance and full-state NEES covariance additionally require a numerically positive-
definite Cholesky factor. No jitter, pseudoinverse, negative-variance clipping, adaptive
noise scaling or truth-dependent recovery is used. Read-only flags protect ordinary
ownership; they are not a security boundary against deliberate buffer mutation.

## Independent motion and noise fixtures

Synthetic motion does not call ESKF propagation or numerical plant integration. For each
NED axis, the exact position, velocity and acceleration are

$$p_i(t)=A_i\sin(w_i t+\alpha_i)+u_i t,$$
$$v_i(t)=A_i w_i\cos(w_i t+\alpha_i)+u_i,\qquad
a_i(t)=-A_i w_i^2\sin(w_i t+\alpha_i).$$

Here `A` is in metres, `w` in radians/second, `alpha` in radians and `u` in m/s. The
roll/pitch/yaw angles have an analogous sinusoidal form plus optional constant Euler
rates. For roll `phi`, pitch `theta` and yaw `psi`, the body-to-world rotation is
`Rz(psi) Ry(theta) Rx(phi)`. Its differentiated body angular velocity is

$$\omega_B=\begin{bmatrix}
\dot\phi-\dot\psi\sin\theta\\
\dot\theta\cos\phi+\dot\psi\sin\phi\cos\theta\\
-\dot\theta\sin\phi+\dot\psi\cos\phi\cos\theta
\end{bmatrix},\qquad f_B=R_{WB}^{T}(a_W-[0,0,g]^T).$$

Central differences independently verify `p_dot=v`, `v_dot=a` and
`R_dot=R skew(omega_B)`. Noise-free strapdown integration is checked under step refinement;
the existing left-held nominal integration has first-order error for changing inputs.
The stationary control uses zero motion. The translating/yaw control uses NED velocity
`[.5,-.25,.1]` m/s and yaw rate `.2` rad/s. Excited motion uses the predeclared independent
amplitude/frequency/phase distributions in ADR 0009. It is a rigid-body kinematic
estimator test, not evidence of achievable thrust, closed-loop tracking or rotor feasibility.

Measurements add true bias and independent Gaussian white sample noise. Bias increments
have variance `eta² dt`, beginning after the initial epoch. Motion, initial error, initial
true biases, four white-noise streams, two bias walks and fault offsets have separate
PCG64 stream IDs. Stream derivation is explicit, versioned and independent of execution
order. The measured input and nominal configuration are the only arguments passed to
replay; reference states and fault labels are consumed afterward by evaluation.

The filter expects continuous process-noise spectral density. For held independent
IMU samples with standard deviation `sigma`, its leading increment variance is matched
by `Q_c = diag(sigma_a² dt, sigma_g² dt, eta_a², eta_g²)`. Observation `R` instead uses
the squared per-observation standard deviation without `dt`. This conversion does not
supply the higher-order position/cross terms omitted by the baseline discretization.

## Fault evidence and its denominators

`EskfObservationFault` identifies a source sensor and row. A dropout removes that
observation, while an offset is additive in the sensor's units and delay adds integer
steps to its delivery epoch. Dropout cannot simultaneously offset or delay the same
source. Reindexed survivors preserve acquisition order; the ledger retains original
identities. Delays beyond the horizon become pending. Unknown/duplicate source IDs,
invalid offsets and arithmetic overflow fail atomically. No IMU samples are altered.

Precision is `TP/(TP+FP)`, recall is `TP/(TP+FN)` and false-positive rate is
`FP/(FP+TN)`. A positive label means an injected nonzero outlier; a positive decision
means NIS rejection. Dropped, stale, pending, disabled and unscored observations are
counted separately and never treated as true negatives. An empty denominator is `null`,
not 1. Unavailable outliers are separately counted. Burst detection delay is the time
from burst start to its first rejected labeled outlier, or `null` when none is detected.

All scored NIS values, including rejected values, remain in reports. Gated and ungated
variants share the same corrupted data and prior. Gates change subsequent state history,
so even an uncensored sequence from a gated filter is not guaranteed chi-square distributed.
Precision/recall apply to the declared magnitude and prevalence distribution; they are
not general fault-classification guarantees. A poor model/prior can reject good readings.

## Reproduce the experiments

Use Python 3.12 and the repository's pinned uv 0.12.3. From the repository root:

```bash
uv sync --locked
make check
uv run pytest -W error
uv run python -m experiments.kalman_sandbox --output /tmp/kalman-sandbox
uv run python -m experiments.eskf_validation --partition smoke --output /tmp/eskf-smoke.json
uv run python -m experiments.eskf_validation --partition development --workers 4 --output /tmp/eskf-development.json
uv run python -m experiments.eskf_validation --partition validation --workers 4 --output /tmp/eskf-validation.json
uv run python -m experiments.plot_eskf_validation --input /tmp/eskf-validation.json --output /tmp/eskf-validation-plots
```

Use new paths: reports and plot directories refuse overwrite. Worker count changes job
throughput, not seed assignment or output ordering. Core computations do not require
Matplotlib; the development group installs it for headless plotting and plot tests.
Pytest includes the repository root on its test import path to exercise the `experiments`
package; that package is not included in the runtime wheel.

The CLI emits the protocol hash before running, captures actual Git/installed-software
provenance and hashes source/configuration bytes before and after execution. A changed
source invalidates publication of the report. JSON retains all planned seeds and variants;
any failure suppresses its complete-group ensemble rather than selecting survivors.
Finite divergent cases remain included. Full NEES/NIS histories, physical error metrics,
sampled error/bias norms, covariance checks, fault decisions and representative component
uncertainty support inspection.
Plot metadata records the exact input report hash and Matplotlib version. No generated
result, figure or large log is committed.

## Interpreting completion

The acceptance target is a functioning basic ESKF under its documented input, prior,
noise and timing contracts, with measured estimation and fault evidence. The code reports
execution success separately from numerical performance and statistical findings. The
90–98% consistency investigation band is not an automatic tuning objective. Repeated
epochs are correlated; the independent-replicate count is the number of seeds at an
epoch, not the number of time samples. Pointwise bands are not simultaneous guarantees.

Stationary and pure-yaw motion do not expose every attitude/bias direction as strongly
as changing translational acceleration and attitude. Bias-convergence claims therefore
apply to the excited evaluation distribution, not every flight or every initial condition.
The filter retains its first-order propagation and local Gaussian assumptions. Persistent
observation loss, sustained wrong-model rejection, large attitude errors, hardware faults,
unknown datums and unobservable states remain technical limits. The separate control gate,
live transport, estimator continuation/persistence, delayed correction and flight-stack
integration are not established by this estimator-only completion.
