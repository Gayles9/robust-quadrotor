# Measurement-Driven Error-State Kalman Filter

The estimator combines fast but drifting inertial measurement unit (IMU) readings
with slower position and altitude observations. The IMU contains an accelerometer
and gyroscope. Integrating their readings predicts motion between observations;
position and altitude measurements then correct that prediction.

An error-state Kalman filter (ESKF) tracks uncertainty in small corrections to
its best estimate. I use a three-component local rotation error so attitude
corrections respect the quaternion's unit-length constraint. A quaternion is
a four-number representation of orientation; its unit-length constraint leaves
three independent rotation coordinates.

The core is in [eskf.py](../../src/quadrotor_math/eskf.py), with the sampled-IMU
extension in [eskf_endpoint.py](../../src/quadrotor_math/eskf_endpoint.py) and live
epoch handling in [eskf_online.py](../../src/quadrotor_math/eskf_online.py).
[System design](../guides/system-design.md) explains how its output reaches control;
[controller tradeoffs](../results/controller-tradeoffs.md) explains why an accurate estimate
at one instant does not guarantee a small physical tracking error.

The filter requires an initial estimate and covariance supplied by the caller
and assumes known, fixed gravity. Its 15 error coordinates describe position,
velocity, attitude, accelerometer bias and gyro bias, with three coordinates
each. Position and velocity use north-east-down (NED) axes; sensor biases use
forward-right-down (FRD) body axes. Runtime mathematics and measurement replay
use NumPy only. The [evaluation protocol](../decisions/0009-eskf-completion-validation.md)
defines the tested motion, noise and acceptance conditions.

A covariance matrix describes uncertainty and how errors in different coordinates
are related. **NEES** (normalized estimation error squared) compares actual
state error with that covariance. **NIS** (normalized innovation squared)
compares a measurement's prediction error with its expected uncertainty.
NEES requires simulated truth and is used for evaluation; NIS can be calculated
from measurements and the filter alone. Neither a successful run nor a single
low error proves that the uncertainty model is calibrated.

## Demonstrated estimator performance

The [first-order evaluation (ZIP archive)](../../evidence/development-records.zip) contains 100 held-out
30-second nominal trajectories with no divergence and the completed fault/bias/sensitivity
evaluation. Full-state NEES coverage is 88.84%, below the declared investigation band.
The first-order method therefore does not support an unqualified
statistical-consistency claim.
The [endpoint method](../decisions/0010-eskf-endpoint-propagation.md)
addresses numerical propagation and its sampled-noise covariance together. The
first-order method is the API default; select endpoint mode explicitly
as described below. Its [separate evaluation (ZIP archive)](../../evidence/development-records.zip)
uses 100 held-out seeds and 380 replays across distinct evaluation groups.
For the nominal endpoint group, central-95% NEES coverage was **95.29%** with
mean **14.60**, versus 91.99% and 16.96 for the paired first-order method.
At each sampled time, coverage is the fraction of nominal runs whose NEES
lies inside the central 95% chi-square interval for 15 degrees of freedom.
The reported statistic averages those fractions over time. It does not pool
all 380 nominal and fault replays.
Mean position RMSE was .06739 m versus .06809 m. There were no numerical failures or
nominal/gated divergences, and the declared bias/fault/recovery targets passed. This
supports improved consistency for the tested distribution; it is not a universal guarantee.

The [supported alignment component](prearm-component.md) can construct an
endpoint initial state under its explicit stationary-support contract. Ordinary
replay and mission APIs still consume a supplied prior; they do not infer that
support automatically.

## Causal online execution and mission feedback

`EskfOnlineEstimator` in `eskf_online.py` exposes the same prediction/correction
mathematics as caller-driven, measurement-only epochs. `step` receives time,
paired IMU samples and the current epoch's delivered slow observations. Its first
sample must coincide with the explicit prior; subsequent times strictly increase.
Invalid or numerically failed calls do not consume an epoch or partially update
continuation state. Returned snapshots own their arrays independently.

Both propagation modes remain supported. Endpoint mode preserves the full
21-coordinate joint covariance and six conditional sample-noise means privately
between calls. The returned `EskfOnlineEstimate` contains the 15-state physical
covariance and the posterior instantaneous rate estimate: measured gyro minus
estimated gyro bias minus conditional current gyro sample-noise mean. First-order
mode has no sample-noise mean. The online/offline shared correction path preserves
position-before-altitude fusion, innovation gates and stale/disabled dispositions.

`simulate_estimated_mission` connects endpoint estimates to the cascade or the
explicit geometric controller without truth-derived controller inputs or
completion. See the
[integration guide](estimated-feedback.md) for exact timing, sensor generation,
independent prior, ownership, evidence format and the separate truth safety oracle.
This synchronous numerical integration is not a real-time flight service and adds
neither automatic startup alignment nor delayed-state rewind. Its performance
record remains separate from the known-prior statistical-calibration campaign above.

## State, prediction and correction

Prediction advances the estimate using the IMU; correction adjusts it when
an observation arrives. The **prior** is the estimate before that correction,
and the **posterior** is the estimate afterward. An **epoch** is one sampled
time.

The nominal state is `(position_W, velocity_W, q_WB, accelerometer_bias_B,
gyroscope_bias_B)`. The Hamilton scalar-first unit quaternion `q_WB` maps FRD body
coordinates to NED world coordinates. There are 16 stored scalars but only 15 local
degrees of freedom. The error order is position, velocity, body-local rotation vector,
accelerometer bias and gyro bias, each with three components. Units are m, m/s, rad,
m/s² and rad/s. Position, velocity and biases use true-minus-nominal additive errors;
attitude uses $`R_{WB}^{\mathrm{true}}=R_{WB}\mathrm{Exp}([\delta\boldsymbol\theta_B]_\times)`$.

For measured specific force $`\mathbf f_m`$ and angular rate $`\boldsymbol\omega_m`$, default prediction subtracts the
estimated biases, forms $`\mathbf a_W=g\mathbf e_3+R_{WB}(\mathbf f_m-\mathbf b_a)`$, and integrates position,
velocity and the quaternion over a left-held IMU interval. Specific force excludes
gravity: a stationary level accelerometer observes `[0,0,-g]`, not zero or `[0,0,g]`.
Quaternion normalization maintains its constraint. The continuous error Jacobians,
noise signs, first-order transition and covariance equations are specified in
[ADR 0004](../decisions/0004-eskf-error-state-conventions.md). Bias random walks increase
uncertainty even though the nominal biases remain constant during prediction.

The predicted position measurement is `p_W + assumed_position_bias_W`;
the altitude prediction is
`reference_altitude - p_W[2] + assumed_altitude_bias`. Thus a positive altitude innovation
corrects the NED-down coordinate negatively. Sensor biases here are explicit assumptions,
not additional estimated states. For observation $`\mathbf z`$, prediction $`\mathbf h`$, Jacobian $`H`$, prior
covariance $`P`$ and independent per-observation covariance $`R`$, the innovation is $`\mathbf r=\mathbf z-\mathbf h`$
and its covariance is $`S=HPH^T+R`$. A diagonally scaled Cholesky solve computes the gain
without forming an inverse. The correction is injected once, Joseph form updates the
covariance, and the SO(3) right Jacobian resets its attitude coordinates and every
attitude cross-covariance. See [ADR 0005](../decisions/0005-eskf-measurement-updates.md).

The caller must supply a meaningful initial pose/velocity/bias estimate and covariance
at the first IMU timestamp. These APIs do not perform automatic stationary
alignment or unknown-pose initialization. There is no magnetometer heading
or Earth-rate/geodetic correction. Small local attitude errors and meaningful
uncertainty are part of the filter model; passing array validation
does not make arbitrary large-error initialization reliable.

## Endpoint integration and matched sample-noise covariance

For instantaneous paired samples, `EskfReplayConfiguration.sampled_imu_noise` selects
the second-order `eskf_endpoint.py` path. Its `EskfSampledImuNoise` contains two explicit
6-by-6 matrices in accelerometer-then-gyro FRD order: `sample_covariance_B` is the
per-sample covariance, and `bias_walk_spectral_density_B` is bias covariance per second.
Samples at different epochs, bias increments and the initial prior are independent.
Within each six-vector, correlations are allowed. The unused legacy
`continuous_noise_covariance` must be zero; mixed declarations are rejected.

At $`t_{k+1}`$ both endpoint samples are available. After subtracting estimated bias and
the old sample's conditional noise mean, let the two specific forces be $`f_0,f_1`$ and
body rates $`\omega_0,\omega_1`$. For $`h=t_{k+1}-t_k>0`$, $`R=R_{WB}[k]`$ and NED gravity $`g_W`$,

```math
\phi=\tfrac h2(\omega_0+\omega_1),\qquad
R_1=R\mathrm{Exp}([\phi]_\times),
```

```math
a_0=g_W+Rf_0,\qquad a_1=g_W+R_1f_1,
```

```math
v_1=v+h(a_0+a_1)/2,\qquad
p_1=p+hv+h^2(2a_0+a_1)/6.
```

Here $`\phi`$ is a body-local rotation vector in radians, $`R`$ and $`R_1`$ map body to world,
accelerations are NED m/s², velocity is NED m/s and position is NED metres. Translation
integrates a linear world-acceleration interpolant; attitude uses an exponential of
the averaged rate. Nominal biases stay constant during prediction. These equations
give second-order convergence on smooth motion, with remaining higher-order rotation
and interpolation errors. They are not an exact continuous-time model.

The covariance follows the derivative of this discrete map. Sample `k` is used by
intervals `k-1→k` and `k→k+1`. Treating successive average noises as independent would
undercount uncertainty. The implementation maintains `EskfEndpointState`: the physical
nominal state, the six-component `imu_noise_mean_B`, and `joint_covariance` of shape
`(21,21)`. The last six coordinates are temporary sample noise, not new physical states.

The prior is `diag(P0,Sigma)`, where `Sigma` is sample covariance. Each prediction
retains physical/sample cross terms, adds independent next-sample noise and the
bias endpoint increment with covariance `W*h`, and discards the previous noise variable
after its final use. `eskf_endpoint_map` returns the nominal state and analytic
matrices `A (21,21)` and `B (21,12)`; prediction uses

```math
C_1=AC A^T+B\mathrm{diag}(\Sigma,Wh)B^T.
```

`C` is joint covariance and `W` is bias-increment spectral density. The 12 driver
coordinates are new accelerometer/gyro noise followed by accelerometer/gyro bias
increments. All derivatives, including signs of endpoint bias effects and the SO(3)
right Jacobian, are specified in the
[endpoint derivation](../decisions/0010-eskf-endpoint-propagation.md).
No empirical covariance multiplier is used.

Corrections have joint Jacobian `[H,0]`. `update_eskf_endpoint` conditions both the
physical estimate and the current sample-noise mean, uses a full Joseph covariance,
then transports every cross term with `diag(J_reset,I6)`. Gates use the physical
15-by-15 marginal, which is also what replay results and NEES expose.
Rejected/stale/disabled/pending observations never condition the noise memory.

Select this path on an existing measurement-only configuration as follows. The four
standard-deviation/density arrays below come from the caller's declared sensor assumptions
in the units above; they are not fitted from evaluation truth.

```python
from dataclasses import replace
import numpy as np
from quadrotor_math.eskf_endpoint import EskfSampledImuNoise
from quadrotor_math.eskf_replay import replay_eskf

sample_covariance = np.diag(np.r_[sigma_accel_B**2, sigma_gyro_B**2])
walk_density = np.diag(np.r_[eta_accel_B**2, eta_gyro_B**2])
endpoint_configuration = replace(
    configuration,
    continuous_noise_covariance=np.zeros((12, 12)),
    sampled_imu_noise=EskfSampledImuNoise(sample_covariance, walk_density),
)
result = replay_eskf(measurements, endpoint_configuration)
```

The nominal run adapter also accepts explicit `sampled_imu_noise`. Stored run-artifact
formats and measurement extraction are unchanged. Direct callers can use
`initialize_eskf_endpoint`, `predict_eskf_endpoint` and `update_eskf_endpoint`; they must
retain the complete `EskfEndpointState` between calls. Reinitializing it from physical
covariance after every step would lose sample correlations. Replay output is a history
for evaluation, not a restart checkpoint or persisted live-service state.

Tests independently check all 21 prior and 12 driver derivative columns, analytic
constant/linear acceleration and yaw cases, smooth-motion convergence, exact repeated
sample-noise variance, batch linear-Gaussian conditioning, seeded nonlinear moments,
invalid inputs, ownership, causal timing and recorded-run compatibility.

## Interfaces, timing and failure contracts

| Boundary | Responsibility and contract |
| --- | --- |
| `EskfNominalState`, `predict_eskf` | Typed state and one deterministic IMU prediction |
| `update_eskf_local_position`, `update_eskf_barometric_altitude` | Same-epoch corrections; owned posterior/gain/residual/reset diagnostics |
| `EskfReplayInput` | Increasing `time_s (n,)`, paired specific force and rate `(n,3)`, typed observation tuple; no truth fields |
| `EskfReplayConfiguration` | Explicit epoch/state/P, gravity, Q, nominal observation models/R and optional gate policy |
| `replay_eskf` | Explicit first-order or endpoint propagation; physical state/covariance histories and an exhaustive observation event ledger |
| `eskf_replay_input_from_run_artifact` | Validated adaptation of full-rate, simultaneous, zero-delay recorded IMU; no truth-state access |
| `evaluate_eskf_replay` | Separate truth-based error/NEES calculation, requiring exact epoch equality |
| `inject_eskf_observation_faults` | Copy measured observations with explicit faults and retain a separate source-label ledger |
| `make_eskf_synthetic_case` | Independent analytic kinematics, randomized measurement generation and known-prior verification fixture |

At $`t_{k+1}`$, default replay predicts with IMU row `k`; endpoint replay uses rows `k,k+1`.
Both then correct with fresh position followed by fresh altitude. No prediction precedes
`t[0]`. The final row is unused by default prediction and closes the endpoint method's last interval.
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

```math
p_i(t)=A_i\sin(w_i t+\alpha_i)+u_i t,
```

```math
v_i(t)=A_i w_i\cos(w_i t+\alpha_i)+u_i,\qquad
a_i(t)=-A_i w_i^2\sin(w_i t+\alpha_i).
```

Here `A` is in metres, `w` in radians/second, $`\alpha`$ in radians and `u` in m/s. The
roll/pitch/yaw angles have an analogous sinusoidal form plus optional constant Euler
rates. For roll $`\phi`$, pitch $`\theta`$ and yaw $`\psi`$, the body-to-world rotation is
$`R_z(\psi)R_y(\theta)R_x(\phi)`$. Its differentiated body angular velocity is

```math
\omega_B=\begin{bmatrix}
\dot\phi-\dot\psi\sin\theta\\
\dot\theta\cos\phi+\dot\psi\sin\phi\cos\theta\\
-\dot\theta\sin\phi+\dot\psi\cos\phi\cos\theta
\end{bmatrix},\qquad f_B=R_{WB}^{T}(a_W-[0,0,g]^T).
```

Central differences independently verify $`\dot{\mathbf p}=\mathbf v`$, $`\dot{\mathbf v}=\mathbf a`$ and
$`\dot R=R[\boldsymbol\omega_B]_\times`$. Noise-free strapdown integration is checked under step refinement;
the existing left-held nominal integration has first-order error for changing inputs.
The stationary control uses zero motion. The translating/yaw control uses NED velocity
`[.5,-.25,.1]` m/s and yaw rate `.2` rad/s. Excited motion uses the predeclared independent
amplitude/frequency/phase distributions in the
[evaluation protocol](../decisions/0009-eskf-completion-validation.md). It is a rigid-body kinematic
estimator test, not evidence of achievable thrust, closed-loop tracking or rotor feasibility.

Measurements add true bias and independent Gaussian white sample noise. Bias increments
have variance `eta² dt`, beginning after the initial epoch. Motion, initial error, initial
true biases, four white-noise streams, two bias walks and fault offsets have separate
PCG64 stream IDs. Stream derivation is explicit, versioned and independent of execution
order. The measured input and nominal configuration are the only arguments passed to
replay; reference states and fault labels are consumed afterward by evaluation.

Default prediction expects continuous process-noise spectral density. For held independent
IMU samples with standard deviation $`\sigma`$, its leading increment variance is matched
by $`Q_c=\mathrm{diag}(\sigma_a^2\Delta t,\sigma_g^2\Delta t,\eta_a^2,\eta_g^2)`$. Observation $`R`$ instead uses
the squared per-observation standard deviation without `dt`. This conversion does not
supply the higher-order position/cross terms omitted by the baseline discretization.

## Fault evidence and its denominators

`EskfObservationFault` identifies a source sensor and row. A dropout removes that
observation, while an offset is additive in the sensor's units and delay adds integer
steps to its delivery epoch. Dropout cannot simultaneously offset or delay the same
source. Reindexed survivors preserve acquisition order; the ledger retains original
identities. Delays beyond the horizon become pending. Unknown/duplicate source IDs,
invalid offsets and arithmetic overflow fail atomically. No IMU samples are altered.

Precision is the fraction of rejected readings that are injected outliers;
recall is the fraction of injected outliers that are rejected. With TP/FP
denoting true/false positives and TN/FN true/false negatives, precision is
`TP/(TP+FP)`, recall is `TP/(TP+FN)` and false-positive rate is
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

For the endpoint protocol, append `--propagation endpoint` to the validation commands
and use new output paths. It uses smoke seeds 20–21, development seeds 5000–5004 and
held-out seeds 50000–50099; the held-out report contains 380 variants including 100
paired original-method comparisons. The same plot CLI accepts either protocol and
adds a propagation-comparison figure for endpoint evidence.

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

Report schema version 2 records each completed variant's actual `time_s` and the actual
epochs used for dropout diagnostics. Before plotting, `validate_validation_report` checks
the frozen protocol/hash, complete unique trial/variant identities and clocks, then
recomputes aggregate results, counters and assessments. Inconsistent evidence is rejected
before output creation. Version 1 remains readable through its verified fixed-clock
convention; the original report and source provenance are preserved. These checks detect
truncation or stale summaries, but do not authenticate the underlying execution.

## Interpreting the evidence

The evidence applies to the documented input, prior, noise and timing contracts.
The code reports execution success separately from numerical performance and
statistical findings. The 90–98% consistency investigation band is not an automatic tuning objective. Repeated
epochs are correlated; the independent-replicate count is the number of seeds at an
epoch, not the number of time samples. Pointwise bands are not simultaneous guarantees.

Stationary and pure-yaw motion do not expose every attitude/bias direction as strongly
as changing translational acceleration and attitude. Bias-convergence claims therefore
apply to the excited evaluation distribution, not every flight or every initial condition.
Default propagation remains first order; endpoint mode has its explicit discrete sample
contract and second-order smooth-motion accuracy. Both retain local Gaussian assumptions. Persistent
observation loss, sustained wrong-model rejection, large attitude errors, hardware faults,
unknown datums and unobservable states remain technical limits. Estimator-only
tests do not establish closed-loop flight performance, live transport, saved
estimator continuation, delayed correction or flight-stack integration.
