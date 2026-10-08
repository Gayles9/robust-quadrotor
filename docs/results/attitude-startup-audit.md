# Attitude-estimation startup audit

This study explains why the estimated tilt can initially become less accurate
after a valid position measurement. The error-state Kalman filter (ESKF) combines
measurements with a motion prediction; it can reduce its expected uncertainty
while one particular estimate moves farther from the true value.

The results do not demonstrate an estimator defect or justify a new correction
law under the freely flying initialization assumptions. The
[verification record (ZIP)](../../evidence/development-records.zip) contains the
checks, and the [study protocol](../decisions/attitude-startup-audit.md)
defines the scope.

## What changes at the first position observation

The first altitude measurement arrives at 0.04 s; the first position measurement
arrives at 0.2 s. At a shared epoch the filter propagates the paired IMU samples,
then corrects position, then altitude. No future sample is used.

The original thrust-axis error grows from 2.294076 to 3.161399 degrees by 0.2 s.
A telescoping accounting of the **scalar angle**, evaluated before and after
each operation, is:

| Contribution through 0.2 s | Angle change [degrees] |
| --- | ---: |
| IMU prediction and true motion | +0.024801 |
| First position correction | +0.844265 |
| All altitude corrections | -0.001744 |

This is exact sequential accounting, not additive counterfactual causation.
The position innovation is approximately `[-2.495, -3.161, -0.734] cm`.
The sensor-noise part is `[-1.828, -1.974, -0.442] cm`, with declared independent
2 cm per-axis noise. The remaining part is the predicted position error.
The normalized innovation squared (NIS) compares a measurement residual with
its predicted uncertainty. Here NIS is 1.215812, well below the rejection
threshold of 11.345, so this observation is not a justified outlier.

The gain links position to attitude through accumulated inertial uncertainty.
It injects a right-local roll/pitch correction of `[-0.675374, +0.534306]`
degrees and a velocity correction of `[-0.021460, -0.027173, -0.001293] m/s`.
For this realization the attitude correction points away from the true attitude.
Yet roll/pitch standard deviations correctly fall from about 3.000548 to
2.888449 degrees. A Bayesian covariance reduction does not promise that every
individual true error decreases. Rejecting or damping this valid observation
after seeing truth would be post-hoc tuning.

By 1 s the original thrust-axis error is 0.499222 degrees, by 5 s it is
0.038467 degrees, and at the 5.395 s hover peak it is 0.055712 degrees.
The true vehicle has already accumulated horizontal motion. The oracle filter
has nearly the same startup growth (3.156732 degrees at 0.2 s), while its inner
controller sees truth. That distinction supports the [attitude-feedback diagnosis](early-flight-diagnosis.md); it still does not isolate startup from all later feedback.

## What the existing measurements can identify

For the calm rotor-only plant, an accelerometer measures specific force,
approximately `f_B = [0, 0, -T/m]`, not a stationary gravity vector. Changing
attitude changes world acceleration; it does not independently rotate gravity
into the free-flight accelerometer measurement. Applying a stationary-gravity
pseudo-measurement here would add an unsupported constraint and reuse IMU data.

Position over time does carry tilt information through acceleration. Around
constant level hover the right-local error equations include

```text
delta_v_N_dot = -g * delta_theta_y - delta_b_a_x
delta_v_E_dot = +g * delta_theta_x - delta_b_a_y
delta_theta_dot = -delta_b_g
```

The 15-state constant-coefficient model with position output has rank 11.
Two tilt/accelerometer-bias combinations, yaw and yaw gyro bias lie in its
four-dimensional unobservable subspace. Altitude adds a sign-reversed down
position row, not a new attitude direction. Actual time-varying flight can add
information: this local calculation does **not** prove global unobservability.

Even a naive second difference of three independent 2 cm position measurements
spaced 0.2 s apart has acceleration-noise standard deviation
`sqrt(6)*0.02/0.2^2 = 1.224745 m/s^2`, equivalent to 7.153182 degrees at small
angles. This is illustrative, not a lower bound on the ESKF or a proposed
estimator. The actual filter uses its prior and all correlated information.

The legacy initial covariance is conservative: its variance is 6.75 times the
declared uniform truth population's moment for position, velocity, attitude and
accelerometer bias, and 8.333333 times for gyro bias. This is an existing,
documented assumption, not a newly found implementation bug. Earlier
[bandwidth designs](../design/feedback-design.md#moment-matched-initialization) already studied a
moment-matched prior alongside different gains. This audit changes neither
assumptions nor gains and does not claim population calibration from one seed.

## Implications for initialization

The endpoint ESKF and original cascade remain unchanged. The evidence does not
justify treating the free-flight accelerometer as a gravity measurement,
skipping a valid position update, or changing process/measurement uncertainty
(`Q`/`R`) after inspecting the true error. The separate mass-load transient
remains unresolved.

[Stationary pre-arm alignment](../design/prearm-alignment.md) provides a different
source of information: a mechanically supported, nonaccelerating interval before
flight. Its contract specifies sample timing, motion rejection, bias uncertainty,
yaw ambiguity and failure behavior. Quiet IMU readings alone cannot prove the
absence of constant world acceleration; stationary support is an explicit
external operating assumption.

The [supported-start flight study](supported-start-flight.md) evaluates this
added stage while retaining startup in the flight scores and preserving the
original thresholds. Its results apply to the supported initial condition;
they do not qualify the unchanged freely flying initialization case.

## Reproduce the audit

Extract the authenticated original campaign and recovered early-flight evidence.
From the repository's pinned environment:

```bash
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.attitude_startup_audit \
  --baseline ORIGINAL/campaign --oracle EARLY_FLIGHT/oracle --output NEW_AUDIT
```

The command never simulates a flight. It authenticates inputs, independently
rescores all original executions, reconstructs both full endpoint histories,
compares joint conditioning/reset with independent algebra, and writes full
event records plus error/covariance signal arrays. Truth appears only in
evaluation, never in the reconstructed prediction or correction.
