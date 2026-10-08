# Stationary pre-arm alignment

Pre-arm alignment uses a short, externally supported period to estimate the
vehicle's inclination and gyro bias before flight. Mechanical support makes the
accelerometer's gravity information interpretable. Quiet measurements alone do
not prove the vehicle is stationary.

This page gives the operating contract and the first-order feasibility criteria.
The implemented uncertainty treatment is described in
[nonlinear pre-arm uncertainty](nonlinear-prearm-uncertainty.md), and the session
API in [the alignment component](standalone-prearm-alignment.md).

The first-order feasibility study failed its uncertainty criterion: maximum
normalized variance was 1.112164 against a 1.10 limit. The
[nonlinear uncertainty results](../design/prearm-nonlinear-uncertainty.md#integration-accuracy-and-empirical-calibration)
explain the implemented correction and its separate evaluation.

## What the support assertion means

An external source asserts mechanical support, motors off, constant attitude,
zero world acceleration and zero angular velocity throughout acquisition and
through release. A false assertion invalidates the result. True attitude, true
bias, commanded attitude and flight scores are not alignment inputs.

The fixed sensor profile is:

| Quantity | Value |
| --- | --- |
| Paired IMU sampling | 400 Hz |
| Gravity | 9.81 m/s² |
| Accelerometer sample standard deviation | 0.04 m/s² |
| Gyro sample standard deviation | 0.002 rad/s |
| Accelerometer bias-walk deviation | 0.0002 m/s²/√s |
| Gyro bias-walk deviation | 0.00002 rad/s/√s |
| Initial accelerometer-bias standard deviation | 0.03 m/s² |
| Prior heading standard deviation | 3 degrees |

The estimate uses the accelerometer mean minus its declared prior bias to obtain
gravity direction, retains prior heading, and uses the gyro mean to estimate
gyro bias. Gravity constrains inclination but not heading. At one orientation,
inclination and transverse accelerometer bias cannot be estimated independently.
The method therefore carries accelerometer-bias uncertainty, bias walks and the
induced attitude/bias cross-covariances instead of fitting a second bias estimate
from the same gravity direction.

## Acquisition and rejection budgets

Duration follows analytic noise budgets: accelerometer white noise contributes
at most 0.02 degree per tangent axis at one standard deviation, and terminal
gyro-bias uncertainty is at most 0.03 degree/s per axis at three standard
deviations, including bias walk. The minimum duration is rounded upward to a
0.5 s acquisition block. The selected profile uses one block: 201 samples over
0.5 s.

The local 99% world-thrust-axis ellipse radius must be at most 0.75 degree.
Heading uncertainty and its coupling at tilted attitudes are retained. This is
an initialization budget, not proof of the 8 cm flight requirement. Invalid
clocks, data or support, ill-conditioned gravity direction, inclination above
15 degrees, or failed uncertainty budgets cause rejection.

Three fixed upper-tail compatibility tests assess mean gyro, gravity magnitude
and demeaned IMU variation. Each has alpha=0.001 and includes temporal correlation
from bias walks. Their union false-rejection bound depends on the stated
Gaussian model; it is not a guarantee for arbitrary disturbances. Thresholds use
`c(d, alpha)=d+2*sqrt(d*ln(1/alpha))+2*ln(1/alpha)`.
Steady acceleration and rotation/bias ambiguities can remain undetectable, so a
gate cannot substitute for the external support assertion.

Alignment samples are consumed once. While support remains asserted, only
bias-walk uncertainty propagates. The first flight endpoint uses a fresh,
disjoint paired IMU sample. Attitude/bias cross-covariances survive release;
position and velocity initialization require a separate, explicitly independent
prior. Clock tolerance is 1e-12 s on the 400 Hz grid, with handoff one fresh
sample interval after the alignment window.

## Feasibility populations and checks

Independent algebra checks inclination signs and Jacobians, heading ambiguity,
random-walk mean/terminal covariances, and correlated handoff. Finite differences,
latent-increment sums and Gaussian calculations provide separate references.

The deterministic feasibility populations use PCG64 with
`SeedSequence([0x50524541, 1, partition, trial])`. Partitions 0 and 1 each contain
5,000 trials:

- The nominal population uses independent rotation-vector components uniform in
  ±2 degrees, accelerometer bias in ±0.02 m/s² and gyro bias in ±0.003 rad/s.
- The Gaussian population has fixed roll +1 degree and pitch -1 degree, heading
  N(0, 3 degrees), accelerometer bias N(0, 0.03 m/s²) and gyro bias
  N(0, 0.005 rad/s), all independent.

Every sample noise and discrete bias increment is drawn explicitly. Prior means
and retained heading are zero; simulated truth is used only for evaluation.
These supported intervals are separate from flight replays.

Each Gaussian trial combines nonlinear right-local attitude error and terminal
bias error in nine dimensions. Errors are whitened by the reported covariance's
Cholesky factor at the estimated orientation. The largest eigenvalue of the
centered empirical covariance must be at most 1.10. All trials, including
rejections, enter this check. A separate linearized-error calculation using the
same latent draws diagnoses approximation error but cannot replace the nonlinear
acceptance test. Nominal gate rejection must be at most 1%, and the covariance
must be positive semidefinite.

The stress fixtures cover missing support, malformed timing, a 1 m/s² single
accelerometer impulse, 0.2 m/s² vibration at 40 Hz, a roll ramp from 0 to
3 degrees, and 0.05 rad/s mean gyro rate. Each adds its signal to the same
level, zero-bias white-noise record from partition 2, trial 0, without bias walk.
A 1-degree pitch-equivalent constant acceleration is retained as an
IMU-indistinguishable false-stationarity example.

The criteria use fixed duration, priors, seeds and noise, without covariance
inflation or post-result retuning. These finite studies establish only a bounded
alignment design; hardware support and complete-flight performance require
separate evidence.
