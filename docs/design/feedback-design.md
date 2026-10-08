# Estimated-feedback cascade design

This guide explains how the position loop, attitude loop and motors influence
one another near hover. I used that model to choose a small number of explicit
profiles rather than treating each gain as an independent knob. The local
model helps explain damping, but the complete noisy simulation determines
whether a profile meets the flight requirements.

The model is implemented in [cascade_analysis.py](../../src/quadrotor_math/cascade_analysis.py)
and the profiles in [feedback_bandwidth_validation.py](../../experiments/feedback_bandwidth_validation.py).
For the broader interpretation, read [controller tradeoffs](../results/controller-tradeoffs.md).

Select `design_version=2` explicitly for the documented numerical reference.
It does not satisfy every flight requirement: both available profiles have
recorded failures against the 8 cm hover-peak limit. Version 1 remains the API
default for reproducibility.

The profiles in `experiments.feedback_bandwidth_validation` use the same plant,
controller equations and error-state Kalman filter (ESKF). They specify the gains
and initial covariance explicitly. The [feedback protocol](../decisions/0014-estimated-feedback-bandwidth.md)
defines the evaluation conditions and acceptance rules.

## Why the loops must be designed together

The outer position law requests acceleration, but motors and attitude cannot
produce that acceleration instantaneously. Designing a second-order position
loop while ignoring those dynamics can therefore overestimate damping. A small
attitude error during initialization can first push the vehicle laterally, then
excite an oscillatory position recovery even after the estimate improves.

Near level matched hover with zero yaw, define one horizontal position $`p`$ [m],
velocity $`v`$ [m/s], signed tilt $`\eta`$ [rad], signed tilt rate $`r`$ [rad/s] and
actual angular acceleration $`a`$ [rad/s²]. With pitch $`\theta`$ and roll $`\varphi`$, for north $`\eta=-\theta`$; for east
$`\eta=\varphi`$. Both definitions give $`\dot v=g\eta`$, consistent with NED/FRD and
negative-body-z thrust. $`g`$ is gravity magnitude [m/s²], not specific force.

Linearizing differential rotor thrust about nonzero hover speed preserves the
rotor time constant $`\tau`$: thrust is quadratic in speed, but its first-order
perturbation is proportional to speed perturbation. With matched inertia and
inactive limits, the inner controller produces commanded angular acceleration $`u`$:

```math
\begin{aligned}
\dot p&=v,&\dot v&=g\eta,\\
\dot\eta&=r,&\dot r&=a,\\
\tau\dot a&=u-a,&u&=k_r(k_a(\eta_d-\eta)-r),\\
\eta_d&=\frac{-k_pp-k_vv}{g}.
\end{aligned}
```

The last relation uses a zero horizontal reference.

$`k_p`$ has units s^-2; $`k_v`$, $`k_a`$ and $`k_r`$ have units s^-1. Eliminating the
intermediate variables gives the fifth-order characteristic polynomial

```math
D(s)=\tau s^5+s^4+k_rs^3+k_rk_as^2+k_rk_ak_vs+k_rk_ak_p.
```

For the original horizontal gains, the slow oscillatory pair has damping ratio
about .574, compared with .9 in the ideal outer-loop calculation. This identifies
a concrete weakness in the original design approximation; it is not evidence that
the underlying controller implementation has the wrong sign or frame.

## Coefficient-matched profile and sampled implementation

Version 2 places all five continuous-model poles at a decay rate of 8 s^-1.
A pole describes a mode of the local response: a negative real pole decays
without oscillation. The target polynomial is

```math
\begin{aligned}
D_2(s)&=0.025(s+8)^5\\
 &=0.025s^5+s^4+16s^3+128s^2+512s+819.2,\\
k_p&=6.4\ \mathrm{s}^{-2},\quad k_v=4\ \mathrm{s}^{-1},\quad k_a=8\ \mathrm{s}^{-1},\quad k_r=16\ \mathrm{s}^{-1}.
\end{aligned}
```

The sum of the desired pole decay rates is 40 s^-1, exactly $`1/\tau`$, making
the s^4 coefficient compatible with the fixed physical motor lag. All remaining
coefficients map directly to the four gains. Only the horizontal position gains
and roll/pitch attitude/rate gains change. Vertical and yaw settings and all
actuator, acceleration, tilt and geofence limits remain unchanged.

For five positive real decay rates $`a_i`$ summing to $`1/\tau=40`$, coefficient
matching gives $`k_p=\left(\sum_{i<j}\frac1{a_i a_j}\right)^{-1}`$. Cauchy-Schwarz bounds the denominator
below by $`100/\sum_{i<j}a_i a_j`$, while the fixed sum bounds the pair-product sum
above by 640. Hence $`k_p\leq6.4`$, attained by five equal rates of 8 s^-1. This is
maximum position stiffness within the stated all-real continuous pole family,
not a global optimum over arbitrary controllers. The sampled implementation
and nonlinear stochastic performance must still be checked independently.

Version 1 remains available with target
$`.025(s+3)^2(s+6)(s^2+28s+392)`$ and gains `3528/1003`, `3192/1003`,
`6018/773`, `773/40`. Its held-out result is 29/30 passing cases: the failing
case has not settled sufficiently by the 5-s hold entry. Version 2 differs only
in these four gains; its moment-matched prior is identical. Each profile has
a separate validation batch.

`cascade_analysis.HorizontalCascade` constructs both the continuous generator
and a sampled-data transition. Its state is $`[p,v,\eta,r,a]^T`$. The latter is sampled
immediately before an outer update. A temporary sixth coordinate holds $`\eta_d`$:
reset it at the outer tick, perform two 10-ms inner-loop updates with held commands,
then retain the five physical coordinates at the next 20-ms outer tick. This
models the actual 50/100-Hz control-clock relationship, not a fictitious continuous
controller. `hover_axis_zero_order_hold` computes the exact local held-input map
to floating-point precision using a bounded augmented-matrix exponential.

All five discrete poles lie inside the unit circle for this profile. Their
equivalent continuous values $`\log(z)/.02`$ differ from the design poles because
of sampling: approximately -3.927, -4.685 +/- 3.781j and -11.590 +/- 9.660j.
The minimum damping ratio is .7682 and slowest decay rate 3.9267 s^-1, compared
with .6012 and 2.4599 s^-1 for version 1.
These establish local stability of the specified linear controller/plant model.
They do **not** establish nonlinear closed-loop stability with an ESKF, saturation,
large attitudes or arbitrary mismatch.

Let each $`e_*`$ denote estimate minus truth. In the continuous local model,

```math
D\!\left(\frac{d}{dt}\right)p=-k_rk_a(k_pe_p+k_ve_v+ge_\eta)-gk_re_r.
```

Thus the static sensitivity to tilt error is $`-g/k_p`$; the sensitivity to velocity
error is $`-k_v/k_p`$. The joint design reduces these magnitudes from 9.81 to about
1.533, and from 1.8 to .625, respectively. Position-estimation error still
has unit static sensitivity. Faster feedback also passes more measurement noise
to commands, which is why the evidence records moment effort and limiting along
with tracking accuracy. No integral state or zero-offset robustness claim is added.

## Moment-matched initialization

The original prior was conservative relative to the experiment's known initial
error population. The population-matched prior uses the same mean: zero position,
velocity and biases, identity body-to-world attitude. For each independent
zero-mean uniform error on `[-a,a]`,

```math
\mathbb E[x]=0,\qquad \mathrm{Var}(x)=\int_{-a}^{a}\frac{x^2}{2a}\,dx=\frac{a^2}{3}.
```

The 15 component half-widths are .02 m (position), .02 m/s (velocity),
2 degrees (right-local rotation vector), .02 m/s² (accelerometer bias) and
.003 rad/s (gyro bias), each repeated for three axes. The covariance is their
squared half-widths divided by three on the diagonal. At the identity prior,
the initialized rotation vector is exactly the right-local attitude error.

This covariance is a fixed population assumption; it is not computed from a
trial's realized truth state, bias, reference or future measurement. The true
initial conditions and all random draws are unchanged. Process, per-sample sensor
noise, bias walks, innovation thresholds and observation schedules are unchanged.
The noiseless case still has zero prior covariance. Moment matching does not make
a uniform population Gaussian or transfer the estimator-only NIS/NEES calibration
claim to this feedback distribution.

Yaw remains weakly observable during near-stationary hover with these sensors.
No magnetometer, heading observation, hidden truth alignment or yaw reset is
introduced. Tests and finite-duration campaigns assess the stated operating
regime; they cannot establish indefinite heading accuracy without excitation.

## Interfaces and verification

The runtime remains `simulate_estimated_mission` with explicitly supplied
parameters. The experiment's `make_configuration(job, design_version=2)` creates
an independent configuration for a declared trial.
`designed_horizontal_cascade(2)` returns the
local design assumptions; no controller silently selects gains based on seed,
mission phase, estimated covariance or observed performance.

The local analysis validates positive finite scalar units, integral clock ratio
and a bounded exponential interval. Returned matrices own read-only float64
storage. Unsupported intervals and nonfinite arithmetic fail explicitly. This is
an analysis utility, not a general-purpose matrix-exponential implementation.

Verification includes independent characteristic coefficients, exact motor decay,
constant-input polynomial trajectories, the semigroup identity, explicit
multirate execution, a continuous-time limit, and central finite differences of
the nonlinear rotor/controller/RK4 execution for both horizontal frame signs.
The population-matched initial covariance is checked by independent quadrature and
seed-independence tests. A bounded CI startup regression preserves the original
takeoff and 5-s hover start; the separate campaign scores the complete 60-s hold.

Every trial pairs estimated feedback with true-state feedback using **the same
gains**. This avoids attributing a changed controller to estimator degradation.
Each profile is a separate reproducible reference. Stored evidence is checked
by measurement replay, explicit configuration validation and reconstruction
of every inner command and limit flag in the paired true-state run.
Corruption tests demonstrate that changed commands, covariances, metrics, profiles,
trial identities or archive bytes cannot be silently accepted.

Design version 1 remains the default so existing calls and all three frozen
protocol hashes are preserved. Select version 2 explicitly. The experiment
uses held-out hover seeds 93000..93019 and square seeds 94000..94009.
Its thirteen development jobs include four hover seeds used for diagnostics;
those cases are not independent validation. The fixed partition contains five
cases, with the same acceptance criteria as the baseline feedback experiment.
Report validation can use independent worker processes;
every full-history check must finish before publication. Tests prove exact
serial/parallel saved bytes for identical report metadata and failure propagation
from later workers. CLI provenance separately records the chosen worker count.

## Qualification status

Both implemented profiles fail the complete acceptance criteria: version 1
passes 29/30 held-out cases, and version 2 passes 28/30. The failures concern the unchanged .08 m hover
peak criterion; numerical, replay and independent physical audits pass.
Complex-pole and bounded-integral development probes do not close the gap
and are not maintained controller options. The 95000/96000 seed families belong to the
[final geometric comparison](../results/final-geometric.md); rerunning them
is reproduction of known cases, not new independent validation. Local poles
or individual plots do not establish that the complete system meets its requirements.

## Reproduction

Exact protocol bytes and numerical replay also depend on the floating-point
backend. The same NumPy build with another OpenBLAS kernel can change a derived
initial-quaternion component by one float64 unit and therefore change the raw
protocol digest. Such a byte-level difference can occur without a change to
the seeded population or feedback equations. NumPy's
[compatibility policy](https://numpy.org/doc/stable/reference/random/compatibility.html)
likewise limits exact reproducibility to tightly matched execution conditions.

The source-definition regression tests use an authenticated initial-attitude
fixture and permit at most four float64 units only in those derived quaternion
components before comparing the original golden digest. A five-unit change and
changes to other fields remain detectable. This is a test-only mathematical-input
comparison: production protocol hashes, archive digests and full-history replay
validation remain strict. Match the original numerical backend to reload an
exact historical replay; a different backend is a distinct numerical realization
and must not be described as byte-identical. No test tolerance changes a flight
performance criterion or converts a failed campaign into a pass.

Use Python 3.12, uv 0.12.3 and the existing lockfile. Output directories must be new.
Generated full histories stay outside Git. Freeze source and protocol before
opening the held-out partition; a previously observed seed set is no longer fresh.

```bash
uv sync --locked
PYTEST_ADDOPTS='-W error' make check
EVIDENCE=/tmp/quadrotor-feedback-design
uv run python -m experiments.feedback_bandwidth_validation --design-version 2 --partition development --workers 3 --output "$EVIDENCE/development"
uv run python -m experiments.feedback_bandwidth_validation --design-version 2 --partition fixed --workers 3 --output "$EVIDENCE/fixed"
# Freeze source/protocol and development/fixed evidence before this command.
uv run python -m experiments.feedback_bandwidth_validation --design-version 2 --partition validation --workers 3 --output "$EVIDENCE/validation"
uv run python -m experiments.plot_feedback_bandwidth --workers 3 --input "$EVIDENCE/fixed" --output "$EVIDENCE/fixed-plots"
uv run python -m experiments.plot_feedback_bandwidth --workers 3 --input "$EVIDENCE/validation" --output "$EVIDENCE/validation-plots"
```

Read the [controller tradeoffs](../results/controller-tradeoffs.md) alongside
the measured results. This profile does not establish hardware performance,
real-time execution, delayed fusion, automatic alignment, ground contact or
fault accommodation.
