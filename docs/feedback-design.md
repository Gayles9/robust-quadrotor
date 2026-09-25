# Estimated-feedback cascade design

Current reference decision (2026-09-25): retain explicit `design_version=2` as
the limited numerical reference after the [same-case comparison](progress/2026-09-25-repository-audit.md).
The historical 8 cm qualification failures below remain unchanged.

This is the bounded numerical profile in
`experiments.feedback_bandwidth_validation`, developed to address the noisy-hover
qualification in [ADR 0013](decisions/0013-estimated-state-mission-feedback.md).
[ADR 0014](decisions/0014-estimated-feedback-bandwidth.md) records its scope,
development decisions and frozen acceptance rules. The original experiment is
preserved, including its failed result. The plant, controller algorithms and ESKF
equations are unchanged; the new profile has explicit gains and initial covariance.

## Why the loops must be designed together

The outer position law requests acceleration, but motors and attitude cannot
produce that acceleration instantaneously. Designing a second-order position
loop while ignoring those dynamics can therefore overestimate damping. A small
attitude error during initialization can first push the vehicle laterally, then
excite an oscillatory position recovery even after the estimate improves.

Near level matched hover with zero yaw, define one horizontal position `p` [m],
velocity `v` [m/s], signed tilt `eta` [rad], signed tilt rate `r` [rad/s] and
actual angular acceleration `a` [rad/s²]. For north `eta=-pitch`; for east
`eta=roll`. Both definitions give `v_dot=g*eta`, consistent with NED/FRD and
negative-body-z thrust. `g` is gravity magnitude [m/s²], not specific force.

Linearizing differential rotor thrust about nonzero hover speed preserves the
rotor time constant `tau`: thrust is quadratic in speed, but its first-order
perturbation is proportional to speed perturbation. With matched inertia and
inactive limits, the inner controller produces commanded angular acceleration `u`:

```text
p_dot = v             v_dot = g*eta
eta_dot = r           r_dot = a
tau*a_dot = u-a
u = kr*(ka*(eta_d-eta)-r)
eta_d = (-kp*p-kv*v)/g          (zero horizontal reference)
```

`kp` has units s^-2; `kv`, `ka` and `kr` have units s^-1. Eliminating the
intermediate variables gives the fifth-order characteristic polynomial

```text
D(s) = tau*s^5 + s^4 + kr*s^3 + kr*ka*s^2 + kr*ka*kv*s + kr*ka*kp.
```

For the original horizontal gains, the slow oscillatory pair has damping ratio
about .574, compared with .9 in the ideal outer-loop calculation. This identifies
a concrete weakness in the original design approximation; it is not evidence that
the underlying controller implementation has the wrong sign or frame.

## Coefficient-matched profile and sampled implementation

Version 2 uses the all-real target

```text
D2(s) = .025*(s+8)^5
      = .025*s^5+s^4+16*s^3+128*s^2+512*s+819.2.
kp = 6.4 s^-2, kv = 4 s^-1, ka = 8 s^-1, kr = 16 s^-1.
```

The sum of the desired pole decay rates is 40 s^-1, exactly `1/tau`, making
the s^4 coefficient compatible with the fixed physical motor lag. All remaining
coefficients map directly to the four gains. Only the horizontal position gains
and roll/pitch attitude/rate gains change. Vertical and yaw settings and all
actuator, acceleration, tilt and geofence limits remain unchanged.

For five positive real decay rates `ai` summing to `1/tau=40`, coefficient
matching gives `kp=1/sum(i<j,1/(ai*aj))`. Cauchy-Schwarz bounds the denominator
below by `100/sum(i<j,ai*aj)`, while the fixed sum bounds the pair-product sum
above by 640. Hence `kp<=6.4`, attained by five equal rates of 8 s^-1. This is
maximum position stiffness within the stated all-real continuous pole family,
not a global optimum over arbitrary controllers. The sampled implementation
and nonlinear stochastic performance must still be checked independently.

Version 1 remains available with target
`.025*(s+3)^2*(s+6)*(s^2+28*s+392)` and gains `3528/1003`, `3192/1003`,
`6018/773`, `773/40`. Its 29/30 held-out result is an explicitly retained failed
qualification. The observed miss is startup settling at the unchanged 5-s hold
entry. Version 2 changes only these four gains relative to version 1; its
moment-matched prior is identical. The earlier failed batch is not reused as
fresh validation for the revised design.

`cascade_analysis.HorizontalCascade` constructs both the continuous generator
and a sampled-data transition. Its state is `[p,v,eta,r,a]`. The latter is sampled
immediately before an outer update. A temporary sixth coordinate holds `eta_d`:
reset it at the outer tick, perform two 10-ms inner-loop updates with held commands,
then retain the five physical coordinates at the next 20-ms outer tick. This
models the actual 50/100-Hz control-clock relationship, not a fictitious continuous
controller. `hover_axis_zero_order_hold` computes the exact local held-input map
to floating-point precision using a bounded augmented-matrix exponential.

All five discrete poles lie inside the unit circle for this profile. Their
equivalent continuous values `log(z)/.02` differ from the design poles because
of sampling: approximately -3.927, -4.685 +/- 3.781j and -11.590 +/- 9.660j.
The minimum damping ratio is .7682 and slowest decay rate 3.9267 s^-1, compared
with .6012 and 2.4599 s^-1 for version 1.
These establish local stability of the specified linear controller/plant model.
They do **not** establish nonlinear closed-loop stability with an ESKF, saturation,
large attitudes or arbitrary mismatch.

Let each `e_*` denote estimate minus truth. In the continuous local model,

```text
D(d/dt)*p = -kr*ka*(kp*e_p + kv*e_v + g*e_eta) - g*kr*e_r.
```

Thus the static sensitivity to tilt error is `-g/kp`; the sensitivity to velocity
error is `-kv/kp`. The joint design reduces these magnitudes from 9.81 to about
1.533, and from 1.8 to .625, respectively. Position-estimation error still
has unit static sensitivity. Faster feedback also passes more measurement noise
to commands, which is why the evidence records moment effort and limiting along
with tracking accuracy. No integral state or zero-offset robustness claim is added.

## Moment-matched initialization

The original prior was conservative relative to the experiment's known initial
error population. The new prior retains the identical mean: zero position,
velocity and biases, identity body-to-world attitude. For each independent
zero-mean uniform error on `[-a,a]`,

```text
E[x] = 0,       Var[x] = integral(-a..a, x^2/(2*a) dx) = a^2/3.
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
The new profile's initial covariance is checked by independent quadrature and
seed-independence tests. A bounded CI startup regression preserves the original
takeoff and 5-s hover start; the separate campaign scores the complete 60-s hold.

Every trial pairs estimated feedback with true-state feedback using **the same
new gains**. This avoids attributing a changed controller to estimator degradation.
The old profile remains a separate reproducible reference. The existing archive
and online/offline replay audits are reused with explicit configuration; the
paired true-state audit now also reconstructs every inner command and limit flag.
Corruption tests demonstrate that changed commands, covariances, metrics, profiles,
trial identities or archive bytes cannot be silently accepted.

Design version 1 remains the default so existing calls and all three frozen
protocol hashes are preserved. Select version 2 explicitly. The experiment
uses new held-out hover seeds 93000..93019 and square seeds 94000..94009.
Its thirteen development jobs explicitly include four already observed hover
seeds as diagnostics. The five fixed identities and every acceptance condition
are inherited unchanged. Report validation can use independent worker processes;
every full-history check must finish before publication. Tests prove exact
serial/parallel saved bytes for identical report metadata and failure propagation
from later workers. CLI provenance separately records the chosen worker count.

## Qualification status

Both implemented profiles remain **unqualified**: version 1 passes 29/30 fresh
cases, and version 2 passes 28/30. The failures concern the unchanged .08 m hover
peak criterion; numerical, replay and independent physical audits pass.
Subsequent complex-pole and bounded-integral development probes do not close the
gap and are not maintained controller options. The reserved 95000/96000 seeds
have not been evaluated. See the complete record before interpreting local poles
or individual passing plots as system qualification.

## Reproduction

Exact protocol bytes and numerical replay also depend on the floating-point
backend. The same NumPy build with another OpenBLAS kernel can change a derived
initial-quaternion component by one float64 unit and therefore change the raw
protocol digest. This was reproduced locally for the final CI mismatch; it is
not a change to the seeded population or feedback equations. NumPy's
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

The [verification record](progress/2026-09-24-feedback-design.md) distinguishes
measured results from the local model, retains development misses and documents
the exact tested source and commands. No hardware, real-time, delayed-fusion,
automatic alignment, ground-contact or fault-accommodation capability is implied.
