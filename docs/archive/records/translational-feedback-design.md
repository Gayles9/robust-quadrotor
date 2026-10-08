# Translational uncertainty and motor-feedback experiment — 2026-09-24

Audited commit: `c96e9f2aec9192d9059fc2aa6a8d764fa2463d10`.
The preceding channel diagnostic passes all 19 of its tests again (0.38 s).
This step preserves the two frozen feedback profiles and all production source.
It evaluates one fixed candidate after examining existing uncertainty records.
No reserved validation seed is opened and no failed result is promoted.

## Uncertainty audit

The 20 already observed version-2 hover trials provide 1,000 position innovations
from 0.2 through 10 s. Their mean pre-gate normalized innovation squared (NIS) is
3.017608; 12 of 1,000 exceed the unchanged 11.345 gate. For residual `r` in metres
and its covariance `S` in m², `NIS = r.T solve(S, r)`. A zero-mean residual with
covariance `S` has expected NIS `trace(S^-1 S)=3`. This expectation follows from
the second moment; the chi-square gate interpretation additionally needs the
assumed Gaussian model.

For the six additive NED errors `e=[p_hat-p_true, v_hat-v_true]`, using the
corresponding 6-by-6 covariance marginal `P_pv`, define
`NEES_pv = e.T solve(P_pv, e)`. This quadratic form is dimensionless despite
the different position and velocity units. Its mean across the 20 trials and
the 1..10-s grid is 6.352061, against a calibrated zero-mean reference of 6.
Time samples are strongly correlated. These descriptive averages are not fresh
validation, confidence intervals, or a proof of calibration. In particular, the
high errors in the two selected failed trials do not alone establish a filter
defect. The audit gives no clear basis for changing sensor noise, prior covariance,
process noise or rejection thresholds to make those two trials pass.

Input report SHA-256:
`86d2c651b31eaf7a4afada0c0987a16008d05c6dd2be7ce10f13fbec51cadf90`.
The data archive and first covariance chunk for each trial were checked against
their recorded SHA-256 before decoding. The saved uncertainty audit contains
the individual innovations, selected-time NEES values and input identities.

## Why straightforward smoothing was not promoted

A first-order digital smoothing law for the requested horizontal tilt is
`eta_d[k] = alpha*eta_d[k-1] + (1-alpha)*eta_PD[k]`, at the existing 50-Hz outer
clock. The six-coordinate local transition retains the previous held tilt,
updates that row at the outer tick, and executes two unchanged 100-Hz inner
holds. With the version-2 gains, the minimum equivalent-pole damping ratio falls
from 0.7682 without smoothing to 0.5463 for `alpha=.5` and 0.0686 for `alpha=.9`.
At `.95` the local sampled model is unstable. This adds a consequential lag;
there is no basis here for presenting generic command smoothing as a free
noise reduction. No smoothing option is added to production.

## One fixed motor-response design

The experiment `experiments.feedback_motor_damping_probe` reconstructs nominal
rotor speed from **previous commanded** speeds with the existing first-order
motor model. Its initial rotor prior is explicit nominal hover,
`sqrt(m_nom*g_nom/(4*k_f_nom))` for each rotor. It never reads true rotor speed.
This prior matches the declared experiment initialization; it is not a general
claim that actual motor speed is known on hardware.

Using nominal inertia `J` and nominal rotor moment `tau_rotor` in FRD coordinates,
the candidate computes a causal modeled angular acceleration

```text
a_hat_B = solve(J, tau_rotor - omega_hat_B cross (J omega_hat_B)),
tau_requested = tau_original - J diag(.5,.5,0) a_hat_B.
```

Moments are in N m, angular acceleration in rad/s² and inertia in kg m².
The original rate, moment and allocation bounds still act on the modified
request. Neither the ESKF nor any sensor model is changed.

The local zero-yaw horizontal state is `[p,v,eta,r,a]`, where `eta=-pitch` for
north and `eta=roll` for east. Here `a` is angular acceleration, not translational
acceleration. With matched nominal motor prediction and inactive limits,

```text
p_dot=v,  v_dot=g*eta,  eta_dot=r,  r_dot=a,
tau*a_dot = kr*(ka*(eta_d-eta)-r) - (1+gamma)*a,
eta_d = (-kp*p-kv*v)/g.
D(s) = tau*s^5 + (1+gamma)*s^4 + kr*s^3
       + kr*ka*s^2 + kr*ka*kv*s + kr*ka*kp.
```

Unlike the existing four-gain cascade, motor-response feedback changes the
fourth-order coefficient. The one tested design fixes `gamma=.5`, `tau=.025 s`
and matches `.025*(s+12)^5`: `kp=14.4 s^-2`, `kv=6 s^-1`, `ka=12 s^-1`,
`kr=36 s^-1`. Vertical and yaw gains are unchanged. This is coefficient matching,
not an optimum inferred from the failed seeds. It lowers the local static
velocity-error sensitivity `kv/kp` from .625 s to .4167 s; position-error static
sensitivity remains one.

The equivalent sampled poles are approximately -5.620, -6.663 +/- 6.127j and
-19.607 +/- 14.502j, all in s^-1, with minimum damping .7361. These are poles of
the matched local plant/controller, not of the nonlinear estimator/controller.
The nominal rotor-observer error is assumed zero for this five-state map. Sensor
errors, mismatch and actuator limiting still require nonlinear evaluation.

## Result and stop decision

Only six already observed hover seeds were evaluated, using the original mission
prefix from 0 through 10 s. Screening requires every 5..10-s peak below .08 m and
no consecutive actuator-limiting interval above .5 s. This is only a prerequisite
to further development; it cannot qualify the full 5..65-s hover window.

| Seed | Peak position error (cm) | Longest actuator limiting (s) |
| --- | ---: | ---: |
| 30 | 5.09203 | .03 |
| 91001 | 5.63725 | .03 |
| 93003 | 7.76191 | .02 |
| 93012 | **8.38576** | .13 |
| 8200 | 6.18746 | .01 |
| 8201 | 3.63793 | .02 |

The two version-2 misses improve from 9.14027/8.86862 cm to 7.76191/8.38576 cm,
but the candidate fails its screen (5/6). Requested pitch moment reaches 1.7098
N m in seed 93012 before the unchanged .8-N-m clipping. A locally stable pole
placement therefore does not establish the required nonlinear tracking result.
The predeclared stop rule is applied: no follow-on gain sweep, no fresh validation
campaign and no production promotion. The final reusable probe reproduces the
initial prototype's traces; all failed data are retained outside Git.

## Tests and reproduction

The new tests independently check the hand-expanded polynomial against the
physical generator, explicit multirate execution, preservation of all nongain
configuration fields, protected seed/horizon boundaries, nominal initialization,
short-prefix nonqualification, output collision rejection and hook restoration.
All 36 focused tests pass (0.57 s), including the 17 new tests and the preceding
19 diagnostic tests. Ruff lint/format and mypy pass. Production source is unchanged.

```bash
uv run ruff check .
uv run ruff format --check .
uv run mypy src experiments
uv run pytest -q -W error tests/unit/test_feedback_motor_damping_probe.py \
  tests/unit/test_feedback_startup_diagnostic.py
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.feedback_motor_damping_probe \
  --output /path/to/NEW_DIR --workers 3
```

The experiment intentionally returns status 1 for the documented failed screen.
The final execution-source digest is
`bf283cdb17a2451e7e49c313c7c71994a701e299531f75a637696051f234142c`.
This diagnostic is not design version 3 of `feedback_bandwidth_validation`; its
only supported runtime design versions remain 1 and 2.

PR #10 remains draft, full performance qualification remains 28/30 for version 2,
and progress percentages are unchanged. The next design needs a joint tracking,
measurement-error and actuator-effort objective on the sampled cascade; pole
location or increased stiffness alone has not provided the required margin.
Any subsequent candidate must pass observed-case regressions before a complete
source freeze and untouched full-hold qualification. The reserved seeds remain
unused; minimum-snap planning and later integration remain outside this step.
