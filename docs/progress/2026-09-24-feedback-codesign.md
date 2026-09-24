# Bounded joint feedback design — 2026-09-24

## Frozen scope and decision rule

Audited parent: `c19985bc8fda5279285c4901c05e97a22df688ff`.
Its GitHub CI passes 3,107 tests in 382.93 s, Ruff lint/format and mypy
(run 36054749863, job 107818795321). The preceding fixed motor-damping
candidate remains rejected: 5/6 observed prefixes pass 8 cm, without margin.
No production defect was demonstrated by that result. This cycle changes no
production source, filter, noise/prior assumptions, plant, reference or limits.

The **existing** five-gain motor-damped cascade is the only candidate family.
The variables are horizontal position gain `kp` in [4,40] s^-2, velocity gain
`kv` in [2,12] s^-1, roll/pitch attitude gain `ka` in [4,24] s^-1,
rate gain `kr` in [8,80] s^-1, and dimensionless motor damping `gamma` in [0,2].
No integral, command smoother, gain scheduling or new sensor is introduced.

Development inputs are the 20 **already observed** version-2 hover histories,
seeds 93000..93019, with fixed report SHA-256
`86d2c651b31eaf7a4afada0c0987a16008d05c6dd2be7ce10f13fbec51cadf90`.
Only their 0..10-s prefixes are used. Their measurement-minus-truth errors
are external forcings of the local sampled linear model, not predictions of
the ESKF on a changed trajectory. Every input data archive is authenticated.

Search budget: 512 initial points (the two previous designs plus 510
stratified points from an explicit PCG64 seed 250924), then exactly three
ten-neighbor coordinate rounds at normalized steps .10, .05 and .025.
The first four coordinates are logarithmic; damping is linear. Total budget
is at most 542 evaluations. The objective and bounds are not revised after
results. One selected design, not a nonlinear gain sweep, may be run.

For horizontal position `p`, modeled requested moment `m`, and noise-response
RMS `n`, the dimensionless objective is

```text
J = (max_trial,max_5..10s ||p|| / .07 m)^2
  + .20 * mean_trial,time_5..10s(||p||^2) / (.03 m)^2
  + .10 * mean_trial,time_0..10s(||m||^2) / (.15 N m)^2
  + .10 * (max_trial,time,axis |m| / .8 N m)^2
  + .10 * (n / .02 m)^2.
```

`n` is the one-axis infinite-horizon local position response to independent
unit-variance, zero-mean disturbances scaled by .02 m position, .02 m/s
velocity, 1 degree tilt, and .002 rad/s rate. Position/velocity disturbances
refresh at 50 Hz; tilt/rate disturbances refresh at 100 Hz. These are declared
design scales, **not** an assertion that actual ESKF errors are white or have
these covariances. The actual colored, coupled errors enter the first four
objective terms separately. Noise response is computed from the discrete
Lyapunov equation of the lifted two-inner-tick model.

Model admissibility requires all equivalent sampled poles to have real part
below -3 s^-1 and damping at least .55, requested rate at most 2 rad/s, and
pre-clipping requested moment at most 1.6 N m (twice the physical .8-N-m
bound). That screening ceiling does not change physical clipping. Actual
nonlinear limiting must still pass the inherited .5-s consecutive limit.
The North/East moment mapping uses pitch/roll inertias .025/.020 kg m².

The single selected design is screened on exactly the six previously exposed
seeds 30, 91001, 93003, 93012, 8200 and 8201, using the unchanged mission's
0..10-s prefix. **Every 5..10-s 3D peak must be below 7 cm**, providing one
centimetre of headroom below the unchanged 8-cm qualification requirement.
All six must also have consecutive limiting <=.5 s. Any failure ends this
cycle without another candidate, production promotion, or fresh validation.
Passing this prefix gate would only authorize complete observed-case
regression and a source/protocol freeze before any fresh qualification.
Reserved 95000..95019 and 96000..96009 seeds remain unopened unless those
preceding gates pass. No local result proves general robustness or infeasibility.

## Results

The fixed 542-evaluation budget completed. Five of the initial 512 points passed
the local pole constraints, and two of those passed the modeled rate/moment
ceilings. Including the 30 coordinate-neighbor evaluations, 21 evaluations
passed the pole constraints and 18 were admissible. These are evaluation counts,
not necessarily distinct gain vectors. **None of the admissible evaluations
reached even 8 cm in the frozen-error linear screen.** The narrow admissible
sample and finite budget are important limitations; this is not a proof that
all possible gains, controllers or sensing arrangements must fail.

The objective selected `kp=6.4`, `kv=4`, `ka=7.314462614094624`,
`kr=15.104974020574769`, `gamma=0`. In particular, motor-acceleration feedback
was not selected. No new state is needed by this selected control law.
The dimensionless objective changed from 1.933583 for version 2 to 1.921661
(only 0.617%). Modeled requested-moment mean square decreased 22.77%, but
the worst linear peak barely changed, from 9.08071 to 9.07810 cm.
These are local development metrics, not measured full-mission improvements.
Squared moment is a control-effort proxy with units N² m², not electrical energy.

The existing version-2 local replay differs from its own original nonlinear
horizontal position history by at most 1.680 mm over these 20 prefixes.
Only this same-controller comparison measures local-model discrepancy;
comparing another design with the old trajectory also includes control changes.
The approximation omits finite-angle/vertical coupling, limits, motor-predictor
error and changes in the ESKF errors induced by a new trajectory. The separate
nonlinear experiment therefore remains decisive.

| Observed seed | Nonlinear 5..10-s 3D peak (cm) | Consecutive limiting (s) | Below 7-cm margin |
| --- | ---: | ---: | --- |
| 30 | 6.20571 | 0 | yes |
| 91001 | 6.59672 | 0 | yes |
| 93003 | **9.14022** | 0 | no |
| 93012 | **9.03842** | 0 | no |
| 8200 | 6.00136 | 0 | yes |
| 8201 | 4.13704 | 0 | yes |

The single nonlinear candidate fails both the stricter margin gate and the
original 8-cm threshold in two cases. There is no actuator limiting in any of
the six prefixes; reduced effort has not repaired the tracking miss. All six
reach the diagnostic endpoint without numerical failure. This is **4/6 in an
observed-case prefix screen**, not a new full-hold qualification percentage.

## Mathematical and implementation audit

The exact held-input local plant is the existing
`hover_axis_zero_order_hold` with state `[p,v,eta,r,a]`, inner period .01 s,
outer period .02 s, `g=9.81 m/s²` and motor lag .025 s. At each outer epoch,
`eta_d=-(kp*(p+e_p)+kv*(v+e_v))/g`; the value is held across two inner ticks.
Each inner command is
`u=kr*(ka*(eta_d-eta-e_eta)-r-e_r)-gamma*a`, with angular-acceleration units.
The matched local model uses the true linear motor state `a` as the nominal
prediction: zero predictor error is a linear-analysis assumption only.
In the nonlinear probe the predictor uses previous commands and the explicit
nominal-hover rotor prior, never actual rotor observations.

`lifted_model` assembles `x[k+1]=A*x[k]+B*w[k]` over the complete outer interval.
Its six disturbance coordinates are the shared position/velocity pair and the
two independent inner-tick tilt/rate pairs. The design covariance solves
`P=A*P*A.T+B*B.T`, and `n=sqrt(P[0,0])`. State scaling improves the numerical
solve; covariance symmetry, positive semidefiniteness and equation residual
are checked. An independent impulse-response energy sum verifies the result.
This algebra does not assert independence or Gaussianity of real filter errors.

`experiments.feedback_codesign` owns the input authentication, deterministic
candidate set, explicit objective, bounded search, selected-gain record and
observed-only nonlinear entry point. Source changes, wrong input identity,
changed winner or excessive recorded budget prevent using a selection.
An empty admissible set preserves its failed ledger and cannot start a probe.
The preceding probe's execution body is factored into a private helper so that
the same nominal motor model, allocator and limit accounting are reused.
Its public behavior remains fixed at the preceding gains and damping .5;
an exact archive comparison verifies preservation on the short regression case.
No public controller or configuration version is introduced.

The focused tests cover gain types/bounds, deterministic stratification,
coordinate round trips, explicit multirate disturbance signs, constant-position
error equilibrium, independent Lyapunov/impulse agreement, North/East inertia,
every objective term, finite arrays/shapes, source/selection boundaries,
budget exhaustion, failed-ledger preservation, unchanged nongain configuration,
reserved-seed refusal, the stricter margin gate and restoration of temporary
execution hooks. All 82 focused tests pass, including 46 new tests and all 36
tests of the two preceding diagnostics. Ruff lint/format and mypy pass.
Before the final six selection-record/failed-ledger regressions were added,
the complete local suite passed 3,147 tests in 225.12 s with warnings as errors.
The final local full-suite rerun became unavailable before its completion could
be verified; its partial output is retained and is not counted as a pass.
The final commit's complete verification result is recorded in GitHub CI and
the accompanying publication receipt, rather than inferred from partial output.

## Reproduction and stop decision

```bash
uv run ruff check .
uv run ruff format --check .
uv run mypy src experiments
uv run pytest -q -W error
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.feedback_codesign synthesize \
  --original /path/to/design-validation-v2 --output /path/to/NEW_SELECTION
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.feedback_codesign probe \
  --selection /path/to/NEW_SELECTION/selection.json \
  --output /path/to/NEW_PREFIXES --workers 3
```

The probe deliberately returns status 1 for the failed margin screen.
The selection, all 542 evaluations, authenticated reduced input histories,
six nonlinear archives and repeat-check records are retained outside Git.
The initial execution is retained as well as the final execution after
selection-record validation was hardened; no objective, gain, bound or seed
was changed in that hardening.
All 542 evaluation records and the selected gains repeat exactly; the reduced
input archive and all six nonlinear archives repeat byte for byte. Independent
rescoring confirms every peak, moment maximum, zero limiting interval, complete
400-Hz prefix and failed gate. This is not a full rigid-body-history audit of a
qualified mission. The final execution-source SHA-256 is
`f1a34fc294f741ed6803676666f14826a96b06d0252aa5bfdec23d1db1b91ea0`.

**Stop this gain-tuning cycle.** It did not satisfy the declared requirement.
Do not introduce compensators, tune the filter around the selected trials,
change the scoring window, open the reserved seeds, or call the block qualified.
Keep the causal filter implementation and both frozen controller baselines;
retain this candidate as a rejected experiment only. PR #10 stays draft and
main is unchanged. The previously published full-hold result remains 28/30
for version 2. No new completion percentage is warranted.

Another implementation cycle needs an explicit architecture/requirements
decision, not another automatic gain sweep: either retain this measured
limitation as an unqualified baseline, or separately scope a justified change
to the information/initialization/control architecture against the same target.
This experiment establishes neither a minimum achievable error nor a need to
replace the ESKF. Trajectory planning, new sensors, new flight phases and
requirement relaxation were not authorized by this design cycle and are not
silently introduced as fixes.
