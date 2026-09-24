# Position/Velocity Cascade and Baseline Mission Verification

Date: 2026-09-24 (UTC).

- Audited baseline: `4dded6210a6e07953f309e6426ec7d615060c4fd` (merged attitude/rate milestone).
- Design and frozen acceptance: [ADR 0012](../decisions/0012-position-control-and-missions.md).
- Mathematics, interfaces, limits and reproduction: [position/mission guide](../position-control.md).
- Scope source: canonical six-month plan, Week 8 and Gate G2 numerical targets,
  checked against current code/tests and the preceding control verification record.

## Audit of the preceding milestone

The complete attitude controller, stage-resolved simulation, experiment, plotter, focused
tests, frame/actuation/configuration contracts and associated documentation were reviewed.
Remote and local main both identified the baseline above. The fresh full suite passed
**2,659 tests with warnings treated as errors** in 68.05 s. A fresh fixed attitude
campaign passed all **14 cases**, including signed axis steps, finite torque rejection,
saturation recovery, persistent-torque offset and three plant refinements.

No scoped mathematical or behavioral defect was found that warranted altering that
milestone. Its production source, tests and frozen protocol remain unchanged. The new
mission harness reuses its internal stage-resolved plant step rather than duplicating
or changing motor, rotor, drag and rigid-body equations. This is an automated technical
self-audit and numerical verification, not an independent human review or a proof of
global stability.

## Implemented increment

Three core modules add:

- `position_control`: pure NED position/velocity PD with acceleration feedforward,
  component acceleration limits, tilt-cone and thrust-magnitude limiting, exact heading
  construction and a robust largest-component body-to-world quaternion conversion.
- `missions`: owned hold/quintic/step references, connected phase schedule, sampled
  geofence/tilt guards and an absorbing completion/abort supervisor with landing dwell
  reset and timeout.
- `mission_simulation`: true-state position/attitude feedback on explicit integer clocks,
  independent truth/nominal configurations, complete command/state histories and no
  terminal command. The historical sensor-run artifact/manifest schemas are untouched.

Two experiment modules provide the frozen full-mission campaign, digest-bound per-trial
compressed histories, full ledger validation and reproducible headless plots. Four test
modules add **200 tests**. No dependency, tool version, CI configuration, pre-existing
source module or pre-existing test was changed. Documentation links now distinguish the
standalone attitude layer from its new mission composition.

## Model, tuning and declared scope

The existing illustrative model remains mass 1 kg, inertia diag(.02,.025,.04) kg m²,
the documented X rotor geometry/spins, k_f=1e-5, k_m=2e-7, motor speeds 0..900 rad/s,
motor time constant .025 s and gravity 9.81 m/s². Inner-loop gains and bounds remain
unchanged. Position gains are K_p=[1,1,2.25] s^-2 and K_v=[1.8,1.8,3] s^-1, chosen
from the ideal local damped second-order model. Position natural frequencies [1,1,1.5]
rad/s are slower than the inner-loop frequencies; execution rates are 400/100/50 Hz
for plant/attitude/position. No gains or acceptance thresholds were retuned after
development or after seeing held-out results.

Command acceleration bounds are [2,2,2] m/s², desired tilt 20 degrees, and collective
2..18 N. The actual-state guard is a 35-degree tilt limit and NED geofence
[-3,-3,-3]..[3,3,.5] m. Nominal parameters equal truth for the calm-air campaign.
The mild-wind diagnostic alone supplies wind [.5,-.3,0] m/s and quadratic body drag
[.1,.1,.15] kg/m. The outer law does not consume either wind or drag as feedforward.

Missions initialize for 1 s, rise 1 m over 4 s, track their declared reference, return
to the virtual launch plane over 4 s and satisfy the .5 s terminal dwell. The square
uses 1 m edges with 6 s smooth segments and 1 s holds, preceded by a 2 s cruise hold.
The step diagnostic changes altitude by .5 m at t=7 s. Position/speed completion bands
are .08 m/.08 m/s, with an 8 s landing timeout. No physical contact, disarming,
estimated-state feedback or hardware safety response is represented.

## Fixed and development results

All five fixed jobs pass, including one square refinement with half plant dt and
unchanged controller periods. Three development seeds (2000–2002) also pass. All fixed
and development missions have no numerical failures, guard aborts or actuator limiting.

| Fixed case | Duration [s] | Full-mission position RMSE [m] | Peak position error [m] | Final position error [m] |
| --- | --- | --- | --- | --- |
| 60 s hover, plus takeoff/landing | 69.5 | .00093917 | .00428845 | .00152359 |
| Square | 39.5 | .01876144 | .03489833 | .00157194 |
| Vertical step | 19.5 | .10324388 | .50065463 | .00228419 |
| Mild-wind square | 39.5 | .03164647 | .05676947 | .02691292 |
| Square, half plant step | 39.5 | .01876144 | .03489833 | .00157194 |

The complete 60-second hover interval t=5..65 s has maximum position error
**.00163469 m**, below the frozen .08 m band. The vertical step's roughly .5 m
instantaneous reference error is expected; simultaneous .08 m/.08 m/s settling
occurs **2.2925 s** after the step, below the 6 s target. The mild-wind final offset
is retained as a limitation of PD control, not mislabeled as zero-error disturbance
rejection. Square position RMSE is the norm-based, time-weighted error across the
**whole mission**, including takeoff, tracking, landing and terminal dwell.

Full-common-grid coarse/fine square differences are at most **1.338e-10 m** in
position and **6.969e-10 degree** in attitude. These are numerical integration
differences in a gentle deterministic simulation, not physical flight accuracy.
The earlier analytical motor spin-up and independent quaternion/derivative tests
remain important complementary evidence.

Development RMSE values are .02161123, .02198708 and .02287226 m. The controller,
model and criteria were unchanged following those runs. Independent physical
reconstruction passed all eight fixed/development histories before held-out execution.

## Frozen held-out identity and results

The execution-source SHA-256 is:

`24e6e7c045fd3de1dcb43e126079aad4c702e97ff5da28a96f0ba27ab85054f8`

It covers ordered names, lengths and bytes of all package/experiment Python sources,
pyproject.toml and uv.lock, including new untracked source. Tests/docs are reviewed
separately. The prevalidation record was written at **2026-09-24T11:12:33.603187+00:00**,
after the 2,858-test warnings-as-errors gate and physical audit. One further near-tilt-
boundary quaternion regression was added without changing execution source, gains,
seeds or thresholds; the final gate has 2,859 tests.

| Partition | Trials | Protocol SHA-256 |
| --- | --- | --- |
| Fixed | 5 | `43f0deece9cbf4f9d7f2157d6f5426b7a5d032df3ee784d1fe6d155e124697fb` |
| Development | 3 | `5a9c5ff496a3066600b30a28025ba2d73a523af6f98164627021133d7b35d8d1` |
| Held-out | 10 | `951e6d234207ab2378224d909cffd0d229a9cc14bb9779655ee93e2a1684e161` |

Held-out seeds 70000–70009 use explicit PCG64/SeedSequence streams. Their independent
initial position/velocity components are uniform in ±.02 m/±.02 m/s, initial rotation-
vector components in ±2 degrees, and body rates in ±.02 rad/s. These are initial-condition
perturbations, **not** sensor-noise, parameter-mismatch or fault trials.

All **10/10 nominal square missions** complete under the frozen acceptance: each
full-mission position RMSE is below .15 m, no sampled safety violation occurs, no
actuator limiting lasts over .5 s, and the final second is limit-free. In fact, no
moment clipping, allocation scaling or commanded rotor-bound limiting occurs in
these missions. There are zero numerical or acceptance failures.

| Held-out quantity | Observed value |
| --- | --- |
| Position RMSE range | .01967840–.02292190 m |
| Worst peak position error | .07051686 m |
| Worst final position error | .00157194 m |
| Worst final speed | .00125449 m/s |
| Maximum actual tilt | 2.474634 degrees |
| Longest actuator limiting | 0 s |

The small errors reflect a matched, noise-free true-state simulation and gentle 1 m
waypoint motions. Ten seeds support this bounded deterministic campaign, not a
statistical reliability guarantee or comparative/hardware performance claim.

## Mathematical, boundary and evidence audit

| Contract | Verification |
| --- | --- |
| NED force and signs | Independent world-force reconstruction, six signed PD-axis checks, upward thrust sign, acceleration feedforward identity |
| Orientation | Proper rotation, exact projected heading, wrapped/half-turn yaw, all four largest-quaternion-component branches; conditioning-aware roundoff near 85-degree tilt |
| Feasibility | Separate analytic acceleration/tilt/thrust projection, minimum-thrust case, seeded bounded demands, zero-moment allocation feasibility at both collective endpoints |
| State/reference ownership | Frozen objects, independent C-contiguous read-only arrays, invalid shape/dtype/finiteness/Boolean/overflow rejection, no input mutation |
| Reference mathematics | Exact endpoint rest conditions; independent quintic p/v/a and finite-difference derivative checks; hold/step and right-continuous boundaries |
| Supervisor | Dwell reset, speed-band requirement, timeout, guard precedence, absorbing terminals, initial and between-controller-tick aborts |
| Numerical composition | Exact hover, explicit multi-rate clocks/holds, old plant reuse, future-reference causality, independent truth/nominal sensitivity |
| Physical CI regressions | Two seeded short flights with broad RMSE/final-state bounds; deliberately acceleration-limited recovery with no residual limiting |
| Evidence | Worker-count repeatability, disjoint seeds, exact ledger, stale metric/clock/initial/reference/phase/outer-demand rejection, failure retention, digest corruption and overwrite rejection |
| Visual output | Complete stored histories, headless rendering, validated inputs, input/plotter hashes and manual visual inspection |

An additional audit script reconstructs all saved command/actual rotor wrenches from
an independent numerical allocation matrix, verifies each exact motor interval,
reconstructs the rate/position laws and moment clips, checks quaternion norms and
rotor/thrust/geofence bounds, and independently recomputes position RMSE. Fixed and
development reconstruction residuals are at most 4.55e-13 rad/s for motor response,
1.78e-15 N for collective, 5.09e-16 N m for allocation-ray agreement and 2.23e-16
for quaternion squared norm. The recomputed RMSE agrees to stored precision.

The audit also compares central-difference velocity/rate derivatives with physical
accelerations. At the abrupt vertical command change, a centered window straddles
two different motor-forcing slopes. Its .02651935 m/s² residual is a window-average
effect, not an instantaneous plant mismatch. The exact exponential motor integral
predicts that window average within **1.15e-8 m/s²**. Smooth-window maxima are
.001577 m/s² and .002030 rad/s² in the prevalidation set. The first inappropriate
all-window assertion is retained with this explanation; the corrected audit separates
smooth intervals from command knots and preserves both diagnostics. No controller,
plant, gain or acceptance threshold was changed for this audit observation.

## Local verification commands

All commands ran on 2026-09-24 against the worktree based on the recorded baseline,
with the existing locked Python 3.12 environment. `MISSION_EVIDENCE` is outside Git.

| Command | Verified outcome |
| --- | --- |
| Baseline `.venv/bin/python -m pytest -q -W error` | 2659 passed in 68.05 s |
| Previous `experiments.attitude_control_validation --partition fixed --workers 4` | 14/14 pass; unchanged protocol and implementation |
| `uv tool run --from uv==0.12.3 uv run make check` (final) | Ruff/format pass, strict mypy 41 files, 2859 passed in 91.12 s |
| `.venv/bin/pytest -W error` (prevalidation) | 2858 passed in 94.06 s; no warnings/failures/skips |
| `.venv/bin/pytest -q -W error` (final, captured directly) | 2859 passed in 88.48 s; no warnings/failures/skips |
| `.venv/bin/pytest -q -W error tests/unit/test_position_control.py` (added boundary regression) | 106 passed |
| `experiments.position_control_validation --partition development --workers 3 --output "$MISSION_EVIDENCE/development-v1"` | 3/3 pass |
| Same CLI, `--partition fixed --workers 4` | 5/5 pass, including full-grid refinement |
| Same CLI, `--partition validation --workers 4` | 10/10 pass; frozen source/criteria unchanged |
| `load_report` plus independent `audit_histories.py` | Complete ledgers and physical reconstructions pass |
| `experiments.plot_position_control` | Five fixed figures and held-out ensemble; no generated results in Git |

## Technical completion boundary

The implemented true-state baseline satisfies the declared G2 numerical mission targets.
No claim is made about a separate explain-back assessment, global stability, flight
readiness or safe physical landing. No ESKF/sensor/manifest behavior was changed.
The next bounded integration task is estimated-state feedback with explicit causal
sensor acquisition/delivery, estimator update, controller sampling and initialization
contracts, compared with this retained true-state baseline. That integration is not
part of this implementation.
