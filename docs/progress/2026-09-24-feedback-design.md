# 2026-09-24: Estimated-feedback design and qualification

**Outcome: qualification remains open; PR #10 stays in draft.** Passing source
checks and physical replay do not override the retained hover-performance
failures. A separate failure-reporting correction is isolated in
[PR #11](https://github.com/Gayles9/robust-quadrotor/pull/11).

## Scope and preceding-step audit

Audited branch commit: `94c074ef20d6e862f7143c39b00418799b5c4c6e`.
The corresponding draft PR #10 starts from main
`23197e95feace859c4d7405c4e2574e06f2c33bb`. The mathematical design and acceptance
rules are in [ADR 0014](../decisions/0014-estimated-feedback-bandwidth.md);
the [technical guide](../feedback-design.md) maps them to code.

A fresh pinned `PYTEST_ADDOPTS='-W error' make check` passed **2952 tests in
152.14 s**, plus lint, format and typing. A fresh unchanged fixed campaign
reproduced **all five original metrics and exact archive-byte hashes**, including
the failed .10756338079020142 m hover peak. Original protocol hashes are also
protected by new regression tests. The full mathematical ESKF, sensor producers,
mission clocks, plant, control laws, limits and historical artifacts are retained.

The audit identified an evidence-validation gap, not a plant/control defect:
the paired true-state history validator checked outer commands and references but
did not reconstruct every inner command. It now compares requested/clipped/allocated
moments, desired rates, rotor commands, scaling and limit flags with their defining
law. Tests reject tampered paired histories in both the original and new campaigns.
The strengthened `position_control_validation.load_report` also accepts all five
previously published true-state fixed/refinement cases, with zero numerical or
acceptance failures. The fresh result is retained in
`design-legacy-position-reload.log`; stricter checking does not invalidate the
authentic historical evidence.

## Mathematical diagnosis and design

The original hover's north/east tracking error develops during initial tilt
uncertainty and subsequently overshoots. At its 5.395-s peak, north error is about
-.1003 m although total position-estimation error is only about .0219 m. The
five-state motor/inner/outer model predicts damping .574 for the original slow
oscillatory mode, compared with .9 from an ideal double-integrator approximation.

An unfitted local forced reconstruction uses recorded position, velocity, attitude
and rate estimation errors as inputs. It predicts a horizontal hold peak
**.109163 m**, compared with measured **.107563 m**. Over the first ten seconds,
local/nonlinear horizontal position discrepancy has **.002254 m RMS** and
**.004999 m maximum**. This supports the coupled-loop explanation. It is not an
independent alternative closed-loop run or proof that one error channel is the
sole cause. Changing takeoff thrust, nonlinear rotations and yaw coupling are
among the linear model's approximations.

The first profile matches coefficients of
`.025*(s+3)^2*(s+6)*(s²+28s+392)` and uses the declared initial uniform population's
exact second moments for P0. The prior mean and true population are unchanged;
P0 is not computed from a trial's realized state. Sensor and process noise, gates,
clocks, references, scoring window and thresholds remain unchanged. Development
misses and the design refinement are explicitly retained in ADR 0014.

The only new mathematical-core module is `cascade_analysis.py`, a bounded local
analysis utility. The runtime controllers remain pure functions of explicit
parameters; the selected profile is provided by the new experiment's configuration
builder. The original experiment and its measured failure remain reproducible.

## Version 1 verification record

All commands use Python 3.12.14 and the unchanged `uv.lock` with uv 0.12.3.
`E` below denotes the external evidence directory; outputs are never overwritten
or committed. The audited parent commit is recorded above; the exact execution
SHA-256 for version 1 is
`d5e38268c0dd396db157833d51923f32d9656245138ae6886ef303f230c9f398`.

| Command (2026-09-24 UTC) | Confirmed outcome |
| --- | --- |
| Parent: `PYTEST_ADDOPTS='-W error' make check` | 2,952 passed in 152.14 s; static checks pass |
| Parent: `python -m experiments.estimated_feedback_validation --partition fixed --workers 3 --output "$E/audit-fixed-reproduction"` | Exact original metrics and archive hashes reproduced; 4/5 as expected |
| New: `PYTEST_ADDOPTS='-W error' make check` | 3,037 passed in 178.66 s; Ruff lint, 157 formatted files and mypy 49 source files pass |
| New: `python -m experiments.feedback_bandwidth_validation --partition fixed --workers 3 --output "$E/design-fixed-v1"` | 5/5; zero numerical or acceptance failures |
| New: `python -m experiments.feedback_bandwidth_validation --partition development --workers 3 --output "$E/design-development-v1"` | 9/9; zero numerical or acceptance failures |

The local full gate was rerun after correcting two test-authoring issues (an
import-format check and mutation of an intentionally read-only fixture). Neither
required a production behavior change. `design-full-gate-v4.log` is the complete
successful gate used for the freeze; incomplete earlier logs are not counted as
passes. There are 85 additional tests: 61 local-analysis tests and 24 experiment,
integrity and bounded-startup tests.

### Fixed and development results

| Fixed case | Full-mission true position RMSE [m] | Peak attitude-estimation error [deg] |
| --- | ---: | ---: |
| Noiseless square | .003069 | .000006819 |
| Noisy 60-second hover, seed 30 | .031030 | 9.4622 |
| Noisy square, seed 31 | .032945 | 2.6265 |
| Vertical step, seed 32 | .111541 | 1.6941 |
| Mild wind, seed 33 | .030545 | 4.4466 |

The unchanged 5–65 s hover hold peak is **.0747738793 m**, below .08 m,
versus **.1075633808 m** for the original profile: about **30.5% lower**.
This is not the full-mission peak, which includes the takeoff transient and is
.160019 m. The hold margin is only .005226 m; it must not be described as a
large robustness margin. All fixed cases complete with no actuator limiting.

The six development hovers span **.051326–.057487 m** hold-peak error and
**.025756–.028539 m** full-mission RMSE. The three development square missions
span **.028793–.030208 m** RMSE. Maximum development attitude-estimation error
is **12.0171 degrees**, below the unchanged 15-degree criterion. Every trial is
retained and evaluated, not just the best seed.

Higher bandwidth has a measurable cost. In fixed hover, the integral of squared
actual moment increases from **.00162598 to .0121999 N² m² s**, about **7.50×**.
That quantity is a control-effort metric, not motor electrical energy. No moment,
rate or allocation limit is reached; absence of limiting does not imply that
unmodeled structural dynamics or real hardware would tolerate these gains.

### Source freeze

`design-source-freeze-v1.json` was recorded at **15:01:03 UTC**, after the fixed,
development and quality gates and before opening new validation seeds. It includes
every execution/test/toolchain file hash and hashes of prerequisite reports/logs.

| Partition | Protocol SHA-256 |
| --- | --- |
| Fixed | `80df4d0315667dc87e57078ae74bfa43bfa59cc9e7f1a0c81dd04c69f381d93e` |
| Development | `4acb0aa56eb4e75fb3097dbb7a4b3fda4404a666599464d3abdcd7fe2877859b` |
| Validation | `f7be987e273d1095ee20b3376ac10c949f78e82f550f8dcea9fe361798642eca` |

### Independent physics and numerical checks

The campaign replays every saved measurement through the independent offline
estimator boundary and requires exact state, covariance and event agreement.
It reconstructs both estimated-feedback and true-feedback commands, mission
transitions, clocks and references, and recomputes every scored metric.

An additional external audit imports only the authenticated archive reader. It
independently rebuilds rotor allocation, stage motor lag, FRD drag, body-to-world
acceleration, Hamilton quaternion derivatives and all four projected-RK4 stages.
It also regenerates the six named random streams, bias walks and sensor samples,
checks control laws and covariance eigenvalues, and repeats the integration-map
check on the separately executed true-state comparator. All **14 fixed/development
pairs** pass, covering **297,214 estimated-feedback truth epochs** plus their
paired true-state histories.

An earlier approximate central-difference angular-acceleration diagnostic produced
.01988 rad/s² against its old .01 diagnostic tolerance at the higher bandwidth.
That is not treated as a passed diagnostic, nor was its tolerance widened.
The stronger check evaluates the actual projected-RK4 discrete map independently,
requiring component residuals below 1e-10 on every interval. Central-difference
values remain descriptive evidence of finite-difference truncation, not a new
acceptance criterion or a hidden relaxation of mission targets.

The noiseless square's true-feedback comparator was also recomputed at **1.25 ms**
instead of **2.5 ms**, retaining the same 10-ms/20-ms controller clocks and all
parameters. Phases and command timestamps agree exactly. Maximum aligned component
differences are **1.338e-10 m** in position, **2.075e-10 m/s** in velocity,
**1.518e-11** in quaternion, **3.066e-10 rad/s** in body rate and **2.135e-8 rad/s**
in rotor speed. This supports numerical resolution of this trajectory; it is not
a stochastic sensor-discretization study or a universal integration bound.

External audit scripts, their SHA-256 values and complete outputs accompany the
retained evidence, rather than adding generated histories to Git.

### First frozen validation outcome: not qualified

`python -m experiments.feedback_bandwidth_validation --partition validation --workers 6 --output "$E/design-validation-v1"`
completed with **29/30 acceptance passes**, zero numerical failures, and a failing
CLI status as required. Hover seed **91001** reaches **.0813936606 m** at exactly
**5.0 s**, exceeding .08 m for .1825 s. Its full-mission RMSE is .030323 m and
maximum attitude-estimation error is 8.1665 degrees. Every attitude condition and
all ten square missions pass, but that does not erase the hover failure.

The failure is a startup-settling margin issue: the east tracking component is
-.079390 m, with east position-estimation error +.023387 m and velocity-estimation
error +.025557 m/s. After 10 s its hold error remains below .039612 m. The scoring
window must not be moved. All 30 saved pairs pass independent physics/sensor/control
reconstruction over **714,030 estimated-feedback epochs** plus their paired
histories. All **299** fixed/development/validation NPZ files pass SHA-256 and ZIP
CRC checks, and all execution/test files still match the prevalidation freeze.

This candidate remains **unqualified**, despite 43 of its 44 combined planned
trials passing. The full failed batch is retained. A subsequent design revision
must disclose these observed seeds as diagnostic data and use new held-out seeds.

## Version 2: settling-margin revision

The complete version-1 source and failed qualification are retained in commit
`6eadfa0438663f2fd26bee6298b62ff3b4e0e11d`; its hosted CI run
[36025423940](https://github.com/Gayles9/robust-quadrotor/actions/runs/36025423940)
passed. Passing code checks did not qualify its failed performance result.

The revision is scoped to the four horizontal/roll/pitch gains. The population
prior, estimator equations, noise, plant, vertical/yaw settings, limits and
complete hold window remain identical to version 1. Four observed hover seeds
(91001, 91011, 91016, 91019) are explicitly added to development diagnostics;
they cannot count as fresh validation. The new held-out identities are hover
93000..93019 and square 94000..94009.

The first all-real development target, roots -5,-5,-8,-11,-11, reaches
.0706366 m on startup regression 91001. This clears the original .08 m target
but fails the stricter .07 m development-margin test. That failure is retained
in `design-v2-focused.log`. No held-out version-2 data had been examined.

The final target `.025*(s+8)^5` attains the maximum `kp=6.4 s^-2` within the
fixed-motor-lag, all-real continuous pole family, by the pair-product
Cauchy-Schwarz bound in ADR 0014. Its other gains are `kv=4`, `ka=8`, `kr=16`
in s^-1. The actual multirate local map has minimum damping ratio **.76816**
and slowest equivalent decay **3.92665 s^-1**, versus .60122 and 2.45992 for
version 1. These describe the local sampled controller/plant approximation;
nonlinear stochastic acceptance is evaluated separately.

`pytest -q -W error tests/unit/test_feedback_design_revision.py` passes all
**24 revision tests in 48.21 s**. The full pinned
`PYTEST_ADDOPTS='-W error' make check` passes **3,061 tests in 242.31 s**, Ruff
lint/format (**159 files**) and mypy (**49 source files**). There are **109 new
tests** relative to the audited 2,952-test integration checkpoint. The exact
version-2 execution SHA-256 is
`04d685799c357fb33df2120e6c35b9199ade8389de3e226dbca2a1f7bf59c017`.

The revision tests independently expand the polynomial, compare both nonlinear
horizontal Jacobians, verify that only the four gains change, preserve every
version-1 protocol hash, enforce seed separation and retain the 7-cm startup
checks at the original hold entry. Full-history audits now run independently in
worker processes. Exact serial/parallel histories and saved bytes agree for
identical report metadata (CLI worker-count provenance remains explicit); corrupt
metrics, estimated commands, paired commands or covariance in a later worker
propagate an exception before any report directory is created. The accelerated
audit performs all original replay and physics/control-contract checks.

### Fixed mission results

`python -m experiments.feedback_bandwidth_validation --design-version 2 --partition fixed --workers 2 --output "$E/design-fixed-v2"`
passes **5/5**, with zero numerical or acceptance failures. Every saved pair also
passes the campaign's complete replay, command and metric validation before
publication. All five cases complete without actuator limiting.

| Fixed case | Full-mission true position RMSE [m] | Peak attitude-estimation error [deg] |
| --- | ---: | ---: |
| Noiseless square | .002113 | .000007169 |
| Noisy 60-second hover, seed 30 | .026235 | 9.5736 |
| Noisy square, seed 31 | .028593 | 2.6265 |
| Vertical step, seed 32 | .110486 | 1.6941 |
| Mild wind, seed 33 | .027089 | 4.4251 |

The original hold-peak miss improves from **.1075633808 m** to
**.0621390364 m**, about **42.2% lower**, measured over the identical 5–65 s
window. The full-mission peak is .142110 m and is not confused with this
hold-specific target. Full-mission hover RMSE falls from .044578 to .026235 m.
Squared actual moment integrated over the mission rises from **.00162598** to
**.0243405 N² m² s**, about **14.97×** the original profile and **2.00×**
version 1. The gain design improves tracking at a substantial
control-effort cost; that integral is not electrical energy. No rate, moment or
allocation limiting occurs, but hardware suitability has not been established.

The separately rendered `design-control-effort-comparison` figure shows the
original, version-1 and version-2 trajectories for the identical fixed seed.
It uses full-rate actual moments, without smoothing, and independently integrates
their squared norm to reproduce each recorded effort value. The largest increase
is concentrated in the startup transient; residual moment activity also increases.

All five fixed pairs also pass the separate exact physics/sensor/control audit
in `design-v2-fixed-paired-audit.json`. A fresh step-halving check of the revised
noiseless square comparator keeps the same 10/20-ms control clocks while reducing
the plant step from 2.5 to 1.25 ms. Aligned phases and control epochs agree
exactly. Maximum component differences are **1.338e-10 m** position,
**2.075e-10 m/s** velocity, **2.101e-11** quaternion, **3.418e-10 rad/s** body
rate and **2.221e-8 rad/s** rotor speed. This establishes resolution for this
deterministic trajectory, not stochastic discretization calibration.

### Development results

`python -m experiments.feedback_bandwidth_validation --design-version 2 --partition development --workers 3 --output "$E/design-development-v2"`
passes **13/13**, with zero numerical or acceptance failures and no actuator
limiting. The original six development hovers span **.039632–.059166 m** hold
peak and **.023053–.025475 m** full-mission tracking RMSE. The three development
squares span **.026203–.026735 m** RMSE. Maximum attitude-estimation error over
all thirteen missions is **10.5007 degrees**.

The four explicitly observed diagnostic hovers have hold peaks .065987 m
(91001), .073130 m (91011), .071286 m (91016) and .065127 m (91019). Thus the
first frozen candidate's failed seed now clears the unchanged full-hold target,
and the largest development hold peak is .073130 m. These are development
results and are not counted as fresh held-out evidence.

The independent artifact audit detected three post-write digest mismatches in
the development directory, including one empty file. Qualification paused before
any new held-out seed was used. The damaged bytes and original failed audit are
retained separately. The three affected trials were regenerated from the same
source and configuration; **all 21 regenerated archive parts** and every metric
matched the original recorded values exactly. Only the three damaged files were
restored, using their original SHA-256 values as acceptance conditions. The report,
metrics and expected hashes were not edited. All **95 development NPZ files**
then passed digest and ZIP CRC verification. The cause of the post-write damage
is unverified; no numerical-source defect is inferred from it.

A broader **472-file digest scan** also found one empty covariance chunk in the
earlier baseline reproduction copy. Its original fixed-campaign archive was
intact and matched the unchanged expected digest; the copy was restored from
those authenticated original bytes. `design-original-reproduction-exact-recovery.json`
and the separately retained empty file record that recovery. All version-1
design archives and all restored version-2 prerequisites matched their hashes.

`design-v2-development-exact-recovery.json` records the exact reproduction and
retained damage. The held-out campaign uses the unchanged maintained CLI in a
private output directory, checks every completed archive, then publishes the
complete directory under its final name. Its execution receipt records the
actual command and source hash. This is an evidence-publication measure, not a
change to simulated dynamics, random realizations or scoring.

The repeated independent physics audit passes all **13** development pairs.
Together with the five fixed pairs, all **121 NPZ files** match their recorded
SHA-256 values and pass ZIP CRC checks. The complete execution, test and toolchain
freeze was recorded at **16:52:21 UTC**, after these prerequisites and before
the first version-2 held-out trial.

| Version-2 partition | Protocol SHA-256 |
| --- | --- |
| Fixed | `08fed2f9bded2462ae06991333e4f1449376a8dcbb77262d7c83e2f8db6864ec` |
| Development | `e55ed6f3cef35f4e9baa1e062c489661f558393be7c83d580ba8c4d719a7b7fb` |
| Validation | `7c4800d34d12b89d4476cbd9ca46d9e39df8ac2900789c148f37ca1ec76b7230` |

### Second frozen validation outcome: not qualified

The version-2 command used `--design-version 2 --partition validation --workers 8`
and completed at **17:10:36 UTC** with **28/30 acceptance passes**, zero numerical
failures and exit status 1. The two failed hovers are seed **93003**, with
**.0914026857 m** hold error at **7.4125 s**, and seed **93012**, with
**.0886861791 m** at **5.5200 s**. Both complete without limiting; all attitude
criteria and all ten square missions pass. After 10 s those two hold errors stay
below .038970 and .037034 m, respectively. Their complete records remain failures
against the unchanged full 5–65 s criterion.

The separate physical audit passes all 30 saved pairs. All **331 version-2 NPZ
files** pass SHA-256 and ZIP CRC checks; execution, tests and toolchain still match
the 16:52:21 freeze. Report SHA-256 is
`86d2c651b31eaf7a4afada0c0987a16008d05c6dd2be7ce10f13fbec51cadf90`.
The source is numerically reproducible, but this profile is **not qualified**.
Its demonstrated startup response remains insufficient across the fresh batch.
Further design work must retain this batch as observed evidence and use a new
held-out seed set.

## Further development audit: no qualifying replacement

All further probes used already observed seeds and the same 5 s hold entry.
They shortened the hold to inspect 5–10 s startup behavior and therefore are
not full 60-second hover qualifications. Their strict development condition
required <.065 m for 30, 91001, 93003 and 93012, <.08 m for the other ten
diagnostic identities, no limiting and no abort. The conditions were not
retroactively changed to turn a failed probe into a pass.

| Development prototype | Seed-93012 startup peak (m) | Result |
| --- | ---: | --- |
| Complex pole family, b=8 | .0844626 | Above inherited .08 m target; limiting present |
| Bounded integral, .025*(s+20/3)^6 | .1114516 | Above target; integral acceleration reaches its .5 m/s² bound |
| Complex pole family, b=10 | .0822047 | Above target; longest actuator limiting across 14 probes .05 s |
| Complex pole family, b=12 | .0826201 | Above target; longest actuator limiting .10 s |
| Complex pole family, b=14 | .0913843 | Above target; 91016 and 8200 also exceed .08 m; longest limiting .15 s |

The b=10/12/14 diagnostics additionally report the original allowance for brief
limiting, whose longest consecutive interval must be <=.5 s. Even under that
inherited condition, the position target fails. Their sampled local minimum
damping ratios are .51408, .44339 and .38175. Increasing stiffness further trades
away damping and raises control effort without resolving the observed case.
The integral hook existed only in its external development process. No extra
controller state, production profile or evidence schema was introduced.

The corrected recorded-error screen uses the actual pitch/roll inertias
.025/.02 kg m² and includes PD and causal bounded-integral feedback. None of
its 129 stable entries meets .08 m while keeping requested rates and moments
inside their bounds. Its version-2 reconstruction differs from the recorded
93003/93012 horizontal response by at most .000366/.000516 m over 0–10 s.
This supports the local forcing diagnosis but is neither a global impossibility
proof nor a substitute for new nonlinear estimated-feedback simulation. The
first screen's use of .02 kg m² on both axes understated pitch moments; that
output is retained with this limitation. Its position predictions are unaffected.

The later diagnostic exposed a real publication defect: for a hover ending
before 65 s, a short-circuit condition returned a NumPy boolean that canonical
JSON could not encode. Two early-abort regressions reproduce this issue; an
explicit `bool` conversion preserves both the failed result and its serializable
record. Successful-case scoring and all thresholds are unchanged. Initial
incomplete diagnostic JSON outputs and error logs are retained; the complete
42-case diagnostic was rerun with explicit scalar conversion in its external
serializer. Incomplete outputs are not counted as successful evidence.

After the scoring correction, the draft's full pinned
`PYTEST_ADDOPTS='-W error' make check` passes **3,063 tests in 236.09 s**,
Ruff lint, formatting of 160 files and mypy over 49 source files. The focused
abort/scoring/design regressions pass **53 tests in 49.20 s**. These are software
verification outcomes, not acceptance of either failed frozen profile. The
isolated correction on main's source passes its own **2,861-test** gate.

The reserved 95000/96000 validation seeds were not opened. The immediate technical
requirement is a separately scoped startup estimation/control design with an
explicit error and control-effort budget, followed by observed-case regressions,
complete fixed/development missions and another frozen fresh campaign. The
unqualified integration remains a draft. The isolated reporting correction was
merged through PR #11 as `f45f610776ec1c6877e896d7f63ec17c87f1d322`, after successful
GitHub CI run 36037670598 and exact-tree verification. Its merged tree is identical
to the locally tested correction. The two abort regressions also pass on that
clean merged checkout. Its main-branch CI is checked separately in the publication
receipt; this merge does not include the unqualified feedback integration.

## Final CI audit: backend-dependent input roundoff

GitHub CI run **36038666466** at `23d950e1f43d2b5306450b66a8400380e49a68bf`
reported **3,062 passes and one failure**, in the test that assumed an identical
golden protocol digest across numerical backends. The failed log is retained.
Selecting the HASWELL OpenBLAS kernel locally reproduces the exact CI digest
`de28fa6bedf6986df434a515181c954a28f5dc54f7bcd229ffb8ced90c338f5b`.
For that version-1 validation protocol, only seed 91010's initial quaternion
y component changes, from -.011563447064241896 to -.011563447064241898.
The population draw and all non-quaternion fields are identical.

The seven-selection audit checks the legacy protocol and both design versions.
Every difference is confined to derived initial-quaternion components. The
generation environment reports NumPy 2.5.2 and OpenBLAS 0.3.34.0.0 with the
SkylakeX kernel; the full backend record is retained separately. The published
raw protocol hashes describe that numerical realization. They were not edited
to match another machine.

The corrected test uses **87 frozen initial-attitude inputs** extracted from the
authenticated recorded protocols. Only those derived components may differ by
up to four float64 units before the original golden digest is checked. The
helper is confined to tests; no runtime protocol, simulation, RNG, filter,
scorer or artifact validator changes. New negative tests reject a five-unit
attitude change, preserve sensitivity to a one-unit position change, and prove
that the production validator still rejects even a one-unit attitude alteration.
The [reproduction guide](../feedback-design.md) states the exact-backend boundary.

Ten focused identity checks pass under the default, HASWELL and PRESCOTT kernel
selections. The first saved HASWELL log was found truncated despite its recorded
zero subprocess exit status; it is retained as incomplete. An independent repeat
passes **10 tests in 1.04 s** and writes a complete authenticated log. No truncated
log is presented as a complete gate transcript.

The revised full pinned `PYTEST_ADDOPTS='-W error' make check` passes
**3,071 tests in 213.92 s**, Ruff lint, format verification of 163 files and mypy
over 49 source files. The execution-source digest remains
`000bb213a7e14cefe2ea6a6a631ee8d2cf13ac15f08d5a3e869548a89d94067b`;
the portability correction changes only tests, their input fixture and documentation.
Fresh CI on that exact committed tree is recorded in the final publication receipt.
These source-check results do not close the unchanged hover-performance gate.

## Limits and next scope

This is a matched-model numerical profile with a known bounded initialization
population and a separately labeled truth safety oracle. It is not automatic
alignment, indefinite hover heading observability, universal consistency,
hardware flight readiness, or robustness to arbitrary mismatch/faults. Local
controller poles do not prove stability of the full nonlinear stochastic system.

After qualification, the next planned technical package is the minimum-snap
trajectory formulation: normalized-time polynomial derivatives, exact snap cost,
equality constraints, a checked linear-system solve and continuity/optimality
tests. It begins with an audit of this milestone. Geometric control, fault-aware
mission handling, ROS 2/C++ and PX4 integration remain separate later packages.
