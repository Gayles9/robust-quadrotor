# 2026-09-24: Estimated-feedback design and qualification

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

The selected profile matches coefficients of
`.025*(s+3)^2*(s+6)*(s²+28s+392)` and uses the declared initial uniform population's
exact second moments for P0. The prior mean and true population are unchanged;
P0 is not computed from a trial's realized state. Sensor and process noise, gates,
clocks, references, scoring window and thresholds remain unchanged. Development
misses and the design refinement are explicitly retained in ADR 0014.

The only new mathematical-core module is `cascade_analysis.py`, a bounded local
analysis utility. The runtime controllers remain pure functions of explicit
parameters; the selected profile is provided by the new experiment's configuration
builder. The original experiment and its measured failure remain reproducible.

## Verification record

All commands use Python 3.12.14 and the unchanged `uv.lock` with uv 0.12.3.
`E` below denotes the external evidence directory; outputs are never overwritten
or committed. The audited parent commit is recorded above; the exact execution
SHA-256 for the new profile is
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
