# Supported-start geometric comparison — 2026-09-29

## Baseline audit and frozen scope

I audited merged PR #40 at `b855329d9d83f7c211883fcd24e390bdb2977218`.
The clean local main matched the remote head. That milestone changed only
documentation/reporting; existing geometric shaping, pre-arm alignment, supported
velocity conditioning and nonlinear release remained separately implemented.
Fresh warning-strict checks passed 102 relevant component tests.

The user-directed scope superseded middleware-interface design with one final
bounded comparison before pausing controller improvement. The professor-facing
report remains a later task. [ADR 0041](../../decisions/0041-supported-geometric-comparison.md)
was written before implementation and scientific execution, with SHA-256
`7e3559843bbceecc47e4785e04967622f0f69c4c8e32660672266431573e26ef`.

## Implemented experiment

I composed the existing coherent-force geometric controller at 10 rad/s with
the supported alignment, exact one-time velocity prior and nonlinear release
predictor. All later prediction, gains, reference timings, sensor distributions
and correction policies stayed fixed. There is no additional sensor, derivative
variant, integral compensation or truth-fed control input.

The three clean arms share initial truth, stationary support, motors initially
off and named random streams. The unaligned geometric control isolates the
complete startup-estimation package under matched physics. The combined cascade
uses the same initialization and original gains for a fair controller comparison.
It is not the differently tuned v2 cascade in the older hover campaign.

The new runner saves each complete flight before auditing/scoring. I extended
the existing cascade auditor to reconstruct geometric force, reference jets,
projected thrust, moments, allocation and domain aborts from recorded posterior
estimates and measured gyro feedback. Complete ESKF states, covariance, events,
conditional sample memory, supervision, nonlinear plant/motors, actual moments,
support ownership and original sensor draws are also reconstructed. Every combined
arm checks order-five first-release moments against order seven with the previous
mathematical accuracy limits. Context patches restore on normal and exceptional
exit and remain confined to separate worker processes.

The new source fingerprint at execution is
`8bad0a12f6d4062809ecd7ccf5380713ba8b2b48bde3e5e96f6cc4c8d63c3275`.
The frozen ledger contains 28 flights: 12 matched clean runs and 16 candidate
supervision off/on runs covering all eight existing fault patterns. Complete
5..65 s hover windows are retained; the shorter supported-start campaign is not
substituted for this test.

## Verification before scientific execution

The combined smoke paths reconstruct all three clean arms and an off/on fault
pair. An initial test assertion incorrectly required more than 100 samples even
after a correct early supervised abort. I replaced it with the exact recorded
duration/grid sample count. No model, controller or acceptance threshold changed.
Typing and formatting findings were corrected before scientific execution.

Commands from the repository root:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q \
  tests/unit/test_supported_geometric_comparison.py \
  tests/unit/test_robustness_validation.py tests/unit/test_supported_start.py \
  tests/unit/test_supported_repeatability.py tests/unit/test_combined_supported_prior.py \
  tests/unit/test_geometric_transient_study.py tests/unit/test_geometric_missions.py
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
```

The targeted suite passes 103 tests in 53.50 s; lint, formatting and all 102 typed
source files pass. The [experiment guide](../../results/supported-geometric.md) supplies the
scientific execution and saved-only verification commands.

The complete software regression command was:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q
```

It passes 3,965 tests in 380.33 s. These checks verify software behavior and
evidence handling; the separately counted scientific flights decide performance.

## Development hover results

All six hover-arm flights complete at 69.5 s without limiting or saturation.
Every inclusive 5..65 s hover sample is scored. The unaligned controls fail
the 8 cm hover limit; both combined geometric and combined cascade arms pass.

| Seed / arm | Full-flight RMSE [cm] | Full-hold peak [cm] | Actual moment effort [N² m² s] |
| --- | ---: | ---: | ---: |
| 30 / unaligned geometric | 4.773335 | 8.728383 | 0.0014187730 |
| 30 / combined geometric | 3.476155 | 5.210059 | 0.0001117724 |
| 30 / combined cascade | 3.215911 | 5.231308 | 0.0001302097 |
| 93012 / unaligned geometric | 5.428953 | 18.100977 | 0.0018279195 |
| 93012 / combined geometric | 3.136953 | 4.624977 | 0.0001148145 |
| 93012 / combined cascade | 2.850588 | 4.760973 | 0.0001115837 |

The worst combined geometric peak is 5.210059 cm, versus 18.100977 cm for
the matched unaligned geometric control. This is a measured benefit of the
complete initialization package under stationary supported physics, not proof
that any one of its three mechanisms accounts for the whole change.

Against combined cascade, geometric hover RMSE ratios are 1.080924 and 1.100459;
effort ratios are 0.858403 and 1.028954. Thus geometric average hover error is
slightly worse despite slightly better peaks. The predeclared hover gate permits
that tradeoff within the absolute limits; it was not relaxed after these results.

## Development trajectory results and stopping decision

All six spline-arm flights complete at 25.76 s without limiting or saturation,
and pass every absolute physical/accuracy requirement.

| Case / arm | Full-flight RMSE [cm] | Full-flight peak [cm] | Final error [cm] | Actual moment effort [N² m² s] |
| --- | ---: | ---: | ---: | ---: |
| Nominal / unaligned geometric | 6.796272 | 20.736819 | 1.746541 | 0.0009576642 |
| Nominal / combined geometric | 4.115024 | 15.594879 | 1.567999 | 0.0001338263 |
| Nominal / combined cascade | 5.082367 | 9.022562 | 1.575068 | 0.0001590920 |
| Wind / unaligned geometric | 8.108337 | 19.591186 | 6.310509 | 0.0006798280 |
| Wind / combined geometric | 6.873877 | 16.860082 | 4.712197 | 0.0000978063 |
| Wind / combined cascade | 6.626774 | 12.121962 | 4.781774 | 0.0001064661 |

The candidate's nominal/cascade RMSE ratio is 0.809667 and its effort ratio is
0.841188. In wind those ratios are 1.037289 and 0.918661. Wind therefore fails
the predeclared requirement to match or beat cascade RMSE. The difference is
0.247103 cm (2.471031 mm) in full-flight RMSE; both absolute RMSE values remain
well below 15 cm. The comparison still fails, as declared prospectively.

All four candidate clean flights meet their absolute requirements and all four
effort comparisons pass; three of four clean comparisons pass. Geometric spline
peak errors remain larger than cascade's even though nominal RMSE is better.
The complete initialization package improves the matched geometric baseline,
but this evidence does not establish across-metric superiority over cascade.

The failed wind gate ends development without opening the 40 reserved fresh
validation flights. All already planned development fault pairs are retained
and completed; their response results are distinct from this clean-flight gate.
No gain, filter, prior, noise, seed or acceptance threshold is changed in response.

## Complete fault ledger and evidence closure

All eight fault-response comparisons pass. Every unsupervised arm completes at
17.5 s. Supervised persistent position dropout, rejection and delay abort at
6.8025 s; the corresponding altitude faults abort at 6.245 s. Landing position
dropout aborts at 14.005 s. Brief position recovery completes at 17.5 s in both
arms. Each required response is reconstructed at its first expired epoch, with
identical causal prefixes and no command at or after the terminal sample.
These are numerical supervisor checks, not physical emergency-flight evidence.

The complete report retains all 28 scientific outcomes and all 28 saved audits.
All 24 combined first-release order-five/order-seven comparisons pass: maximum
normalized covariance error is `1.3202571976719731e-08` against `0.001`, maximum
standardized mean error is `1.407815611986719e-15` against `1e-6`, and measured
attitude-mean difference is zero against `1e-8` rad. Execution source and the
frozen decision bytes remain unchanged throughout the campaign.

A separate saved-array calculation authenticates all 194 referenced payloads
and independently rescores the 12 clean arm flights and paired ratios. Explicit
trapezoid sums and vector dot products recover the runner's metrics to a maximum
absolute difference of `1.3877787807814457e-17` in their SI units. It independently
checks full-window coverage and the absolute physical conditions.

| Evidence | SHA-256 |
| --- | --- |
| Complete development report | `fa93de167c3b788e711655023906bf056c0a26b9caf91820eb8511b4ac4aa0e3` |
| Independent saved-array scores | `2521cfacd0ef9943ccd1992cc2f5efcdbfc4e44ef91df55abca8c0d2152c14b3` |

The evidence is split into two ordinary ZIP archives; extracting both into the
same directory restores the complete `development` tree. Both include identical
root report/protocol metadata, an authenticated manifest, independent scores,
execution/software logs and reproduction instructions. No repository source or
generated scientific payload is committed to Git.

| Archive | Bytes | SHA-256 |
| --- | ---: | --- |
| `Quadrotor_Geometric_Hover_Evidence_2026-09-30.zip` | 450233179 | `430cb4f1fab4cec9823f0877aeec90aa7e53310d6526f6769ca932e1ae285fdb` |
| `Quadrotor_Geometric_Tracking_Fault_Evidence_2026-09-30.zip` | 271438322 | `a26f652800c9b710174d7551cfda9febe7816aae12e9f1b99d894ae82245895f` |

Archive CRC checks pass with 97 and 171 entries respectively and no duplicate
member names. Final documentation validation checks 156 documents, 707 local
links and 23 Python/JSON examples; lint, formatting, strict typing and
`git diff --check` pass. Production `src`, dependencies and the lockfile remain
byte-identical to the audited baseline.

Scientific execution finished on 2026-09-30 UTC. The command exits 1 for its
preserved failed comparison, not an execution or replay error. The ledger is
complete: 3/4 clean comparisons and 8/8 fault comparisons pass, the worst hover
improves, every audit passes, and 0/40 conditional fresh-validation flights run.

## Decision and next action

The implementation/evidence step meets its scope; the candidate fails the frozen
performance acceptance. Keep it as an explicit reproducible research option and
retain the cascade default. The full-hover improvement is real under the modeled
support assumptions; this is not a conclusion that geometric control is incapable
of useful performance or that no future design can improve it.

Pause controller improvement now. In a later task, prepare the professor-facing
report from the published source and these complete records, explaining the
hover and nominal-tracking improvements alongside wind, peak-error and average-
hover tradeoffs. Existing report revision 3.0 is unchanged by this experiment.
No new tuning, sensor, integration, report revision or message to the professor
is part of this closeout.
