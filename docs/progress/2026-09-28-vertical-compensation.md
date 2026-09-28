# Bounded vertical disturbance-compensation study — 2026-09-28

The engineering step is complete; the single candidate is **rejected**. All
24 executions and reconstructions succeeded, all 12 response cases passed,
and 10 of 12 candidate comparisons passed. Mass landing improves substantially,
but its tracking RMSE still fails; hover also fails its no-regression condition.
The original cascade remains the default.

## Audit and frozen scope

Starting main: `0f1702f080690b572bafee819deb6522acaadf25`, tree
`d1814de6b03cfa3bd8ff930ea00609737bbe7372`. Its post-merge
[CI](https://github.com/Gayles9/robust-quadrotor/actions/runs/36434948375) passed.
The workspace was clean and matched GitHub. Fresh checks passed **163 tests in
62.80 s**:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run pytest -q tests/unit/test_eskf_live_faults.py tests/unit/test_robustness_validation.py tests/unit/test_position_control.py
```

The saved mass pair was authenticated and independently reconstructed with
`experiments.robustness_validation.audit_case`; every saved metric matched.
No preceding in-scope defect required repair. The immutable baseline report
SHA-256 is `ec85a0d6b55acc278acb18b1c7af7b02cb56c6e985dc48eaf66fbc7fd520bb09`.

| Baseline diagnostic | Measured value |
| --- | ---: |
| Mean vertical error, 17..25 s | 0.436975 m |
| PD equilibrium prediction | 0.436000 m |
| Mean estimated-minus-true vertical position, 23..25 s | 0.001055 m |
| Outer / inner limit flags | 0 / 0 |
| Commanded thrust range | 9.810000..11.317405 N |
| Actual rotor-speed range | 493.050348..540.308700 rad/s |

This supports a steady-force-deficit diagnosis. It does not imply that
estimation and actuator transients can be ignored in a changed flight.
The earlier horizontal bounded-integral probe was reviewed: it hit its
0.5 m/s² bound and failed the startup-hover condition. This new scope does not
relabel that rejected prototype as qualified.

[ADR 0024](../decisions/0024-bounded-vertical-compensation.md) freezes one
vertical-only candidate at gain 0.5 s^-3, bound ±1.5 m/s² and period 0.02 s.
Its canonical campaign protocol SHA-256 is
`6bab1e089730485310a9d69a3d49361618c73982663b75f2a9efe8fc9fa9635e`.
References, estimator, PD gains, response budgets, original flight limits and
seeds remain unchanged. No sweep or reserved qualification case is authorized.

## Implementation and initial checks

The [guide](../vertical-compensation.md) explains the scalar law, health gating,
conditional anti-windup and lifecycle. It is an explicit option for estimated
cascade missions; the original default and geometric controller stay intact.
Separate authenticated diagnostics retain every applied and prepared state.
The existing history and mission dataclasses are unchanged.

The initial component checks passed **39 tests in 5.80 s**, covering signs,
current/next-state timing, bounds, freeze/unwind/reset, atomic invalid calls,
default payload parity and preflight before random-stream creation. A short
four-execution smoke run passed both response comparisons and retained the
original stationary fixture's failed flight-completion result. Smoke never
qualifies the candidate.

The combined component, candidate-evidence and original robustness checks
passed **72 tests in 218.29 s**. They include changed protocol/source/metric
claims, altered authenticated integral traces, and a failed supervised run
whose successful unsupervised partner must remain available. A subsequent
independent sampled-model check passed **1 test in 1.46 s**: its exact held-input
linear motor plant is stable and rejects the constant load using the actual
compensator without giving that compensator true mass or disturbance inputs.

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run pytest -q tests/unit/test_vertical_compensation.py tests/unit/test_vertical_compensation_validation.py tests/unit/test_robustness_validation.py
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run pytest -q tests/unit/test_vertical_compensation.py -k sampled_lag_model
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.vertical_compensation_validation --partition smoke --workers 2 --baseline results/robustness-smoke --output results/vertical-smoke
```

Smoke exits 1 because it is explicitly not qualification. All four execution
histories and their reconstructions succeed; this is not a numerical failure.

## Complete software gate

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
```

The complete local gate passed **3,575 tests in 1164.64 s**, with warnings
treated as errors. Ruff lint and formatting passed for 248 Python files, and
strict mypy found no issues in 74 source, experiment and maintenance files.
This adds 52 tests to the preceding milestone. No dependency or tool version
changed: Python 3.12.14, NumPy 2.5.2 and pinned uv 0.12.3 were used.
The final documentation check passed for 101 Markdown files, 438 local links
and 23 Python/JSON examples; the final lint, format and whitespace checks passed.

The independent sampled plant check, default payload parity, causal health
gating, state bounds, invalid-call atomicity and saved-evidence tampering checks
support the implementation contract. Passing software checks does not override
a failed flight comparison.

## Complete campaign and decision

```bash
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.vertical_compensation_validation --baseline results/robustness-campaign --partition campaign --workers 2 --output results/vertical-candidate
```

The exact original baseline report and all its payloads authenticated before
comparison. The runner executed and independently reconstructed all 24 new
histories, including ESKF, observation health, mission decisions, every command
and every integral update. The report was written only after those checks.
There were **zero numerical/evidence failures**, **12/12 response passes**,
**10/12 candidate comparison passes**, and **3/5 required flight passes**.
Exit status 1 records the retained performance failure, not an execution error.

| Case | Supervised outcome [s] | Response | Candidate comparison |
| --- | --- | --- | --- |
| `nominal_hover` | COMPLETE at 15.5 | Pass | Fail |
| `nominal_tracking` | COMPLETE at 17.5 | Pass | Pass |
| `position_dropout` | ABORT at 6.8025 | Pass | Pass |
| `position_rejection` | ABORT at 6.8025 | Pass | Pass |
| `position_delay` | ABORT at 6.8025 | Pass | Pass |
| `altitude_dropout` | ABORT at 6.245 | Pass | Pass |
| `altitude_rejection` | ABORT at 6.245 | Pass | Pass |
| `altitude_delay` | ABORT at 6.245 | Pass | Pass |
| `position_recovery` | COMPLETE at 17.5 | Pass | Pass |
| `landing_position_dropout` | ABORT at 14.005 | Pass | Pass |
| `wind_tracking` | COMPLETE at 17.5 | Pass | Pass |
| `mass_tracking` | COMPLETE at 17.5 | Pass | Fail |

Persistent-fault rows require correct timed abort rather than flight completion.
All seven such stops occur at the original times, without a terminal command.
The five nonpersistent off/on pairs have identical complete flight payloads.
Recovery is confirmed at 6.6 s. The following flight metrics are the same in
both supervision modes:

| Flight case | Original RMSE [cm] | Candidate RMSE [cm] | Original final error [cm] | Candidate final error [cm] | Candidate flight gate |
| --- | ---: | ---: | ---: | ---: | --- |
| `nominal_hover` | 7.660495 | 7.666007 | 3.951813 | 3.977706 | Fail |
| `nominal_tracking` | 6.244457 | 6.249235 | 5.736318 | 5.741237 | Pass |
| `position_recovery` | 6.601302 | 6.606082 | 5.760791 | 5.765385 | Pass |
| `wind_tracking` | 6.717405 | 6.722552 | 8.021562 | 8.018492 | Pass |
| `mass_tracking` | 42.393311 | 17.568364 | 43.463425 | 6.023494 | Fail |

Whole-flight RMSE uses each complete saved mission. The original mass flight
lasts 25 s; the candidate completes at 17.5 s. At the common 17.5 s horizon,
original RMSE is 41.641083 cm versus candidate 17.568364 cm. Thus the improvement
is real under either comparison, but still misses the original 15 cm limit.
Nominal tracking, recovery and wind retain their passing gates despite small
RMSE increases of approximately 0.048, 0.048 and 0.051 mm respectively. Every
scalar delta, including all persistent-fault modes, is retained in the report.

The mass candidate reduces final position error from 43.463425 to 6.023494 cm
and final vertical error from 43.301945 to 1.513906 cm. Its learned acceleration
reaches -0.977543 m/s², close to the -0.981 m/s² load prediction. No integral
state reaches its bound. The 37.256841 cm peak occurs at 2.28 s; first-five-second
RMSE is 29.736082 cm, while last-five-second RMSE is 3.285672 cm. The terminal
correction addresses the steady load, but the earlier transient still consumes
too much of the whole-flight error budget. This is evidence for the measured
mechanism, not permission to retune or qualify it.

Hover peak changes from **10.756338 to 10.759618 cm**, an increase of
**0.032802 mm**. It is small but violates the frozen no-increase condition.
Both values also exceed the original 8 cm requirement. The original passing
hover completion, RMSE, final-position and final-speed conditions remain intact.
These two failed case comparisons reject the single candidate under ADR 0024.

All campaign integral traces stay within ±1.5 m/s² and reconstruct exactly.
Unhealthy periods freeze learning; the mass run has 10 frozen startup epochs
and 865 integrating epochs. No campaign integral clipping is observed; separate
component checks exercise bound clipping, saturation blocking and unwinding.
No truth mass, fault label, gain sweep, reserved seed or relaxed limit was used.

## Evidence identity and next step

Execution-source SHA-256:
`008470bec6b8b1137f7cb90db2760925e24329a475fcda5c2ef579b6618fceac`.
Candidate report SHA-256:
`46a506cc25881cb86a22fd7c4a3aeb85ff1f6e28244d51b492305922dae8b4bc`.
The run records its audited base commit and dirty working tree; the published
source contains that exact execution fingerprint. Tests and documentation are
versioned alongside it, while large generated histories remain outside Git.

The companion `quadrotor-vertical-compensation-2026-09-28.zip` retains all
candidate payloads, diagnostics, protocol, exact baseline report, comparison
figures, source identity and verification logs. Full original baseline payloads
remain in `quadrotor-integrated-robustness-2026-09-28.zip`, SHA-256
`139518b7d6356a1cdd2ba14fca73704af21f7d1c720a3701b7adcd39927241f4`.
Both archives are required for complete offline comparison verification.

The implementation/evaluation scope is completed successfully; the candidate
performance acceptance is not. Retain the option only for explicit research,
keep the original cascade default, and leave geometric, broad noisy-flight and
hardware qualification open. The next bounded task is the
[technical report and operating boundary](../next-steps.md#completed-technical-report-and-operating-boundary),
reconciling verified capabilities, failures and the later integration contract.
