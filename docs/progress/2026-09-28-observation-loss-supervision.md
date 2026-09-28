# Observation loss supervision — 2026-09-28

## Preceding milestone audit

Starting commit: `d01e440594c4a588f78291455242618e6db5e41b`, tree
`0b2530047af914dae4a04cac57ef9c4be48b3a9d`. The workspace was clean and matched
GitHub main. The preceding monitor merge's
[CI run](https://github.com/Gayles9/robust-quadrotor/actions/runs/36419386080)
completed successfully.

I audited the health state machine, delivered-event contract, estimated-mission
hook, mission phase/guard ordering and existing artifact schema. Fresh checks:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run pytest -q tests/unit/test_observation_health.py tests/unit/test_observation_health_mission.py tests/unit/test_missions.py
```

Result: **103 passed in 21.77 s**. No preceding in-scope defect needed repair.
Python 3.12.14, pinned uv 0.12.3 and the existing lock were used throughout;
versions and dependencies are unchanged.

## Scope and implementation

[ADR 0022](../decisions/0022-observation-loss-supervision.md) froze the policy
and acceptance criteria before implementation. Position is required during
every active phase because the controller also runs during initialization
hold. Altitude has an independent, explicitly optional requirement. Required
fusion cannot be disabled. Positive caller-supplied timeouts are separate from
health warning/loss ages; there is no default safe-flight deadline.

WAITING spends its budget from startup. Once health has been established,
DEGRADED, LOST and RECOVERING share a continuous unhealthy-time budget. Only
confirmed health before expiry clears it. Expiry is checked before recovery,
so an acceptance arriving at the deadline cannot conceal the expired interval.
Abort is latched, with deterministic position-first observation ties.

The supervisor binds to the monitor and consumes each epoch exactly once. It
accepts no truth, estimate, measurements, commands or fault labels. Its immutable
decision history remains external to the existing mission/evidence schema.
The optional mission hook records a decision after each health update, before
mission transitions and commands. Truth/estimate safety guards retain priority;
observation abort precedes completion and retains the terminal epoch without
issuing a terminal command. A virtual landing still needs usable feedback, so
this action is numerical termination, not a claimed safe hardware maneuver.

## Focused verification and results

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run pytest -q tests/unit/test_observation_supervision.py tests/unit/test_observation_supervision_mission.py
```

Result: **65 passed in 63.51 s**: 40 component cases and 25 integration cases.
The component checks initially also passed alone, 40 tests in 7.05 s. Strict
mypy found no issues in 68 source/experiment/script files. Initial lint found
one overlong test name; shortening it did not change behavior.

| Check | Protocol and observed result |
| --- | --- |
| Real estimator faults | Stationary ideal IMU, seed-31 noiseless prior, h=1/64 s, position period 4h, altitude 2h; explicit source-index dropouts, 100 m offsets and one-epoch delays feed the real online ESKF before health/supervision |
| Independent persistent loss | Six combinations of sensor and fault type abort exactly 0.25 s after detected degradation; the unaffected stream stays healthy |
| Outlier and recovery | One actual rejected outlier per stream causes no abort; two accepted recovery observations clear the timer only before expiry; partial recovery and a new rejection keep the original timer |
| Causality and lifecycle | Identical delivered prefixes with different future fault labels give identical histories; skipped/duplicate epochs fail atomically; reset repeats decisions; required-disabled policies fail; optional altitude cannot veto position |
| Nominal integration | Six paired noisy configurations across cascade/geometric and nominal/optional-disabled/optional-delayed altitude preserve every original numerical/event payload byte and saved archive digest |
| Fault integration | Eight paired noisy cases across both controllers and independent delayed/rejected position/altitude streams abort at the explicit startup deadlines: position 0.08 s, altitude 0.06 s; command/state prefixes match the unsupervised comparator |
| Phase and guard ordering | Deadlines at 0.1/0.2/0.3 s survive phase boundaries; observation expiry beats same-epoch completion; truth and estimate guards retain higher priority; no terminal command is issued |
| Evidence | All eight fault histories authenticate and reload; delivered events reconstruct every health snapshot and supervisor decision; deliberately corrupted payloads are refused |

The integration fixtures use 0.1 s stationary HOLD segments bearing the four
mission phase labels, not physical takeoff or tracking maneuvers. Position is
sampled every 0.02 s and altitude every 0.01 s, on the 0.0025 s plant clock.
Declared delay is 0.013 s for position or 0.03 s for altitude; the selected
rejection gate is 1e-15 for one stream, leaving the other's original gate.
Health counts are warning=2, loss=4, rejection=3, recovery=2, evidence-window=5.
Faulted integration streams never establish accepted health, so 60/80 ms are
**startup budgets**, not measured bounds after an arbitrary in-flight dropout.
The component protocol separately tests loss and recovery after acceptance.

Generated test archives stay outside Git. Every run retains comparator results;
no acceptance threshold, estimator model or controller was retuned. The reserved
geometric qualification seeds remain unopened.

A final evidence review added saved complete unsupervised comparator histories
alongside the supervised histories, and checked both reloads. The affected
checks passed **8 tests, 17 deselected, in 29.58 s**:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run pytest -q tests/unit/test_observation_supervision_mission.py -k fault_abort_timing
```

The four unsupervised position-fault comparators reach virtual completion at
0.41000000000000003 s; the four unsupervised altitude-fault comparators retain
`landing_timeout` at 0.44 s. All eight supervised cases terminate at the earlier
declared observation deadline. These are correct planned aborts, not eight
successful flight completions. Comparing peak errors over these different
horizons would not demonstrate improved tracking.

Project instructions now require each completed step to report the quality and
limits of its results, whether acceptance was met, and the next concrete action.

## Complete gate

With pinned uv 0.12.3 first on PATH:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
```

The complete gate passed **3,466 tests in 712.48 s**. Ruff lint and formatting
passed for 232 Python files, strict mypy passed for 68 source/experiment/script
files, and documentation checks passed for 95 Markdown files, 387 local links
and 22 Python/JSON examples. The final comparator-archive assertions were also
verified separately with the eight affected checks above. `git diff --check`
is clean. No dependency, tool version, estimator or controller implementation
changed. In existing source, only the two mission execution functions changed;
the mission and evidence dataclasses retain their exact definitions.

Publication uses a tree matching the local reviewed files, with GitHub CI
required to pass on the pull request before merge. The merged tree must match
that tested tree. The pull request and its checks preserve the hosted result.

## Assessment and next action

This bounded step **meets its acceptance criteria**: the focused cases support
the supervisor interface and its timing/command semantics, and the complete
software gate passes. They do not close the existing estimated-hover gap,
qualify geometric control, or establish a safe physical fallback. The cascade
remains the default and geometric control remains experimental for noisy
estimated feedback.

Next: audit this step and freeze an integrated robustness campaign across actual
hover/tracking/virtual-landing maneuvers, causal live observation faults and
declared model mismatch. Use unchanged controllers, supervised/unsupervised
comparators, explicit pass limits and authenticated histories; retain failures.
