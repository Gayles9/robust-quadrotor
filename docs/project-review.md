# Documentation and project review — 2026-09-26

I audited the merged project before reorganizing the documentation. The core
simulation, estimator, controllers and planning code remain unchanged. The main
engineering limitation is still estimated-state control: no current profile
meets every original hover condition, and the six geometric derivative/gain
candidates did not satisfy the combined tracking, hover and effort requirements.

## Scope and source

The starting point is merged `main` at
`29a37b017114b58810ea8f7535448934365e39bc`, tree
`8dfd2cc8579f125a2a6aa37c9b384872abc783b2`. All 214 tracked files were recovered
from that commit and checked against their Git blob hashes before editing.
The preceding geometric work is merged through
[PR #16](https://github.com/Gayles9/robust-quadrotor/pull/16); its
[post-merge CI](https://github.com/Gayles9/robust-quadrotor/actions/runs/36262451382)
passed.

This review covers the repository's Markdown, public interfaces, experiment
entry points, reproducibility setup, tests and current qualification claims.
It separates current guides from dated verification records. The earlier
standalone report and external raw flight archives are not part of this Git
tree. This documentation change does not produce a new flight qualification.

## Documentation findings and corrections

| Finding | Correction |
| --- | --- |
| The front page mixed setup, detailed mathematics and historical status | A short README now leads to system design, subsystem guides, controller tradeoffs and current results |
| Readers had to infer how the modules fit together | A system guide explains the flight loop, rates, frame signs, information boundaries and design choices |
| Detailed control results were difficult to interpret | A plain-language controller guide connects the metrics to estimator corrections, delay, startup and disturbance rejection |
| Planning guides described estimated spline execution as unsupported | The guides now describe the existing geometric path and explicit cascade opt-in, with qualification limits retained |
| Eleven interface descriptions and one function-body excerpt were labelled as standalone Python | They are now labelled as text; executable Python examples receive compilation checks |
| Some decisions and verification records were missing from their indexes | The indexes include every decision and dated record, grouped by subsystem |
| Historical next actions and publication states could read as current instructions | The documentation map explains their scope; the geometric audit has a merge addendum |
| Personal references and workstation paths appeared in project files | Names were removed and historical command paths made portable without changing numerical evidence |
| Documentation errors were outside the normal quality gate | `make docs-check` checks local targets, heading anchors, fences and Python/JSON syntax; `make check` runs it in CI |

The [documentation map](README.md) links explanations directly to their source
modules and design decisions. The [environment guide](environment.md) describes
the locked setup and the difference between a planning example and a flight
campaign. No project dependency or tool version was changed for this review.

## What the project has completed

| Area | Implemented and checked | Remaining boundary |
| --- | --- | --- |
| Dynamics and actuation | NED/FRD rigid-body motion, rotor forces/moments, allocation, motor lag, wind/drag, Euler and projected RK4; equilibrium, invariants and convergence checks | Simplified simulated plant without contact, battery or identified hardware dynamics |
| Sensors and run evidence | Seeded IMU/position/altitude sensing, bias/noise, fixed-rate schedules, named random streams and validated save/load formats | Simulated sensor models; large experiment payloads are stored outside Git |
| Estimation | 15-state ESKF, endpoint propagation, position/altitude corrections, innovation diagnostics/gating, causal online operation and measurement-only replay | Validated within the declared sensor/model/fault cases, without a complete onboard health manager |
| Baseline control | Attitude/rate and position cascade, multirate mission execution, virtual takeoff/landing and bounded safety checks | Estimated-feedback hover qualification retains failures; the truth safety monitor is simulation-only |
| Planning | Fixed-duration minimum-snap curves, analytical derivatives, conservative nominal bounds, bounded retiming and mission adapters | Reference feasibility does not guarantee closed-loop tracking or obstacle avoidance |
| Geometric control | Force-to-attitude derivatives, rotation error moment law, causal feedback derivative filter and two explicit experimental derivative options | Strong tested true-state tracking; no accepted noisy-feedback replacement |
| Engineering workflow | Locked Python environment, deterministic fixtures, unit/integration tests, lint, formatting, strict typing and hosted CI | Passing a software gate does not establish hardware readiness or general flight robustness |

The [status page](status.md) gives the evidence for each area. In the preceding
controller audit, all 15 true-state regression executions passed. The six noisy
development profiles were rejected. Across that session, 42 executions and 202
saved payloads were independently authenticated and rescored. Those are retained
historical results, not fresh simulations performed by this documentation edit.

## Remaining weak points

1. **Coupled noisy feedback.** Estimator revisions affect the desired-force
   derivatives, while filtering, actuator lag and attitude response affect
   damping. Rebasing reduces one source of effort but worsens the tested hover
   response. A single gain or filter metric is insufficient to select a design.
2. **Startup and initial uncertainty.** Early attitude error can create real
   lateral motion before later measurements improve the estimate. The failing
   hover peak can persist after estimation error has become small.
3. **Persistent disturbances.** The present position law has no integral
   disturbance estimate. Integral action brings reset and anti-windup choices,
   and earlier bounded probes did not close the qualification gap.
4. **Evidence availability.** Seven payloads required authenticated recovery in
   the preceding audit. Their damage cause remains unknown. Fail-closed payload
   verification protects interpretation, but does not explain the storage event.
5. **Scope of validation.** The noisy-feedback requirements remain open. There
   is no custom-stack hardware validation, complete health/fallback policy,
   obstacle planning, or completed ROS 2/PX4 integration. The earlier integration
   compatibility exercise does not establish those capabilities.

The [controller guide](controller-tradeoffs.md) explains the measured tradeoffs
and identifies which mechanisms are observations and which remain hypotheses.
I have not weakened a threshold, relabelled a development case as fresh
validation, or promoted an unqualified experimental option.

## Verification of this change

The reproducible gate is:

```bash
uv sync --locked
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
```

The warning-strict gate passed **3,326 tests in 210.77 seconds**, including
11 new documentation-checker cases. Ruff lint and formatting passed, and strict
mypy found no issues in 63 source, experiment and maintenance-script files.
The documentation check passed for **87 Markdown files, 349 local links and
20 Python/JSON examples**. A separate Markdown parser found the same 349 local
links, and all 30 shell blocks passed `bash -n`. All 178 mathematical expressions
compiled with KaTeX without an error. All 87 pages rendered to HTML and all
three Mermaid diagrams rendered to SVG. The README, documentation map and new
design, controller and review pages were checked in a browser for readable
tables, diagrams and page layout.

I executed the README planning command and the complete Python examples in
the planning and geometric guides. The planning command produced its plot,
archive and metrics, with snap cost **99.44149495945453 m²/s⁷**, independent
quadrature **99.44149495945385 m²/s⁷**, and maximum waypoint residual
**2.5757174171303632e-14 m**. These reproduce the documented example within
roundoff. It is a polynomial calculation, not a new flight test.

All 129 baseline implementation, experiment, test, CI and environment files
checked against their Git blobs are unchanged. The execution source fingerprint
remains `cc4f982a40f90bc775c03703aa92007549a21f4509e296eb83743d63ef5aa9fb`.
The added scripts and tests maintain documentation only. Personal-reference and
home-directory scans are clear, both record indexes are complete, and
`git diff --check` passes.

## Next scope

The [next-step plan](next-steps.md) permits one coupled diagnosis of the known
startup/hover failures and at most one justified candidate. It requires the
unchanged development limits to pass before the original true-state regression
and any fresh qualification. If the diagnosis cannot support a credible change,
or that candidate fails, the controller study stops with its present limits
documented. Health-monitoring interface work would then be a separate scoped
task; middleware integration and hardware work remain later milestones.
