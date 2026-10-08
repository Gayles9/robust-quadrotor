# Results and validation

This project separates correct numerical implementation from demonstrated flight
performance. Analytical tests, estimator calibration, true-state control and
noisy closed-loop flight provide different kinds of evidence. The
[capability and requirement ledger](operating-envelope.md) connects the current
claims to source, assumptions and retained failures.

## Main control result

The [final geometric comparison](final-geometric.md) evaluates independently
selected geometric and cascade profiles on twelve reserved clean cases: four
hovers, four nominal trajectories and four mild-wind trajectories. Both selected
controllers pass every clean absolute condition. The original cascade remains
the default.

| Metric | Geometric change versus selected cascade |
| --- | ---: |
| Balanced whole-flight RMSE | 4.7717% lower |
| Nominal trajectory RMSE | 13.7942% lower |
| Mild-wind trajectory RMSE | 1.4117% lower |
| Hover whole-flight RMSE | 1.6100% higher |
| Balanced whole-flight peak error | 15.2880% higher |
| Balanced squared-moment effort | 1.6736% higher |

The balanced metric weights the three categories equally using log ratios. The
4.7717% improvement misses the predeclared 10% target. Eight fault-response
comparisons pass. These are bounded results under the declared simulation and
supported-start assumptions; they do not establish general superiority or
hardware qualification. Squared-moment effort is a control-effort proxy, not
battery energy. [Controller tradeoffs](controller-tradeoffs.md) explains the
mechanisms and why better average tracking can coexist with larger peaks.

## Read the evidence at the right level

| Evidence | What it supports | Where to inspect it |
| --- | --- | --- |
| Analytical and numerical tests | Equations, interfaces, deterministic behavior and rejection contracts | [Tests](../../tests/unit), [foundations](../design/foundations.md) |
| True-state flights | Controller behavior when the simulated state is available | [Position control](../design/position-control.md), [trajectory missions](../design/trajectory-missions.md) |
| Sensor replay and ESKF calibration | Estimator consistency within declared motion, prior and noise assumptions | [Estimation](../design/estimation.md) |
| Noisy estimated-feedback flights | Closed-loop behavior under the exact evaluated scenarios | [Final comparison](final-geometric.md), [integrated robustness](integrated-robustness.md) |
| Saved reports and full histories | Authentication, independent rescoring and replay of recorded executions | Each study's reproduction section and [retained records](../archive/records/README.md) |
| Consolidated technical write-up | Implemented methods, evaluation and reported results | [Official report guide](../report.md) |

The endpoint estimator's nominal study reports 95.29% central-95% NEES coverage.
NEES compares estimation error with the filter's predicted uncertainty. The 380
replays cover distinct nominal and fault groups; that coverage number is not a
pooled statistic across all of them and does not qualify closed-loop flight.

## Supporting studies

| Question | Study |
| --- | --- |
| What happens during observation loss and model mismatch? | [Integrated robustness](integrated-robustness.md), [bounded vertical compensation](vertical-compensation.md) |
| What causes the original mass and hover failures? | [Early-flight diagnosis](early-flight-diagnosis.md), [attitude startup audit](attitude-startup-audit.md) |
| What does stationary support improve? | [Supported-start flight](supported-start-flight.md), [independent repeatability](independent-supported-start.md) |
| Why does residual hover error remain? | [Residual diagnosis](residual-supported-hover.md), [navigation-feedback isolation](navigation-feedback-isolation.md) |
| What changes when support is removed? | [Velocity-prior screen](supported-velocity-prior.md), [nonlinear release](nonlinear-release.md), [combined prior](combined-supported-prior.md) |
| How do estimation channels interact? | [Combined-prior diagnosis](combined-prior-diagnosis.md), [whole-flight error budget](whole-flight-error-budget.md) |
| Would an additional inclination measurement close the gap? | [Independent-inclination feasibility](independent-inclination-feasibility.md) |
| What preceded the final controller comparison? | [Supported geometric study](supported-geometric.md) |

Earlier failures retain their original scope and acceptance criteria. A later
study does not retroactively turn them into passes. Test counts in the archived
records describe those recorded baselines, not a fresh check of your checkout.

## Reproduce and inspect

Start with the [small planning example](../guides/getting-started.md#install-and-run-the-first-example)
and fixed true-state flight set. Larger campaigns often require authenticated
parent artifacts and staged execution. Follow the relevant study's reproduction
section before running its module.

Generated histories, reports and plots go under the ignored root `results/`.
The repository retains study explanations, exact recorded metrics, protocols and
small test fixtures. Historical raw campaign archives named in records are
separate artifacts and are not included in a clone; a filename or hash alone is
not a download link. See [development practices](../development.md) for source
fingerprints, numerical-environment limits and independent rescoring.
