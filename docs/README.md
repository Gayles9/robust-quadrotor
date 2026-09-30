# Technical documentation

Start with [system design](system-design.md) for the complete flight loop. Then
read [controller problems and tradeoffs](controller-tradeoffs.md) for the main
unresolved engineering question. Each guide below moves from its purpose and
design choices to equations, interfaces and verification.

## Subsystem guides

| Topic | Explanation | Main implementation | Design record |
| --- | --- | --- | --- |
| Coordinate frames | [Frame contract](architecture/frame-contract.md) | [Rotations](../src/quadrotor_math/rotations.py) | [0002](decisions/0002-frame-conventions.md) |
| Plant, motors and sensors | [Foundations](foundations.md) | [Dynamics](../src/quadrotor_math/dynamics.py), [actuation](../src/quadrotor_math/actuation.py), [IMU](../src/quadrotor_math/imu.py) | [0003](decisions/0003-environmental-wind-and-drag.md) |
| Recorded runs and replay | [Run configuration and persistence](foundations.md#reproducible-run-configuration-and-named-random-streams) | [Generation](../src/quadrotor_math/run_generation.py), [artifacts](../src/quadrotor_math/run_artifact.py) | [Generation evidence](progress/2026-09-22-complete-run-generation.md) |
| State estimation | [ESKF guide](estimation.md) | [Prediction/correction](../src/quadrotor_math/eskf.py), [endpoint model](../src/quadrotor_math/eskf_endpoint.py) | [0004–0010](decisions/README.md#state-estimation) |
| Observation health | [Availability, rejection and recovery](observation-health.md) | [Passive monitor](../src/quadrotor_math/observation_health.py) | [0021](decisions/0021-observation-health-monitoring.md) |
| Observation supervision | [Required streams and timed abort](observation-supervision.md) | [Supervisor](../src/quadrotor_math/observation_supervision.py) | [0022](decisions/0022-observation-loss-supervision.md) |
| Integrated robustness | [Faulted flight and paired evidence](integrated-robustness.md) | [Live faults](../src/quadrotor_math/eskf_live_faults.py), [campaign](../experiments/robustness_validation.py) | [0023](decisions/0023-integrated-robustness-evaluation.md) |
| Vertical compensation | [Bounded experimental integral](vertical-compensation.md) | [Scalar law](../src/quadrotor_math/vertical_compensation.py), [comparison](../experiments/vertical_compensation_validation.py) | [0024](decisions/0024-bounded-vertical-compensation.md) |
| Attitude control | [Cascade inner loop](control.md) | [Attitude control](../src/quadrotor_math/attitude_control.py) | [0011](decisions/0011-baseline-attitude-control.md) |
| Position control and missions | [Position loop and supervisor](position-control.md) | [Position control](../src/quadrotor_math/position_control.py), [missions](../src/quadrotor_math/missions.py) | [0012](decisions/0012-position-control-and-missions.md) |
| Feedback from estimates | [Sensor-to-controller integration](estimated-feedback.md), [cascade design](feedback-design.md) | [Online ESKF](../src/quadrotor_math/eskf_online.py), [estimated missions](../src/quadrotor_math/estimated_mission.py) | [0013–0014](decisions/README.md#feedback-control) |
| Geometric control | [Force, attitude and derivative design](geometric-control.md) | [Moment law](../src/quadrotor_math/geometric_control.py), [reference](../src/quadrotor_math/geometric_reference.py), [filter](../src/quadrotor_math/geometric_filter.py) | [0017–0019](decisions/README.md#geometric-control) |
| Trajectory planning | [Minimum-snap solver](trajectories.md) | [Minimum snap](../src/quadrotor_math/minimum_snap.py) | [0015](decisions/0015-minimum-snap-trajectory.md) |
| Trajectory execution | [Bounds, retiming and missions](trajectory-missions.md) | [Feasibility](../src/quadrotor_math/trajectory_feasibility.py), [mission execution](../src/quadrotor_math/mission_simulation.py) | [0016](decisions/0016-trajectory-feasibility-and-missions.md) |

## Running and checking the project

The [setup guide](environment.md) covers installation, checks and numerical
reproducibility. Run experiment modules from the repository root with
`uv run python -m experiments.NAME --help` before choosing a campaign.

| Experiment | Purpose |
| --- | --- |
| `minimum_snap_example` | Small planning example with a plot and independently checked cost |
| `euler_rk4_convergence` | Numerical integration error and observed convergence |
| `eskf_consistency`, `eskf_validation` | Estimator accuracy, uncertainty, bias and observation-fault studies |
| `attitude_control_validation`, `position_control_validation` | Baseline recovery and complete virtual flights |
| `estimated_feedback_validation`, `feedback_bandwidth_validation` | Original estimated-feedback profiles, including retained hover failures |
| `trajectory_mission_validation` | Reference retiming and five true-state spline flights |
| `geometric_reimplementation_validation` | Original geometric true/estimated comparisons |
| `geometric_correction_validation` | Reproduce the six rejected derivative/gain profiles |
| `supported_geometric_comparison` | One fixed geometric/startup combination, matched cascade and full hover/fault evidence |
| `robustness_validation` | Frozen paired observation-fault and model-mismatch flights with complete evidence reconstruction |
| `vertical_compensation_validation` | One frozen vertical candidate compared with the authenticated original robustness campaign |
| `early_flight_diagnostic`, `early_flight_oracle` | Authenticated mass/hover decomposition and one attitude-only diagnostic intervention; never qualification |
| `attitude_startup_audit` | Offline startup propagation/update reconstruction, independent joint conditioning and bounded information audit |
| `prearm_alignment_feasibility` | Supported-stationary design, covariance and rejection feasibility; no production arming or flight integration |
| `prearm_nonlinear_uncertainty` | Nonlinear joint-moment derivation, independent calibration and prior-failure reconstruction |
| `prearm_component_validation` | Authenticated numerical equivalence and 200 complete supported acquisition/handoff sessions |
| `supported_start_validation` | Eight physically supported-release flights, paired alignment comparison and complete saved replay |
| `supported_repeatability` | Independent supported-start seeds and observation-fault responses with separate pairing and unchanged acceptance |
| `residual_hover_diagnostic` | Authenticated offline hover response accounting and unfitted sampled cascade model |
| `navigation_feedback_oracle` | Single supported-hover outer p/v isolation with complete saved replay and unchanged scoring |
| `kalman_sandbox` | Small standalone Kalman calculations |

The subsystem guides explain the supported arguments, expected outputs and
meaning of a failed performance gate. `plot_*` modules consume the corresponding
saved evidence. Diagnostic studies such as `feedback_codesign` and
`feedback_startup_diagnostic` remain available to reproduce earlier findings;
they are not additional production controller options. Previously inspected
seed sets are reproduction cases, not fresh validation.

## Status and evidence

- [Operating envelope](operating-envelope.md): source-indexed default/research capabilities, passing evidence and open requirements.
- [Current status](status.md): implemented capabilities and measured limitations.
- [Technical report](technical-report.md): consolidated LaTeX report, source/evidence identity and operating boundary.
- [Controller tradeoffs](controller-tradeoffs.md): interpretation of the results.
- [Supported geometric comparison](supported-geometric.md): frozen initialization/controller combination and conditional validation.
- [Early-flight diagnosis](early-flight-diagnosis.md): distinct mass and hover mechanisms, oracle limits and reproduction.
- [Attitude startup audit](attitude-startup-audit.md): first-position update explanation, information limits and justified no-go.
- [Stationary pre-arm alignment](prearm-alignment.md): support contract, uncertainty, rejection, sample ownership and calibration limitation.
- [Supported-start flight comparison](supported-start-flight.md): explicit fixture physics, alignment benefit, retained mass failure and validation boundary.
- [Independent supported-start validation](independent-supported-start.md): repeatability, observation-fault responses and the retained fresh-seed hover failure.
- [Residual supported-hover diagnosis](residual-supported-hover.md): navigation-error forcing, exact response accounting and the next isolation test.
- [Navigation-feedback isolation](navigation-feedback-isolation.md): demonstrated channel headroom with diagnostic truth feedback.
- [Supported velocity prior](supported-velocity-prior.md): exact conditioning, its failed release uncertainty screen and the next boundary correction.
- [Release-aware prediction](release-prediction.md): corrected first interval, verified derivatives and the nonlinear covariance limitation.
- [Nonlinear release uncertainty](nonlinear-release.md): conditional joint moments, full calibration and isolated boundary-only flight comparison.
- [Combined supported prior](combined-supported-prior.md): one supported velocity constraint, nonlinear release and matched comparisons against both previous versions.
- [Combined-prior diagnosis](combined-prior-diagnosis.md): observation corrections, signed physical error budgets and startup-study closeout.
- [Whole-flight error budget](whole-flight-error-budget.md): correlated response energies, sampled sensitivities and an independent-inclination measurement decision.
- [Independent inclination feasibility](independent-inclination-feasibility.md): tangent measurement, correlated uncertainty, calibration/timing requirements and a no-go for expanding the current sensor scope.
- [Standalone pre-arm component](prearm-component.md): support evidence, acquisition, rejection and one-time estimator handoff.
- [Nonlinear pre-arm uncertainty](prearm-nonlinear-uncertainty.md): rotation/bias coupling, positive quadrature and passing independent calibration.
- [Next-step plan](next-steps.md): the current bounded task and stopping rules.
- [Project review](project-review.md): the September 26 documentation and source audit.
- [Decision index](decisions/README.md): assumptions, alternatives and design rationale.
- [Verification index](progress/README.md): all dated records and exact measured results.
- [Changelog](../CHANGELOG.md): notable repository changes.

Dated records describe their own commits. Their old next actions and test counts
are historical; use the current status and plan for today's scope. The standalone
technical report is a separately maintained, source-pinned LaTeX deliverable;
its [guide](technical-report.md) identifies the current revision. These Markdown
guides are the current repository documentation.
