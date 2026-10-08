# Capability and requirement ledger

This table shows what the project implements, what its tests and simulations
demonstrate, and which requirements remain unmet. Each claim is limited to the
scenarios actually checked; it is not a guarantee across every operating
condition or a hardware qualification. The sensors are an inertial measurement
unit (IMU), local position measurements and barometric altitude.

## Capabilities and requirements

Passing component checks, passing an absolute flight limit and passing a paired
candidate comparison are separate claims. For example, a flight can remain
within an error limit while still being worse than its paired control. A
"research option" is implemented for explicit experiments and is not the default.
RMSE is root-mean-square error across a flight; NEES compares estimation error
with the filter's stated uncertainty. The [official report](../report.md)
discusses the methods and results.

| ID / capability | Current status and passing evidence | Failed or conditional requirement | Source |
| --- | --- | --- | --- |
| E01 Plant, sensors and records | **Implemented and checked numerical foundation.** Six-degree-of-freedom dynamics, motor lag, noisy IMU/position/altitude, clocks and exact records are implemented and checked. | Illustrative parameters; no contact, battery or identified hardware model. | [Guide](../design/foundations.md); [implementation](../../src/quadrotor_math/dynamics.py) |
| E02 State estimation | **Implemented under declared assumptions.** 15-state error-state Kalman filter (ESKF) with correction/reset, gates, causal replay and 21-coordinate endpoint noise memory; the nominal endpoint study reports 95.29% central-95% NEES coverage. The 380 replays cover distinct evaluation groups, not a single pooled coverage population. | Explicit priors and local noise/motion assumptions; no delayed-state rewind or general closed-loop qualification. | [Guide](../design/estimation.md); [implementation](../../src/quadrotor_math/eskf_endpoint.py) |
| E03 Trajectory planning | **Implemented reference generator.** Degree-seven minimum-snap, continuous reference bounds and bounded uniform retiming pass their declared checks. | No obstacle planning, time optimization or complete actuator-feasibility proof. | [Guide](../design/trajectories.md); [implementation](../../src/quadrotor_math/minimum_snap.py) |
| E04 Cascade missions | **Original default; broader flight qualification incomplete.** True-state cases pass. Original integrated campaign passes 12 response cases and 3 of 5 required flight cases. | Estimated hover reaches 10.7563 cm versus 8 cm; heavier mass reaches 42.3933 cm RMSE versus 15 cm. | [Guide](integrated-robustness.md); [implementation](../../src/quadrotor_math/estimated_mission.py) |
| E05 Geometric control | **Explicit research option.** Fifteen true-state regression executions pass their comparisons; rotation-based control and derivative strategies are implemented. See E14–E15 for evaluated startup and filtering improvements. | Six tested profiles and the original coherent-force candidate fail their joint noisy-hover/tracking/effort acceptance. | [Guide](controller-tradeoffs.md); [implementation](../../src/quadrotor_math/geometric_control.py) |
| E06 Observation response | **Passive monitor and opt-in supervisor.** Availability, recovery and latched timeout decisions pass the declared fault comparisons, with complete recorded response timing. | Availability is not accuracy. Numerical abort stops simulated commands; physical fallback is unimplemented. | [Guide](../design/observation-supervision.md); [implementation](../../src/quadrotor_math/observation_supervision.py) |
| E07 Mass compensation | **Fails the full requirements; explicit research option.** Bounded vertical integral removes the tested timeout and reduces final error to 6.02 cm. | 17.57 cm whole-flight RMSE exceeds 15 cm; hover comparison regresses. Not evaluated together with supported-start candidates. | [Guide](vertical-compensation.md); [implementation](../../src/quadrotor_math/vertical_compensation.py) |
| E08 Supported alignment | **Standalone component; experimental flights.** Nonlinear joint alignment covariance passes its fixed calibration; 200 acquisition/release trials pass. Known-seed hover improves to 2.506 cm. | External support and fresh sample ownership are mandatory. One independent-seed aligned hover still reaches 10.1808 cm. No hardware support claim or normal integration. | [Guide](independent-supported-start.md); [implementation](../../src/quadrotor_math/prearm_alignment.py) |
| E09 Release prediction | **Experiment-only mathematical component.** One-sided first force removes ballistic boundary bias. Nonlinear 21-coordinate moments pass both-prior calibration across 17 configurations and 680,000 outcomes. | Conditional Gaussian pushforward; ordinary later approximations remain. Boundary-only flight comparison fails. | [Guide](nonlinear-release.md); [implementation](../../experiments/nonlinear_release.py) |
| E10 Combined supported prior | **Experimental candidate; startup study closed.** All nine tested clean flights pass absolute limits; hover peaks are 6.7539, 5.6943 and 5.5644 cm. Eight fault comparisons pass. | Five of nine clean no-regression comparisons fail. These are reused development seeds; fresh qualification and adoption remain incomplete. | [Guide](combined-supported-prior.md); [implementation](../../experiments/combined_supported_prior.py) |
| E11 Whole-flight budget | **Completed offline analysis.** 27 clean histories retain signed Gram accounting and sampled local input sensitivities; mean offsets and cross terms are included. | Single-input ceilings are not joint allowances. No established posterior-error and physical-residual allocation proves an 8 cm bound. | [Guide](whole-flight-error-budget.md); [implementation](../../experiments/whole_flight_error_budget.py) |
| E12 Independent inclination | **Conditional contract only; extension closed.** Direction Jacobian, correlated conditioning and local information ranks are verified. Calibrated direction gives rank 13 of 15. | Unknown inclination bias gives rank 13 of 17. Source/model, calibration, timing and joint allocation remain unsupported; component no-go. | [Guide](independent-inclination-feasibility.md); [implementation](../../experiments/inclination_feasibility.py) |
| E13 ROS 2/PX4 and hardware | **Custom integration unimplemented.** Historical environment compatibility and x500 takeoff/hover/landing spike only. | No custom adapter, command authority, real-time guarantee, contact model or hardware qualification. A custom integration interface is not implemented. | [Evidence archive](../../evidence/development-records.zip) |
| E14 Supported geometric combination | **Implemented experimental configuration.** Full-hold peaks fall from 8.73/18.10 cm to 5.21/4.62 cm. All four candidate clean flights meet absolute limits and effort comparisons; nominal spline RMSE is 19.03% below matched combined cascade. | Wind RMSE is 3.73% above cascade (6.87 versus 6.63 cm), failing the predeclared comparison. Geometric hover RMSE and spline peaks also exceed cascade's. This study's conditional validation stage is not executed; the default is unchanged. | [Guide](supported-geometric.md); [implementation](../../experiments/supported_geometric_comparison.py) |
| E15 Axis-dependent geometric control | **Implemented and evaluated on reserved cases.** Fourteen corrected true-state checks and all twenty-four selected-controller clean flights pass absolute conditions. Fresh geometric hold peaks are 4.8540..5.7295 cm; balanced RMSE is 4.7717% below selected cascade and 20.2899% below original cascade. | The 10% balanced improvement target is missed. Against selected cascade, hover RMSE is 1.6100% higher, balanced whole-flight peaks 15.2880% higher and moment effort 1.6736% higher. This is a bounded accuracy tradeoff, with no default promotion or general superiority theorem. | [Guide](final-geometric.md); [implementation](../../experiments/axis_shaped_geometric.py) |

## Conditions on the measured flight results

The preserved studies use NED/FRD frames and body-to-world rotations. Their
fixed mission profile uses 400 Hz plant/IMU, 100 Hz inner control, 50 Hz outer
control, 5 Hz position and 25 Hz altitude. Position noise is 2 cm per axis;
altitude noise is 3 cm; the IMU's per-sample standard deviations are 0.04 m/s²
and 0.002 rad/s per axis. Bias walks and truth/nominal mismatch remain explicit
in each saved configuration. These assumptions are not identified sensor specs.

Supported-start comparisons additionally require a physically balanced numerical
fixture, motors off, stationary support through 201 IMU samples over 0.5 s and
the fresh release sample at 0.5025 s. Exact velocity conditioning requires a
separate zero-world-velocity assertion; quiet IMU data do not prove it. Normal
missions do not silently acquire those additional assumptions.

For the combined candidate, each clean flight must complete, keep whole-flight
RMSE and final position error at or below 15 cm, final speed at or below 15 cm/s,
and applicable inclusive 5..11 s hover maximum at or below 8 cm. Clean RMSE and
hover peak must also not exceed either matched control by more than 1e-12 in
the metric's SI unit. All nine absolute flight conditions pass; five paired
comparisons fail. The clean set contains hover, nominal translation and wind
at three seeds, not a mass-mismatch case or fresh population qualification.

The whole-flight budget's prospective inequality applies only to the original
local sampled horizontal hover model with established input bounds and an
additional initial/vertical/physical residual allowance. Its individual ceilings
must not be advertised as simultaneous sensor specifications. No complete
allocation currently closes that requirement.

## Current defaults and limitations

The original cascade and ordinary mission initialization remain the defaults.
Geometric control, vertical compensation and supported-start/release variants
are explicit research options. The numerical foundation is implemented and
checked; state estimation retains its declared prior, noise and motion
assumptions. Broader flight qualification remains incomplete, and the available
evidence does not justify the independent-inclination sensor extension.

The [official report](../report.md) explains the mathematics and reported
results. The [report verification record (ZIP)](../../evidence/development-records.zip)
authenticates 436 saved payload references, independently rescores 84 existing
executions and checks 144 Gram windows, with no new scientific flights.
These counts measure audit coverage, not independent trials or project completion.

The [final axis-dependent geometric comparison](final-geometric.md) is complete
and covered by the official report. Broader qualification and interface
integration are separate from those measured results; see
[planned work](../next-steps.md).
