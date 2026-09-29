# Project status

Updated 2026-09-28. The implemented Python stack covers simulation, estimation,
planning and closed-loop virtual missions. The main open performance issue is
hover and trajectory control with noisy estimated feedback.

## Implemented capabilities

| Area | Implemented and checked | Remaining limitation |
| --- | --- | --- |
| Plant | Nonlinear six-degree-of-freedom dynamics, quaternion rotations, rotor allocation, motor lag, wind/drag, Euler and projected RK4 | Illustrative parameters; no contact, battery or identified airframe model |
| Sensors and records | Noisy IMU, position and altitude; bias walks; scheduled acquisition/delivery; declared truth/nominal mismatch; authenticated artifacts and replay | Explicit timing and numerical-backend contracts |
| State estimation | 15-state ESKF, measurement updates, attitude reset, innovation gates, endpoint propagation, online execution and replay | Explicit initial prior, no automatic alignment or delayed-state rewind; hover heading is weakly observable |
| Baseline control | Cascaded position/velocity and attitude/rate feedback; supervised takeoff, tracking, landing and abort | True-state cases pass; original estimated-hover qualifications retain failures |
| Vertical compensation | Explicit bounded integral, health-gated learning, reset, conditional anti-windup and authenticated state reconstruction | One frozen research candidate; no default change or whole-flight qualification |
| Planning | Fixed-duration minimum-snap splines, derivatives through snap, nominal reference bounds, bounded uniform retiming | No obstacle planning, time optimization or complete actuator-feasibility proof |
| Geometric control | Rotation-based moment law, analytic force-to-attitude derivatives, causal filter, true/estimated missions and two optional derivative strategies | Six earlier profiles and one subsequent coherent-force candidate fail the joint requirements; the bounded study is closed |
| Integration | Earlier ROS 2/PX4/Gazebo compatibility checks and an x500 takeoff/hover/landing spike | The custom Python stack is not integrated into PX4 or validated on hardware |
| Fault handling | Innovation rejection, per-stream health, opt-in timed abort and causal live faults with a frozen paired maneuver campaign | Declared campaign failures, broader fault diagnosis and physical fallback flight remain |

Estimated minimum-snap missions are available experimentally through the
geometric controller or explicit cascade `allow_minimum_snap=True`. That support
is an implementation capability, not a passing noisy-trajectory qualification.

## Evidence that matters

The [observation health milestone](progress/2026-09-28-observation-health-monitoring.md)
adds independent availability, rejection and recovery evidence for local
position and altitude. Its passive integration preserves complete numerical
histories and saved flight bytes across six paired regression configurations.
The monitor does not change flight decisions or establish estimator accuracy.
The [guide](observation-health.md) defines the timing and recovery rules.

The separate [observation supervision milestone](progress/2026-09-28-observation-loss-supervision.md)
adds explicit, independent unhealthy-time budgets. Required streams can stop
the numerical mission before another command; optional altitude cannot veto
it. Recovery deadlines, phase boundaries, safety priority, nominal equivalence
and saved-history reconstruction are checked. This is an opt-in simulation
response, not a hardware emergency procedure or a new flight qualification.

The [integrated robustness evaluation](progress/2026-09-28-integrated-robustness.md)
uses twelve frozen cases and 24 actual supervised/unsupervised executions,
covering hover, translation, virtual landing, independent stream faults,
recovery, wind and mass mismatch. Full saved histories support exact ESKF,
health, decision and command reconstruction. Response acceptance is scored
separately from flight completion and tracking limits.

All 12 response cases pass with zero numerical failures. Three of five required
flight cases pass: nominal tracking, brief recovery and wind. Hover peaks at
10.76 cm against 8 cm; the 10% heavier-mass case times out at 25 s with 42.39 cm
tracking RMSE and 43.46 cm final error. This is a completed evaluation with a
failed overall flight campaign, not a controller promotion. Its complete
warning-strict software gate passed 3,523 tests at that milestone. The subsequent
[vertical study](progress/2026-09-28-vertical-compensation.md) tests one bounded
disturbance-compensation candidate against those exact saved histories and limits.

That study completes 24 new executions with zero numerical failures, 12/12
response passes and 10/12 candidate comparisons passing. The heavier-mass
mission now completes at 17.5 s, with final error reduced from 43.46 to 6.02 cm.
Its 17.57 cm RMSE still exceeds the original 15 cm limit. Hover peak also rises
slightly, from 10.7563 to 10.7596 cm, failing the frozen no-regression rule and
remaining above 8 cm. The candidate is rejected; it remains an explicit
research option, and the original cascade stays default. The complete software
gate passes 3,575 tests. The [technical report and operating boundary](technical-report.md)
now consolidate these results and the separate geometric limit. Its audit
recomputed metrics from all 48 archived baseline/candidate executions, with
182 payload references authenticated and no new flights. The subsequent
[causal diagnosis](early-flight-diagnosis.md) reconstructs the four saved
mass/hover histories and uses one new attitude-only oracle. An unfitted vertical
model matches the mass trajectories to about 8 mm RMS; the oracle reduces hover
peak to 7.2291 cm. That truth-assisted result demonstrates headroom and cannot
qualify a flight controller. The subsequent
[startup audit](attitude-startup-audit.md) reconstructs both complete estimator
histories and attributes the early tilt growth mainly to the first, valid noisy
position correction. Independent conditioning/reset agrees to roundoff; no
implementation defect is demonstrated. It rejects a speculative free-flight
gravity correction and retains all production behavior. The subsequent
[pre-arm alignment design](prearm-alignment.md) defines a genuinely supported
stationary 0.5 s interval, uncertainty, rejection and sample ownership. Across
5,000 nominal trials its 99th-percentile axis error is 0.1869 degrees and one
window is rejected. However, the independent nonlinear Gaussian covariance
check reaches 1.1122 maximum normalized variance against a 1.10 limit. Production
implementation was blocked pending a bounded uncertainty derivation. The
[nonlinear model](prearm-nonlinear-uncertainty.md) now passes that gate: maximum
normalized variance falls from 1.1424 to 1.0822 on a fresh paired Gaussian
population, below the unchanged 1.10 cap. The original failing population also
passes at 1.0614. All 15,000 study outcomes are retained. The
[standalone component](prearm-component.md) now reproduces all archived
covariances exactly and completes 200 supported acquisition/handoff sessions.
Support, timing, ownership and latched rejection are explicit. The subsequent
[supported-start flight comparison](supported-start-flight.md) passes its frozen
startup acceptance: hover peak is 2.51 cm versus 8.59 cm in the same supported
release without alignment (10.76 cm in the original moving/spinning baseline).
Nominal and wind tracking pass; the mass case still fails at 42.53 cm RMSE.
These are known-seed development results under modeled fixture support. The
subsequent [independent validation](independent-supported-start.md) improves
hover peaks on all three fresh seeds, but only two pass the 8 cm limit; the
third reaches 10.18 cm. Normal mission integration remains no-go. The
[residual diagnosis](residual-supported-hover.md) now reconstructs all six hover
histories and explains most of the failed peak through navigation-error forcing.
An unfitted sampled cascade model matches the failed flight to 0.761 mm horizontal
RMS. The subsequent [navigation-feedback isolation](navigation-feedback-isolation.md)
reduces that failed peak from 10.18 to 1.99 cm and whole-flight RMSE from 6.49
to 3.93 cm, passing the frozen diagnostic comparison. It changes only the outer
controller's position/velocity inputs to simultaneous truth. The ordinary
estimated-input configuration remains unqualified. Next is one supported
zero-velocity prior candidate, using the existing stationary support boundary
with explicit covariance handling and no in-flight truth or zero-velocity update.
All six nominal/wind tracking cases and all eight fault-response comparisons
pass; the 34-flight campaign and its failed hover condition are retained.
No hardware support procedure or general flight qualification is claimed.

The latest [startup/hover closeout](progress/2026-09-26-geometric-transient-closeout.md)
reproduces the original failures and rejects one frozen coherent-force candidate.
It passes the seed-30 spline comparison at 6.77 cm RMSE and 0.85 times cascade
effort, but its full-hover maxima are 10.87 and 17.33 cm against 8 cm. No production
algorithm or controller default changed. That milestone's software gate passed
3,338 tests; sampled-model checks include both horizontal frame directions.
The reserved qualification seeds remain unopened. Observation health monitoring
and bounded supervisor responses are implemented; the separate integrated
evaluation retains the original controller settings and performance limits.

The [geometric audit](progress/2026-09-26-geometric-project-audit.md) records the
following results for the implementation merged in
[PR #16](https://github.com/Gayles9/robust-quadrotor/pull/16):

- The complete software gate passed 3,315 tests, lint, formatting and strict
  typing. CI also passed on the final implementation and merged tree.
- Fifteen true-state regression flights covered seven cascade/geometric pairs
  and an exact repeat. All required comparisons passed; geometric RMSE ranged
  from 0.20 to 3.75 cm.
- The bounded tuning study rejected all six new profiles. None met the 8 cm
  full-hover maximum, nonregressing spline RMSE and effort ratio of at most two
  simultaneously. Defaults and thresholds were retained.
- Across that session, 42 flight executions and 202 recorded payloads were
  independently checked. Counts include repeated comparators and a repeat run;
  they are not 42 independent validation cases.

Seven damaged payloads were restored to their original expected hashes. Four
recoveries used sensor-only estimator replay and three used byte-identical
fixture data. The underlying damage cause is unknown. Qualification now also
requires all saved payloads to pass authentication.

For the earlier baseline, version 1 passed 29/30 fresh hover cases and version 2
passed 28/30. Version 2 is retained as an explicit limited numerical reference,
not a universally better or fully qualified controller. Its same-case comparison
and remaining misses are in the [baseline closeout](progress/2026-09-25-repository-audit.md).

The [endpoint estimator study](progress/2026-09-23-eskf-endpoint-calibration.md)
achieved 95.29% NEES coverage on its declared evaluation, with zero numerical
failures across 380 replays. This supports the tested standalone estimator
configuration. It does not establish observability in every maneuver or close
the flight-control requirements.

## Gate interpretation

The plant/sensor/replay foundation (G1) is complete. The standalone estimation
work (G3) is supported within its stated prior, motion and noise assumptions.
The true-state baseline and trajectory packages pass their bounded cases, but
estimated-feedback flight qualification (G2) remains open. Geometric control
has the same distinction between implementation and qualified flight behavior.

[Controller problems and tradeoffs](controller-tradeoffs.md) explains why the
remaining failures can coexist with correct equations and good true-state
tracking. [Project review](project-review.md) records the current audit, and
[the next-step plan](next-steps.md) defines the bounded supported-velocity prior candidate.
Broader qualification and middleware integration remain separate deliverables.
The updated report is pinned to its declared source; historical report revisions
and dated records remain snapshots, not instructions to repeat old studies.
