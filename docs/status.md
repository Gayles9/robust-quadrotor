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
| Planning | Fixed-duration minimum-snap splines, derivatives through snap, nominal reference bounds, bounded uniform retiming | No obstacle planning, time optimization or complete actuator-feasibility proof |
| Geometric control | Rotation-based moment law, analytic force-to-attitude derivatives, causal filter, true/estimated missions and two optional derivative strategies | Six earlier profiles and one subsequent coherent-force candidate fail the joint requirements; the bounded study is closed |
| Integration | Earlier ROS 2/PX4/Gazebo compatibility checks and an x500 takeoff/hover/landing spike | The custom Python stack is not integrated into PX4 or validated on hardware |
| Fault handling | Innovation rejection, event diagnostics, mission guards and passive per-stream observation health monitoring | Supervisor responses, fault diagnosis and degraded flight policy remain |

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

The latest [startup/hover closeout](progress/2026-09-26-geometric-transient-closeout.md)
reproduces the original failures and rejects one frozen coherent-force candidate.
It passes the seed-30 spline comparison at 6.77 cm RMSE and 0.85 times cascade
effort, but its full-hover maxima are 10.87 and 17.33 cm against 8 cm. No production
algorithm or controller default changed. That milestone's software gate passed
3,338 tests; sampled-model checks include both horizontal frame directions.
The reserved qualification seeds remain unopened. Observation health monitoring
is now implemented; supervisor responses are the next separate task.

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
[the next-step plan](next-steps.md) defines supervisor response design before
further integration. Broad integrated evaluation and an updated technical report remain
separate deliverables; the earlier report is a historical snapshot.
