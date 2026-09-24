# Technical scope snapshot — 2026-09-24

This is a scope estimate, not a release certification, a measure of elapsed time,
or a claim of hardware readiness. The [latest verification record](progress/2026-09-24-feedback-design.md)
is authoritative for the actual tested profile and results. Historical baselines
remain reproducible and must not be confused with that separately named profile.

**Qualification is open.** The two frozen estimated-feedback profiles pass 29/30
and 28/30 fresh cases. Both miss the unchanged .08 m hover target. Later gain and
integral prototypes do not resolve the observed failures and are not promoted.
The implemented numerical integration and its passing software checks must not
be presented as completed hover-performance qualification.

| Area | Implemented technical capability | Important boundary or remaining work |
| --- | --- | --- |
| Physical and numerical foundation | NED/FRD rigid-body dynamics, quaternion utilities, rotor allocation and lag, wind/drag, Euler/RK4 histories and analytic/convergence checks | Illustrative parameters and numerical physics, not hardware identification or contact dynamics |
| Sensors and reproducibility | IMU/position/altitude models, scheduled delivery, bias walks, named random streams, configurations, provenance, manifests and authenticated artifacts | Defined sensor/timing contracts; no claim that arbitrary hardware streams are supported |
| Basic ESKF | Prediction, position/altitude correction, right-local reset, gating, replay, consistency/fault experiments and causal online execution | Explicit initial prior; full-rate paired zero-delay IMU for the supported composition; stale data rejected rather than rewound; weak hover heading observability |
| Baseline feedback and missions | Attitude/rate and position/velocity cascade, feedforward, explicit limits, true/estimated-state execution, virtual takeoff/track/land/abort and paired evidence | Bounded numerical profiles; truth safety oracle is labeled separately; not a real emergency-flight policy |
| Optimized trajectory generation | Existing hold, step and quintic mission references | Minimum-snap optimization, general waypoint constraint assembly and feasibility/time allocation are not implemented |
| Advanced control | Baseline controller and test interfaces are available | Geometric tracking controller and fair advanced-versus-baseline comparison remain |
| System fault tolerance | Estimator outlier gates and bounded mission guards exist | General fault scenarios, persistent health monitoring and degraded-mode policies remain; estimator rejection alone is not system fault tolerance |
| Middleware and deployment | The mathematical package remains middleware-independent | ROS 2 wrappers, tested C++ component and PX4/Gazebo mission integration remain |
| Final system evidence | Reproducible subsystem and bounded closed-loop campaigns exist | Broad cross-controller/fault evaluation, integrated demonstration and complete final technical report remain |

## Approximate progress

- **Standalone numerical system: approximately 65–70%.** This denominator includes
  the planned trajectory, advanced-control and system fault-handling packages,
  not just the already implemented simulator/filter/baseline controller.
- **Full original technical project: approximately 45–50%.** This additionally
  includes middleware/integration, the broad final experiment matrix and the
  report/release/demonstration package.

These are coarse deliverable-based estimates against the six-month master plan.
The strongest coverage is in the foundations, baseline-control and basic-estimator
groups; several later groups are wholly unimplemented. The range acknowledges
unequal package sizes and partially completed cross-cutting validation work.
It is not obtained by dividing passed tests by a target test count, and it should
not be read as a forecast of remaining effort. Passing a bounded campaign does
not turn all robustness/integration work into completed scope.

## Next bounded package

The immediate work is to resolve the startup estimation/control coupling under
the existing hover, actuator and timing requirements. A new design needs an
explicit error and control-effort budget, observed-seed regressions, full replay
and physical audits, and fresh validation after its complete source freeze.
The current draft cannot be promoted using its failed results.
The [bounded startup diagnostic](progress/2026-09-24-feedback-startup-diagnostic.md)
now shows that replacing position/velocity feedback with truth reduces the two
observed startup peaks below 1 cm, while replacing attitude/rate feedback leaves
both above 8 cm. This narrows the next design target to translational
estimation/control coupling; it is diagnostic evidence, not a production fix.
The subsequent [uncertainty and motor-response study](progress/2026-09-24-translational-feedback-design.md)
finds no clear noise-calibration defect in the observed population. One fixed
causal candidate improves both misses but still fails at 8.386 cm. It remains
an explicitly rejected experiment; the next design must address tracking,
measurement-error amplification and actuator effort together.

After that qualification, audit the feedback milestone before the minimum-snap
formulation: normalized-time seventh-order segments,
derivatives through snap, exact integrated snap cost, waypoint/boundary/continuity
constraints and a checked equality-constrained solve. Its evidence must include
constraint residuals, knot continuity, comparison against a feasible non-optimal
trajectory, time-scaling laws and explicit ill-conditioning failure behavior.

Trajectory feasibility/time allocation and closed-loop trajectory integration
follow as their own bounded package. Geometric control, fault accommodation and
ROS/PX4 integration must not be silently folded into the minimum-snap solver.
