# Technical scope — 2026-09-25

The repository audit and baseline selection are recorded in the
[closeout](progress/2026-09-25-repository-audit.md). Current development proceeds
with a limited baseline; estimated-feedback hover qualification remains open.

| Area | Implemented capability | Remaining boundary |
| --- | --- | --- |
| Plant and numerical foundation | NED/FRD dynamics, quaternions, motors/allocation, wind/drag, Euler/RK4 and analytical/convergence checks | Illustrative hardware parameters; no contact model |
| Sensors and reproducibility | IMU, position/altitude, bias walks, scheduled delivery, truth/nominal models, authenticated artifacts and replay | Defined scheduling and numerical-backend contracts |
| ESKF | Prediction, correction/reset, gates, replay, endpoint calibration and causal online execution | Explicit prior; full-rate paired zero-delay IMU in the supported composition; stale data rejected; weak hover heading observability |
| Baseline control and missions | Attitude/rate and position/velocity cascade, true/estimated feedback, virtual takeoff/track/land/abort | Truth safety oracle is separate; no hardware emergency-flight policy |
| Minimum-snap planning | Fixed-duration optimization, C3 knots, derivatives through snap, conservative whole-curve reference bounds, bounded uniform timing and true-state missions | No minimum-time claim, obstacles, rotor torque/motor feasibility proof or estimated-state polynomial qualification |
| Advanced control | Reusable baseline interfaces | Geometric tracking controller and fair comparison remain |
| System fault tolerance | Estimator outlier gates and bounded mission guards | Persistent health monitoring and degraded-mode policy remain |
| Middleware/deployment | Independent Python core and earlier compatibility spike | ROS 2/C++, PX4/Gazebo mission integration remain |
| Final evidence/report | Reproducible subsystem campaigns and a frozen report v1 | Broad integrated evaluation and an updated final technical report remain |

## Feedback closeout

Use explicit `design_version=2` as the numerical reference. On the same six
observed full hovers it lowers worst peak from 10.4950 to 9.1403 cm and mean peak
from 7.5504 to 6.9123 cm, at about 2.06 times the mean squared-moment effort.
It is not better on every individual case.

Original fresh results remain **29/30 for version 1 and 28/30 for version 2**.
The unchanged 8 cm full-hold condition is still missed. No threshold, scoring
window, gain, prior or sensor distribution changed in this closeout. The original
profiles and failed diagnostics remain reproducible. No additional gain search
or fresh feedback-validation campaign is authorized by this planner milestone.

The reported startup-readiness attempt is not present in the published source.
Its failed outcome was reported in the preceding chat; it is not an accepted
runtime option. No filter defect or fundamental performance limit was proven.

## Current completed step and next scope

[ADR 0015](decisions/0015-minimum-snap-trajectory.md) defines the fixed-duration
position solver. Its [verification record](progress/2026-09-25-minimum-snap.md)
covers interpolation, endpoint/knot constraints, analytic cost, independent
optimization, scaling laws and explicit numerical failures.

The bounded trajectory package is complete under
[ADR 0016](decisions/0016-trajectory-feasibility-and-missions.md): whole-curve
reference bounds, uniform retiming and true-state execution. The five fixed
[mission cases](progress/2026-09-25-trajectory-missions.md) complete with no
limiting, 3.27–3.92 cm position RMSE and at most 7.74 cm peak error. Those are
true-state trajectory results, not a new estimated-feedback hover campaign.

The next bounded package is **geometric tracking control with a fair true-state
comparison**. Begin by auditing the trajectory integration, then fix the controller
equations, reference derivatives, actuator behavior and comparison criteria before
implementation. Preserve the original estimated-feedback failures. Fault
accommodation and ROS/PX4 remain separate milestones.

Progress is tracked by these deliverables. Historical percentage estimates and
test counts do not establish performance qualification or remaining effort.
