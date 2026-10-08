# Trajectory bounds, timing and true-state missions

This layer checks whether a planned path is reasonable to ask the controller to
follow. A bounded retimer slows the minimum-snap trajectory until its nominal
reference limits pass. The mission runner then measures actual tracking,
completion and limiting separately. A feasible reference is only the first
check; actuator lag and feedback error can still affect the flight.

The implementation is in [trajectory_feasibility.py](../../src/quadrotor_math/trajectory_feasibility.py)
and [mission_simulation.py](../../src/quadrotor_math/mission_simulation.py).
[ADR 0016](../decisions/0016-trajectory-feasibility-and-missions.md) defines the
original true-state scope and acceptance criteria. The later
[geometric integration](geometric-control.md) also supports experimental
estimated-state spline missions, whose performance limitations remain explicit.

## Whole-curve reference bounds

For a degree-n polynomial in normalized segment time,

`p(s) = sum(k=0..n, a_k s^k) = sum(i=0..n, b_i B_i,n(s))`,

the Bernstein control points are

`b_i = sum(k=0..i, binomial(i,k)/binomial(n,k) a_k)`.

The Bernstein basis is nonnegative and sums to one on [0,1]. Every polynomial
value therefore lies in the convex hull of its control points. Apply the same
conversion to physical velocity, acceleration and jerk, then split each segment
at midpoints with de Casteljau's algorithm. The default six levels give 64
subintervals per segment. Bounds cover those intervals continuously; they are
not maxima from a time grid. More subdivisions tighten the hull without changing
the path, and the explicit depth limit of eight bounds computation.

Position components are bounded by control-point minima and maxima. Speed is
bounded by the largest velocity-control-point norm. Component acceleration is
bounded by the largest absolute acceleration control point. All include a
floating-point pad of `512 eps (1 + sum(abs(power coefficients)))`, with origin
and gravity contributions included where needed. This is a conservative numerical
engineering check at supported float64 scales, not formal interval arithmetic.

In the nominal drag-free NED model, define `u = g e3 - a`. Reference thrust is
`T = m ||u||`, and desired body-down is `b3 = u/||u||`. On each subinterval let
`z` be a positive lower bound on u_z, H an upper bound on `||u_xy||`, U an upper
bound on `||u||`, and J an upper bound on jerk norm. Sufficient bounds are

- `m z <= T <= m U`;
- `tilt <= atan(H/z)`;
- `||omega_reference|| <= J/z`.

The last inequality matches the existing controller's fixed-heading construction:
`c2 = [-sin(yaw), cos(yaw), 0]`, `b1 = (c2 x b3)/h`, `h = ||c2 x b3||`,
`b2 = b3 x b1`. Differentiating `c2 dot b1 = 0` gives
`omega3 = omega2 (c2 dot b3)/h`. Consequently
`||omega|| <= ||b3_dot||/h <= ||j||/(||u|| h) <= ||j||/u_z`.
Fixed yaw heading does not imply zero angular velocity about body-down.

`check_trajectory_feasibility(trajectory, limits)` returns all bounds and an
ordered tuple of unresolved limit conditions. Nonpositive down-thrust gives
undefined tilt/rate bounds, represented by `None`, and explicit rejection.
Acceptance is sufficient for these nominal reference conditions. Rejection can
be conservative, including a curve too close to a bound for numerical padding.
C3 continuity is checked to an absolute numerical tolerance of 1e-7 in each
derivative's SI units; those residuals are treated as floating-point error.

These checks do not prove rotor moment, motor-lag or feedback feasibility. They
do not include drag compensation, obstacles, uncertain parameters or continuous
truth-state safety. The existing sampled truth guards remain active in flight.
For example, the reference speed stays below its 0.6 m/s planning limit while
the nominal simulated vehicle reaches 0.612397 m/s. The planning limit bounds
the reference; actual speed is not capped by the historical controller.

## Bounded timing policy

`retime_trajectory` accepts C3 rest-to-rest trajectories, including zero endpoint
velocity, acceleration and jerk to the same numerical tolerance. It retains the
normalized coefficients, origin and relative segment durations. Scaling durations
by alpha preserves the spatial path and gives

`p_alpha(t) = p(t/alpha)`, `p_alpha^(r)(t) = alpha^-r p^(r)(t/alpha)`,
`snap_cost_alpha = alpha^-7 snap_cost`.

For a minimum-snap input this remains the minimum-snap solution at the scaled
times. The function cannot infer optimality for manually constructed inputs.
The default search tries 1, 1.25, 1.25², ... up to a final cap of 8, with at most
12 attempts. Every attempted scale and its full bounds are retained. The first
passing candidate is returned; failure has `trajectory=None` and a reason.
It is not a minimum-time optimizer or a general independent-segment allocator.
Geofence rejection ends the search because this retimer never changes geometry.

## Mission adapter and compatibility

`MinimumSnapMissionSegment` carries a polynomial in a TAKEOFF, TRACK or LAND
phase. It validates ownership, C3 internal joins, rest endpoints, supplied endpoint
positions and exact agreement with the stored total duration. One segment can
carry a multi-waypoint curve with nonzero velocity at internal waypoints.
Its separate dataclass leaves the historical `MissionSegment` serialization
unchanged, including the frozen experiment protocol hashes.

The existing `mission_reference` evaluates analytic p/v/a within the segment and
holds its endpoint outside it. Right-continuous phase selection, controller
periods, actuator lag, guard priority, terminal dwell and timeout are unchanged.
Before any plant step, `simulate_mission` checks every polynomial against the
mission box and nominal controller acceleration, thrust, tilt and rate bounds.
The reference rate norm is bounded by the smallest configured component rate
limit, which conservatively fits all three components. There is no speed limit
in the historical controller, so that preflight omits speed; planning can impose
its own tighter speed limit. `simulate_estimated_mission` accepts polynomial
missions with an explicit geometric controller, or with
`allow_minimum_snap=True` for a cascade comparison. The default estimated
cascade call still rejects polynomial missions. These opt-ins support the
comparison experiments; they do not imply noisy-flight qualification.

## Fixed evidence and reproduction

```bash
uv run pytest -q -W error tests/unit/test_trajectory_feasibility.py tests/unit/test_trajectory_missions.py tests/unit/test_trajectory_mission_validation.py
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.trajectory_mission_validation --output /tmp/NEW_trajectory_missions --workers 3
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
```

The frozen mission initializes for 1 s, takes off to 1 m altitude in 4 s, follows
a four-leg 3D loop, holds 1 s and lands in 4 s. The track starts with 1 s per leg.
Planning imposes speed 0.6 m/s, component acceleration 0.6 m/s², thrust [7,13] N,
tilt 10 degrees, reference angular speed 0.4 rad/s and the existing mission box.
The first acceptance is scale **3.814697265625**, after seven attempted scales;
track duration is 15.2587890625 s. Bounds include speed 0.574785 m/s, component
acceleration at most 0.321724 m/s², thrust [9.692721,9.922717] N, tilt 2.063 degrees
and reference rate 0.029173 rad/s. The spline reaches about 1.49215 m North/East
despite waypoints reaching only 1 m, illustrating why waypoint-only geofence
checks are insufficient.

Five deterministic cases cover nominal execution, plant-step refinement, two
opposite initial offsets and mild wind/drag. Controller gains and sensor/estimator
code are unchanged. Full-grid NPZ histories are bound by SHA-256 in the JSON report.
The report records whether source hashes remain unchanged throughout execution.
These are bounded engineering cases, not a Monte Carlo qualification campaign.
See the [verification record](../archive/records/trajectory-missions.md) for results.

The subsequent [geometric comparison](geometric-control.md) is implemented and
has passing bounded true-state results. The [final noisy-feedback comparison](../results/final-geometric.md)
reports a modest balanced accuracy gain with retained peak and effort costs.
[Controller tradeoffs](../results/controller-tradeoffs.md) explains the earlier
failures and current limits; [next steps](../next-steps.md) separates planned work.
