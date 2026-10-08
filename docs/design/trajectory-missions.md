# Trajectory bounds, timing and true-state missions

This layer checks whether a planned path is reasonable to ask the controller to
follow. A bounded retimer searches a fixed range of slower timings for a
minimum-snap trajectory that meets the nominal reference limits. It rejects
the trajectory if no attempted timing passes. The mission runner then measures
actual tracking, completion and limiting separately. A feasible reference is only the first
check; actuator lag and feedback error can still affect the flight.

The implementation is in [trajectory_feasibility.py](../../src/quadrotor_math/trajectory_feasibility.py)
and [mission_simulation.py](../../src/quadrotor_math/mission_simulation.py).
[The mission specification](../decisions/0016-trajectory-feasibility-and-missions.md)
defines the true-state scope and acceptance criteria. True-state feedback gives
the controller the exact simulated state, isolating tracking behavior from
estimation error. [Geometric integration](geometric-control.md) also supports
experimental estimated-state spline missions, whose performance limitations remain explicit.

## Whole-curve reference bounds

Checking a few sampled points can miss a peak between them. Polynomial bounds
instead enclose every point on a curve segment. The method below uses that
property to check the complete reference path, including between waypoints.

For a degree-n polynomial in normalized segment time,

$`p(s)=\sum_{k=0}^n a_k s^k=\sum_{i=0}^n b_i B_{i,n}(s)`$,

the Bernstein control points are

$`b_i=\sum_{k=0}^i\frac{\binom ik}{\binom nk}a_k`$.

The basis is $`B_{i,n}(s)=\binom ni s^i(1-s)^{n-i}`$. It is nonnegative and sums to one on [0,1]. Every polynomial
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

In the nominal drag-free NED model, define $`\mathbf u=g\mathbf e_3-\mathbf a`$. Reference thrust is
$`T=m\lVert\mathbf u\rVert`$, and desired body-down is $`\mathbf b_3=\mathbf u/\lVert\mathbf u\rVert`$. On each subinterval let
$`z`$ be a positive lower bound on $`u_z`$, $`H`$ an upper bound on $`\lVert\mathbf u_{xy}\rVert`$, $`U`$ an upper
bound on $`\lVert\mathbf u\rVert`$, and $`J`$ an upper bound on jerk norm. Writing tilt as $`\vartheta`$ and reference angular velocity as $`\boldsymbol\omega_r`$, sufficient bounds are

- $`mz\leq T\leq mU`$;
- $`\vartheta\leq\arctan(H/z)`$;
- $`\lVert\boldsymbol\omega_r\rVert\leq J/z`$.

The heading angle is $`\psi`$. The last inequality matches the existing controller's fixed-heading construction:
$`\mathbf c_2=[-\sin\psi,\cos\psi,0]^T`$, $`\mathbf b_1=(\mathbf c_2\times\mathbf b_3)/h`$, $`h=\lVert\mathbf c_2\times\mathbf b_3\rVert`$,
$`\mathbf b_2=\mathbf b_3\times\mathbf b_1`$. Differentiating $`\mathbf c_2^T\mathbf b_1=0`$ gives
$`\omega_3=\omega_2(\mathbf c_2^T\mathbf b_3)/h`$. Consequently
$`\lVert\boldsymbol\omega\rVert\leq\frac{\lVert\dot{\mathbf b}_3\rVert}{h}\leq\frac{\lVert\mathbf j\rVert}{\lVert\mathbf u\rVert h}\leq\frac{\lVert\mathbf j\rVert}{u_z}`$.
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
the reference; actual speed is not capped by the controller.

## Bounded timing policy

`retime_trajectory` accepts C3 rest-to-rest trajectories, including zero endpoint
velocity, acceleration and jerk to the same numerical tolerance. It retains the
normalized coefficients, origin and relative segment durations. Scaling durations
by alpha preserves the spatial path and gives

$`\mathbf p_\alpha(t)=\mathbf p(t/\alpha)`$, $`\mathbf p_\alpha^{(r)}(t)=\alpha^{-r}\mathbf p^{(r)}(t/\alpha)`$,
$`J_\alpha=\alpha^{-7}J`$.

Here $`J`$ and $`J_\alpha`$ denote the original and retimed integrated squared snap costs.

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
Its separate dataclass preserves `MissionSegment` serialization and the
experiment protocol hashes that identify reproducible configurations.

The existing `mission_reference` evaluates analytic p/v/a within the segment and
holds its endpoint outside it. Right-continuous phase selection, controller
periods, actuator lag, guard priority, terminal dwell and timeout are unchanged.
Before any plant step, `simulate_mission` checks every polynomial against the
mission box and nominal controller acceleration, thrust, tilt and rate bounds.
The reference rate norm is bounded by the smallest configured component rate
limit, which conservatively fits all three components. There is no speed limit
in the controller, so that preflight omits speed; planning can impose
its own tighter speed limit. `simulate_estimated_mission` accepts polynomial
missions with an explicit geometric controller, or with
`allow_minimum_snap=True` for a cascade comparison. The default estimated
cascade call still rejects polynomial missions. These opt-ins support the
comparison experiments; they do not imply noisy-flight qualification.

## Evaluation and reproduction

```bash
uv run pytest -q -W error tests/unit/test_trajectory_feasibility.py tests/unit/test_trajectory_missions.py tests/unit/test_trajectory_mission_validation.py
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.trajectory_mission_validation --output /tmp/NEW_trajectory_missions --workers 3
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
```

The evaluation mission initializes for 1 s, takes off to 1 m altitude in 4 s, follows
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
See the [verification evidence (ZIP archive)](../../evidence/development-records.zip) for results.

The [geometric comparison](geometric-control.md) has passing bounded true-state
results. The [final noisy-feedback comparison](../results/final-geometric.md)
reports a modest balanced accuracy gain with retained peak and effort costs.
[Controller tradeoffs](../results/controller-tradeoffs.md) explains the earlier
failures and current limits; [next steps](../next-steps.md) separates planned work.
