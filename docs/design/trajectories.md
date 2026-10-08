# Fixed-duration minimum-snap position trajectories

The solver chooses a smooth position curve through specified waypoints at
specified times, expressed in the North–East–Down (NED) world frame. Velocity
describes how quickly position changes; acceleration describes how quickly
velocity changes; jerk is the rate of change of acceleration; snap is the rate
of change of jerk, or the fourth time derivative of position. The objective
penalizes its squared magnitude integrated over time; it is not electrical
energy or an actuator-feasibility certificate.

I keep path optimization separate from timing and flight execution. This makes
it possible to verify the polynomial solution independently before asking a
controller to track it. The solver is in
[minimum_snap.py](../../src/quadrotor_math/minimum_snap.py); the next layer is
[trajectory bounds and missions](trajectory-missions.md).

The [final controller comparison](../results/final-geometric.md) also exercises
minimum-snap references with noisy estimated feedback. Its bounded results do
not turn planning smoothness into a general flight-feasibility guarantee.

## Problem and solution

For segment i with duration T_i and normalized coordinate s in [0,1],

`p_i(t) = origin + sum(k=0..7, c_ik s^k)`, `s=(t-t_i)/T_i`.

Its r-th physical derivative divides the r-th normalized derivative by T_i^r.
For r=4, the integrated squared-snap cost is

`J = sum_i T_i^-7 sum_(k,l=4..7)
     (c_ik dot c_il) [k!/(k-4)!] [l!/(l-4)!] / (k+l-7)`.

The units of J are m²/s⁷. Each segment interpolates both endpoint positions.
Velocity, acceleration and jerk are shared between adjacent segments, enforcing
C3 continuity: position and its first three derivatives have no jumps at the
joins. Their initial and final values are prescribed, zero by default.
Internal derivatives are free optimization variables; internal waypoints are
not forced stops.

A fixed 8-by-8 Hermite map converts endpoint p/v/a/j into normalized polynomial
coefficients. The analytic snap Gram matrix is factored to write the reduced
objective as `||Bz + b||²`. Time, position and column scaling precede a checked
singular value decomposition (SVD) least-squares solve. This exposes weakly
determined directions so poorly conditioned problems can be rejected. The
solver needs no external optimizer or matrix inverse of a Karush–Kuhn–Tucker
(KKT) system, and applies no regularization.

The minimizer is unique: a zero-cost feasible variation would be a C3 piecewise
cubic, hence a global cubic. Its zero initial p/v/a/j force it to vanish. This
proves strict positivity on the feasible variation space. It does not establish
flight stability. For fixed positions and free internal derivatives, the optimum
also has the natural continuity of derivatives four through six; independent
tests check this consequence.

## Use

```python
import numpy as np
from quadrotor_math.minimum_snap import minimum_snap_trajectory

trajectory = minimum_snap_trajectory(
    np.array([[0.0, 0.0, 0.0], [1.0, 0.0, -1.0], [2.0, 1.0, -1.0]]),
    np.array([2.0, 3.0]),
)
position_W = trajectory.evaluate(1.0)
snap_W = trajectory.evaluate(1.0, derivative_order=4)
reference = trajectory.position_reference(1.0, yaw_rad=0.0)
print(trajectory.snap_cost)
```

Optional `initial_derivatives_W` and `final_derivatives_W` have shape (3,3):
rows velocity [m/s], acceleration [m/s²], jerk [m/s³]; columns North/East/Down.
Input arrays must be finite real numeric NumPy arrays. Outputs own read-only
float64 storage. Coefficients have shape `(segments,8,3)` and are relative to
the separately retained `position_origin_W`; adding a large world translation
does not contaminate the derivative optimization.

Evaluation covers the exact stored interval `[0, knot_times_s[-1]]` and orders
0 through 4. Knots use the segment on their right, with the final endpoint taken
from the last segment. Out-of-domain times raise rather than extrapolate. Use
the stored final time: summing scaled durations and scaling an old sum can differ
by one floating-point unit. The fixed-yaw adapter produces the existing
`PositionReference`; [trajectory missions](trajectory-missions.md) adds the
subsequent bounded feasibility, timing and true-state execution layer.

## Numerical boundary

At most 64 segments and a largest/smallest duration ratio of 1e6 are supported.
That ratio limit is a coarse input bound, not a guarantee every such problem is
resolvable. The diagonally scaled reduced system must be full rank with condition
number <=1e8. Its relative stationarity residual must be <=2e-10.

Endpoint checks use global scaled time and position units. The measured residual
must be <=2e-9 times max(1, the expected derivative magnitude); a conservative
floating-point cancellation estimate must be <=2e-8 on the same scale. This
second check matters: durations [1,1e6] can produce coefficients near 1e19 and
lose metres at an endpoint even though the reduced system condition is only 17.
The solver rejects that case explicitly. Cost/evaluation also reject overflow,
underflow that prevents representation, and nonfinite values.

No tolerance is a physical hover requirement. Residual checks apply to the
polynomial constraints; the existing controller's 8 cm target is unchanged.

## Evidence and reproduction

```bash
uv run pytest -q -W error tests/unit/test_minimum_snap.py tests/unit/test_minimum_snap_example.py
uv run python -m experiments.minimum_snap_example --output /tmp/NEW_minimum_snap
```

The example writes a compact NPZ, JSON metrics and a PNG. Its four waypoints and
durations [2,1.5,2.5] s are fixed. The optimized cost is **99.44149496 m²/s⁷**,
compared with **10837.25951605 m²/s⁷** for the feasible seventh-degree reference
that stops at each waypoint: **99.0824% lower** for this example. This comparison
uses identical times and boundary constraints; it does not compare flight energy
or prove actuator feasibility. Independent quadrature gives 99.44149496, maximum
waypoint residual is 2.58e-14 m, and maximum C3 knot mismatch is 4.63e-13 in the
respective derivative units.

Verification includes the analytic rest-to-rest polynomial
`35s^4 - 84s^5 + 70s^6 - 20s^7` and cost `100800 ||delta_p||²/T^7`, an independent
full coefficient-space constrained solve, feasible perturbations, exact cubic
recovery, nonzero endpoint derivatives, quadrature, time/space invariance,
natural continuity, the 64-segment boundary and explicit invalid/numerical failures.
Uniform time scaling by a gives derivative scaling a^-r and cost scaling a^-7.

See the [solver specification](../decisions/0015-minimum-snap-trajectory.md) for
its scope and [verification evidence (ZIP archive)](../../evidence/development-records.zip) for
the measured checks.

## Flight integration and scope

The [trajectory mission layer](trajectory-missions.md) implements nominal
whole-curve thrust/tilt/rate/geofence bounds, bounded uniform retiming and
true-state mission execution. These capabilities are separate from the fixed-time
solver's mathematical contract. Smooth interpolation alone does not establish
flight feasibility. [Geometric tracking](geometric-control.md) and experimental
estimated-state spline execution are implemented as separate consumers.
Noisy-flight qualification, obstacle planning and ROS/PX4 integration remain open.
