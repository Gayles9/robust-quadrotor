# ADR 0015: Fixed-duration minimum-snap position trajectories

## Purpose and scope

A minimum-snap trajectory is a smooth path that minimizes rapid changes in
acceleration. More precisely, snap is the fourth derivative of position, and
the solver minimizes its integrated squared magnitude. The waypoint positions
and segment durations are inputs; the solver determines the connecting curves.

The reusable **fixed-duration position solver** supports:

- NED waypoints of shape `(segments + 1, 3)` and supplied positive durations;
- seventh-degree polynomials on each local normalized interval `[0, 1]`;
- exact waypoint interpolation, continuity through jerk, and prescribed initial
  and final velocity, acceleration and jerk (zero by default);
- minimization of the sum of integrated squared fourth time derivatives;
- deterministic float64 solution using NumPy only, explicit conditioning and
  residual checks, owned read-only outputs, and derivatives through snap;
- a fixed-yaw adapter returning the existing `PositionReference` type;
- a reproducible small numerical example with independently checked metrics.

It does not add automatic segment timing, actuator/geofence feasibility,
obstacle constraints, yaw optimization, a mission-execution adapter, geometric
control, estimator tuning, new dependencies or ROS/PX4 integration. A smooth
polynomial is not a dynamically feasible or qualified flight trajectory.
[ADR 0016](0016-trajectory-feasibility-and-missions.md) describes the separate
feasibility, retiming and mission-execution layer.

## Mathematical contract

For segment i of duration T_i, use s=(t-t_i)/T_i and
`p_i(s)=sum(k=0..7, c_ik s^k)`. Derivative r in physical time is the
normalized polynomial derivative divided by T_i^r. The cost is

`J = sum_i T_i^-7 sum_(k,l=4..7)
     c_ik dot c_il (k!/(k-4)!) (l!/(l-4)!)/(k+l-7)`.

Waypoints and endpoint derivatives are fixed; internal velocity, acceleration
and jerk are the optimization variables shared by adjacent segments. A fixed
Hermite map turns those endpoint data into normalized polynomial coefficients.
This eliminates equality constraints structurally, without inverting an
indefinite KKT matrix. Solve the reduced positive-definite quadratic after time,
position and diagonal variable scaling. Reject ill-conditioned or numerically
unresolved problems rather than regularizing the objective silently.

Uniqueness: a feasible variation with zero snap is piecewise cubic. C3
continuity makes it one global cubic, and zero initial position/velocity/
acceleration/jerk makes that cubic identically zero. Thus the reduced quadratic
is positive definite for positive durations. This is a fixed-time spline
optimality statement, not a nonlinear closed-loop stability proof.

## Verification

1. Recover the independent single-segment rest-to-rest polynomial
   `35s^4 - 84s^5 + 70s^6 - 20s^7` and cost `100800 ||delta_p||^2/T^7`.
2. Match waypoint and endpoint conditions and C3 knot continuity in physical
   units; include nonuniform durations and nonzero boundary derivatives.
3. Match an independently assembled coefficient-space equality-constrained
   solution, and verify nonnegative cost increase under feasible perturbations.
4. Match integrated snap against Gauss-Legendre quadrature; improve on a
   feasible stopped-at-each-waypoint seventh-degree reference.
5. Verify translation/rotation and time-scaling laws, including `J -> J/a^7`
   when all durations scale by a and prescribed derivatives scale by a^-r.
6. Exercise explicit shape, type, finite, duration, out-of-range time,
   conditioning and unrepresentable arithmetic failures and array ownership.
7. Warning-strict lint, formatting, typing and tests check the implementation.
   Planner evaluation keeps feedback gains, sensor distributions and flight
   acceptance limits fixed.

Related literature: Daniel Mellinger and Vijay Kumar, *Minimum snap trajectory
generation and control for quadrotors*, ICRA 2011, doi:10.1109/ICRA.2011.5980409;
Declan Burke, Airlie Chapman and Iman Shames, *Generating Minimum-Snap Quadrotor
Trajectories Really Fast*, https://arxiv.org/abs/2008.00595.
This implementation uses a bounded dense reduced solve; it does not implement
or claim the latter paper's linear-time large-scale algorithm.
