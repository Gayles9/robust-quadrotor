# ADR 0016: Bounded trajectory timing and true-state mission execution

Date: 2026-09-25. Status: accepted for this implementation.

## Audit and scope frozen before implementation

Audited main: `76db6eefea09aa25be236b1246cdc21ba44d1557`, tree
`1e7ccc312ffe3bcbf88b488313e0de1c9bfe31c1`. Post-merge CI run
`36142551257` passed. A fresh run of the 36 minimum-snap tests passed, and source
review found no new solver defect. Its constructor intentionally checks numerical
representation, not C3 continuity or rest endpoints; integration must check those
properties before using caller-constructed trajectories.

Implement only nominal reference feasibility bounds, bounded uniform retiming,
and minimum-snap segments in the existing true-state mission supervisor. Keep
controller gains, plant, motors, observer and original acceptance protocols intact.
No geometric control, estimator tuning, obstacles, torque feedforward, global
time-optimal search, fault accommodation, ROS/PX4 or hardware flight qualification.

## Reference feasibility contract

For NED reference acceleration a and jerk j, define u = g e3 - a. In the nominal
drag-free model, thrust is m ||u|| and body-down is u/||u||. Require u_z > 0.
Bound position inside the declared box, speed, component acceleration, collective
thrust, tilt, and the norm of the fixed-yaw reference's angular velocity.

Convert each derivative polynomial to Bernstein form and apply a fixed number
of midpoint subdivisions. Every subinterval lies in the convex hull of its
control points. Use those hulls, with explicit floating-point padding, to bound
the entire curve, not just sampled times. With z_min > 0, valid conservative
bounds are thrust >= m z_min, tilt <= atan(horizontal_u_max/z_min), and
||omega_reference|| <= ||j||_max/z_min. The last inequality follows from the
existing fixed-heading attitude construction; it includes the changing rotation
about body-down even though the supplied yaw heading is constant.

Acceptance means these sufficient nominal reference bounds fit the limits.
Rejection may be conservative; it does not prove the actual curve violates a
limit. Floating-point padding is an engineering guard, not a formal interval
arithmetic proof. Passing does not establish rotor torque/motor-lag feasibility,
closed-loop tracking, obstacle avoidance or safety under model mismatch.

Uniform retiming is restricted to C3, rest-to-rest trajectories. Scaling all
durations by alpha preserves geometry and the fixed-time optimum, scales the
r-th derivative by alpha^-r and snap cost by alpha^-7. Try alpha = 1 followed
by factors of 1.25, capped at 8, at most 12 attempts. Record every attempt and
return explicit failure if no candidate passes. Geofence failure cannot be
repaired by this geometry-preserving policy. There is no hidden gain adjustment
or claim of minimum feasible time.

## Mission and acceptance

Add a minimum-snap reference kind carrying an owned trajectory. Require C3
internal joins and zero initial/final v/a/j within stated numerical tolerances,
connected endpoints, and matching duration. Preserve existing phase scheduling,
terminal dwell/timeout, truth guards and control clocks. Preflight every new
polynomial against the nominal controller acceleration/thrust/tilt/rate bounds
and the mission geofence before any plant step. This package supports true-state
execution; estimated-state polynomial missions remain a separate validation step.

Fixed evidence mission: 1 s initialization; 4 s minimum-snap takeoff to [0,0,-1];
track [0,0,-1], [1,0,-1.2], [1,1,-0.8], [0,1,-1], [0,0,-1] with initial durations
[1,1,1,1] s and the bounded retimer; 1 s hold; 4 s minimum-snap landing.
Planning limits: speed 0.6 m/s, component acceleration 0.6 m/s², tilt 10 degrees,
reference angular speed 0.4 rad/s, thrust [7,13] N, and the existing mission box.
The exact existing true-state controller and motor parameters remain unchanged.

Five declared simulations: matched nominal; the same mission with half the plant
step and unchanged controller periods; two opposite deterministic initial p/v/
roll/rate offsets; and the existing mild wind [0.5,-0.3,0] m/s with drag
[0.1,0.1,0.15] kg/m. They are bounded engineering cases, not a statistical campaign.
For each: complete, full-mission position RMSE <= 0.15 m, peak <= 0.25 m, original
terminal 0.08 m/0.08 m/s/0.5 s dwell within 8 s timeout, and no inner or outer
limiting. Refinement agreement: <= 0.005 m position and <= 0.05 degree attitude
at shared epochs. Preserve any failed attempts and their explanation.

Unit verification must cover analytic between-endpoint excursions, independent
dense evaluations/finite-difference attitude rates, derivative/time-scaling laws,
conservative rejection, exhausted budgets, numerical/ownership contracts,
boundary sampling, malformed/discontinuous trajectory rejection, and preflight
failure before execution. Finish with warnings-as-errors tests, lint and typing.
