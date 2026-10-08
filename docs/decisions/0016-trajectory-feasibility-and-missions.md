# ADR 0016: Bounded trajectory timing and true-state mission execution

## Purpose and scope

A mathematically smooth path can still demand more speed, thrust or tilt than
the vehicle is configured to provide. This layer checks conservative bounds
over the entire polynomial path and, where possible, slows the path uniformly
until the bounds fit. It also connects minimum-snap references to the true-state
mission supervisor.

The trajectory constructor checks numerical representation, not continuity
through jerk (C3) or rest endpoints. Mission integration therefore checks those
properties before accepting caller-constructed trajectories. Controller gains,
plant equations and estimator assumptions are unchanged by retiming. Obstacle
avoidance, global time optimization and hardware flight qualification are
outside this model.

## Reference feasibility contract

For NED reference acceleration $`\mathbf a`$ and jerk $`\mathbf j`$, define $`\mathbf u=g\mathbf e_3-\mathbf a`$.
In the nominal drag-free model, thrust is $`m\lVert\mathbf u\rVert`$ and body-down is $`\mathbf u/\lVert\mathbf u\rVert`$. Require $`u_z>0`$.
Bound position inside the declared box, speed, component acceleration, collective
thrust, tilt, and the norm of the fixed-yaw reference's angular velocity.

Convert each derivative polynomial to Bernstein form and apply a fixed number
of midpoint subdivisions. Every subinterval lies in the convex hull of its
control points. Use those hulls, with explicit floating-point padding, to bound
the entire curve, not just sampled times. With a lower bound $`z_{\min}>0`$ on $`u_z`$ and upper bound $`U_h`$ on $`\lVert\mathbf u_{xy}\rVert`$,
valid conservative bounds are $`T\geq mz_{\min}`$, tilt $`\vartheta\leq\arctan(U_h/z_{\min})`$,
and $`\lVert\boldsymbol\omega_r\rVert\leq\lVert\mathbf j\rVert_{\max}/z_{\min}`$. The last inequality follows from the
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

A minimum-snap reference carries an owned trajectory. The mission adapter
requires C3 internal joins, zero initial/final v/a/j within the stated numerical
tolerances, connected endpoints and matching duration. It uses the existing
phase scheduling, terminal dwell/timeout, truth guards and control clocks.
Before any plant step, preflight checks compare each polynomial with the nominal
controller acceleration/thrust/tilt/rate bounds and mission geofence. This
interface supports true-state execution; [estimated-state polynomial missions](../design/trajectory-missions.md)
have separate evidence and assumptions.

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

Unit tests cover analytic between-endpoint excursions, independent dense
evaluations, finite-difference attitude rates, derivative/time-scaling laws,
conservative rejection, exhausted budgets, numerical/ownership contracts,
boundary sampling, malformed/discontinuous trajectory rejection and preflight
failure before execution. Warnings-as-errors tests, lint and typing complement
these numerical and interface checks.
