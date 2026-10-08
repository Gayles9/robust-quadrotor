# 0012: True-State Cascaded Position Control and Baseline Missions

See the [interface guide](../design/position-control.md) and
[verification record (ZIP)](../../evidence/development-records.zip).

## Scope

Position control turns a desired path into attitude and thrust requests for
the inner cascade. The mission layer supplies takeoff, tracking and virtual
landing references, checks limits, and decides when the run completes or stops.
It shares the attitude controller's motor, drag and RK4 plant step, so both
simulations use the same physical equations.

The evidence here uses true-state feedback: the controller receives the
simulator's exact state. [Estimated feedback](0013-estimated-state-mission-feedback.md)
is evaluated separately with explicit sensor timing and uncertainty.
Landing means converging to the virtual launch plane in the airborne model,
not making physical ground contact. Abort stops simulation at the detected
epoch; it is not a hardware emergency-control policy.

## Controller and conventions

NED world, FRD body, scalar-first Hamilton q_WB, R_WB body to world. All position,
velocity and acceleration references are NED SI vectors. A pure stateless PD law:

```math
\mathbf a_{\mathrm{req},W}=\mathbf a_{r,W}+K_p(\mathbf p_{r,W}-\mathbf p_W)+K_v(\mathbf v_{r,W}-\mathbf v_W).
```

Here $`K_p,K_v`$ are diagonal gain matrices and subscript $`r`$ denotes reference values.

$`K_p`$ has units 1/s², $`K_v`$ 1/s. No integral: no windup state and no claim of zero
steady error under persistent force or mass mismatch. Limit acceleration per
axis, with positive vertical bound strictly below nominal gravity. Set the
desired down-axis force vector $`\boldsymbol\ell_W=m_n(g_n\mathbf e_3-\mathbf a_{\mathrm{lim},W})`$.
Its vertical component is strictly positive. Reduce its horizontal norm to
$`\ell_z\tan\theta_{\max}`$ if necessary; then clip its magnitude to the positive
configured thrust interval, preserving direction. Record every limit flag and
the resulting feasible nominal acceleration $`g_n\mathbf e_3-(T/m_n)\mathbf b_3`$.

Let $`\mathbf b_3=\boldsymbol\ell/\lVert\boldsymbol\ell\rVert`$ and $`\mathbf y_\psi=[-\sin\psi,\cos\psi,0]^T`$ for yaw $`\psi`$.
Construct $`\mathbf b_1`$ by normalizing $`\mathbf y_\psi\times\mathbf b_3`$; then $`\mathbf b_2=\mathbf b_3\times\mathbf b_1`$ and $`R_{r,WB}=[\mathbf b_1\ \mathbf b_2\ \mathbf b_3]`$. Positive $`b_{3,z}`$
prevents the heading singularity and makes the horizontal body-x heading equal
to the requested yaw. Convert this orthonormal matrix to a unit quaternion with
a largest-component branch, including yaw near a half-turn. Collective T is a
positive magnitude; actual rotor force remains -T along body z.

The existing inner loop receives held references, not angular feedforward.
Nominal collective endpoints must have feasible zero-moment rotor allocations;
convexity of squared-speed allocation then gives feasibility throughout the
interval. Nominal speed bounds must fit truth bounds. Validate hover thrust lies
strictly inside the interval. All public arrays are finite, owned, read-only
float64 with explicit shapes; malformed input fails before execution.

## References, state machine and execution

Hold and rest-to-rest quintic segments provide p/v/a analytically. The quintic
blend is 10*s³-15*s⁴+6*s⁵, s in [0,1], with exact endpoint p and zero v/a.
This is a simple reference primitive, not a minimum-snap optimizer. An explicit
position-step diagnostic is permitted only in TRACK. Constant yaw per mission.

Ordered phases: INITIALIZE, TAKEOFF, TRACK, LAND, then COMPLETE or ABORT.
Time-scheduled phase changes do not imply a waypoint has actually been reached;
tracking error is measured throughout. Completion requires endpoint position and
speed within configured tolerances continuously for the completion dwell.
Landing timeout aborts; any guard failure has priority over completion.
Terminal phases are absorbing. Guards check actual NED geofence and tilt at
every recorded plant epoch, including initialization and the final epoch.
Commands enforce acceleration, tilt, thrust and rotor bounds; command limits
are not guarantees on actual state. Continuous inter-sample safety is not claimed.

Integer strides: plant 400 Hz, attitude/rate 100 Hz, position 50 Hz. The outer
stride is a multiple of the inner stride. At each epoch: record state and analytic
reference; evaluate guards and mission transition; stop if terminal; update outer
reference if due; update inner command if due; advance one plant interval using
the unchanged stage-resolved motor/RK4 step. No terminal command. Log actual and
commanded speeds, requested/limited/allocated moments, thrust and both layers'
limit flags on their own clocks. A failure ledger retains unsuccessful trials.

## Model-based gains and frozen acceptance

Use the preceding illustrative 1 kg model and inner gains unchanged. Position
Diagonal entries $`K_p=[1,1,2.25]`$ s⁻², $`K_v=[1.8,1.8,3]`$ s⁻¹ give ideal local natural frequencies
[1,1,1.5] rad/s, slower than the inner [6,6,4] rad/s. Acceleration bounds [2,2,2]
m/s², reference tilt 20 degrees, collective 2..18 N; actual-tilt guard 35 degrees.
Geofence [-3,-3,-3]..[3,3,.5] m. These are illustrative simulation settings,
not calibrated vehicle limits. No acceptance or gain adjustment on held-out data.

Fixed scenarios: airborne 60-second hover segment, 1 m square with smooth
6-second edges and 1-second waypoint holds, 0.5 m vertical reference step, mild
constant wind [.5,-.3,0] m/s with body drag [.1,.1,.15] kg/m, and a square repeat
at half plant step with control periods fixed. Initialization 1 s, takeoff 4 s
to 1 m altitude, landing 4 s, terminal dwell .5 s with .08 m/.08 m/s tolerances,
8 s landing timeout. Hover/step diagnostics include takeoff and landing too.

Development seeds 2000..2002 and held-out seeds 70000..70009 independently vary
initial position in ±.02 m, velocity in ±.02 m/s, roll/pitch/yaw rotation-vector
components in ±2 degrees and rates in ±.02 rad/s using PCG64 and a declared stream
identity. Randomness changes initial conditions only, not gains or truth physics.
All 10 held-out nominal square missions must complete without guard violations;
each full-mission time-weighted position RMSE must be <.15 m. No contiguous
actuator limiting lasting >.5 s and no limiting in the final second. Each fixed
mission must complete; mild wind remains a diagnostic of finite PD offset.
Hover-segment error must stay below .08 m; vertical-step settling into simultaneous
.08 m/.08 m/s bands must occur within 6 s. Refinement compares the full common
time grid before termination, requiring position <.005 m and attitude <.05 degree.

Metrics: full-grid position RMSE/peak/final error, velocity error, attitude error,
settling after the last reference event, control effort (integral squared thrust
and moment), limit durations and longest consecutive actuator limiting, actual
tilt, waypoint endpoint errors and complete phase/termination record. No plot
decimation may affect metrics. Evidence includes complete histories, protocol and
source digests, software provenance and all planned identities. Before execution,
the evidence format was specialized to finite JSON metadata binding per-trial
compressed NPZ histories by exact-byte SHA-256. This avoids oversized raw exports
without dropping samples. New-directory publication refuses overwrite.

## Verification

1. Independent force reconstruction, NED signs, all headings/near-pi yaw, SO(3)
   orthogonality/determinant, clipping bounds, feasibility endpoints, local PD
   coefficients, finite/shape/type/ownership and extreme arithmetic tests.
2. Quintic derivatives against independent finite differences, endpoint
   continuity, hold/step semantics, exact phase boundaries, terminal dwell/reset,
   guard precedence, timeout and absorbing terminal states.
3. Hover equilibrium, integer clocks/holds, future-reference causality, truth
   versus nominal independence, plant-step reuse, early abort and no terminal
   commands, invalid configuration and read-only result contracts.
4. Fresh full regression/static gates, complete old fixed attitude campaign,
   development before held-out runs, worker-count determinism, failure retention,
   corrupted ledger/metrics/clocks rejection, plot rendering and visual inspection.
5. Reconstruct the actual forces, moments and commands independently from
   recorded histories to check the physical and feedback paths.
