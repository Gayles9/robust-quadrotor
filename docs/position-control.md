# True-State Position Control and Baseline Missions

The position/velocity outer loop closes the translational part of the existing
[attitude/rate cascade](control.md). It consumes NED position and velocity, a
position/velocity/acceleration reference and constant mission yaw. It outputs a
feasible collective thrust and body-to-world orientation for the unchanged inner
loop. [ADR 0012](decisions/0012-position-control-and-missions.md) defines the scope
and predeclared acceptance; the [verification record](progress/2026-09-24-position-control-and-missions.md)
records actual results and limitations.

This guide documents **true-state simulation feedback**. The separate
[estimated-feedback integration](estimated-feedback.md) reuses the same control
laws and plant with causal sensor/ESKF inputs. Truth
plant parameters are separately supplied; the pure controller has only its own
nominal mass/gravity, gains and limits. There is no hidden integral, RNG, sensor,
truth-parameter lookup, drag compensation or reference-rate feedforward.

## Interfaces and ownership

| Module / interface | Responsibility and contract |
| --- | --- |
| `position_control.PositionReference` | Owned NED `(3,)` position m, velocity m/s, acceleration m/s²; finite signed yaw radians |
| `PositionControllerParameters` | Positive diagonal position/velocity gains, nominal mass/gravity, acceleration/tilt/thrust bounds |
| `compute_position_control` | Pure feedback and force-direction construction; returns `PositionControlCommand` |
| `missions.MissionSegment`, `sample_segment` | Hold, rest-to-rest quintic, or explicit step reference and analytic derivatives |
| `MissionPlan`, `mission_reference` | Connected reference segments with ordered phases and right-continuous time selection |
| `MissionSafetyLimits`, `mission_guard_reason` | Closed NED geofence and measured tilt bound; geofence wins simultaneous violations |
| `MissionState`, `advance_mission` | Pure supervisor transition, landing dwell/reset, timeout and absorbing terminal states |
| `mission_simulation.MissionNumerics` | Positive plant step and integer controller strides; explicit execution/allocation budget |
| `simulate_mission` | True-state feedback, guards and multirate execution using the existing motor/RK4 plant |
| `MissionResult` | Owned truth, reference and command histories on explicit clocks, terminal phase and abort reason |

Arrays are copied to independent, C-contiguous, read-only float64 storage; phase
codes use int64 and limit flags bool. Frozen dataclasses prevent field reassignment.
Read-only NumPy flags are an API ownership contract, not a security boundary against
a caller deliberately re-enabling writes. The runner snapshots configurations and
initial arrays. Public boundaries reject wrong shapes, non-real/Boolean array
dtypes, scalar Booleans, nonfinite values, invalid units-domain limits and arithmetic
overflow rather than silently sanitizing them. Unit quaternion inputs are required.

## Position/velocity law and local dynamics

Use the existing NED world and FRD body frame contract: positive world z is down,
gravity is `g e3`, and rotor force is `-T e3` in body coordinates. Let p,v be world
position and velocity, and p_r,v_r,a_r their references. An asterisk denotes
elementwise multiplication in the following executable-form equations:

    a_requested_W = a_r + K_p * (p_r - p) + K_v * (v_r - v)

Here K_p is a positive `(3,)` diagonal gain vector in s^-2, K_v in s^-1, and every
acceleration is in m/s². Acceleration feedforward supplies known reference curvature;
feedback removes measured tracking error. With a perfectly realized acceleration and
a dynamically consistent reference, each axis obeys

    e_ddot + K_v * e_dot + K_p * e = 0,   e = p - p_r.

Thus natural frequency is sqrt(K_p) and damping ratio K_v/(2 sqrt(K_p)). The example
uses K_p=[1,1,2.25], K_v=[1.8,1.8,3]: frequencies [1,1,1.5] rad/s and damping
[.9,.9,1]. These are slower than the existing inner-loop local frequencies [6,6,4]
rad/s. Faster update rates alone do not establish time-scale separation; both the
loop dynamics and sampling are relevant. This local equation assumes no actuator
lag, attitude-tracking error or active limits and is not a stability proof for the
complete nonlinear sampled system. The experiments retain those effects.

There is no integrator. A persistent external force d_W approximately produces
e_W=d_W/(m*K_p) at a matched, unsaturated equilibrium, rather than zero position
error. Constant mass/gravity mismatch likewise produces a vertical offset. Mild-wind
testing characterizes this baseline limitation; it does not justify a universal
disturbance-rejection claim. With no integral state, anti-windup/reset logic is not
needed in this increment.

## Acceleration, tilt and thrust feasibility

The mapping is deliberately explicit and sequential, not an optimal projection:

1. Clip a_requested componentwise to ±`maximum_acceleration_W`, obtaining a_c.
   The vertical bound must be strictly less than nominal gravity.
2. Form the desired body-down force vector l_W=m_n(g_n e3-a_c). It has positive
   down component l_z, even when upward/downward acceleration is limited.
3. If ||l_xy|| > l_z tan(theta_max), scale only l_xy to that radius. This bounds
   the desired tilt without reversing horizontal direction.
4. Let b3=l/||l|| and T=clip(||l||,T_min,T_max), where 0<T_min<T_max. The implied
   feasible nominal acceleration is g_n e3-(T/m_n)b3.

`PositionControlCommand` records `requested_acceleration_W`,
`feasible_acceleration_W`, `q_reference_WB`, `collective_thrust`, and separate
`acceleration_limited`, `tilt_limited`, `thrust_limited` flags. The feasible
acceleration is a command-space quantity under perfect realization, **not** the
current vehicle acceleration. It may differ from the component-clipped acceleration
after tilt or thrust limiting. No current-attitude cosine division is applied.

The example acceleration bound is [2,2,2] m/s², desired tilt 20 degrees and thrust
2..18 N. These are explicit illustrative parameters, not hardware ratings. Nominal
hover thrust must lie strictly inside the thrust interval. The composed runner
verifies that both collective endpoints admit zero-moment allocation with the
nominal rotor model. Since zero-moment squared speeds depend linearly on T and the
rotor intervals are convex, the intervening collective interval is feasible too.
The existing inner allocator can then reduce moment demand while preserving T.
Truth and nominal rotor-speed ranges must be compatible; hidden clipping by the
truth motor is prohibited.

Independent tests reconstruct `R_WB @ [0,0,-T] / m + [0,0,g]`, check all acceleration
axis signs, recover the requested acceleration when unsaturated, and verify the
combined limit projection against a separate trigonometric oracle. A seeded set
checks finite bounded commands for large demands. Overflow is rejected before
clipping, so clipping cannot conceal invalid intermediate arithmetic.

## Desired orientation and yaw

For requested yaw psi, define horizontal y_h=[-sin(psi),cos(psi),0]. Then

    b1 = normalize(y_h cross b3)
    b2 = b3 cross b1
    R_reference_WB = [b1 b2 b3]

The columns are the desired FRD axes expressed in NED. Since b3_z>0, the cross
product is nonzero and the horizontal projection of b1 points along the requested
yaw heading [cos(psi),sin(psi)]. This is an exact heading convention, not merely a
rotation-matrix column label. Positive north acceleration at zero yaw requires
negative pitch; positive east acceleration requires positive roll. Upward acceleration
requires more collective thrust. These signs follow directly from negative body-z thrust.

A largest-component matrix-to-quaternion conversion avoids dividing by a small scalar
component near a half-turn yaw. The quaternion is normalized and its sign canonicalized;
the existing inner-loop error remains invariant to quaternion sign. Tests verify proper
rotation (orthogonality and determinant +1), force reconstruction and exact horizontal
heading at signed, near-half-turn and wrapped yaw angles. The mission keeps yaw constant;
outer-loop orientation references are held between updates. No angular-trajectory
feedforward or global attitude-domain guarantee is implied.

## Reference primitives and mission states

`MissionSegment` declares a phase, positive duration, connected start/end NED positions
and `ReferenceKind`. A hold requires equal endpoints. For a smooth segment of length D,
let s=t/D in [0,1], delta_p=p_end-p_start. Its quintic blend is

    h(s)   = 10 s^3 - 15 s^4 + 6 s^5
    h'(s)  = 30 s^2 (1-s)^2
    h''(s) = 60 s (1-s) (1-2s)
    p = p_start + h(s) delta_p
    v = h'(s) delta_p / D
    a = h''(s) delta_p / D^2.

Position, velocity and acceleration connect continuously to endpoint holds; endpoint
values are returned exactly. This is a simple rest-to-rest primitive, not a minimum-snap
optimizer. Independent finite differences test both derivatives. A STEP is allowed only
in TRACK for the vertical-step diagnostic; its position jumps at the segment start and
its supplied v/a are zero on either open side. No impulsive acceleration is simulated.

The phase order is INITIALIZE → TAKEOFF → TRACK → LAND, then COMPLETE or ABORT.
INITIALIZE holds an already airborne, explicitly initialized state with explicit motor
speeds; it is not stationary sensor calibration or physical arming. TAKEOFF/LAND are
smooth references between a virtual launch plane and cruise altitude. TRACK can combine
holds, smooth waypoint segments and the explicit step diagnostic. Segment boundaries
are time-scheduled and right-continuous. A transition to the next waypoint does not
prove that the vehicle reached the previous one; endpoint errors remain in the metrics.

After the final landing reference ends, the target remains constant. Completion requires
position error ≤.08 m and speed ≤.08 m/s continuously at every observed plant epoch for
.5 s in the declared campaign. Leaving either band resets the dwell. An 8 s landing
timeout aborts. Safety violations take priority over completion; COMPLETE and ABORT
are absorbing. No command or plant advancement occurs at or after termination.

The actual-state geofence is the closed NED box [-3,-3,-3]..[3,3,.5] m, and the actual
body-z tilt guard is 35 degrees in the campaign. The runner checks these at **every
plant epoch**, not only controller ticks. It also stops on the existing inner loop's
local attitude-error domain violation. Desired-tilt/thrust limits constrain commands;
they do not guarantee actual tilt or containment between samples. Abort records the
detected terminal state and reason and stops numerical execution. No emergency descent,
collision response, ground contact or motor-disarming policy is represented.

## Execution and returned data

The example plant/attitude/position rates are 400/100/50 Hz. `MissionNumerics` supplies
the positive plant step and integer strides, with the position stride a multiple of
the attitude stride. The full reference-plus-timeout horizon must fit `maximum_steps`;
budget failure is configuration rejection, never successful mission completion.

At each t_k=k*dt: record truth and the analytic reference; check actual-state guards and
advance the supervisor; stop if terminal; compute the outer demand if due; check the
local attitude domain; compute the inner command if due; advance the plant. The existing
`attitude_simulation._plant_step` supplies stage-resolved exact motor response, rotor
wrench, air-relative quadratic drag, and projected RK4. That shared internal helper
is reused without changing historical run generation or the preceding attitude harness.

| `MissionResult` fields | Clock / interpretation |
| --- | --- |
| `time_s`, `position_W`, `velocity_W`, `q_WB`, `omega_B` | Initial through terminal truth epochs |
| `actual_rotor_omega`, `actual_moment_B` | Actual speeds and rotor-only moment at truth epochs |
| `reference_position_W`, `reference_velocity_W`, `reference_acceleration_W` | Analytic reference at every truth epoch, not the held outer-loop sample |
| `phase`, `abort_reason` | Integer phase per truth epoch; reason only for ABORT |
| `control_time_s`, `q_reference_WB`, `collective_thrust` | Inner epochs; held outer demand |
| `commanded_rotor_omega`, `desired_omega_B` | Inner epochs; rotor rad/s and requested FRD body rate rad/s |
| `moment_requested_B`, `moment_limited_B`, `allocated_moment_B`, `moment_scale` | Inner epochs; nominal N m demand/clip/allocation and ray scale |
| `inner_limit_flags` | Inner epochs; rate-demand clip, moment clip, allocation scale |
| `position_control_time_s`, `requested_acceleration_W`, `feasible_acceleration_W` | Outer epochs; requested and feasible NED acceleration |
| `outer_limit_flags` | Outer epochs; acceleration, tilt, thrust limiting |

Truth histories have N+1 rows; command histories contain only actual updates before
termination and allow a partial last hold interval. An immediate abort has one truth row
and zero commands. Tests verify both clocks, holds, motor lag, future-reference causality,
initial/between-tick guard failures, timeout, terminal-command exclusion, deterministic
repetition and independent truth/nominal sensitivity.

## Reproducibility and evidence

The frozen experiment has fixed, development, validation and smoke partitions. Only
initial conditions are stochastic; explicit PCG64/SeedSequence identifiers and draw order
are in `validation_protocol`. The development seeds are 2000–2002; held-out nominal square
seeds are 70000–70009. No seeded sensor noise or estimator is present. Smoke is short
execution evidence only and cannot establish mission performance.

The CLI stores `report.json` and one compressed `trial-NNN.npz` per numerical success,
with failed identities and diagnostics retained in the ledger. The report binds each
NPZ's exact bytes by SHA-256, states the protocol/source digests, software provenance,
UTC generation time and worker count. It is published last into a new-only directory;
an interrupted directory without the report is incomplete. This is a new experimental
bundle, not a sensor-run artifact schema migration. It does not claim crash durability
or authentication against malicious replacement of both report and history.

The loader rejects duplicate JSON keys, nonfinite metadata, wrong formats/filenames,
digest mismatches and pickle-dependent arrays. It validates shapes/clocks, original
initial conditions, all analytic references, mission phases, held outer demands,
metrics and the complete ordered trial ledger. Digests are checked on the same bytes
subsequently decoded. Missing/reordered/duplicate trials, failed executions and failed
criteria cannot become an aggregate pass. Metrics use all samples; plots read stored
validated evidence rather than rerunning or selecting favorable seeds.

RMSE is sqrt(integral ||p-p_r||² dt / duration), using trapezoidal quadrature over the
complete mission, including takeoff and landing. Command-limit durations use actual
hold widths, including partial final intervals. Actuator limiting is moment clipping,
allocation scaling, or a commanded rotor at a speed bound; longest consecutive duration
and final-second overlap are reported separately. Rate/outer limits have their own
diagnostics. Control effort is integral T_command² dt and integral ||tau_actual||² dt;
these are signal-effort measures, not electrical energy. Settling uses the first epoch
after the last simultaneous position/speed-band violation within the constant vertical
step interval. Refinement halves only plant dt and compares the full common truth grid.

## Run the complete verification campaign

Use the existing locked Python 3.12 / uv 0.12.3 environment; no dependencies were added.
Choose a new evidence directory outside Git:

~~~bash
uv sync --locked
make check
uv run pytest -W error
MISSION_EVIDENCE=/tmp/quadrotor-mission-evidence
mkdir -p "$MISSION_EVIDENCE"
uv run python -m experiments.position_control_validation --partition development --workers 3 --output "$MISSION_EVIDENCE/development"
uv run python -m experiments.position_control_validation --partition fixed --workers 4 --output "$MISSION_EVIDENCE/fixed"
uv run python -m experiments.position_control_validation --partition validation --workers 4 --output "$MISSION_EVIDENCE/validation"
uv run python -m experiments.plot_position_control --input "$MISSION_EVIDENCE/fixed" --output "$MISSION_EVIDENCE/fixed-plots"
uv run python -m experiments.plot_position_control --input "$MISSION_EVIDENCE/validation" --output "$MISSION_EVIDENCE/validation-plots"
~~~

Read the verification record before interpreting the numerical outcomes. This bounded
true-state baseline does not establish arbitrary-gain stability, hardware performance,
sensor robustness, estimator/control timing correctness, or safe real-world landing.
