# Baseline Attitude and Body-Rate Control

The inner loop answers a simple question: which rotor commands will turn the
vehicle toward a desired orientation? I use a cascade so the attitude and rate
responses can be checked separately, then tested together with motor delay.
Attitude means orientation; body rate means how fast that orientation is changing.
The cascade first requests a turning rate, then the moment needed to achieve it.
Position tracking is handled by the outer loop.

Start with [system design](../guides/system-design.md) for the complete loop or
[controller tradeoffs](../results/controller-tradeoffs.md) for the measured performance
and limitations. The equations below are implemented in
[attitude_control.py](../../src/quadrotor_math/attitude_control.py) and exercised by
[attitude_simulation.py](../../src/quadrotor_math/attitude_simulation.py).

The implemented inner loop converts an orientation reference and a collective-thrust
demand into bounded rotor-speed commands. It includes a pure controller and a deterministic
true-state simulation of all three translation and three rotation axes. The
[acceptance protocol](../decisions/0011-baseline-attitude-control.md) defines the
tested cases and criteria.

This page describes the inner loop, which alone does not regulate position or altitude.
The [position/mission layer](position-control.md) supplies translational feedback.
The [estimated-feedback integration](estimated-feedback.md) supplies the estimated
state from the error-state Kalman filter (ESKF). Tests with exact simulated
state isolate controller behavior; they do not establish estimated-state or
hardware performance.

## Frames, signals and interface boundaries

Use the [NED/FRD frame contract](../architecture/frame-contract.md). A Hamilton scalar-first
unit quaternion q_WB maps FRD body vectors into NED world coordinates through R_WB.
The reference q_reference_WB describes the desired body orientation relative to the same
world. Its physical orientation is unaffected by changing its quaternion sign.
Both input quaternions must already be unit length within the existing rotation utility's
tolerance; the controller does not silently normalize malformed inputs.

| Boundary | Responsibility | Input/output contract |
| --- | --- | --- |
| attitude_control.attitude_error_body | Shortest relative rotation vector | Two (4,) unit quaternions → (3,) current-body radians |
| attitude_control.body_rate_moment | Rate P feedback with gyroscopic compensation | Current/desired (3,) FRD rad/s, full (3,3) inertia kg m², positive (3,) gains 1/s → unbounded (3,) FRD N m |
| attitude_control.allocate_limited_body_moment | Preserve collective while reducing infeasible moment | Nonnegative collective N, (3,) FRD N m and nominal RotorParameters → LimitedMomentAllocation |
| attitude_control.compute_attitude_control | Compose the bounded attitude/rate/allocation law | q_WB, omega_B, q_reference_WB, collective_thrust, AttitudeControllerParameters → AttitudeControlCommand |
| attitude_simulation.simulate_attitude_control | Fixed-grid true-state execution | Explicit initial state/motor speeds, independent truth body/rotor/world, nominal controller and AttitudeControlSchedule → AttitudeSimulationResult |

Arrays use independent C-contiguous read-only float64 storage. The execution
boundaries reject malformed shapes, non-real dtypes, NaN/Inf, invalid scalar Booleans,
arithmetic overflow, nonpositive gains and invalid quaternions. Inertia must be exactly
symmetric and positive definite. Rotor coefficients and motor time constant must be
positive, motor parameters complete, and speed bounds ordered and safe to square.
Structural configuration dataclasses and executable controllers validate different
domains; the controller applies the stricter requirements needed for its calculations.

Rate/moment/angle limits and nominal inertia/rotors are explicitly supplied in
AttitudeControllerParameters. No controller parameter is inferred from the truth plant.
The controller has no RNG, clock, hidden state, estimator access or integral state.
Failed calls return no partial command or simulation result and do not change inputs.
Numerical validation establishes a usable domain, not a proof of stability for arbitrary
caller-supplied gains, plant mismatches or sampling rates.

## Relative rotation and cascaded law

Let R = R_WB and R_d = R_reference_WB. The relative rotation RᵀR_d is expressed in
the **current body** coordinates. Its principal rotation vector e_B has direction
along the required rotation axis and norm equal to the shortest angular error:

```math
\mathbf e_B=\mathrm{Log}(R^\mathsf{T}R_d)^\vee.
```

Here Log maps a rotation to a skew-symmetric matrix; vee extracts its three-vector.
The implementation computes the equivalent relative quaternion
q_e = q_WB* ⊗ q_reference_WB, where * is quaternion conjugation and ⊗ Hamilton
multiplication. Write q_e = (w_e, v_e) and choose the representation with nonnegative
scalar part. At an exact half-turn, use the first nonzero positive vector component.
For s = ||v_e|| > 0:

```math
\theta=2\mathrm{atan2}(s,w_e),\qquad
\mathbf e_B=\frac{\theta}{s}\mathbf v_e,\qquad 0\leq\theta\leq\pi.
```

At s < 10⁻⁸, the code uses the continuous small-angle limit
2 v_e / ||q_e|| to avoid division by a very small s. The local controller rejects
||e_B|| above maximum_attitude_error_rad, which must itself be strictly below π.
The utility defines a half-turn tie, but the controller does not operate at that
singularity. Tests compare the error with an independent relative-matrix/Rodrigues
expression at nonidentity attitudes, signed principal axes and either input sign.

The outer proportional loop requests body rate; the inner proportional loop requests
angular acceleration. With componentwise clipping and multiplication (⊙),

```math
\boldsymbol\omega_d=
\mathrm{clip}(K_a\odot\mathbf e_B,-\boldsymbol\omega_{\max},\boldsymbol\omega_{\max}),
\qquad
\boldsymbol\alpha_d=K_r\odot(\boldsymbol\omega_d-\boldsymbol\omega_B),
```

```math
\boldsymbol\tau_{\rm req}
=I_n\boldsymbol\alpha_d+
\boldsymbol\omega_B\times(I_n\boldsymbol\omega_B),
\qquad
\boldsymbol\tau_{\rm lim}
=\mathrm{clip}(\boldsymbol\tau_{\rm req},-\boldsymbol\tau_{\max},\boldsymbol\tau_{\max}).
```

All vectors above are (3,) current-body vectors. K_a and K_r are positive diagonal
gain vectors in s⁻¹; omega is rigid-body rate in rad/s, alpha angular acceleration in
rad/s², I_n the full nominal inertia in kg m², and tau moment in N m. Rotor angular
speeds are separate (4,) variables. The cross product compensates the gyroscopic term
in I omega_dot = tau − omega × (I omega). For a matched plant, feasible unsaturated
allocation and instantaneous actuation, the resulting angular acceleration is alpha_d.
An independent off-diagonal-inertia test verifies that cancellation with the existing
physical acceleration function.

AttitudeControlCommand retains attitude_error_B, desired_omega_B,
moment_requested_B, moment_limited_B, rate_limited, moment_limited and allocation.
Limiting a **rate demand** does not guarantee a bound on actual vehicle rate under a
disturbance or motor lag. Reference orientations are piecewise constant; angular
trajectory feedforward and reference-rate transport are not implemented.

The attitude-to-rate cascade has an established engineering precedent in the
[official PX4 controller documentation](https://docs.px4.io/main/en/modules/modules_controller#mc_att_control).
This repository's local logarithmic proportional/proportional (P/P) law and allocator are
defined above; it does not implement PX4's complete flight-control law.

## Gain rationale and persistent disturbances

Near a constant reference, on a matched principal inertia axis, with no limits or motor
lag, a small signed orientation deviation theta obeys

```math
\ddot\theta+K_r\dot\theta+K_rK_a\theta=0,\qquad
\omega_n=\sqrt{K_rK_a},\qquad
\zeta=\frac{K_r}{2\sqrt{K_rK_a}}.
```

Finite differences of the composed controller and physical acceleration verify this local
linearization. The declared example uses K_a = [3,3,2] and K_r = [12,12,8] s⁻¹:
K_r = 4 K_a gives ζ = 1 and natural frequencies [6,6,4] rad/s in the zero-lag local
model. These are model-based starting gains, not fitted held-out results. Actual motor
lag, sampling, coupling and limits remain in all execution experiments. The fixed
example uses rate-demand bounds [2,2,1.5] rad/s, moment bounds [.8,.8,.3] N m and a
π/2 local attitude domain. These values are explicit illustrative configuration.

With a constant principal-axis external torque d, equilibrium requires

```math
\theta_{\rm eq}=\frac{d}{I K_rK_a}.
```

There is no integral accumulator and therefore no integral windup or anti-windup state.
The controller resists persistent torque with a nonzero orientation offset. The fixed
diagnostic checks this prediction explicitly. Pulse rejection means recovery after the
external moment is removed; it does not mean zero steady error under arbitrary constant
disturbances. The saturation test verifies recovery when demands return to a feasible
region.

## Collective-preserving allocation

The existing rotor model has FRD thrust along negative z and reaction-moment sign
−spin. For squared speed s_i = Omega_i², its allocation matrix is

```math
A=
\begin{bmatrix}
k_f&k_f&k_f&k_f\\
-y_1k_f&-y_2k_f&-y_3k_f&-y_4k_f\\
x_1k_f&x_2k_f&x_3k_f&x_4k_f\\
-\sigma_1k_m&-\sigma_2k_m&-\sigma_3k_m&-\sigma_4k_m
\end{bmatrix},\qquad
A\mathbf s=\begin{bmatrix}T\\\boldsymbol\tau_B\end{bmatrix}.
```

x_i,y_i are rotor positions in body metres, sigma_i ∈ {−1,+1} the documented rotor
spin directions, k_f thrust coefficient and k_m reaction-moment coefficient. T ≥ 0 is
collective thrust **magnitude**, not body-z force; F_rotor,B = [0,0,−T] N.

For a feasible zero-moment collective anchor, define

```math
\mathbf s_0=A^{-1}[T,0,0,0]^\mathsf{T},\qquad
\mathbf d=A^{-1}[0,\boldsymbol\tau_{\rm lim}^\mathsf{T}]^\mathsf{T}.
```

Find the largest a ∈ [0,1] satisfying
Omega_min² ≤ s_0,i + a d_i ≤ Omega_max² for every rotor.
For each d_i > 0 its candidate upper bound is (Omega_max²−s_0,i)/d_i;
for d_i < 0 it is (s_0,i−Omega_min²)/(−d_i). Zero directions add no restriction.
The minimum of these bounds and 1 preserves T and the direction of the already clipped
moment while reducing its magnitude. Componentwise moment clipping may itself change
the original requested direction; only this subsequent allocation ray preserves direction.

The implementation obtains the anchor from the unchanged strict allocator and squares
its returned speeds. Thus anchor values include that boundary's roundoff repair.
When a < 1, it backs a off by the relative factor 1−64 epsilon_float64 before invoking
the same strict allocator on (T, a tau_lim). Its achieved moment is recomputed with the
forward rotor model. LimitedMomentAllocation records commanded_rotor_omega,
allocated_moment_B and moment_scale. These are nominal steady command quantities.
Independent tests check physical forward force/moment, boundary anchors, feasible identity
and maximality of the limiting ray over a seeded set. No rotor solution is independently
clipped to conceal infeasibility. Singular geometry or an infeasible zero-moment anchor
is an error, even if another nonzero-moment allocation could support the same T.

## Numerical execution and timing

AttitudeControlSchedule owns a positive time_step_s, integer controller_stride L ≥ 1,
reference_q_WB (m,4), collective_thrust (m,) and disturbance_moment_B (N,3),
where N ≥ 1 and m = ceil(N/L). Plant timestamps are t_k = k h, k=0,...,N.
Reference/collective row j applies from t_jL through its control interval. Body disturbance
row k is constant in current-body components through [t_k,t_k+1). Partial final control
intervals are allowed. No terminal command is computed at t_N.

At each controller epoch the harness reads the current truth attitude/rate, computes
a command and holds it until the next controller epoch. It advances motor and rigid-body
states over each plant interval. Nominal motor-command limits must lie within truth
motor limits, so the harness never hides command clipping in a mismatched truth actuator.
Truth inertia, mass, wind, drag, coefficients and motor lag are independently supplied.
Unit tests show that changing only truth inertia preserves the first command and changes
the subsequent trajectory; changing nominal inertia changes the first command.
The declared acceptance campaign itself uses matched parameters.

With held target Omega_c and actual speed Omega_0 at the interval start, each motor is
evaluated exactly at an RK4 stage's elapsed time u:

```math
\Omega(u)=\Omega_c+(\Omega_0-\Omega_c)\exp(-u/\tau_m).
```

Stages at u = 0, h/2, h/2, h compose the existing rotor wrench, stage-specific
wind-relative quadratic drag, supplied body disturbance and body-wrench rigid-body
derivative. Intermediate and final quaternions are normalized. The final actual motor
speed uses the exact same held-target solution at u=h. This evaluates changing thrust
within a step. The constant-command sensor-run generator uses a separate
integration schedule; see [foundations](foundations.md).

The constant-speed/zero-disturbance limit agrees with the existing RK4 integrator.
Equal-motor spin-up from rest has an independent analytical vertical position/velocity
solution obtained by integrating [Omega_c(1−exp(−t/tau_m))]²; step halving demonstrates
fourth-order convergence in that test. Future-reference changes cannot alter earlier
states or commands. Additional tests cover initial/final alignment, exact held motor
response, a partial last control interval, finite failures and nonzero drag.

The controller checks its attitude domain at each control call. It does not enforce a
continuous-time state safety envelope. The campaign separately verifies every recorded
plant-epoch error is inside the declared domain. There is no ground contact, arming logic,
flight termination, transport latency, sensor sampling or estimator in this harness.

## Returned histories and evidence

| AttitudeSimulationResult field | Shape | Meaning |
| --- | --- | --- |
| time_s | (N+1,) | Includes the initial and final truth epochs, seconds |
| position_W, velocity_W | (N+1,3) each | NED m and m/s |
| q_WB, reference_q_WB | (N+1,4) each | Actual and expanded reference quaternions; new reference at each tick |
| omega_B | (N+1,3) | FRD rad/s |
| actual_rotor_omega | (N+1,4) | Actual rad/s |
| actual_moment_B | (N+1,3) | Truth rotor moment N m, before external disturbance |
| control_time_s | (m,) | Actual controller epochs, seconds |
| commanded_rotor_omega | (m,4) | Held rotor commands rad/s |
| desired_omega_B | (m,3) | Bounded rate demand rad/s |
| moment_requested_B, moment_limited_B, allocated_moment_B | (m,3) each | Requested, componentwise clipped and nominally allocated moment N m |
| moment_scale | (m,) | Allocation ray scale in [0,1] |
| limit_flags | (m,3), bool | Rate-demand clipping, moment clipping, allocation scaling |

The result validates shape, finiteness, unit quaternions, nonnegative speeds, scale domain
and control epochs drawn from the increasing truth clock. It is an in-memory interface,
separate from the sensor-run artifact format. The experiment JSON stores the complete
result plus the frozen protocol, every job identity, per-trial metrics/failures, source hash,
software provenance, UTC time and worker count. Its protocol digest excludes source/date/
workers; its separate executable-source digest includes all package and experiment Python
files, pyproject.toml and uv.lock, including new files not yet committed.

The CLI checks source bytes before and after execution and refuses to overwrite a result.
Parallel workers receive deterministic independent jobs; order and seed consumption are
fixed. Held-out seeds 60000–60029 are separate from development 1000–1004 and smoke 10–11.
Smoke checks finite execution only and is not recovery evidence.

The metric definitions are explicit: rotation-vector norm for attitude error; time-weighted
RMS by trapezoidal quadrature; final/peak norms; simultaneous 1-degree/.05-rad/s settling
after the last departure; 10–90% first-crossing rise time and positive overshoot for signed
axis steps; sum of actual control-interval durations for each limit flag; sampled
rotor-bound occupancy; and integral of squared actual rotor moment. These are finite-grid,
finite-horizon descriptions. A command flag is not a physical rotor-bound flag.
Position displacement is retained to make the uncontrolled translation visible.

validate_report requires every planned identity exactly once in order, checks the protocol
and clocks, and recomputes metrics and the summary from histories. Failure rows prevent
an overall pass. This is an integrity/consistency check, not independent regeneration or
cryptographic authentication of physical truth. The plotter validates first, uses a
headless backend and records input/plotter SHA-256. In coupled plots, x/y/z attitude
errors are current-body rotation-vector components, not Euler-angle differences.

## Reproduce

Use the locked Python 3.12 environment and uv 0.12.3. Set ATTITUDE_EVIDENCE
to a new directory outside the repository; all result/figure destinations must be new.

~~~bash
uv sync --locked
make check
uv run pytest -W error
ATTITUDE_EVIDENCE=/tmp/attitude-control-evidence
mkdir -p "$ATTITUDE_EVIDENCE"
uv run python -m experiments.attitude_control_validation --partition fixed --workers 4 --output "$ATTITUDE_EVIDENCE/fixed.json"
uv run python -m experiments.attitude_control_validation --partition development --workers 2 --output "$ATTITUDE_EVIDENCE/development.json"
uv run python -m experiments.attitude_control_validation --partition validation --workers 4 --output "$ATTITUDE_EVIDENCE/validation.json"
uv run python -m experiments.plot_attitude_control --input "$ATTITUDE_EVIDENCE/fixed.json" --output "$ATTITUDE_EVIDENCE/fixed-plots"
uv run python -m experiments.plot_attitude_control --input "$ATTITUDE_EVIDENCE/validation.json" --output "$ATTITUDE_EVIDENCE/validation-plots"
~~~

Generated reports, plots and logs stay outside Git. Physical parameters are illustrative,
motor lag is first order, rotor forces are quasi-static, and the local P/P baseline has
no persistent-disturbance integrator. Constant collective does not compensate tilt or
transient motor redistribution; tilted cases can move far from their initial position.
These limits are material when interpreting the numerical precision of final attitudes.
