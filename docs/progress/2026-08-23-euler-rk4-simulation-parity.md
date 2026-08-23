# 2026-08-23: Euler and RK4 Simulation Parity

## Milestone summary

The mathematical core now provides matching deterministic, constant-input multi-step
simulators for explicit Euler and projected RK4. Both public functions accept the same
frame-explicit arrays and physical parameters, apply their established one-step integrator
sequentially, and return time-aligned histories with identical shapes and ordering.

This milestone creates the production interface needed for a fair Euler-versus-RK4 study. It
does not yet provide convergence measurements, invariant monitoring, or evidence that the
projected RK4 implementation achieves fourth-order convergence for the complete state.

## Motivation

The RK4 simulator established deterministic history construction, but a quantitative method
comparison also needs Euler histories produced through an equivalent source-level contract.
Adding `simulate_rigid_body_euler_from_rotor_speeds` makes the numerical method the controlled
difference: both simulations use the same initial-state representation, rotor inputs,
physical parameters, time semantics, and returned state layout.

**Think about it this way.** A fair comparison gives both methods the same starting state,
inputs, duration, and output format, then changes only how each step is calculated.

## Architectural placement

The simulation layer remains in `src/quadrotor_math/simulation.py`, above the one-step
integrators in `src/quadrotor_math/integration.py`. Its composition is:

```text
constant rotor speeds
    -> FRD rotor actuation
    -> complete NED/FRD rigid-body state derivative
    -> explicit Euler or projected RK4 state transition
    -> sequential time-aligned NED/FRD state histories
```

The Euler simulator delegates every transition to
`rigid_body_state_euler_step_from_rotor_speeds`; the RK4 simulator delegates every transition
to `rigid_body_state_rk4_step_from_rotor_speeds`. Neither simulator reproduces rotor
actuation, rigid-body dynamics, quaternion kinematics, or integration equations.

## Shared API and history contract

The two public functions are:

```python
simulate_rigid_body_euler_from_rotor_speeds(
    position_W,
    velocity_W,
    q_WB,
    omega_B,
    rotor_omega,
    rotor_positions_B,
    rotor_spin_directions,
    mass,
    inertia_B,
    gravity_acceleration,
    thrust_coefficient,
    moment_coefficient,
    time_step,
    number_of_steps,
)

simulate_rigid_body_rk4_from_rotor_speeds(
    position_W,
    velocity_W,
    q_WB,
    omega_B,
    rotor_omega,
    rotor_positions_B,
    rotor_spin_directions,
    mass,
    inertia_B,
    gravity_acceleration,
    thrust_coefficient,
    moment_coefficient,
    time_step,
    number_of_steps,
)
```

The initial state comprises `position_W` in NED metres, `velocity_W` in NED m/s, `q_WB` as a
dimensionless Hamilton scalar-first FRD-body-to-NED-world unit quaternion, and `omega_B` in
FRD rad/s. Rotor speeds, rotor geometry, spin directions, mass, body-frame inertia, gravity,
and thrust and moment coefficients remain constant across all transitions. `time_step` is a
fixed positive interval in seconds, and `number_of_steps` must be a positive Python integer
that is not a Boolean.

Both functions return float64 NumPy arrays in this exact order:

1. `time_s`, shape `(N + 1,)`, in seconds;
2. `position_history_W`, shape `(N + 1, 3)`, in NED metres;
3. `velocity_history_W`, shape `(N + 1, 3)`, in NED m/s;
4. `q_history_WB`, shape `(N + 1, 4)`, as dimensionless Hamilton scalar-first
   FRD-body-to-NED-world unit quaternions; and
5. `omega_history_B`, shape `(N + 1, 3)`, in FRD rad/s.

Here `N = number_of_steps`. Row zero contains the supplied initial state, rows one through `N`
contain successive integration results, and `time_s[i] = i * time_step`. Therefore `N`
requested transitions produce `N + 1` recorded rows. Quaternion normalization is inherited
from the selected one-step integrator.

## Sequential propagation

Each simulator calls its one-step integrator exactly `number_of_steps` times. The complete
position, velocity, quaternion, and angular-velocity result of one call becomes the complete
input state of the next call. No transition restarts from the original state. The rotor inputs
and physical parameters are passed unchanged on every call.

Both implementations calculate one validated step before allocating and assigning their
fixed-shape history arrays. This preserves lower-level validation ownership: malformed state,
rotor, or physical inputs fail through the established stepper and composed functions rather
than through an unrelated NumPy broadcasting failure during history assignment.

**Think about it this way.** Each history row is a link in a chain. The simulator must use the
newest link to create the next one, and it lets the established stepper check that the first
link can be created before building the table that will hold the chain.

## Numerical distinction

Explicit Euler evaluates the complete state derivative once at the beginning of each step and
uses that slope across the full interval. It is a first-order integration method. Projected
RK4 evaluates four complete derivative stages at the current state, two intermediate states,
and an end-like state before applying the classical RK4 weights. Its intermediate and final
quaternions are normalized.

RK4 is expected to be substantially more accurate than Euler at a given sufficiently small
step size, but this repository has not yet measured either method's errors or observed
convergence rates. In particular, quaternion projection changes the complete numerical map,
so the observed order of the projected full-state implementation must be established by a
convergence study rather than asserted from the unprojected classical formula alone.

**Think about it this way.** Euler estimates the interval from one slope; RK4 combines four
slopes. That gives RK4 a stronger theoretical starting point, but only measured error across
smaller steps can show how this complete implementation behaves.

## Validation ownership

Each simulator directly validates only `number_of_steps`. Integer-valued floats are rejected;
`True` and `False` are rejected explicitly because Boolean values are subclasses of `int`; and
zero and negative integers are rejected. These checks occur before the first integration step
or history allocation.

Validation of the fixed positive `time_step`, initial state, quaternion, rotor inputs, mass,
inertia, gravity, and coefficients remains delegated to the selected one-step integrator and
its composed functions.

## Deterministic testing

The Euler history test mirrors the established RK4 scenario: a nonzero initial position and
velocity, identity body-to-world attitude, nonzero yaw rate, and one active rotor offset from
the centre of mass. Two transitions produce three time-aligned rows. Exact float64 references
check every time, position, velocity, quaternion, and angular-velocity component.

The first propagated row verifies composition with the one-step Euler integrator. The second
transition proves that the complete first result becomes the next input; restarting from the
initial state would not produce the recorded third row. The scenario also verifies row-zero
semantics, `N + 1` shapes, return order, and constant-input sequential propagation. A separate
five-case parameterized test covers integer-valued float, Boolean, zero, and negative
`number_of_steps` values.

The test demonstrates deterministic history construction for this scenario. It does not
measure accuracy, convergence order, physical-invariant behavior, or long-horizon stability.

## Verification

At commit `5870663`, the complete project gate passed:

- Ruff linting;
- Ruff formatting verification;
- strict mypy checking over 8 source files;
- 163 pytest tests; and
- `git diff --check`.

## Engineering decisions

Two small method-specific propagation loops remain in production. This duplication is being
tolerated deliberately because the common abstraction should be extracted from two proven
implementations rather than predicted prematurely. The explicit functions keep the numerical
method visible in the public API and keep strict typing straightforward for the upcoming
comparison.

The following abstractions remain deferred:

- a shared callable or private history-propagation helper, until another behavior change shows
  which details are genuinely common;
- a string method selector or enum, because two explicit public functions are clearer and
  already type-safe;
- a rigid-body state dataclass, because current consumers use explicit arrays and have not yet
  demonstrated enough state-container requirements; and
- scheduled inputs, callbacks, controllers, events, and adaptive stepping, because this
  milestone covers constant-input fixed-step propagation only.

## Production capability now available

The package can now generate deterministic, time-aligned complete rigid-body histories using
either explicit Euler or projected RK4 under constant rotor speeds and physical parameters.
The matching interfaces allow the same scenario to be propagated by either method without
adapting state representation or result handling.

## What has not yet been demonstrated

- Quantitative Euler-versus-RK4 error or convergence behavior has not been measured.
- Fourth-order convergence has not been established for the projected RK4 complete-state map.
- No invariant-monitoring framework exists.
- Long-horizon accuracy and stability have not been characterized.
- Time-varying inputs, closed-loop control, estimation, callbacks, events, and adaptive step
  sizes are not implemented.

## Next exact milestone

Run a deterministic Euler-versus-RK4 convergence study across decreasing fixed time steps
using physically meaningful position, velocity, quaternion-attitude, and angular-velocity
error metrics, then add state-validity and physical-invariant monitoring appropriate to the
applied forces and moments.
