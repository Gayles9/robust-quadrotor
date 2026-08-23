# 2026-08-23: Deterministic Multi-Step RK4 Simulation

## Milestone summary

The mathematical core now composes the fixed-step projected RK4 state propagator into a
deterministic, constant-input multi-step simulator. The new layer records time, position,
velocity, attitude, and angular-velocity histories without reproducing actuation, dynamics,
quaternion-kinematics, or integration equations.

## Motivation

The one-step RK4 interface established how to advance a complete rigid-body state consistently
with the NED/FRD frame contract. Quantitative numerical studies also require a repeatable
sequence of steps and an explicit record of every generated state. The simulation layer
provides that sequence while keeping the existing one-step integrator as the owner of state
propagation.

**Think about it this way.** The RK4 stepper answers “what is the next state?” The simulator
repeats that operation and records “how did the state evolve over time?”

## Architecture and composition

The implemented composition is:

```text
constant rotor speeds
    -> FRD rotor force and moment
    -> complete NED/FRD rigid-body state derivative
    -> fixed-step projected RK4 state transition
    -> sequential time-aligned state histories
```

`simulate_rigid_body_rk4_from_rotor_speeds` calls
`rigid_body_state_rk4_step_from_rotor_speeds` exactly `number_of_steps` times. The result of
each call becomes the complete position, velocity, quaternion, and angular-velocity input to
the next call. Rotor speeds, rotor geometry, spin directions, mass, inertia, gravity, and
actuation coefficients remain constant across all transitions. Intermediate and final
quaternion normalization is inherited from the RK4 stepper.

The derivative, step, and simulation layers have distinct responsibilities. A state derivative
is an instantaneous rate. One RK4 step combines four complete derivative evaluations to
produce one next state. The simulator executes multiple sequential state transitions and
stores the initial state and every result.

**Think about it this way.** A derivative is a local slope, an RK4 step advances one interval,
and the simulation history joins those intervals without resetting to the original state.

## Exact API contract

```python
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

The initial state is `position_W` in NED metres, `velocity_W` in NED m/s, `q_WB` as a
dimensionless Hamilton scalar-first FRD-body-to-NED-world unit quaternion, and `omega_B` in FRD
rad/s. `time_step` is a fixed positive interval in seconds. `number_of_steps` is the number of
RK4 transitions and must be a positive Python integer that is not a Boolean.

The function returns float64 NumPy arrays in this exact order:

1. `time_s`;
2. `position_history_W`;
3. `velocity_history_W`;
4. `q_history_WB`; and
5. `omega_history_B`.

## History shapes and time semantics

For `N = number_of_steps`, the returned shapes and units are:

| Array | Shape | Frame | Units |
| --- | --- | --- | --- |
| `time_s` | `(N + 1,)` | — | s |
| `position_history_W` | `(N + 1, 3)` | NED world | m |
| `velocity_history_W` | `(N + 1, 3)` | NED world | m/s |
| `q_history_WB` | `(N + 1, 4)` | FRD body to NED world | dimensionless |
| `omega_history_B` | `(N + 1, 3)` | FRD body | rad/s |

Row zero contains the supplied initial state. Rows one through `N` contain the successive RK4
results, so `N` requested transitions produce `N + 1` history rows. Time begins at zero and
obeys `time_s[i] = i * time_step`.

## Validation ownership

The simulator directly validates only `number_of_steps`. Integer-valued floats are rejected,
and `True` and `False` are rejected explicitly because Python Booleans are subclasses of
`int`. Zero and negative integers are also rejected.

Validation of `time_step`, state arrays, quaternion validity, rotor inputs, mass, inertia,
gravity, and coefficients remains delegated to the RK4 stepper and its composed functions.
The simulator computes one validated RK4 step before allocating or assigning the fixed-shape
history arrays. Consequently, malformed lower-level inputs fail with their established
validation messages instead of first causing an unrelated NumPy broadcasting error during
history assignment.

**Think about it this way.** The simulator verifies its new loop count, then lets the existing
mathematical boundaries verify the state and physics before arranging any results into history
tables.

## Deterministic testing

The deterministic history test uses a nonzero initial position and velocity, identity
body-to-world attitude, nonzero yaw rate, and one active offset rotor. Two RK4 transitions
produce three time-aligned rows. Exact float64 references check every returned time, position,
velocity, quaternion, and angular-velocity component.

The first propagated row matches the established one-step RK4 result. The second row verifies
that propagation continues from the first result instead of restarting from the initial state.
The scenario also confirms the `N + 1` history contract, row-zero initial state, constant
inputs, return order, and full-state evolution. A separate five-case parameterized test checks
integer-valued float, Boolean, zero, and negative `number_of_steps` values.

This deterministic test establishes composition and repeatability for the chosen scenario. It
does not establish convergence rate, invariant preservation, long-horizon accuracy, or
closed-loop behavior.

## Verification

At commit `81b38f9`, the complete project gate passed:

- Ruff linting;
- Ruff formatting verification;
- strict mypy checking over 8 source files;
- 157 pytest tests; and
- `git diff --check`.

## Engineering decisions

- Reuse the existing RK4 stepper rather than duplicating lower-level equations.
- Keep constant rotor input and fixed time steps for the first simulation interface.
- Include the initial state so `N` transitions map unambiguously to `N + 1` samples.
- Return separate frame-explicit arrays compatible with existing public APIs.
- Keep explicit Euler as the transparent one-step baseline.
- Defer separate Euler simulation, a generic step callable, method strings or enums, a state
  dataclass, scheduled inputs, controllers, callbacks, and event handling until demonstrated
  consumers define their requirements.

## Current limitations

- Rotor speeds, rotor geometry, and physical parameters are constant over the full history.
- The time step is fixed; no adaptive step size or error control exists.
- No scheduled input, controller, callback, or event handling exists.
- No quantitative Euler-versus-RK4 convergence study has been completed.
- Physical invariants and long-horizon accuracy have not been characterized.
- No closed-loop control or state estimation is implemented.

## Next exact milestone

Quantify Euler-versus-RK4 convergence across decreasing time steps and monitor state validity
and physical invariants during propagation.
