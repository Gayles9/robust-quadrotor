# 2026-08-19: Foundational Dynamics and Integration Pipeline

## Objective

Establish a frame-explicit, independently tested pipeline from four rotor speeds through one
numerical update of the complete rigid-body state, while keeping the mathematical core
independent of ROS 2 and PX4.

## Completed pipeline

The implemented path is:

```text
rotor speeds
    -> individual static rotor thrusts
    -> FRD body force and moment
    -> NED translational and FRD rotational acceleration
    -> position, velocity, quaternion, and angular-velocity derivatives
    -> explicit-Euler update
    -> normalized body-to-world quaternion
```

Actuation is decomposed into quadratic rotor thrust, collective body force, thrust moments
from rotor offsets, and the body yaw reaction moment. These contributions are combined into
`force_B` and `moment_B`. The dynamics layer consumes that wrench and the current rigid-body
state. The integration layer advances all state components from derivatives evaluated at the
same current state, then normalizes the updated quaternion.

**Practical interpretation.** The code now connects actuator commands to one complete motion
update without hiding the force, moment, acceleration, attitude-rate, or integration stages.

## Mathematical foundations

The world frame `W` is north-east-down (NED), and the body frame `B` is
forward-right-down (FRD). `R_WB` actively maps body-coordinate vectors into world
coordinates. `q_WB` is the equivalent Hamilton scalar-first `[w, x, y, z]` body-to-world
quaternion. Positive world `z` points downward. The authoritative convention remains the
[frame and state contract](../../architecture/frame-contract.md).

### Translational dynamics

```text
velocity_derivative_W =
    gravity_W + (R_WB @ force_B) / mass
```

`gravity_W = [0, 0, gravity_acceleration]` in m/s². `force_B` is expressed in the FRD body
frame and rotated into the NED world frame before division by mass.

**Practical interpretation.** Gravity and rotor force determine the change in world-frame
velocity, while the current velocity is the position derivative.

### Rotational dynamics

```text
inertia_B @ angular_velocity_derivative_B =
    moment_B - cross(omega_B, inertia_B @ omega_B)
```

The inertia tensor is expressed about the centre of mass in body coordinates. The equation
includes gyroscopic coupling and is solved with `np.linalg.solve`; it does not form an explicit
matrix inverse.

**Practical interpretation.** Body moments change angular velocity, and the coupling term
captures how existing rotation interacts with angular momentum.

### Quaternion kinematics and propagation

```text
quaternion_derivative_WB =
    0.5 * q_WB ⊗ [0, omega_B]

state_next = state_current + time_step * state_derivative
```

All Euler updates use derivatives from the same current state. After every Euler update, the
updated `q_WB` is normalized to unit length.

**Practical interpretation.** Angular velocity changes attitude, Euler integration advances
that instantaneous change through a short interval, and normalization keeps the quaternion a
valid rotation representation despite numerical drift.

## Architectural decisions

### Compose small tested functions

Each layer delegates to existing lower-level functions rather than repeating equations:

- rotor thrust, body force, thrust moment, and reaction moment compose into a body wrench;
- rotation, translation, quaternion kinematics, and rotational dynamics compose into the
  complete state derivative;
- the rotor-speed derivative and quaternion normalizer compose into the Euler step.

This structure keeps frame/sign defects local, preserves focused validation, and lets tests
cover both individual equations and the complete pipeline.

### Defer a state dataclass

The current state components remain explicit typed arrays: `position_W`, `velocity_W`,
`q_WB`, and `omega_B`. A state dataclass was deferred because the present composition can be
expressed without introducing a container abstraction, and there is not yet a second
simulation/control/estimation consumer that establishes the right long-term state API.

**Practical interpretation.** The project avoided locking in a state container before its
actual users and lifecycle requirements are known.

### Implement explicit Euler before a higher-order method

Explicit Euler was selected as the first propagation method because it exposes the direct
relationship between the continuous derivative and the discrete state update. That makes the
new dynamics pipeline easy to test end to end before introducing the intermediate derivative
evaluations required by RK4.

Explicit Euler remains a baseline, not the intended final high-accuracy simulation method.
Its first-order truncation error and accumulated drift must be assessed against a higher-order
integrator.

**Practical interpretation.** Euler provides the simplest trustworthy moving simulation;
RK4 is the next step for better accuracy rather than the first place to debug the equations.

## Validation and TDD process

Development used strict small-step RED/GREEN increments. Focused tests first established each
missing interface or validation behavior, then the minimum production change made that test
pass. The implemented boundaries check the relevant shapes and finite values, physical scalar
signs, nonnegative rotor speeds, rotor spin directions, inertia symmetry and positive
definiteness, unit-quaternion requirements where rotation matrices are formed, and finite
positive integration intervals. Validation inside composed functions is reused instead of
duplicated at higher levels.

Quaternion normalization rejects invalid shape, non-finite values, and zero norm. The Euler
integrator normalizes the updated quaternion after every accepted positive finite time step.

At commit `aac5412`, `make check` passed Ruff linting, Ruff formatting verification, strict
mypy checking, and 146 pytest tests.

## Relevant commits

- `09a56ca` — translational acceleration dynamics
- `6c25565` — rotational acceleration dynamics
- `ad2e0cc` — rigid-body state derivative
- `95bba08` — rotor speeds to rigid-body dynamics
- `aac5412` — explicit Euler state integration

## Current limitations

- The only numerical propagator is first-order explicit Euler; RK4 is not implemented.
- Rotor thrust is a static quadratic mapping; rotor/motor transients are not propagated.
- The source has no closed-loop controller, trajectory tracker, sensor model, or state
  estimator.
- No multi-step deterministic scenario suite or full Monte Carlo robustness campaign exists.
- Additional aerodynamic, contact, and environmental effects are not represented.
- ROS 2/PX4 compatibility work exists, but the mathematical core has no completed adapter or
  runtime integration layer.

## Next exact engineering milestone

Implement a higher-order rigid-body integration method, beginning with a focused RED test that
compares one RK4 rotor-speed state step against a deterministic reference case while preserving
the established NED/FRD contracts and quaternion unit norm.
