# 2026-08-23: RK4 State Integration

## Objective

Add a higher-order complete-state propagator without duplicating the lower-level actuation,
rotation, quaternion-kinematics, translational-dynamics, or rotational-dynamics equations.

## Implemented interface

`rigid_body_state_rk4_step_from_rotor_speeds` advances one rigid-body state over a fixed time
step. Its state arguments and return values are, in order:

1. `position_W`, shape `(3,)`, expressed in the NED world frame in metres;
2. `velocity_W`, shape `(3,)`, expressed in the NED world frame in m/s;
3. `q_WB`, shape `(4,)`, a dimensionless Hamilton scalar-first body-to-world quaternion; and
4. `omega_B`, shape `(3,)`, expressed in the FRD body frame in rad/s.

The rotor angular speeds, rotor positions, rotor spin directions, mass, body-frame inertia,
gravity magnitude, thrust coefficient, and moment coefficient remain constant during all four
derivative evaluations in one step. Each returned component is a float64 NumPy array.

## RK4 mathematics

Let `state` contain `position_W`, `velocity_W`, `q_WB`, and `omega_B`; let `input` contain the
rotor speeds held constant over one step; and let
`f = rigid_body_state_derivative_from_rotor_speeds`. Every `k` is a complete state derivative:

```text
k1 = f(state_current, input)

k2 = f(
    state_current + 0.5 * time_step * k1,
    input,
)

k3 = f(
    state_current + 0.5 * time_step * k2,
    input,
)

k4 = f(
    state_current + time_step * k3,
    input,
)

state_next =
    state_current
    + (time_step / 6)
    * (k1 + 2*k2 + 2*k3 + k4)
```

The `k1` derivative is evaluated at the current state. The `k2` intermediate state is the
original state plus the `k1` half-step; the `k3` intermediate state is the original state plus
the `k2` half-step; and the `k4` intermediate state is the original state plus the `k3`
full-step. Position, velocity, quaternion, and angular velocity are all advanced when each
intermediate state is constructed.

The quaternion component is projected back to unit norm after constructing the `k2`, `k3`,
and `k4` intermediate states and before evaluating their derivatives. The quaternion component
of the final classically weighted update is also normalized. Quaternion derivatives themselves
are not normalized. This is projected quaternion handling within a fixed-step RK4 method; the
repository does not claim that it is exact or establish a formal global order for this
projected implementation.

## Practical interpretation

Euler uses one derivative sample to extend the initial slope across the whole step. RK4 uses
four derivative samples at the start, two intermediate states, and an end-like state. Their
weighted combination follows changes in the slope and therefore represents curved motion
better over a step, without changing the underlying physical model.

## Validation

The RK4 interface rejects a non-finite `time_step` before any derivative evaluation,
quaternion normalization, or multiplication of the time step by an array. It then rejects zero
and negative time steps. This ordering prevents invalid intermediate arithmetic and the runtime
warnings that would otherwise arise for an infinite step. Shape, finite-value, quaternion,
rotor, mass, inertia, gravity, and coefficient validation remains delegated to the existing
composed dynamics and quaternion-normalization functions.

## Testing

- One deterministic full-state test checks the propagated position, velocity, `q_WB`, and
  `omega_B` against agreed float64 reference values.
- One four-case parameterized test checks `NaN`, positive infinity, zero, and negative time
  steps with exact error messages.
- Existing explicit-Euler tests remain unchanged.
- At commit `c3d79f1`, the complete project gate passed 151 tests; Ruff linting, Ruff formatting
  verification, and strict mypy checking also passed.

## Architectural decisions

- Reuse `rigid_body_state_derivative_from_rotor_speeds` for all four derivative evaluations
  instead of reproducing lower-level equations.
- Keep rotor inputs, rotor geometry, and physical parameters constant during one step.
- Construct new intermediate and returned arrays without mutating the input state arrays.
- Do not introduce a state container before additional consumers establish its requirements.
- Keep explicit Euler as a transparent baseline rather than replacing it.

## Limitations

- Integration uses a fixed time step only.
- Quaternion handling uses intermediate and final projection to unit norm.
- No Euler-versus-RK4 convergence or accuracy study exists yet.
- No long-horizon deterministic scenarios exist yet.
- No adaptive error control exists.
- No controllers or estimators exist yet.

## Relevant commit

- `c3d79f1` — feat: add RK4 state integration

## Next exact milestone

Build deterministic multi-step scenarios, then perform an Euler-versus-RK4 convergence study.
