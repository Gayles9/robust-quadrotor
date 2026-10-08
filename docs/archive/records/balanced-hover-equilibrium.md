# 2026-08-30: Balanced-Hover Equilibrium Characterization

## Milestone summary

Focused tests now characterize one exact balanced-thrust, zero-moment hover equilibrium at
both the continuous derivative boundary and the multi-step simulator boundary. The evidence
shows that the implemented model recognizes the specified equilibrium and that explicit Euler
and projected RK4 numerically preserve it for the tested fixed time grid.

This milestone adds no production API. The hover configuration remains local to the tests,
and Gate G1 remains open.

## Frame, force, and rotor conventions

The world frame is north-east-down (NED), so gravity is positive world `z`:

```text
gravity_W = [0, 0, +gravity_acceleration]
```

The body frame is forward-right-down (FRD), so upward rotor thrust is negative body `z`:

```text
force_B = [0, 0, -total_thrust]
```

The tests use this local rotor ordering, with `L` measured from the centre of mass:

1. front: `[+L, 0, 0]`;
2. right: `[0, +L, 0]`;
3. rear: `[-L, 0, 0]`; and
4. left: `[0, -L, 0]`.

This ordering is local to the tests. It does not establish a package-wide rotor numbering
convention.

For four equal rotors, the test calculates the equilibrium speed as

```text
hover_rotor_speed = sqrt(
    mass * gravity_acceleration / (4 * thrust_coefficient)
)
```

## Deterministic physical configuration

The derivative and simulation tests use the same parameters:

| Quantity | Value |
| --- | --- |
| Mass | `1.0 kg` |
| Gravity magnitude | `9.81 m/s²` |
| Thrust coefficient | `0.75 N/(rad/s)²` |
| Moment coefficient | `0.5 N·m/(rad/s)²` |
| Arm length | `0.5 m` |
| Inertia | `diag(2, 3, 4) kg·m²` |
| Spin directions | `[+1, -1, +1, -1]` |

The initial position is `[10, 20, 30] m`, the attitude is identity `q_WB = [1, 0, 0, 0]`,
and both `velocity_W` and `omega_B` are zero.

The equal-rotor thrust calculation is

```text
individual_thrust = mass * gravity_acceleration / 4
                  = 1.0 * 9.81 / 4
                  = 2.4525 N
```

Therefore the four rotors produce total body force

```text
force_B = [0, 0, -4 * 2.4525]
        = [0, 0, -9.81] N.
```

At identity attitude, `R_WB = I`, so translational force balance is

```text
gravity_W + (R_WB @ force_B) / mass
    = [0, 0, +9.81] + [0, 0, -9.81] / 1.0
    = [0, 0, 0] m/s².
```

Each rotor force is `[0, 0, -2.4525] N`. The front and rear offset-thrust
moments are equal and opposite about body `y`, while the right and left moments are equal and
opposite about body `x`. Their symmetric sum is therefore zero in roll and pitch. For equal
rotor speeds, the spin-direction sum is

```text
+1 - 1 + 1 - 1 = 0,
```

so the implemented reaction-moment convention also gives zero yaw moment. Total `moment_B`
is `[0, 0, 0] N·m`.

## Derivative-level evidence

At identity attitude with `velocity_W = 0`, `omega_B = 0`, balanced force, and zero moment:

- the position derivative equals the zero velocity;
- the velocity derivative equals zero because thrust cancels gravity;
- the quaternion derivative equals zero because angular velocity is zero; and
- the angular-velocity derivative equals zero because both the applied moment and gyroscopic
  term are zero.

The focused dynamics test asserts each derivative independently against a correctly shaped
zero array using `rtol=0.0` and `atol=1e-12`.

## Multi-step simulator evidence

One parameterized test applies the same constant hover rotor speeds to:

- `simulate_rigid_body_euler_from_rotor_speeds`; and
- `simulate_rigid_body_rk4_from_rotor_speeds`.

For both methods, the test uses:

- time step: `0.05 s`;
- number of steps: `20`;
- total duration: `1.0 s`;
- relative tolerance: `0.0`; and
- absolute tolerance: `1e-12`.

The returned histories are compared directly, without normalization or repair, against
constant position, zero velocity, identity attitude, and zero angular velocity. Both methods
preserve the equilibrium for all 21 stored samples.

## Scientific claim and limitations

This evidence proves that the implemented model recognizes and numerically preserves the
specified exact hover equilibrium. It does not prove hover stability. Because no feedback
controller exists, a perturbed vehicle will not automatically return to equilibrium.

The tests do not validate:

- arbitrary yaw or tilted hover;
- perturbed-hover response or stability;
- controllers;
- motor dynamics;
- aerodynamic effects, drag, wind, or other disturbances;
- sensors or state estimation; or
- parameter uncertainty or robustness.

Gate G1 remains open.

## Architecture decision

The first hover scenario remains test-local. No reusable scenario helper, configuration
dataclass, or standalone experiment was added because there is not yet a second production
consumer. Extracting an abstraction before another consumer establishes its required contract
would add ownership and maintenance cost without demonstrated reuse.

## Verification evidence

For the current working tree on 2026-08-30:

- the complete repository gate passed 224 pytest tests;
- Ruff lint passed;
- Ruff formatting verification passed;
- strict mypy passed over `src` and `experiments`;
- no warnings were reported; and
- `git diff --check` passed.

## Next exact action

A read-only design review for a gravity-only ballistic reference scenario, separating:

- analytical constant-gravity trajectory behavior;
- horizontal-momentum conservation;
- mechanical-energy conservation;
- Euler versus RK4 numerical behavior;
- and deciding the smallest first test without mixing these distinct claims.
