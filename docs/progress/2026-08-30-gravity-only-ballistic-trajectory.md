# 2026-08-30: Gravity-Only Ballistic Trajectory Characterization

## Milestone summary

Two focused simulation tests now characterize one deterministic gravity-only ballistic
scenario. The public projected-RK4 simulator is compared with the analytical constant-gravity
trajectory, while the public explicit-Euler simulator is compared with its independently
derived discrete trajectory.

This milestone establishes method-specific trajectory behavior before making separate
conservation claims. It adds no production API, and Gate G1 remains open.

## Scenario

The world frame is north-east-down (NED), so gravity acts along positive world `z`. All four
rotor speeds are zero, which produces zero body force and zero body moment. The complete
deterministic inputs are:

| Quantity | Value |
| --- | --- |
| Initial `position_W` | `[10.0, 20.0, 30.0] m` |
| Initial `velocity_W` | `[1.0, -2.0, -3.0] m/s` |
| Initial `q_WB` | Identity `[1.0, 0.0, 0.0, 0.0]` |
| Initial `omega_B` | `[0.0, 0.0, 0.0] rad/s` |
| Rotor speeds | `[0.0, 0.0, 0.0, 0.0] rad/s` |
| Mass | `2.0 kg` |
| Inertia | `diag(2.0, 3.0, 4.0) kg·m²` |
| Gravity magnitude | `9.81 m/s²` |
| Thrust coefficient | `0.75 N/(rad/s)²` |
| Moment coefficient | `0.5 N·m/(rad/s)²` |
| Time step | `0.25 s` |
| Steps | `4` |
| Duration | `1.0 s` |

The tests retain the established valid test-local rotor geometry and balanced spin directions,
although neither affects the zero-speed wrench. With zero moment and zero initial angular
velocity, `omega_B` remains zero and `q_WB` remains identity.

## Analytical continuous solution

The NED gravity vector is

```text
gravity_W = [0, 0, +g]
```

With no other force, the exact translational solution is

```text
velocity_W(t) = velocity_W_0 + gravity_W * t

position_W(t) = (
    position_W_0
    + velocity_W_0 * t
    + 0.5 * gravity_W * t**2
)
```

At `t = 1.0 s`, this gives:

- exact position: `[11.0, 18.0, 31.905] m`;
- exact velocity: `[1.0, -2.0, 6.81] m/s`;
- identity attitude; and
- zero angular velocity.

## Projected-RK4 evidence

The projected-RK4 test constructs the expected position and velocity histories directly from
the analytical equations at all five grid times. It independently compares returned time,
position, velocity, attitude, and angular-velocity histories using `rtol=0.0` and
`atol=1e-12`.

All comparisons pass. Quaternion projection has no physical effect here because `q_WB` is
already unit length and its derivative remains zero.

## Explicit-Euler derivation and evidence

Explicit Euler advances constant-gravity translation as

```text
velocity_(k+1) = velocity_k + h * gravity_W
position_(k+1) = position_k + h * velocity_k
```

Repeated substitution gives the discrete closed form

```text
velocity_n = velocity_0 + gravity_W * t_n

position_n = (
    position_0
    + velocity_0 * t_n
    + 0.5 * gravity_W * (t_n**2 - t_n * h)
)
```

Consequently, defining position error as Euler minus analytical,

```text
position_error_n = -0.5 * gravity_W * t_n * h
```

At `t = 1.0 s` with `h = 0.25 s`:

- Euler position is `[11.0, 18.0, 30.67875] m`;
- Euler velocity is `[1.0, -2.0, 6.81] m/s`; and
- final Euler-minus-analytical position error is `[0.0, 0.0, -1.22625] m`.

The Euler test constructs this discrete formula as its oracle and independently compares all
five returned histories using `rtol=0.0` and `atol=1e-12`. It does not compare Euler position
with the continuous solution and weaken the tolerance. The position difference is expected
first-order integration behavior caused by using the velocity at the beginning of each step;
it is not a dynamics defect. Euler velocity is exact at the grid times in this scenario because
acceleration is constant.

## Scientific claim and limitations

The evidence supports these bounded claims:

- the public RK4 simulator reproduces the exact constant-gravity ballistic trajectory over the
  tested grid within the stated tolerance;
- the public Euler simulator reproduces its mathematically derived discrete trajectory;
- Euler velocity is exact here because acceleration is constant; and
- Euler position is not analytically exact because each update uses the velocity at the
  beginning of the step.

The tests do not yet claim:

- mechanical-energy conservation;
- horizontal- or full-momentum monitoring;
- aerodynamic realism;
- stability or recovery from perturbations;
- control;
- state estimation; or
- robustness to uncertainty or disturbances.

Gate G1 remains open.

## Architecture decision

The gravity-only scenario remains test-local. No ballistic scenario helper, invariant module,
energy metric, or standalone experiment was added because there is not yet a demonstrated
second production consumer. Analytical and discrete trajectory behavior was established
before introducing separate conservation claims or deciding their eventual ownership.

## Verified gate

For the current working tree on 2026-08-30:

- the complete repository gate passed 226 pytest tests;
- Ruff lint passed;
- Ruff formatting verification passed;
- strict mypy passed over `src` and `experiments`;
- no warnings were reported; and
- `git diff --check` passed.

## Exact next action

A read-only design review for gravity-only conservation characterization, separating:

- horizontal momentum conservation;
- RK4 mechanical-energy conservation;
- Euler's analytically predicted energy drift;
- test-local calculations versus a reusable invariant API;
- and choosing the smallest first conservation behavior.
