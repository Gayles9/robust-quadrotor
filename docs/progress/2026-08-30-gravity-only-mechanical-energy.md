# 2026-08-30: Gravity-Only Mechanical-Energy Characterization

## Milestone summary

Two focused simulation tests characterize mechanical energy for the established deterministic
gravity-only ballistic scenario. Projected RK4 preserves the continuous mechanical-energy
value across the tested grid, while explicit Euler follows its independently derived positive
energy-drift law.

This is a conditional, scenario-specific physical property rather than a universal invariant.
The calculations remain local to the tests, no production API was added, and Gate G1 remains
open.

## Conditional physical assumptions

The characterization applies only under these assumptions:

- all four rotor speeds are zero;
- there is no thrust or applied body moment;
- gravity is constant and uniform in the north-east-down (NED) world frame;
- there is no drag, wind, motor loss, ground contact, or other nonconservative effect; and
- mass is constant at `2.0 kg`.

These assumptions are required for the stated conservation law. Mechanical energy is not a
universal invariant of the existing forced dynamics model.

## NED mechanical-energy definition

Positive world `z` points downward in NED coordinates. Gravitational potential energy
therefore carries a negative sign:

```python
potential_energy = -mass * gravity_acceleration * position_history_W[:, 2]

kinetic_energy = 0.5 * mass * np.sum(velocity_history_W**2, axis=1)

mechanical_energy = kinetic_energy + potential_energy
```

Equivalently, for world-frame velocity `v_W` and NED down position `z_W`,

```text
E = 0.5 * m * ||v_W||² - m * g * z_W.
```

Under gravity-only motion, `gravity_W = [0, 0, +g]` and
`dz_W/dt = velocity_W[2]`. Thus

```text
dE/dt = m * v_W · gravity_W - m * g * dz_W/dt
      = m * g * velocity_W[2] - m * g * velocity_W[2]
      = 0.
```

## Initial energy

The established scenario starts with:

- `position_W = [10.0, 20.0, 30.0] m`;
- `velocity_W = [1.0, -2.0, -3.0] m/s`;
- mass `2.0 kg`; and
- gravity magnitude `9.81 m/s²`.

The initial energies are

```text
kinetic_energy_0 = 0.5 * 2.0 * (1.0² + (-2.0)² + (-3.0)²)
                 = 14.0 J

potential_energy_0 = -2.0 * 9.81 * 30.0
                   = -588.6 J

mechanical_energy_0 = 14.0 - 588.6
                    = -574.6 J.
```

## Projected-RK4 conservation evidence

The projected-RK4 simulator already reproduces the analytical constant-gravity trajectory.
The new energy test calculates kinetic, potential, and mechanical energy directly from its
returned position and velocity histories. The calculated mechanical-energy history remains
`-574.6 J` across all five samples and is compared with `rtol=0.0` and `atol=1e-12 J`.

This tight behavior is expected for this short polynomial constant-gravity case. It is not a
universal claim that projected RK4 exactly conserves energy for arbitrary nonlinear systems.
Quaternion projection has no physical effect here because attitude remains constant and unit
length.

## Explicit-Euler drift derivation and evidence

Euler velocity is exact at the grid times in this scenario because acceleration is constant.
Euler position has the known bias from using the velocity at the beginning of each step. When
that discrete position and the exact grid-time velocity are substituted into the NED energy
definition, the resulting energy law is

```text
E_n - E_0 = (
    0.5
    * mass
    * gravity_acceleration**2
    * t_n
    * time_step
).
```

For mass `2.0 kg`, gravity `9.81 m/s²`, time step `0.25 s`, and grid times
`[0.0, 0.25, 0.5, 0.75, 1.0] s`, the expected energy history is approximately

```text
[
    -574.6,
    -568.58524375,
    -562.5704875,
    -556.55573125,
    -550.540975,
] J.
```

At `t = 1.0 s`, the energy drift is `+24.059025 J` and final Euler energy is
`-550.540975 J`. The test compares the calculated history with this derived law using
`rtol=0.0` and `atol=1e-12 J`.

This positive drift is predictable numerical error caused by Euler's position discretization.
It is not physical energy entering the vehicle and is not a defect in the dynamics model.

## Horizontal momentum evidence

The committed analytical RK4 and discrete Euler trajectory tests already assert at every grid
time that

- `velocity_W[:, 0] = 1.0 m/s`;
- `velocity_W[:, 1] = -2.0 m/s`; and
- mass is constant at `2.0 kg`.

They therefore implicitly verify constant horizontal momentum

```text
horizontal_momentum_W = mass * velocity_W[:, 0:2]
                      = [2.0, -4.0] kg·m/s
```

at every sample. A duplicate momentum-only test was deliberately not added.

## Architecture decision

Energy is calculated locally in the two simulation tests. This milestone adds no public energy
function, `metrics.py` extension, `invariants.py` module, generic monitor, report object, or
tolerance policy.

Important public-API questions remain unresolved, including:

- raw energy values versus energy residuals;
- translational versus rotational energy;
- gravitational potential reference conventions;
- validation ownership; and
- how external work and scenario assumptions should be represented.

Without a second production consumer and answers to those questions, extracting a reusable
physical-diagnostics API would be premature.

## Verified gate

For the current working tree on 2026-08-30:

- the complete repository gate passed 228 pytest tests;
- Ruff lint passed;
- Ruff formatting verification passed;
- strict mypy passed over `src` and `experiments`;
- no warnings were reported; and
- `git diff --check` passed.

## Scientific limitations

- This characterization does not establish energy conservation under rotor thrust, drag,
  wind, motor losses, ground contact, or other external work.
- It does not prove that projected RK4 is generally energy-preserving.
- It does not establish stability, control, state estimation, or robustness.
- Gate G1 remains open.

## Exact next action

A read-only design review for torque-free rigid-body rotation, separating:

- rotational kinetic-energy conservation;
- body-frame angular-velocity evolution;
- inertial-frame angular-momentum conservation;
- quaternion attitude evolution;
- Euler versus projected-RK4 behavior; and
- whether test-local calculations or a reusable physical-diagnostics API are justified.
