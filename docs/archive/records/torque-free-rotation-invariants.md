# 2026-08-31: Torque-Free Rotation Invariants

## Milestone summary

Two focused tests characterize nonlinear torque-free rotation of one asymmetric rigid body.
The dynamics-boundary test verifies the instantaneous rotational-energy identity, while the
projected-RK4 simulation test verifies small bounded inertial-angular-momentum drift over one
declared grid.

This milestone adds no production invariant or diagnostic API. Its calculations remain local
to the tests, and Gate G1 remains open.

## Scenario

The deterministic rotational configuration is:

| Quantity | Value |
| --- | --- |
| `inertia_B` | `diag(2.0, 3.0, 4.0) kg·m²` |
| Initial `omega_B` | `[0.7, -0.4, 1.1] rad/s` |
| Initial `q_WB` | Identity `[1.0, 0.0, 0.0, 0.0]` |
| Applied body moment | `[0.0, 0.0, 0.0] N·m` |
| Rotor speeds | `[0.0, 0.0, 0.0, 0.0] rad/s` |
| Projected-RK4 time step | `0.05 s` |
| Projected-RK4 steps | `200` |
| Total duration | `10.0 s` |

Gravity affects translation, but it does not enter the rotational equations in the current
centre-of-mass rigid-body model. Zero rotor speeds produce zero body force and zero applied
body moment, so the rotational claims remain dynamically separate from the gravity-driven
translation.

## Continuous rotational dynamics

For zero applied moment, the Euler rigid-body equation is

```text
I_B @ omega_dot_B + omega_B × (I_B @ omega_B) = 0.
```

Body-frame angular momentum is

```text
H_B = I_B @ omega_B.
```

For the initial state,

```text
H_B_0 = diag(2, 3, 4) @ [0.7, -0.4, 1.1]
      = [1.4, -1.2, 4.4] kg·m²/s.
```

The gyroscopic term is

```text
omega_B × H_B = [-0.44, -1.54, -0.28] N·m.
```

Therefore,

```text
I_B @ omega_dot_B = -omega_B × H_B
                  = [0.44, 1.54, 0.28]

omega_dot_B = [0.22, 0.5133333333333333, 0.07] rad/s².
```

The angular acceleration is nonzero despite zero applied moment. Gyroscopic coupling
redistributes the body-frame angular-velocity components while satisfying the torque-free
equations.

## Rotational kinetic energy

Rotational kinetic energy is

```text
T = 0.5 * omega_B.T @ I_B @ omega_B.
```

Because `I_B` is constant and symmetric,

```text
dT/dt = omega_B.T @ I_B @ omega_dot_B
       = -omega_B.T @ (
             omega_B × H_B
         )
       = 0.
```

The cross product is perpendicular to `omega_B`, which makes the final scalar product zero.
For the characterized initial condition,

```text
T_0 = 0.5 * (
          2 * 0.7²
          + 3 * (-0.4)²
          + 4 * 1.1²
      )
    = 3.15 J.
```

The derivative-level test verifies both:

- the nonzero hand-derived `omega_dot_B`; and
- an instantaneous rotational-energy rate of zero within `1e-12 W`.

The simulation test does not test or establish exact multi-step rotational-energy
conservation.

## Inertial-frame angular momentum

The frame contract defines `R_WB` as mapping body-frame vector components into the NED world
frame. Inertial-frame angular momentum is therefore

```text
H_W = R_WB @ H_B.
```

For the repository's Hamilton scalar-first body-to-world quaternion convention,

```text
R_dot_WB = R_WB @ skew(omega_B).
```

Differentiating `H_W` gives

```text
H_dot_W
    = R_WB @ (
          omega_B × H_B
          + H_dot_B
      )
    = 0,
```

because torque-free body dynamics give

```text
H_dot_B = -omega_B × H_B.
```

Thus `H_W` remains fixed in inertial space. `H_B` changes because it is expressed in rotating
body axes, the components of `omega_B` change, and the Euclidean magnitude of `omega_B` need
not remain constant when the principal inertias differ. None of those changes violates
rotational-energy or inertial-angular-momentum conservation.

At the identity initial attitude,

```text
H_W_0 = [1.4, -1.2, 4.4] kg·m²/s.
```

## Numerical characterization

The projected-RK4 simulator runs for `10.0 s` with time step `0.05 s`. For every stored
sample, the test calculates

```text
H_B_i = I_B @ omega_B_i
H_W_i = R_WB(q_WB_i) @ H_B_i.
```

The complete `H_W` history is compared with the repeated initial vector using

```text
rtol = 0.0
atol = 5e-7 kg·m²/s.
```

This is an absolute componentwise numerical bound. A prior deterministic diagnostic
measurement for this exact scenario, duration, and grid found a maximum world-frame
angular-momentum vector-error norm of approximately `2.6485544e-7 kg·m²/s`. The `5e-7`
assertion provides controlled deterministic headroom while retaining a physically meaningful
bound.

Projected RK4 does not exactly conserve angular momentum at finite step size. The test
characterizes small bounded drift only for this scenario, duration, and grid; it is not a
general proof of long-term stability or conservation. Quaternion projection keeps quaternion
norms valid, but it does not turn classical RK4 into a geometric or invariant-preserving
integrator.

## Test ownership

The dynamics-boundary test is

```text
tests/unit/test_dynamics.py
test_angular_acceleration_body_from_moment_has_zero_rotational_energy_rate_when_torque_free
```

The multi-step simulation test is

```text
tests/unit/test_simulation.py
test_rk4_torque_free_rotation_nearly_conserves_inertial_angular_momentum
```

They remain separate because:

- energy rate is a scalar dynamics-boundary identity in watts;
- inertial angular momentum is a world-frame vector that crosses rotations, dynamics,
  integration, and simulation; and
- separate tests preserve diagnostic clarity and avoid mixing units.

## Architecture decision

No production functions were added to `metrics.py`, `validation.py`, or a new `invariants.py`
module. Test-local calculations remain preferable until actual production consumers establish
whether a reusable API should return raw quantities, residual histories, pass/fail judgments,
tolerances, or explicit scenario assumptions.

## Limitations and deferred scope

This milestone explicitly defers:

- multi-step rotational-energy drift characterization;
- Euler torque-free drift characterization;
- arbitrary initial attitude;
- general non-diagonal inertia;
- intermediate-axis instability;
- analytical elliptic-function solutions;
- long-duration conservation studies;
- external moments and work-energy balance;
- rotor gyroscopic effects;
- motor and aerodynamic moments; and
- reusable invariant-monitoring APIs.

## Verification

For the current working tree on 2026-08-31:

- Ruff lint passed;
- Ruff formatting verification passed;
- strict mypy passed over `src` and `experiments`;
- the complete pytest suite passed 230 tests;
- no warnings were reported; and
- `git diff --check` passed.

## Gate status and exact next action

Gate G1 remains open.

Perform a read-only remaining-Gate-G1 evidence review to determine which deterministic
physical characterization is still necessary before closing the plant-model and
numerical-foundation gate, including whether multi-step rotational-energy drift needs its own
test.
