# 2026-08-30: Rigid-Body State-History Validation

## Milestone summary

The mathematical core now provides reusable unconditional structural and numerical validation
for complete rigid-body state histories. The new public module
`src/quadrotor_math/validation.py` contains
`validate_rigid_body_state_history(...)`, which rejects malformed or corrupted histories and
returns `None` for valid histories. It does not normalize or repair invalid inputs.

Both deterministic explicit-Euler and projected-RK4 simulator outputs pass this validator
directly. This milestone establishes a reusable state-history boundary; it does not prove
dynamic accuracy or implement a physically conditional invariant. Gate G1 is not claimed as
fully passed.

## Public contract

The validator accepts the simulator history arrays in their established order:

```python
validate_rigid_body_state_history(
    time_s,
    position_history_W,
    velocity_history_W,
    q_history_WB,
    omega_history_B,
)
```

The arrays retain the repository's frame and unit contract:

| Array | Required shape | Frame | Units |
| --- | --- | --- | --- |
| `time_s` | `(N,)` | — | s |
| `position_history_W` | `(N, 3)` | NED world | m |
| `velocity_history_W` | `(N, 3)` | NED world | m/s |
| `q_history_WB` | `(N, 4)` | FRD body to NED world | dimensionless |
| `omega_history_B` | `(N, 3)` | FRD body | rad/s |

## Implemented validation order

The function checks:

1. `time_s` has shape `(N,)`;
2. the histories contain at least two samples;
3. `position_history_W` has shape `(N, 3)`;
4. `velocity_history_W` has shape `(N, 3)`;
5. `q_history_WB` has shape `(N, 4)`;
6. `omega_history_B` has shape `(N, 3)`;
7. every state history has the same sample count as `time_s`;
8. `time_s` and every state history contain only finite values;
9. time is strictly increasing;
10. time is uniformly spaced; and
11. every quaternion row has unit Euclidean norm.

Uniform spacing compares every adjacent interval with the first interval using internal
`rtol=1e-12` and `atol=0.0`. Quaternion norms are compared with `1.0` using internal
`rtol=1e-12` and `atol=1e-12`. These tolerances are implementation details rather than new
public parameters.

Invalid inputs raise stable, boundary-specific `ValueError` messages. The validator rejects
nonunit quaternion rows rather than normalizing them, because validation must reveal corrupted
state instead of silently changing it. Negative unit quaternions remain valid, including
negative identity, because quaternion sign does not change the represented attitude.

## Simulator composition evidence

A focused parameterized test reuses the established deterministic two-step simulation
configuration for both:

- `simulate_rigid_body_euler_from_rotor_speeds`; and
- `simulate_rigid_body_rk4_from_rotor_speeds`.

Each simulator's five returned arrays are passed directly to
`validate_rigid_body_state_history(...)` in public return order. Neither quaternion histories
nor any other output is normalized, reshaped, or repaired by the test. Both cases pass.

This proves compatibility between the current simulator outputs and the structural validator
for the tested deterministic configuration. It does not establish trajectory accuracy beyond
the separate convergence evidence.

## Architecture decision

Unconditional structural validity belongs in `validation.py`. Shapes, sample alignment,
finiteness, time-grid structure, and unit quaternion norms have the same meaning regardless of
the applied forces, moments, or scenario, so later controllers, estimators, experiments, and
Monte Carlo studies can reuse one public boundary.

Physically conditional conservation quantities and equilibria remain deferred to later,
explicitly named scenario or invariant work. Energy, momentum, hover, position, velocity,
speed, and angular-velocity magnitude must not be labeled invariant without imposing and
documenting the physical assumptions that make the claim valid.

Rotation-matrix orthogonality and determinant `+1` checks are not duplicated in this history
validator. `R_WB` is not stored in the public rigid-body history, unit quaternion validity is
checked here, and quaternion-to-matrix correctness remains owned and tested by
`src/quadrotor_math/rotations.py`.

## Scientific limitation

Passing this validator proves only that a history satisfies the implemented structural and
numerical contract. It does not prove that the trajectory:

- is dynamically accurate;
- conserves energy or linear or angular momentum;
- represents hover;
- satisfies an equilibrium; or
- satisfies any scenario-specific physical invariant.

Those properties require explicit force, moment, gravity, input, and initial-condition
assumptions plus separately justified tolerances and tests.

## Verification evidence

For the current working tree on 2026-08-30:

- the complete repository gate passed 221 pytest tests;
- Ruff lint passed;
- Ruff formatting verification passed;
- strict mypy passed over `src` and `experiments`;
- no warnings were reported; and
- `git diff --check` passed.

The two new code/test files remain untracked at this documentation milestone. This record
reports the verified working-tree state rather than attributing the milestone to a commit.

## Current limitations and deferred scope

- No physically conditional conservation or equilibrium monitor is implemented.
- No rotation-matrix history is accepted or generated by this validator.
- No simulator automatically invokes the validator; consumers call the reusable boundary
  explicitly.
- No controller, estimator, robustness campaign, or Monte Carlo study consumes it yet.
- Gate G1 is not declared complete by this milestone.

## Next exact action

Conduct a read-only design review for the first deterministic physical reference scenario,
beginning with balanced-thrust, zero-moment hover equilibrium and defining its assumptions,
expected state behavior, tolerances, and smallest TDD test.
