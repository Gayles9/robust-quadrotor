# Reproducible Run Configuration and Named Random Streams

## 1. Milestone summary

Reproducible simulation requires more than a seed. A run must have one immutable description
of the plant and sensor values used by truth, the values assumed by nominal consumers, the
initial state, numerical horizon, sensor timing, commanded input, intentional mismatch, and
the ownership of every stochastic sequence. Without those boundaries, caller mutation or a
change in one sensor's draw count can silently change another part of a run.

Run 1 establishes that foundation. It adds an immutable validated run configuration, explicit
truth/nominal separation, exact mismatch accounting over the currently supported schema, and
six persistent named random generators derived from one root seed. The configuration validates
requested sample periods against the fixed truth grid but neither constructs a scheduler nor
composes a simulation.

Run manifests, canonical JSON, saved NPZ artifacts, artifact hashing, composed simulation,
and replay remain deferred to Runs 2 and 3. Wind, drag, deliberate mismatch effects in the
simulator, estimation, control, Monte Carlo campaigns, ROS 2, and PX4 are also outside this
milestone. Gate G1 remains open.

## 2. Repository baseline and scope

Work began on branch `main`. Both refs resolved to:

```text
HEAD:        0ee34ba52be5a739a85d1cb1c0c437fc1a750c55
origin/main: 0ee34ba52be5a739a85d1cb1c0c437fc1a750c55
commit:      0ee34ba feat: add fixed-rate sensor scheduling
```

The starting worktree was:

```text
## main...origin/main
 M src/quadrotor_math/randomness.py
?? src/quadrotor_math/run_configuration.py
?? tests/unit/test_run_configuration.py
?? tests/unit/test_run_randomness.py
```

The starting checksums were:

```text
18065d08fd02bd086cf6a6b47bc39d617d171b78f33ed8f14cd4693da6da06da  README.md
8236e86607da74fe49da11966914d137c201eda2025fb0c743d3e46162b88f02  src/quadrotor_math/randomness.py
1e150d03cb513916c45d6efbcca2a353b74d1cccfc9bf435cdc173a5f263a3b4  src/quadrotor_math/run_configuration.py
a0165f80c6088a47f2357ecb9b412aa9db3c61e7dccef170e6b327cb3ef2b3a7  tests/unit/test_randomness.py
3486e92a8f3e331be665c456bc8205f296d93cc14f980395464dbff2d4fa63d2  tests/unit/test_run_configuration.py
00a2dbc7ad3adab2e7edde80cc5e98e70d079eaea9e34ea5b9d734bbb5a13f13  tests/unit/test_run_randomness.py
```

Both staged and unstaged diff-integrity checks passed, the index was empty, and this progress
record did not exist. After the repository gate passed, this documentation run changed only:

```text
README.md
docs/progress/2026-09-14-run-configuration-and-random-streams.md
```

It did not change Python source, tests, dependencies, project configuration, or an existing
progress record. It added no manifest, serialization, artifact, simulation-composition,
replay, wind, drag, estimator, controller, ROS 2, or PX4 behavior. Nothing was staged during
development.

## 3. Configuration architecture

The schema follows real ownership and consumer boundaries rather than a generic recursive
configuration hierarchy:

- `RigidBodyParameters` owns mass and the FRD body inertia tensor.
- `RotorParameters` owns four FRD rotor-position vectors, four spin directions, and thrust
  and moment coefficients.
- `WorldParameters` owns the nonnegative magnitude of uniform NED gravity.
- `ImuParameters` owns initial accelerometer and gyroscope bias, measurement-noise standard
  deviation, and bias-random-walk density vectors.
- `PositionSensorParameters` owns NED local-position bias and noise vectors plus positive-up
  barometric reference altitude, bias, and noise standard deviation.
- `TruthConfiguration` groups the rigid-body, rotor, world, IMU, and position-sensor values
  used by truth.
- `NominalConfiguration` independently groups the corresponding values assumed by future
  model-based consumers.
- `IntegrationMethod` gives the stable public values `euler` and `projected_rk4`.
- `RigidBodyInitialState` owns initial NED position and velocity, Hamilton scalar-first
  body-to-world `q_WB`, and FRD body angular velocity.
- `RunNumerics` owns the integration method, fixed truth time step, and authoritative number
  of transitions.
- `SensorSchedule` stores one requested sample period and one fixed delivery delay in seconds.
- `SensorSchedules` explicitly groups accelerometer, gyroscope, local-position, and
  barometric-altitude schedules.
- `ConstantRotorSpeedInput` owns four nonnegative rotor speeds in rad/s.
- `DeclaredMismatch` stores one exact supported parameter path and the rationale for its
  intentional truth/nominal difference.
- `RunConfiguration` groups all of those values with one root seed and the complete declared
  mismatch sequence, and owns the run-level cross-field checks.

The world frame is north-east-down (NED), the body frame is forward-right-down (FRD), and
`R_WB` maps body-coordinate vectors into world coordinates. `position_W` and `velocity_W` are
world-frame vectors, `omega_B` is expressed in the FRD body frame, and `q_WB` is the
body-to-world attitude quaternion. Barometric altitude is positive-up relative to its
explicit reference datum even though NED position `z` is positive-down.

## 4. Ownership and immutability

Every configuration dataclass is declared `frozen=True`, `slots=True`, and `eq=False`.
Generated dataclass equality is intentionally absent: array-bearing records require explicit
domain comparison, and NumPy array equality does not generally produce one scalar Boolean.

Array shapes are validated at their appropriate boundaries, and array-bearing constructors
take independent float64 ownership. Most remaining value validation occurs before ownership
where implemented. `RigidBodyParameters` instead checks the inertia shape, creates an owned
float64 copy, and validates finiteness, symmetry, and positive definiteness on that copy. Only
successfully validated arrays are made non-writeable and stored; caller arrays are never
mutated. Truth and nominal configurations can therefore own distinct values even when they
were built from the same caller array. `RunConfiguration` snapshots mismatch declarations
into a tuple only after every validation succeeds, retaining caller order without retaining a
mutable caller-owned list.

The NumPy writeable flag protects the supported ownership boundary and guards against ordinary
accidental mutation. It is not an adversarial security boundary: sufficiently determined
code outside the public contract may still attempt to bypass NumPy protections.

## 5. Validation contract

Validation below is ordered as implemented. Invalid input raises before a constructed object
is returned and before its owned arrays are exposed.

### `RigidBodyParameters`

Validation order and exact messages are:

```text
mass must be finite
mass must be positive
inertia_B must have shape (3, 3)
inertia_B must contain only finite values
inertia_B must be symmetric
inertia_B must be positive definite
```

After validating mass and the inertia shape, the constructor creates an independently owned
float64 inertia copy. It validates finiteness, symmetry with `np.allclose` using NumPy
defaults, and positive definiteness by Cholesky factorization on that owned copy, then marks
the copy non-writeable and stores it.

### `RotorParameters`

Validation order and exact messages are:

```text
rotor_positions_B must have shape (4, 3)
rotor_spin_directions must have shape (4,)
rotor_positions_B must contain only finite values
rotor_spin_directions must contain only finite values
rotor_spin_directions must contain only -1.0 or 1.0
thrust_coefficient must be finite
moment_coefficient must be finite
thrust_coefficient must be nonnegative
moment_coefficient must be nonnegative
```

Both arrays receive independent read-only float64 copies after validation.

### `WorldParameters`

Validation order and exact messages are:

```text
gravity_acceleration must be finite
gravity_acceleration must be nonnegative
```

Zero gravity is valid.

### `ImuParameters`

The six vectors are checked for shape `(3,)` in this order, then checked for finiteness in
the same order:

```text
initial_accelerometer_bias_B
accelerometer_noise_standard_deviation_B
accelerometer_bias_random_walk_density_B
initial_gyroscope_bias_B
gyroscope_noise_standard_deviation_B
gyroscope_bias_random_walk_density_B
```

The exact shape and finiteness message templates are:

```text
{field_name} must have shape (3,)
{field_name} must contain only finite values
```

The four noise or density vectors are then checked for nonnegative elements in this order:

```text
accelerometer_noise_standard_deviation_B
accelerometer_bias_random_walk_density_B
gyroscope_noise_standard_deviation_B
gyroscope_bias_random_walk_density_B
```

The exact message template is:

```text
{field_name} must be nonnegative
```

All six vectors are then copied independently into read-only float64 storage. Accelerometer
bias and measurement-noise quantities use m/s²; gyroscope quantities use rad/s. Each
continuous-time random-walk density multiplied by the square root of seconds produces its
corresponding bias-increment unit.

### `PositionSensorParameters`

Validation order and exact messages are:

```text
local_position_bias_W must have shape (3,)
local_position_noise_standard_deviation_W must have shape (3,)
local_position_bias_W must contain only finite values
local_position_noise_standard_deviation_W must contain only finite values
local_position_noise_standard_deviation_W must be nonnegative
barometric_reference_altitude must be finite
barometric_altitude_bias must be finite
barometric_altitude_noise_standard_deviation must be finite
barometric_altitude_noise_standard_deviation must be nonnegative
```

The two NED vectors use metres and receive independent read-only float64 copies. Barometric
reference altitude, bias, and noise standard deviation use metres in the positive-up altitude
convention.

### Grouping types

`TruthConfiguration`, `NominalConfiguration`, and `SensorSchedules` are explicit typed
grouping boundaries. They add no runtime validation beyond construction of their validated
children. Python type annotations do not themselves perform runtime type enforcement.

### `RigidBodyInitialState`

`position_W`, `velocity_W`, and `omega_B` require shape `(3,)`; `q_WB` requires shape `(4,)`.
Shapes are validated in position, velocity, quaternion, angular-velocity order, followed by
finiteness in that same order. The exact templates are:

```text
{field_name} must have shape {expected_shape}
{field_name} must contain only finite values
```

Quaternion norm is then checked with `np.isclose(norm, 1.0, rtol=1e-12, atol=1e-12)` and the
exact message:

```text
q_WB must have unit norm
```

The quaternion is rejected rather than silently normalized. All four valid arrays are copied
into read-only float64 storage.

### `RunNumerics`

Validation order and exact messages are:

```text
integration_method must be an IntegrationMethod
truth_time_step_s must be finite
truth_time_step_s must be positive
number_of_steps must be a non-Boolean integer
number_of_steps must be positive
```

The enum violation and invalid integer type raise `TypeError`; numerical-domain violations
raise `ValueError`. `number_of_steps` is the authoritative run horizon.

### `SensorSchedule`

Validation order and exact messages are:

```text
sample_period_s must be finite
sample_period_s must be positive
delivery_delay_s must be finite
delivery_delay_s must be nonnegative
```

Both values are elapsed seconds.

### `ConstantRotorSpeedInput`

Validation order and exact messages are:

```text
rotor_omega must have shape (4,)
rotor_omega must contain only finite values
rotor_omega must be nonnegative
```

The valid rad/s vector receives an independent read-only float64 copy.

### `DeclaredMismatch`

Validation order and exact messages are:

```text
parameter_path must be nonempty
rationale must be nonempty
```

Empty and whitespace-only strings are rejected by testing `.strip()`. Valid strings are
retained exactly as supplied; they are not stripped, normalized, or rewritten.

### `RunConfiguration`

Local validation begins in this exact order:

```text
root_seed must be a non-Boolean integer
root_seed must be in [0, 2**128)
declared_mismatches must contain only DeclaredMismatch values
```

The root-seed type and mismatch-item type errors raise `TypeError`; the range error raises
`ValueError`. Schedule-grid and structural mismatch validation then proceed as documented in
the next two sections. Only after all checks pass is the declaration sequence stored as an
owned tuple.

## 6. Truth-grid cross-field validation

Schedules are validated in this stable order:

1. Accelerometer
2. Gyroscope
3. Local position
4. Barometric altitude

For each channel the calculation is:

```text
sample_stride_ratio = sample_period_s / truth_time_step_s
sample_stride = round(sample_stride_ratio)
effective_sample_period_s = sample_stride * truth_time_step_s
```

A non-finite ratio is rejected before conversion to an integer. Acceptance requires a
strictly positive stride and:

```python
np.isclose(
    sample_period_s,
    effective_sample_period_s,
    rtol=1e-12,
    atol=0.0,
)
```

Every grid failure uses the exact message:

```text
sample_period_s must be an integer multiple of truth_time_step_s
```

The test characterization accepts `0.03 / 0.01`, avoiding an exact-binary-division
requirement. It rejects `0.0125 / 0.005` for each of the four channels. Delivery delays are
validated as finite and nonnegative locally but are not required to align with the truth
grid. `RunConfiguration` does not rewrite a period, store a stride, construct a scheduler, or
advance scheduler state.

## 7. Structural mismatch contract

The private registry contains exactly these 18 paths in schema order:

```text
rigid_body.mass
rigid_body.inertia_B
rotors.rotor_positions_B
rotors.rotor_spin_directions
rotors.thrust_coefficient
rotors.moment_coefficient
world.gravity_acceleration
imu.initial_accelerometer_bias_B
imu.accelerometer_noise_standard_deviation_B
imu.accelerometer_bias_random_walk_density_B
imu.initial_gyroscope_bias_B
imu.gyroscope_noise_standard_deviation_B
imu.gyroscope_bias_random_walk_density_B
position_sensors.local_position_bias_W
position_sensors.local_position_noise_standard_deviation_W
position_sensors.barometric_reference_altitude
position_sensors.barometric_altitude_bias
position_sensors.barometric_altitude_noise_standard_deviation
```

Scalar values use exact `==`. Arrays use `np.array_equal`. Every comparison result is
explicitly converted to a Python `bool`; no generated dataclass equality, approximate
truth/nominal comparison, reflection, serialization, or generic tree walker is involved.

After the schedule checks, the explicit truth/nominal equality registry is constructed, the
caller declarations are snapshotted into a tuple, and their paths are derived in caller
order. Validation and storage then occur in this exact order:

1. Reject the first unsupported path in caller order.
2. Reject duplicate path values, independent of rationale.
3. Build the declared-path set and reject the first undeclared actual difference in schema
   order.
4. Reject the first declared-but-equal path in schema order.
5. Store the already-owned declaration tuple without sorting or rewriting it.

The exact structural messages are:

```text
declared mismatch parameter_path is not supported: {parameter_path}
declared_mismatches must not contain duplicate parameter_path values
truth and nominal differ without a declared mismatch: {parameter_path}
declared mismatch has equal truth and nominal values: {parameter_path}
```

Declaration order is historical input and remains observable in the stored tuple. It does
not affect set equality: the declared path set must equal the actual array-aware difference
set. A deliberate mismatch has no third numerical override; its value exists only as the
difference between truth and nominal, accompanied by one exact-path declaration and
rationale.

## 8. Named RNG architecture

`RunRandomStream` defines six stable enum names, string values, and explicit numeric IDs:

| Enum name | Stable string value | ID |
| --- | --- | ---: |
| `ACCELEROMETER_MEASUREMENT_NOISE` | `accelerometer.measurement_noise` | 1 |
| `ACCELEROMETER_BIAS_RANDOM_WALK` | `accelerometer.bias_random_walk` | 2 |
| `GYROSCOPE_MEASUREMENT_NOISE` | `gyroscope.measurement_noise` | 3 |
| `GYROSCOPE_BIAS_RANDOM_WALK` | `gyroscope.bias_random_walk` | 4 |
| `LOCAL_POSITION_MEASUREMENT_NOISE` | `local_position.measurement_noise` | 5 |
| `BAROMETRIC_ALTITUDE_MEASUREMENT_NOISE` | `barometric_altitude.measurement_noise` | 6 |

Explicit IDs make stream derivation independent of enum iteration order. The implemented
protocol version is `1`. `create_run_rng` constructs exactly:

```python
seed_sequence = SeedSequence(
    [
        1,
        stable_numeric_stream_id,
        root_seed,
    ],
    pool_size=4,
)
generator = Generator(PCG64(seed_sequence))
```

This is the implemented integer-sequence entropy input, not a `spawn_key` scheme. Each child
explicitly uses `PCG64`. `create_run_random_streams` constructs and retains six distinct
generators in a frozen, slotted, `eq=False` `RunRandomStreams` bundle. The bundle exposes one
persistent generator per measurement-noise or bias-random-walk consumer; it does not recreate
a generator per sample.

`create_run_rng` validates in this order:

```text
root_seed must be a non-Boolean integer
root_seed must be in [0, 2**128)
stream must be a RunRandomStream
```

The type violations raise `TypeError`; the root-seed range violation raises `ValueError`.
Bundle construction inherits root-seed validation through its named factory calls.

Tests establish same-root, same-name replay; exact agreement with the versioned reference
construction for all six explicit IDs; agreement between each bundled generator and its
individual factory; six distinct bundled generator objects; and independent consumption, so
drawing 100 values from one named generator does not advance another. The longstanding
`create_rng(seed)` remains `np.random.default_rng(seed)` and retains its original same-seed
replay contract.

## 9. RED/GREEN progression

The failures described here are completed historical TDD evidence, not active failures.

- Initial RED increments established the missing run-configuration module and missing named
  stream APIs.
- The ownership-schema GREEN introduced the frozen grouped configuration boundaries and
  independent read-only array ownership.
- Grouped parameter-validation RED/GREEN increments established rigid-body, rotor, world,
  IMU, and position-sensor local contracts.
- Remaining schema and stream-bundle RED/GREEN increments added state, numerics, schedules,
  constant input, declarations, root-seed handling, named stream derivation, and the
  persistent six-generator bundle.
- A fifty-case local-validation RED was followed by the 140-test configuration GREEN.
- The cross-field test increment added 29 cases: four off-grid channel cases, an aligned
  floating-point case, 18 supported mismatch paths, and six structural/order cases.
- The exact cross-field RED was `9 failed, 160 passed`: the four off-grid schedules and five
  mismatch-consistency checks failed because cross-field validation did not yet exist.
- The minimum production GREEN completed all 169 configuration cases in 0.41s.
- The bounded regression passed 186 tests in 0.40s, and bounded mypy reported
  `Success: no issues found in 2 source files`.

Ruff's canonical import and layout rules were used only within each increment's explicit
scope. During the cross-field RED, the safe fixer reported no fixes and the formatter
reformatted the test file. During the final production GREEN, the safe fixer reported all
checks passed and the formatter reformatted the production file. The subsequent
non-modifying checks reported `All checks passed!` and `2 files already formatted`; neither
formatting event changed the behavioral contract.

## 10. Repository gate

After the complete Python work was present and before documentation changed, the
repository-wide gate was run exactly once:

```text
$ UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache make check
uv run ruff check .
All checks passed!
uv run ruff format --check .
59 files already formatted
uv run mypy src experiments
Success: no issues found in 15 source files
uv run pytest
============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.1.1, pluggy-1.6.0
rootdir: /home/luke/projects/robust-quadrotor
configfile: pyproject.toml
testpaths: tests
collected 645 items

tests/unit/test_actuation.py ........................................... [  6%]
.................                                                        [  9%]
tests/unit/test_dynamics.py ............................................ [ 16%]
...                                                                      [ 16%]
tests/unit/test_imu.py ................................................. [ 24%]
.....................................................................    [ 34%]
tests/unit/test_integration.py ..........                                [ 36%]
tests/unit/test_metrics.py ....................................          [ 42%]
tests/unit/test_position_sensors.py .................................... [ 47%]
..................                                                       [ 50%]
tests/unit/test_randomness.py .                                          [ 50%]
tests/unit/test_rotations.py ..................................          [ 55%]
tests/unit/test_run_configuration.py ................................... [ 61%]
........................................................................ [ 72%]
..............................................................           [ 82%]
tests/unit/test_run_randomness.py ................                       [ 84%]
tests/unit/test_sensor_scheduling.py ................................... [ 89%]
.......................                                                  [ 93%]
tests/unit/test_simulation.py ...................                        [ 96%]
tests/unit/test_validation.py ......................                     [ 99%]
tests/unit/test_vectors.py .                                             [100%]

============================= 645 passed in 14.03s =============================
```

This is completed historical evidence: Ruff lint passed; Ruff format verification found 59
files already formatted; mypy found no issues in 15 source files; and pytest collected and
passed 645 tests in 14.03 seconds. The total is the previously published 460 tests plus 169
run-configuration tests and 16 named run-randomness tests. There were no failures, errors,
skips, or warnings. Verification changed no file. Documentation followed the successful gate,
and the gate was not rerun.

## 11. Reproducibility claim and limitations

The tested claim is deliberately narrow:

> The same configuration, root seed, stream protocol, NumPy environment, and consumption
> order reproduce the same stochastic sequences.

The tests establish this claim for the current environment and the sampled operations. They
do not guarantee:

- replay across arbitrary NumPy or Python versions;
- replay across arbitrary platforms, BLAS implementations, CPUs, or future random
  distribution implementations;
- identity of saved artifacts or NPZ bytes;
- full simulation replay;
- truth isolation from nominal configuration before the Run 3 perturbation composition test;
- wind or drag mismatch behavior;
- estimator behavior; or
- reproducibility of a Monte Carlo campaign.

No scheduler, run configuration, RNG state, manifest, or result artifact is serialized by
Run 1. A root seed does not by itself record consumption order, numerical environment, or all
future composition choices.

## 12. Gate G1 and roadmap status

Run 1 of the reproducible run architecture is complete. It establishes immutable run input,
structural truth/nominal separation, exact mismatch declarations, and independent named
random-stream ownership. Gate G1 remains open.

Run 2 is the exact next action: add an explicit run-manifest schema, canonical JSON, a saved
NPZ artifact, and strict validation, hashing, and ownership. Run 3 will compose those pieces
and test replay, including the nominal-perturbation proof that nominal changes do not leak
into truth. Wind, drag, and deliberate environmental mismatch behavior follow. Estimation,
control, Monte Carlo campaigns, ROS 2, and PX4 remain later work.

## 13. Final repository state

The protected Python milestone files retained these final checksums:

```text
8236e86607da74fe49da11966914d137c201eda2025fb0c743d3e46162b88f02  src/quadrotor_math/randomness.py
1e150d03cb513916c45d6efbcca2a353b74d1cccfc9bf435cdc173a5f263a3b4  src/quadrotor_math/run_configuration.py
a0165f80c6088a47f2357ecb9b412aa9db3c61e7dccef170e6b327cb3ef2b3a7  tests/unit/test_randomness.py
3486e92a8f3e331be665c456bc8205f296d93cc14f980395464dbff2d4fa63d2  tests/unit/test_run_configuration.py
00a2dbc7ad3adab2e7edde80cc5e98e70d079eaea9e34ea5b9d734bbb5a13f13  tests/unit/test_run_randomness.py
```

The final README checksum after its documentation update is:

```text
46e567a99746061fb6aad5ec4900aad91c039ba69030ca6526f271e42857edff  README.md
```

This record is the seventh milestone file. Its own final SHA-256 is necessarily recorded by
the external final-integrity command and task handoff: embedding that digest into this file
would change the file and invalidate the embedded value. That external seven-file checksum
set is the authoritative final record.

The expected actual Git status after this documentation increment is:

```text
## main...origin/main
 M README.md
 M src/quadrotor_math/randomness.py
?? docs/progress/2026-09-14-run-configuration-and-random-streams.md
?? src/quadrotor_math/run_configuration.py
?? tests/unit/test_run_configuration.py
?? tests/unit/test_run_randomness.py
```

Only README is tracked in this documentation diff; ordinary `git diff --stat` does not count
the untracked progress record or the other untracked milestone files. Nothing is staged,
committed, or pushed. Run 2 has not started.
