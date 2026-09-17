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

At this Run 1 checkpoint, run manifests, canonical JSON, saved NPZ artifacts, artifact
hashing, composed simulation, and replay remained deferred to Runs 2 and 3. Wind, drag,
deliberate mismatch effects in the simulator, estimation, control, Monte Carlo campaigns,
ROS 2, and PX4 were also outside this milestone. Gate G1 remained open.

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

At this Run 1 checkpoint, Run 2 was the exact next action: add an explicit run-manifest
schema, canonical JSON, a saved NPZ artifact, and strict validation, hashing, and ownership.
Run 3 will compose those pieces and test replay, including the nominal-perturbation proof
that nominal changes do not leak into truth. Wind, drag, and deliberate environmental
mismatch behavior follow. Estimation, control, Monte Carlo campaigns, ROS 2, and PX4 remain
later work.

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
committed, or pushed. Run 2 had not started at that checkpoint.

## Run 2B closeout — 2026-09-16

### Status

Run 2B implementation and the subsequent read-only completeness audit are complete. The
audit found no blocking correctness defect or missing Run 2B test. This section records the
prepared working tree; it does not claim that these files have been staged, committed, or
pushed. Gate G1 remains open because run composition, replay, estimation, and control are
later work.

### Files prepared

The complete Run 2B implementation and test scope before documentation closeout is:

```text
src/quadrotor_math/run_manifest.py
src/quadrotor_math/run_artifact.py
tests/unit/test_run_manifest.py
tests/unit/test_run_artifact.py
```

`README.md` and this existing progress record are the documentation-closeout scope. No
production or test file is changed by documentation closeout.

### Manifest result

`RunManifest` retains backward-compatible version-1 decoding and stable canonical version-1
bytes. An artifact-bound version-2 manifest adds only `data_artifact.sha256`, with exactly 64
lowercase hexadecimal characters. Decoding enforces the declared version's exact schema and
keys; encoding is deterministic canonical UTF-8 JSON. Invalid schema values and digests are
rejected instead of repaired. Save creates a distinct bound manifest with
`dataclasses.replace`; the caller's unbound manifest remains unchanged.

### Artifact model result

`RunArtifactData` has exactly 35 explicitly ordered arrays: 25 one-dimensional fields, eight
width-three fields, and two width-four fields. The groups are truth and command histories,
four six-array sensor streams, accelerometer and gyroscope bias histories, and the three
global-delivery columns. Exact NumPy array type, little-endian float64 or int64 dtype, rank,
and trailing shape are required. Every stored array is an owned, C-contiguous, read-only
copy. The dataclass is frozen, slotted, and identity-equal.

### Intrinsic validation result

Construction validates the static array schema, truth/bias/command leading-dimension
relationships, and the six aligned arrays within each sensor stream. Each stream's sequence
and observation indices are consecutive from zero. Delivered-at-truth entries are either
pending `-1` or a truth update row from 1 through the final row, and all acquisition and
delivery timestamp arrays are finite. The global table accepts only stable sensor IDs 1–4,
contains each delivered observation exactly once with multiplicity, excludes pending
observations, and orders records by delivered truth row, sensor ID, and sequence index.

### Manifest/artifact compatibility result

Save and authenticated load call the same compatibility validator. It checks truth rows
against the configured step count plus one, command rows against the step count, the exact
configured truth-time grid, and every row of the constant four-rotor command. All four sensor
streams must match their configured acquisition counts and acquisition/delivery timestamps.
Sensor timestamp comparisons use `rtol=0` and `atol=1.0e-12` seconds. Delivered-at-truth
rows must be the first truth update reaching the scheduled delivery time under the same
absolute boundary tolerance, or `-1` when no update reaches it. The shared rigid-body
history validator requires finite, strictly increasing and uniformly spaced truth times,
finite state histories, and unit quaternions. Row-zero position, velocity, attitude, angular
velocity, and the initial accelerometer and gyroscope biases must exactly match the
manifest.

### NPZ result

Encoding uses one deterministic in-memory, uncompressed `np.savez` call with the 35 members
named explicitly in data-model order. Decoding uses one in-memory `np.load` call through
`BytesIO` with `allow_pickle=False`. It checks the exact logical member set before reading
any member, then constructs owned immutable arrays while the archive is open. Valid decode
and re-encode produce byte-identical NPZ payloads in the tested environment.

### Save and publication result

`save_run_directory` refuses an existing destination before validation or encoding. It
validates compatibility before creating staging, encodes NPZ bytes once in memory, hashes
those exact bytes, and canonically encodes the bound version-2 manifest. Staging is created
in the destination's parent and contains only `data.npz` and `manifest.json`. Each file is
written and fsynced, then the staging directory is fsynced. One Linux `renameat2` operation
with `RENAME_NOREPLACE` publishes the whole directory; afterward the parent directory is
fsynced before save returns. A racing destination is never replaced. Pre-publication and
publication failures propagate and clean staging. If parent fsync fails after publication,
the failure propagates while the complete published directory remains present and loadable.

### Load result

`load_run_directory` follows this order:

1. Require a directory.
2. Require exactly `manifest.json` and `data.npz` as its entries.
3. Read and decode `manifest.json`.
4. Require a bound manifest.
5. Read `data.npz` once.
6. Verify SHA-256 against the bound manifest before NPZ parsing.
7. Decode those exact authenticated bytes with `allow_pickle=False` and the exact member set.
8. Validate decoded data against the manifest.
9. Return the bound manifest and immutable artifact.

### TDD development record

The bounded RED/GREEN progression covered manifest versioning and digest binding; the data
model and static array schema; leading dimensions; deterministic encoding; save/load happy
paths; destination and cleanup failures; atomic no-replace publication; decoder and
directory-shell validation; exact NPZ member sets; zero-based stream indices; delivered-index
domain; finite sensor timestamps; global delivery-table membership and order; manifest
compatibility; initial-state and bias compatibility; shared rigid-body validation; configured
sensor counts and timestamps; delivery-row and pending semantics; and crash-durable
publication. Each accepted RED failed only for its intended missing behavior, and each GREEN
retained the protected tests and files. The read-only closeout audit then checked the full
implementation and test matrix.

### Final verification

The 2026-09-16 audited gate produced:

| Check | Result |
| --- | ---: |
| Focused artifact and manifest tests | 409 passed |
| Bounded artifact, manifest, configuration, and randomness tests | 594 passed |
| Sensor-scheduling tests | 58 passed |
| Rigid-body validation tests | 22 passed |
| Full suite | 1,054 passed; 0 skipped; 0 xfailed |
| Ruff lint | Passed |
| Ruff format check | 64 files already formatted |
| Mypy over `src/quadrotor_math` | No issues in 16 source files |

The Run 2B source and tests contained no `TODO`, `FIXME`, `NotImplementedError`, `xfail`,
or skip markers. These results are a verified working-tree snapshot, not a standing promise
about later revisions.

### Design decisions

- Version 1 remains the unbound compatibility format; artifact binding requires version 2.
- Schema, digest, array, and configuration checks reject invalid data rather than repairing
  or normalizing it.
- Persisted arrays are independently owned and marked read-only.
- Digest authentication precedes NPZ parsing. It binds data to the supplied manifest; it is
  not a signature over a potentially malicious manifest.
- Publication uses Linux no-replace `renameat2` with file, staging-directory, and
  parent-directory fsyncs for crash durability. Missing `renameat2` support fails closed.
- Save and load share one configuration-compatibility validator.
- Run 2B does not claim hostile-input hardening.

### Run 2B handoff

Run 2C followed as a separate, bounded manifest-decoder validation increment. Its completed
scope and verification are recorded below.

## Run 2C manifest-decoder validation closeout — 2026-09-17

### Scope and result

The complete Run 2C code and test scope before this documentation closeout is exactly:

```text
src/quadrotor_math/run_manifest.py
tests/unit/test_run_manifest.py
```

Manifest decoding rejects duplicate keys before a JSON object's pairs become a dictionary.
The check applies independently to every object, including nested objects. Its exact
`ValueError` format is:

```text
manifest JSON contains duplicate key: <key>
```

JSON parsing rejects the nonstandard numeric constants `NaN`, `Infinity`, and `-Infinity`.
Their exact `ValueError` format is:

```text
manifest JSON contains nonstandard constant: <token>
```

The decoded `run_configuration.numerics.duration_s` must exactly equal
`truth_time_step_s * number_of_steps`. A mismatch raises `ValueError` with this exact message:

```text
manifest run_configuration.numerics.duration_s must equal truth_time_step_s * number_of_steps
```

Canonical encoding, manifest versions, and public APIs are unchanged.

### RED/GREEN progression

The sequence was a top-level duplicate-key RED test, duplicate-key GREEN implementation,
nested duplicate-key characterization, `NaN` RED test, nonstandard-constant GREEN
implementation, positive- and negative-infinity characterization, inconsistent-duration RED
test, and exact derived-duration GREEN validation. Each RED failure was limited to its
intended missing behavior; each GREEN retained the previously accepted tests.

### Audited verification

| Check | Result |
| --- | ---: |
| Manifest tests | 138 passed |
| Manifest and artifact tests | 415 passed |
| Full suite | 1,060 passed; 0 skipped; 0 xfailed; 0 warnings |
| Ruff lint | Passed |
| Ruff format check | 64 files already formatted |
| Mypy | No issues in 16 source files |

These results cover the byte-identical production and test files present before this
documentation-only closeout.

### Design decisions

- JSON object keys must be unique at every nesting level.
- Manifest decoding accepts only standard JSON numeric syntax.
- Redundant derived configuration values are validated rather than trusted or repaired;
  the derived-duration comparison is exact.
- The validation remains internal to decoding. No public API or encoded schema changed.

### Next exact project action

1. Review the documentation-only diff.
2. Run the final complete quality gate.
3. Stage exactly these four Run 2C code, test, and documentation files for publication:

   ```text
   README.md
   docs/progress/2026-09-14-run-configuration-and-random-streams.md
   src/quadrotor_math/run_manifest.py
   tests/unit/test_run_manifest.py
   ```

4. Commit and push only after explicit publication authorization.
5. Begin Run 3 as the next separate milestone.
