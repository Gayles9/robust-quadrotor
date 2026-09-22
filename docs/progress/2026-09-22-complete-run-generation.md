# Complete-run generation — 2026-09-22

## Objective and scope

This milestone joins the already validated plant, motor, sensor, scheduler, randomness, and
artifact boundaries into one deterministic in-memory run. It follows the six-month project's
configuration and persistence work and advances Gate G1 without closing it. The public API is:

```python
from quadrotor_math.run_generation import generate_run_artifact_data

data = generate_run_artifact_data(configuration)
```

`configuration` is an already validated `RunConfiguration`; the result is a complete
`RunArtifactData`. Generation does not write files, capture software provenance, build a
manifest, invoke the control allocator, or alter the 35-array artifact schema. The configured
`ConstantRotorSpeedInput.rotor_omega` already supplies commands, so allocation belongs
upstream when a future controller provides thrust and moment demands.

## Truth-grid event order

The authoritative grid is `truth_time_s[i] = i * truth_time_step_s`, including indices zero
through `number_of_steps`. Row zero copies the configured initial position, velocity,
body-to-world `q_WB`, FRD angular velocity, and initial IMU biases exactly. For each transition
from row `k` to `k + 1`, the generator:

1. Stores the configured four-rotor command in `commanded_rotor_omega[k]`.
2. Selects actual speed. Complete motorized mode advances the previous actual speed with
   `motor_speed_first_order_step` and **truth** speed limits and time constant; historical mode
   uses the command directly.
3. Advances the rigid-body state with the selected speed and truth rigid-body, rotor, and
   world parameters, dispatching to Euler or projected RK4 as configured.
4. Advances accelerometer and gyroscope bias random walks once with their dedicated streams.
5. Updates accelerometer, gyroscope, local position, and barometric altitude schedulers in
   stable sensor-ID order at the completed truth row.

All seven motor-configuration values must be absent or present: three truth motor values,
three nominal motor values, and initial actual speed. A partial combination raises
`run generation motor configuration must be either entirely omitted or complete` before RNG
construction. Nominal motor values classify completeness only. A test changes nominal mass,
thrust coefficient, gravity, accelerometer bias, and position bias while leaving truth fixed;
all 35 generated arrays remain exactly equal.

The six named RNG streams are constructed once per accepted run. Separate streams own
accelerometer measurement noise, accelerometer bias walk, gyroscope measurement noise,
gyroscope bias walk, local-position measurement noise, and barometric-altitude measurement
noise. Same-seed runs reproduce all 35 arrays exactly in the tested environment; a changed
root seed changes the intended stochastic fields while truth, commands, sensor indices, and
configured timestamps remain equal. This does not assert cross-version NumPy reproducibility.

## Sensor acquisition, delivery, and artifact

Schedulers acquire before delivering at one update. An acquisition at truth row `i` uses its
truth state and bias row `i`. Accelerometer acquisition reconstructs actual rotor speed at
row `i`, obtains instantaneous world acceleration from the existing rigid-body derivative,
converts it to FRD specific force with `R_WB`, and applies truth bias and noise. Gyroscope
sampling uses truth FRD angular velocity and its truth bias and noise. Local position uses
truth NED position; barometric altitude uses truth position and the truth reference altitude.

Each stream stores consecutive sequence and observation indices, scheduled acquisition and
delivery times, measurements, and the first truth row at which a delivery becomes available.
Pending deliveries have index `-1`. The global table contains delivered observations once,
ordered by delivered truth row, stable sensor ID (1–4), and sequence index; pending records
are absent. The audit confirmed simultaneous deliveries follow that order.

All 35 existing fields retain their exact little-endian float64 or int64 dtype, shape, and
order. `RunArtifactData` takes owned, C-contiguous, read-only copies without aliasing the
configuration. One-step runs with every sample period beyond the horizon produce empty
`(0, 3)` vector streams and `(0,)` scalar and metadata streams; save/load accepts them. Runs
with all deliveries beyond the horizon retain every acquisition and measurement, mark every
delivery `-1`, and have an empty global table; save/load accepts them too. Actual rotor-speed
history is reconstructed privately and is not an artifact field.

## Persistence and evidence

The generator returns data only. The caller can supply an unbound `RunManifest` to the
existing `save_run_directory` API and load it through `load_run_directory`. Historical ideal
actuation uses unbound manifest version 1 and bound version 2. Complete motorized runs use
unbound version 3 and bound version 4. Both paths passed compatibility validation. The
motorized round trip preserved every array exactly, and re-encoding the loaded manifest and
NPZ produced byte-identical files in the tested environment. The original unbound manifest
and generated data remained unchanged.

The TDD stub first produced 10 intended `NotImplementedError` failures. After implementation,
one test expected decimal `0.3` instead of the required grid product `3 * 0.1`; correcting
that test expectation produced 10 passing tests. A nominal-isolation test raised the count
to 11. Independent audit probes found no production defect. They exposed missing persistent
coverage for wholly empty acquisition streams and wholly pending deliveries; two focused
regression tests raised the generator count to 13. A separate probe confirmed that partial
motor rejection creates no RNG streams and an accepted run creates the named bundle once.
Final review also extended the historical persistence test from save-only coverage to
save/load coverage without adding motor fields. No production correction was needed.

With `UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache`, the code audit passed:

| Command | Result |
| --- | --- |
| `uv run pytest -q tests/unit/test_run_generation.py` | 13 passed |
| `uv run pytest -q` on the ten related test files | 1,082 passed |
| `uv run pytest -q` | 1,239 passed |
| `uv run ruff check .` | Passed |
| `uv run ruff format --check .` | 66 files already formatted |
| `uv run mypy src experiments` | No issues in 18 source files |
| `make check` | Passed with 1,239 tests |

No failures, errors, skips, xfails, or warnings were reported. The final publication-readiness
gate reran `make check` and both Git diff checks after the README and this record were written.

## Limits and next milestone

The generated artifact does not persist actual rotor-speed history. The existing fixed-input
simulators still do not propagate motor state, and there is no separate replay API. Wind,
drag, deliberate mismatch effects and their validation evidence, control, and estimation
remain future Gate G1 work. Gate G1 remains open. The next bounded project milestone should
build evidence for deliberate truth/nominal mismatch and environmental effects on top of this
complete-run boundary.

This generator and documentation closeout have **not** been staged, committed, pushed, or
published.

## Publication and replay closeout — 2026-09-22

The generator was published in commit `4cc132af291f9352b8145e6397482e8a632e76fd`,
whose parent is `9579bb920ab528bb6e78ba29e4568844d9e7f508`. That commit contained exactly
`README.md`, `docs/progress/2026-09-22-complete-run-generation.md`,
`src/quadrotor_math/run_generation.py`, and `tests/unit/test_run_generation.py`.
[GitHub Actions run 35674673151](https://github.com/Gayles9/robust-quadrotor/actions/runs/35674673151)
concluded successfully for that commit. The hosted environment reported Python 3.12.14,
NumPy 2.5.2, and pytest 9.1.1. Ruff passed, Ruff format reported 67 files already formatted,
mypy passed for 18 source files, and all 1,239 tests passed.

The follow-up characterization loads a saved historical version-2 or motorized version-4
manifest and regenerates from its decoded `run_configuration` through the public generator.
Both fixtures have nonzero sensor noise and bias random walks. In each case, all 35 original,
loaded, and regenerated artifact arrays are exactly equal. Saving the regenerated data with
the loaded bound manifest reproduces the canonical `manifest.json` and `data.npz` bytes
exactly. The historical unbound/bound versions are 1/2; the motorized versions are 3/4.
This is deterministic replay in the tested environment, without a new replay API.

The follow-up verification used `UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache`:

| Command | Result |
| --- | --- |
| `uv run pytest -q tests/unit/test_run_generation.py` | 15 passed |
| `uv run pytest -q tests/unit/test_run_configuration.py tests/unit/test_run_manifest.py tests/unit/test_run_artifact.py tests/unit/test_run_generation.py` | 668 passed |
| `uv run pytest -q` | 1,241 passed |
| `uv run ruff check .` | Passed |
| `uv run ruff format --check .` | 67 files already formatted |
| `uv run mypy src experiments` | No issues in 18 source files |
| `make check` | Passed with 1,241 tests |

No production code or artifact schema changed. Generation remains an in-memory boundary,
with explicit saving and upstream allocation; actual rotor-speed history remains private and
reconstructable. Gate G1 remains open for deliberate truth/nominal mismatch effects, wind and
drag evidence, and the remaining planned work.
