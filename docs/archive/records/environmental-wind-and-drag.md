# Environmental Wind and Quadratic Drag — 2026-09-22

## Published starting baseline

Gate G1 started from published `main` commit
`65a48da9ddd64bd98e4b2faf473c33cd514c3485` (`test: prove complete run replay`).
The local implementation was developed as four bounded slices on top of that commit. At the
final audit, `HEAD` and fetched `origin/main` still named that commit, the index was empty,
and exactly the expected thirteen source and test files were modified and unstaged.

## Gate objective

Gate G1 adds a deterministic environmental disturbance to the existing NED/FRD rigid-body
core without adding an aerodynamic moment, changing rotor allocation, or expanding the
35-array run artifact. The gate includes the pure force law, configuration and deliberate
truth/nominal mismatch support, propagation and accelerometer consistency, canonical
manifest persistence, and exact replay in the tested environment.

## Selected model and contract

Wind is constant and expressed in the NED world frame. Vehicle-relative air velocity is
rotated into the FRD body frame before applying one nonnegative lumped coefficient per body
axis:

```text
velocity_air_W = velocity_W - wind_velocity_W
velocity_air_B = R_WB.T @ velocity_air_W
force_drag_B = (
    -quadratic_drag_coefficient_B
    * abs(velocity_air_B)
    * velocity_air_B
)
```

`velocity_W`, `wind_velocity_W`, and both relative-air velocities use m/s.
`quadratic_drag_coefficient_B` uses kg/m, making `force_drag_B` a body-frame force in
newtons. The elementwise law gives

```text
force_drag_B @ velocity_air_B
    = -sum(coefficient_B[i] * abs(velocity_air_B[i]) ** 3)
    <= 0
```

so the force cannot add energy relative to the air. It acts at the modelled centre of mass
and adds no aerodynamic moment. The accepted scope is constant wind and axis-lumped drag,
not gusts, turbulence, an air-density/Cd/area decomposition, aerodynamic moments, CFD, or a
blade-element model.

## Slice A — pure force and dynamics boundary

The RED boundary required a public, independently testable force law with exact signs,
frames, validation order, ownership, finite-arithmetic handling, and zero behavior. The
GREEN implementation:

- validates all four array shapes before finiteness, then rejects negative coefficients;
- transforms relative-air velocity through `R_WB.T` before applying anisotropic drag;
- returns an owned, C-contiguous float64 vector without mutating or aliasing inputs;
- returns exact zero for zero relative airspeed and avoids extreme inactive-axis arithmetic
  when a coefficient is zero;
- translates finite-input arithmetic overflow to
  `quadratic drag calculation must remain finite`;
- validates rotor actuation before environmental inputs at the rotor-speed derivative;
- preserves the historical rotor-force arithmetic path when drag is exactly zero;
- sums rotor and drag forces while leaving the rotor moment unchanged; and
- translates force-sum overflow to
  `combined rotor and quadratic drag force must remain finite`.

North-axis anchors make the sign explicit. With identity attitude, coefficient
`[0.5, 0, 0] kg/m`, calm air, and north velocity `+2 m/s`, the drag force is
`[-2, 0, 0] N`. A stationary vehicle in a `+1 m/s` north wind instead receives
`[+0.5, 0, 0] N`. A rotated anisotropic example maps relative-air velocity
`[2, 3, -2]` NED to `[3, -2, -2]` FRD and produces `[-4.5, 4, 8] N`.

## Slice B — configuration and declared mismatch

The RED boundary rejected the absent environmental ownership and mismatch contract. The
GREEN implementation adds independently constructed float64 zero defaults to both truth and
nominal rigid-body/world parameter groups. Explicit arrays are copied, C-contiguous,
read-only, and independent of caller mutation. Wind must be finite; coefficients must be
finite and nonnegative. Adding the fields at the end of their parameter constructors keeps
historical positional calls compatible.

The two exact declared-mismatch paths are:

- `rigid_body.quadratic_drag_coefficient_B`
- `world.wind_velocity_W`

They occupy their defined registry positions without changing the existing validation
precedence for unsupported paths, duplicate declarations, undeclared differences, or
declared-but-equal values. Accepted declaration order is retained.

## Slice C — propagation, sensing, and deliberate mismatch

The propagation RED cases required drag to follow each evaluated state rather than a value
cached at the start of a step. The GREEN implementation evaluates the environmental
derivative once from the current Euler state and separately at projected-RK4 k1, k2, k3,
and k4. Each RK4 evaluation uses that stage's velocity and projected attitude. Both history
simulators forward the environment on every step. Omitting the environment and passing
explicit zero arrays reproduce the historical results exactly.

The complete-run generator passes only truth wind and truth drag coefficients to physical
propagation and to the completed-row derivative used for accelerometer truth. Nominal values
never enter generated physical or stochastic data. The four combinations of nominal wind or
coefficient changes with noise/bias walks disabled or enabled leave every one of the 35
artifact arrays exactly equal. RNG stream construction and consumption order are unchanged,
rotor allocation remains upstream, and actual rotor-speed history remains private and
reconstructable.

### Euler hand calculations and accelerometer consistency

For a level 1 kg fixture with north velocity `2 m/s`, time step `0.1 s`, and north-axis
coefficient `0.5 kg/m`, calm-air drag is `-0.5 * 2² = -2 N`. Euler therefore gives
`v₁ = 2 + 0.1(-2) = 1.8 m/s`. The completed-row derivative is
`-0.5 * 1.8² = -1.62 m/s²`, which is the accelerometer's north specific force.

With truth north wind `+1 m/s`, initial relative airspeed is `1 m/s`: Euler gives
`v₁ = 1.95 m/s`, then completed-row relative airspeed is `0.95 m/s` and the accelerometer
receives `-0.5 * 0.95² = -0.45125 m/s²`. With calm air and coefficient `1 kg/m`, Euler gives
`v₁ = 1.6 m/s` and completed-row acceleration `-1.6² = -2.56 m/s²`. Persistent tests verify
all three trajectory/accelerometer pairs.

### RK4 stage and convergence evidence

An independent one-step audit of the scalar project path for
`dv/dt = -0.5 * abs(v) * v`, `v(0) = 2`, and `h = 0.1 s` observed derivative stages
`[-2.0, -1.805, -1.8235725312500002, -1.6519125776336478]`. The resulting state was
`x₁ = 0.19061904578125002 m` and `v₁ = 1.8181823726644393 m/s`. Different stage values
demonstrate reevaluation rather than cached start-of-step drag.

For the exact solution `v(t) = 2 / (1 + t)` at `t = 1 s`, successively halving the fixed
step produced:

| Step (s) | Absolute velocity error |
| ---: | ---: |
| 0.2 | 8.812316451445312e-06 |
| 0.1 | 5.951604620246798e-07 |
| 0.05 | 3.7794904983456945e-08 |
| 0.025 | 2.370830198827889e-09 |
| 0.0125 | 1.483071443431072e-10 |

The adjacent error ratios were `14.81`, `15.75`, `15.94`, and `15.99`, corresponding to
observed orders `3.888`, `3.977`, `3.995`, and `3.999`. This is audit evidence for the
tested scalar case, not a cross-platform guarantee.

## Slice D — canonical manifests and replay

The manifest RED/GREEN work was divided into schema/version selection, strict decoding, and
public persistence/replay. The first focused RED run reported 14 failures and 4 passes, then
18 passes after the version-selection increment. The second focused RED run reported 23
failures and 12 passes, then 35 passes after strict decoding. The completed manifest module
passes all 219 of its tests; focused artifact and generation replay checks also pass.

The final matrix is:

| Environment | Motor mode | Unbound | Bound |
| --- | --- | ---: | ---: |
| Zero | Historical | 1 | 2 |
| Zero | Motorized | 3 | 4 |
| Nonzero | Historical | 5 | 6 |
| Nonzero | Motorized | 5 | 6 |

The public schema-version constant remains 1. Bound versions are `{2, 4, 6}`; strict
motorized versions are `{3, 4}`; motor-field versions are `{3, 4, 5, 6}`; and environmental
versions are `{5, 6}`. Versions 5/6 add only the approved drag and wind keys to the strict
rigid-body and world key sets. Their motor fields are all null for historical actuation and
all populated for motorized actuation; partial motor state is rejected. A manually labelled
v5/v6 manifest whose truth and nominal environment are entirely zero is also rejected.
Versions 1–4 retain their exact strict schemas and canonical bytes.

For noisy historical and motorized environmental fixtures, the public APIs encode v5 and
save it as SHA-256-bound v6. Loading authenticates the exact NPZ bytes before parsing. The
directory contains only `manifest.json` and `data.npz`; all environmental configuration and
all 35 artifact arrays survive exactly. Regeneration from the loaded configuration reproduces
all arrays exactly, and resaving reproduces both canonical files byte for byte in the tested
environment. Loaded arrays are owned, C-contiguous, read-only, and independent. The artifact
schema remains exactly 35 arrays, `run_artifact.py` is unchanged, and no actual rotor-speed
trajectory was added.

## Final independent audit

The top-to-bottom audit reviewed the complete thirteen-file implementation/test diff and the
full affected files before any documentation edit. A deterministic probe with seed
`20260922` exercised 4,096 finite velocity, wind, proper-rotation, and nonnegative
anisotropic-coefficient combinations. Maximum relative-air power was exactly `0`; minimum
power was `-31643891.954012159 W`, with a largest applied float64 tolerance of
`4.4968675118201633e-07 W`. The probe also checked input immutability, output ownership,
exact zero relative-air force, and zero-coefficient extreme-value behavior.

The review found no frame/sign defect, cached-stage drag, truth/nominal leakage, schema or
replay incompatibility, unrelated refactor, debug output, unfinished marker, generated
artifact, secret material, dependency change, or public API break. No production correction
or additional test was required by the final audit.

## Verification

All commands used `UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache` where shown.

| Command | Result |
| --- | --- |
| Focused seven-file pytest gate before documentation | 865 passed in 5.08s |
| Full pytest before documentation | 1,362 passed in 7.07s |
| `uv run ruff check .` before documentation | Passed |
| `uv run ruff format --check .` before documentation | 67 files already formatted |
| `uv run mypy src experiments` before documentation | No issues in 18 source files |
| `make check` after documentation | Ruff passed; 69 files formatted; mypy passed; 1,362 tests passed |
| Focused seven-file pytest gate after documentation | 865 passed |
| `git diff --check` and `git diff --cached --check` | Passed |

No failures, errors, skips, xfails, or warnings were reported. The thirteen implementation
and test hashes remained unchanged throughout this documentation-only closeout.

## Limits and next milestone

Wind is constant in space and time. Drag is an effective axis-lumped translational force;
there is no cross-axis aerodynamic coupling, aerodynamic moment, rotor wake, ground effect,
gust/turbulence model, Reynolds-number model, identified density/Cd/area decomposition, CFD,
or blade-element model. The evidence is deterministic simulation and replay evidence in the
tested Python/NumPy environment, not cross-platform bitwise reproducibility, real-world
flight validation, or a safety/certification claim. Closed-loop control, estimation, ROS 2,
PX4, and C++ integration remain outside this gate.

At this point Gate G1 was locally complete and ready for publication. The
publication details are recorded in the closeout below.

The exact next bounded milestone is state-estimation design, beginning with the 15-state
error-state Kalman filter conventions, process/noise model, propagation equations, and
Jacobian verification.

## Publication closeout

The implementation was published in commit
[`32e4886c63b38b62f41aa236b4439ca22560f5b7`](https://github.com/Gayles9/robust-quadrotor/commit/32e4886c63b38b62f41aa236b4439ca22560f5b7)
was created with parent `65a48da9ddd64bd98e4b2faf473c33cd514c3485` and exact subject
`feat: add environmental wind and drag`. It contained exactly the audited 17 paths.

The local publication gate passed 865 focused tests, 1,362 full-suite tests, Ruff,
formatting over 69 Python files, mypy over 18 source files, and `make check`. The normal push
succeeded and synchronized `HEAD` with `origin/main`. GitHub Actions push
[run 35793062927](https://github.com/Gayles9/robust-quadrotor/actions/runs/35793062927)
completed successfully, including `uv sync --locked` and `make check` in the hosted `check`
job.

Gate G1 is now closed and published. The next bounded milestone remains the 15-state
error-state Kalman-filter design: conventions, process and noise model, propagation
equations, and Jacobian verification.
