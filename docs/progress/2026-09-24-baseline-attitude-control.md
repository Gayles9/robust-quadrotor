# Baseline Attitude/Rate Control Verification

Date: 2026-09-24 (UTC).

- Audited baseline commit: 8fef0ce6c811b64dc2e2ab4558591111254e5628.
- Scope and frozen criteria: [ADR 0011](../decisions/0011-baseline-attitude-control.md).
- Interfaces, mathematics and reproduction: [control guide](../control.md).
- Source of milestone scope: canonical six-month plan's Week 7 attitude/rate inner loop,
  cross-checked against the current master engineering log and baseline repository.

## Implemented scope

The new pure controller maps a local body-frame quaternion rotation error to a bounded
rate demand, then to a full-inertia proportional rate moment with gyroscopic compensation.
Explicit componentwise moment limits precede a collective-preserving rotor allocation.
The allocator reduces moments along a feasible squared-speed ray and delegates its final
command to the existing strict allocator. It records each demand and limiting decision.

The deterministic true-state harness composes the existing motor, rotor, wind/drag and
body-wrench plant equations. Each projected-RK4 stage evaluates the exact held-command
motor response at that stage's elapsed time. References and collective are held at explicit
controller epochs; body disturbances are held at plant intervals. Immutable histories
include initial/final states, actual motor speeds, commanded speeds and requested/clipped/
allocated/actual moments. Truth and controller nominal parameters remain separate.

This completes the bounded inner-loop implementation and its declared verification
campaign. It does not close G2: position/altitude control and baseline mission validation
remain separate. There is no integral state, estimated-state feedback, flight safety
supervisor, moving-reference feedforward or hardware validation in this increment.

## Mathematical and boundary audit

| Contract | Evidence |
| --- | --- |
| Current-body relative rotation, NED/FRD signs | Independent relative-matrix/Rodrigues comparison, all signed principal axes, both quaternion signs, small angles and half-turn tie |
| Rate control and inertia | Full off-diagonal-inertia gyroscopic cancellation; finite-difference local acceleration coefficients −K_r K_a and −K_r |
| Feasible allocation | Exact identity with the old strict allocator; independent interval optimum; forward wrench and preserved collective/direction; lower/upper anchors; 80 deterministic randomized demands and infeasibility above the ray optimum |
| Limits and numerical domains | Positive gains, local-angle boundary, full SPD/exact symmetry, shape/real dtype/finiteness, scalar Boolean/overflow, singular geometry and infeasible collective rejection |
| Array ownership | Owned C-contiguous read-only arrays, frozen containers, independent nominal/truth storage, unchanged caller input on failure |
| Numerical execution | Old RK4 constant-speed limit; analytical equal-motor spin-up position/velocity; >14× error reduction on each of two step halvings |
| Timing and causality | Initial/final rows, no terminal command, exact motor response over a held command, future references cannot change past output, partial final control interval |
| Physical feedback | All principal-axis recoveries, finite torque-pulse sign/recovery, hover equilibrium, nonzero drag and truth/nominal independence |
| Evidence integrity | Independent settling oracle, worker-count determinism, disjoint seeds, complete ordered trial identities, finite JSON, failure retention, stale metric/summary/clock/protocol rejection, headless plot and overwrite checks |

The pre-held-out audit corrected the persistent diagnostic's event timestamp to its
actual 0.5 s torque onset and clarified requested-versus-actual plot labels. Gains,
physical parameters, acceptance thresholds and held-out seeds were unchanged.
No existing production or test file was edited. A byte comparison against the baseline
checks all 71 pre-existing package/experiment/test/toolchain/CI/instruction files in that
protected set. Existing ESKF, sensor generation, manifest/artifact schemas, dependencies,
tool versions and CI retain their original bytes and pass the full regression suite.
This is automated source review and numerical verification, not an independent human
review or an exhaustive stability proof.

## Fixed physical cases

The illustrative matched model has mass 1 kg; inertia diag(.02,.025,.04) kg m²; X rotors
at (±.15,±.15,0) m in declared order; spins [+1,−1,+1,−1]; k_f=10⁻⁵ and k_m=2×10⁻⁷
in SI coefficient units; speeds 0–900 rad/s; motor time constant .025 s; gravity
9.81 m/s²; calm air and zero drag. Collective is 9.81 N, starting from equal hover
motor speeds. Plant/control rates are 400/100 Hz. Every base case lasts 4 s.

The zero-lag local critical-damping design sets K_a=[3,3,2], K_r=[12,12,8] s⁻¹.
Regular rate limits are [2,2,1.5] rad/s and moment limits [.8,.8,.3] N m.
The saturation case alone uses moment limits [.05,.05,.02] N m. These values were
declared before evaluating the case outputs.

| Case | Final attitude error [deg] | Final rate norm [rad/s] | Settling after final event [s] |
| --- | --- | --- | --- |
| Level hover | 0 | 0 | 0 |
| ±20° roll steps at .5 s | .0000064472 | 4.964×10⁻⁷ | .8900 |
| ±20° pitch steps at .5 s | .0000064463 | 4.963×10⁻⁷ | .8900 |
| ±20° yaw steps at .5 s | .000806746 | 4.328×10⁻⁵ | 1.2050 |
| Coupled 30° recovery, nonzero initial rate | .000246219 | 1.321×10⁻⁵ | 1.3225 |
| [.12,−.1,.06] N m pulse, 1.0–1.2 s | .000793768 | 4.265×10⁻⁵ | .6600 |
| 30° roll reference .5–1.5 s, tight moment limits | .002378454 | .000183112 | 1.2300 |

Signed-step rows report the rounded upper final errors across both signs. Settling
requires simultaneous error ≤1 degree and rate norm ≤.05 rad/s after the last departure
from that band. Step/pulse settling is below the declared 2 s bound. Every nonpersistent
case finishes below .5 degree and .02 rad/s, remains within the local π/2 domain at
recorded epochs and has no moment clipping/allocation scaling during its final second.
The hover case preserves position, velocity, quaternion and rate exactly in this run.

Roll/pitch 10–90% rise times are .5325 s; yaw .8100 s. No sampled positive overshoot
occurs in these signed-step cases. Yaw steps invoke allocation scaling for .04 s;
coupled recovery invokes moment clipping for .03 s and allocation scaling for .11 s.
The deliberate stress invokes moment clipping for 1.06 s and then recovers. These
durations are sums of actual control-interval widths, not counts of plant samples.

The separate constant-torque diagnostic applies +.04 N m about body x from .5 s.
Its predicted equilibrium is .04/(.02×12×3) = 3.183098862 degrees, compared with
3.183097949 degrees measured at 4 s. Final roll-rate magnitude is 7.027×10⁻⁸ rad/s.
This nonzero offset is the expected limitation of a proportional controller. Its
diagnostic acceptance tests the offset prediction; it is not counted as zero-error
persistent-disturbance rejection.

Three paired repeats halve the plant step to .00125 s while retaining the .01 s
control period. Final coarse/fine discrepancies are:

| Case | Attitude difference [deg] | Rate difference [rad/s] |
| --- | --- | --- |
| Coupled recovery | 1.107×10⁻¹¹ | 5.497×10⁻¹³ |
| Torque pulse | 1.189×10⁻¹¹ | 5.034×10⁻¹³ |
| Saturation recovery | 8.620×10⁻¹¹ | 5.925×10⁻¹² |

All are below the .02-degree/.002-rad/s acceptance limits. These small **final**
differences partly reflect decay toward equilibrium; the separate analytical spin-up
test supplies the integration-order evidence. The fixed ledger contains 11 base cases
plus three refinement cases, with zero numerical or acceptance failures.

Translation remains uncontrolled. Four-second roll/pitch steps produce approximately
17.24 m position displacement, the saturation case 12.84 m, and yaw steps about .289 m
from transient motor redistribution. Those histories remain in the evidence. Accurate
attitude tracking under constant collective is not an altitude/position-hold result.

## Frozen validation identity

The executable-source SHA-256 is
b0233770211d339971477c0866d3bf377d3a40109c13981bd31ad88e471d1304.
It covers the ordered names, lengths and bytes of all package/experiment Python files,
pyproject.toml and uv.lock. Documentation and tests are separately reviewed. The
prevalidation freeze was recorded at 2026-09-24T02:02:15.493641+00:00, after the
2,659-test gate. Reports accurately identify the baseline commit and a dirty working
tree; publication verification relates the frozen executable bytes to the published commit.

| Partition | Jobs | Protocol SHA-256 |
| --- | --- | --- |
| Fixed | 14 | 27fcb0ae3eaaf390fc8581078e1867db66934dee5a2f588dbc5ff38b8f016947 |
| Development | 5 | 36ba8654491089b8ede55d96d43f17294e628b552ab6748678d381ff32512cac |
| Held-out | 30 | 49cdd72790ae17d1d9cc63370b08d7f4f9336b5b03737d193642628b7bc1ad5d |

Development seeds 1000–1004 all pass; their largest final attitude error is
.000090774 degree and largest final rate norm 4.869×10⁻⁶ rad/s.
Held-out seeds 60000–60029 draw independent normalized Gaussian rotation axes,
uniform 5–30-degree angles and uniform per-axis initial rates in ±.3 rad/s.
Every seed starts from the declared level reference and matched truth/nominal model.
These are randomized initial-condition tests, not noisy-sensor or mismatch trials.

## Held-out results and complete evidence

All 30 held-out trials satisfy the frozen final error/rate, recorded local-domain and
final-second moment-limit criteria. There are zero numerical failures and zero acceptance
failures. Five trials use rate/moment/allocation limiting at some point; none retains
moment clipping or allocation scaling in its final second.

| Held-out quantity | Mean | Maximum |
| --- | --- | --- |
| Final attitude-error norm [deg] | .0000618732 | .0001678381 |
| Final body-rate norm [rad/s] | 3.31921×10⁻⁶ | 9.00248×10⁻⁶ |
| Settling from initial recovery [s] | .93508 | 1.195 |
| Peak attitude-error norm [deg] | 17.1493 | 29.6238 |
| Final uncontrolled position displacement [m] | 3.08178 | 6.34199 |

The very small final attitude errors arise in deterministic matched-model true-state
recovery with no sensor noise. They do not quantify real-flight accuracy.

The complete held-out JSON was reproduced with identical frozen source and seeds and
verified before compression at 2026-09-24T02:05:40.151940+00:00. Its 26,265,231 raw
bytes have SHA-256
5e8b7cc899e5d2daea1cedcdfbd250fd76a3f1be689a3ac9f75dfe41c947a202.
An earlier raw export was truncated and rejected by the reader; it is not used as
accepted evidence. The compressed replacement was decompressed in a separate read,
checked for exact byte count/digest, and fully revalidated, including all 30 identities
and each trial's 1,601 state rows and 400 control rows. There was no retuning or
selection of seeds between executions.

Fixed-report SHA-256:
d18f3e5db3cf301e0d6437ba390822f52c750d32f9da1c6aa4ffc52ff2126b43.
Development-report SHA-256:
cb0c5eef99e31086d771a1eb517c937621fd768b6b50833ec224667cfe0ba698.
Both pass complete report validation and have the same frozen executable-source hash.
All four fixed-case figures and the held-out summary figure were rendered and visually
inspected for axis units, signs, labels, limits and readable layout. Generated data and
figures remain outside Git; the CLI and protocol reproduce them from the published source.

## Local verification

All commands below ran on 2026-09-24 against the working tree based on the baseline
commit above. Environment: Python 3.12.14, NumPy 2.5.2, pytest 9.1.1 and the existing
uv 0.12.3 locked toolchain. ATTITUDE_EVIDENCE denotes an output directory outside Git.

| Command | Outcome |
| --- | --- |
| Baseline .venv/bin/pytest -W error | 2517 passed in 68.71 s |
| uv tool run --from uv==0.12.3 uv run make check | Ruff/formatting pass; mypy 36 source files; 2659 tests passed in 89.95 s |
| .venv/bin/pytest -W error | 2659 passed in 87.23 s; no warnings, failures or skips |
| .venv/bin/python -m experiments.attitude_control_validation --partition fixed --workers 4 --output "$ATTITUDE_EVIDENCE/fixed.json" | 14/14 cases pass, including three paired refinements |
| Same CLI, --partition development --workers 2 | 5/5 development cases pass |
| Same CLI, --partition validation --workers 4 | 30/30 held-out cases pass; complete compressed evidence revalidated |
| validate_report on fixed, development and decompressed held-out JSON | Exact ledgers, clocks, metrics and summaries pass |
| experiments.plot_attitude_control and render_report | Four fixed figures and one held-out figure; all visually inspected |
| git diff --check and protected-file byte comparison | Pass; all 71 protected pre-existing files unchanged |

The 142 added tests cover the new controller, execution and evidence tooling. Full
regression testing also exercises every existing subsystem without altering its tests.
The source hash is checked before and after each campaign; held-out outcomes were not
used to tune gains, physical parameters or thresholds.

## Remaining technical boundary

The result supports the declared local, matched-model, true-state inner-loop baseline.
It does not establish global attitude stability, arbitrary sampling stability, hardware
performance, noisy-estimate feedback, persistent-disturbance zero error or mission
robustness. Next, the master plan's Week 8 position/velocity outer loop and baseline
hover/waypoint mission validation can use this inner loop. Their G2 acceptance criteria
must be scoped and verified separately.
