# Project audit and bounded geometric-controller study — 2026-09-26

## Audited source and scope

Started from PR #15 at `e7783dfffae221c7f463eec5e34ad2a3f8a09eca`.
During this session GitHub reported it merged as
`b2f107545c80328cdd75b063f0aaa1e21a4db9f8`. Both use tree
`da4441495d55121c5bf0a237855e072560db17a5`; the source under audit did not
change. New work uses `codex/geometric-audit-2026-09-26` based on that merge.
The earlier missing-source blocker is resolved by the rebuilt implementation.

Audited the source contracts, mathematical/controller equations, estimator
correction boundary, mission timing/guards, experiment acceptance, persistence,
documentation and repository/CI state. This is a source and regression audit with
targeted flight evidence, not exhaustive proof of every possible operating state.
No dependency, tool version, plant, estimator equation, sensor distribution,
prior, scoring limit or cascade default changed.

## Completed project capabilities

| Area | What is implemented and verified | Boundary |
| --- | --- | --- |
| Mathematical plant | NED/FRD six-degree-of-freedom rigid-body equations, Hamilton quaternions, rotor force/moment allocation, motor lag, wind and anisotropic drag, Euler/projected RK4 | Illustrative parameters; no identified hardware or contact model |
| Sensors and records | Noisy IMU, position and altitude, bias random walks, scheduled acquisition/delivery, independent named RNG streams, truth/nominal mismatch, versioned authenticated artifacts and replay | Supported schedules and numerical-backend contracts remain explicit |
| State estimation | 15-state ESKF prediction, measurement correction, attitude reset, covariance propagation, innovation gates, endpoint/sample-memory model, online execution and offline replay | Explicit prior; no automatic alignment; stale observations rejected; hover heading remains weakly observable |
| Baseline flight | Position/velocity and attitude/rate cascade; supervised virtual takeoff, tracking, landing, completion and abort; true and estimated feedback | Existing v1 29/30 and v2 28/30 fresh hover results retain their failures |
| Planning | Fixed-duration minimum-snap splines, C3 joins, analytic derivatives through snap, whole-curve nominal reference bounds, bounded uniform retiming and mission composition | No obstacle planning, time optimality or full rotor/motor feasibility proof |
| Advanced control | Geometric moments, analytic force-to-attitude derivatives, causal feedback derivative filter, true/ESKF integration and explicit estimator-correction rebasing | Experimental; flight qualification must be evaluated separately from software tests |
| Integration evidence | Earlier ROS/PX4/Gazebo compatibility checks and an x500 takeoff/hover/landing spike | Custom Python estimator/controller integration, ROS 2/C++ adapters and deployed missions remain future work |
| Fault handling | Measurement outlier rejection and bounded mission guards | Persistent health monitoring, diagnosis and degraded flight policy remain future work |

The frozen formal report v1 covers its recorded earlier commit. Updating the
final report and broad integrated evaluation remain separate deliverables.

## How the project works

A planner produces desired world position, velocity, acceleration, jerk and
snap. At 50 Hz the position loop uses estimated position/velocity to request
lift and a desired attitude. At 100 Hz the attitude loop requests thrust and
body moments. The allocator converts those into four rotor-speed commands;
motor lag and the rigid-body plant evolve on a 400 Hz grid. Simulated IMU,
position and barometer measurements feed the ESKF, closing the loop. Defaults
for these campaign fixtures are 400 Hz IMU, 5 Hz position and 25 Hz altitude.

The geometric path builds desired rotation, angular velocity and angular
acceleration from the lift vector. The new optional path separates discrete
ESKF estimate revisions from force derivatives before this construction.
Only sensor-derived posterior state, gyro estimates and accepted correction
events reach control. Simulator truth supplies sensors, evaluation and a
separately labelled safety guard. Mission supervision checks progress and
aborts on declared domain violations without issuing a terminal command.

All core mathematics remains independent of ROS/PX4. Frames are NED world,
FRD body, active body-to-world R_WB and scalar-first Hamilton q_WB.

## Diagnosed weaknesses and changes

1. The original filter treats posterior position/velocity revisions as force
   changes to differentiate. A measurement-only moment reconstruction matches
   saved commands to 1.12e-16 N m. Offline rebasing reduces the desired angular
   acceleration torque RMS from 0.016614 to 0.002702 N m on nominal spline seed
   30, and from 0.012313 to 0.001839 on hover seed 93012. These are diagnostic
   reconstructions, not closed-loop results.
2. Add immutable `FeedbackDerivativeFilter.rebase(delta_c)`, translating previous
   input and all three section states. Here `delta_c=m*(Kp*delta_p+Kv*delta_v)`
   uses accepted ESKF correction events accumulated between outer ticks. Raw PD
   force still uses the full posterior; physical variations retain the original
   30 rad/s filter. Indices, reset, ownership and finite arithmetic stay explicit.
3. Add `rebase_estimator_corrections=False` to geometric parameters. The original
   controller remains reproducible; this behavior is selected explicitly.
4. Rebasing reduces noise-driven effort but exposes slow attitude feedback and
   worsens startup hover if gains are unchanged. The bounded study tests three
   translational bandwidths and two additional critically damped roll designs,
   with all failed results retained. ADR 0018 freezes the reasoning and limits.
5. The evidence audit found empty covariance chunks after original successful
   hashing. Exact measurement-only replay reproduces saved posterior states and
   restores original chunk hashes. The truncation cause is not established.
   Campaign qualification now also requires explicit saved-payload verification.
6. Test measured physical derivatives using the existing specific-force sample,
   posterior attitude/bias and conditional IMU-noise mean. Its first derivative
   supplies jerk; this avoids twice differentiating posterior corrections.
   The option is mutually exclusive with rebasing and requires estimated
   feedback. It also fails development, so it is retained only as an explicit
   experimental path under ADR 0019.

The raw force and filtered derivatives still form approximate reference jets.
The filter has about 0.1 s low-frequency delay. Rebasing does not remove wind
bias, estimator bias or the need for suitable feedback bandwidth. The ideal
continuous-time geometric identity is a mathematical test oracle; it is not a
proof for this sampled, noisy, motor-lag system.

## Verification and decision

The preceding focused audit passed 154 tests in 139.56 s. Two new rebase tests
first failed because the method did not exist, then passed after implementation.
An invalid sensor-schedule field in a new test and an array-comparison mistake
in another new test were corrected. A strict typing issue in the experiment
report was corrected before the final gate. The final focused geometric gate
passes 46 tests in 10.75 s, including off-outer-grid correction accumulation,
measurement-only command reconstruction, offline replay, unchanged fixtures,
failure ledger completeness and damaged-evidence rejection.

The first complete gate passed **3,304 tests in 506.24 s**, Ruff lint/format
(204 files) and strict mypy (61 source/experiment files), with warnings as errors.
The measured-derivative tests first failed eight cases because the new interfaces
were absent, then passed. Its final nine focused tests include direct inspection
of zero world acceleration in noiseless NED hover; moment-only hover checks can
miss vertical acceleration mistakes because force normalization removes scale.

## Bounded development results

All spline rows below use nominal seed 30 and the same fixed cascade comparator:
7.4059 cm RMSE and 0.001176960 N² m² s actual squared-moment effort.
Frequency is the declared horizontal natural frequency; Kv=1.8*frequency and
Kp=frequency². The original row is reconstruction evidence; its spline and
seed-93012 hover were reproduced in this audit. The 30-seed original hover value
comes from the preceding published record. Every other row is new execution.

| Derivative design | Frequency [rad/s] | kR [N m] | Spline RMSE [cm] | Effort / cascade | Hover 30 peak [cm] | Hover 93012 peak [cm] |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Original filtered force | 1.0 | 0.64 | 6.0517 | 10.860 | 8.7649 | 13.6855 |
| Rebased | 1.0 | 0.64 | 8.3764 | 1.136 | 11.7185 | 17.1268 |
| Rebased | 1.5 | 0.64 | 5.8496 | 2.513 | 10.0064 | 16.3063 |
| Rebased | 2.0 | 0.64 | 4.6956 | 5.213 | 8.8138 | 15.5239 |
| Rebased, critical roll model | 1.0 | 1.28 | 6.4334 | 3.897 | 10.0079 | 15.5003 |
| Rebased, critical roll model | 1.5 | 1.28 | 4.6316 | 8.163 | 8.7919 | 15.0401 |
| Measured physical derivatives | 1.0 | 0.64 | 8.5360 | 1.824 | 11.9179 | 17.4527 |

The required hover maximum is **8 cm over every sample from 5 to 65 s**.
The required spline effort ratio is **<=2**, with no RMSE regression against
cascade. No new profile passes all three requirements. Rebasing at frequency 2
improves this observed spline's RMSE by 22.4% and effort by 52.0% relative to the
original geometric design, but its hover is worse and effort still exceeds the
allowed comparison. That limited improvement is not an accepted replacement.

**Decision: reject all six candidates for promotion; preserve both defaults.**
Further selection on the same observed cases would provide weak evidence for
general improvement. The planned 40-case validation is stopped at development
disqualification; none of seeds 95000..95003 or 96000..96003 was opened.
This retains the original thresholds and avoids claiming a fundamental optimum.
The next technical proposal must justify a joint startup/estimator/controller
design and clear known failures before using those fresh validation cases.

## Final regression and publication

The final warning-strict `make check` passed **3,315 tests in 461.77 s**,
Ruff lint/format (207 files), and strict mypy (61 source/experiment files).
The strengthened measured-acceleration suite separately passed nine tests in
0.61 s. All 213 inspected relative documentation links resolve.

The fresh true-state regression ran seven cascade/geometric pairs plus one exact
geometric repeat. All fifteen complete without limiting or rotor-bound contact.
Every geometric/cascade RMSE comparison passes; the original first-five mean
ratio is **0.32737115046888376** against the unchanged 0.80 bound.

| Geometric true-state case | RMSE [cm] | Peak [cm] |
| --- | ---: | ---: |
| nominal | 0.202813 | 0.595374 |
| refined plant step | 0.202813 | 0.595374 |
| positive offset | 1.017507 | 3.297564 |
| negative offset | 1.012433 | 3.287579 |
| mild wind | 3.569739 | 7.506940 |
| reversed wind | 3.665280 | 7.643064 |
| wind plus offset | 3.749173 | 7.506926 |

Halving the plant step while retaining controller periods changes position by
less than 1.89e-10 m and attitude by at most 2.415e-6 degrees, within the original
5 mm/0.05 degree limits. The repeat matches every saved mission array exactly.
Execution source SHA-256 stayed
`cc4f982a40f90bc775c03703aa92007549a21f4509e296eb83743d63ef5aa9fb`.

Across the entire session, **42 flights were executed**, including baseline
reproductions, repeated comparators and the final true-state regression. All
complete without limiting; these are not 42 distinct held-out trials. The
independent verifier authenticates **202 payload files** and recomputes all
42 flight scores directly from saved arrays, without importing the controller
or production scorer.

Seven payloads were damaged after their original digests had been recorded:
six were empty and one retained bytes that did not match its expected hash.
Four were restored by sensor-only ESKF replay, matching every posterior state,
event disposition and original payload SHA-256. Three were restored from
independent identical-fixture payloads with the exact expected digest. No
measurement, metric, expected digest, failure or threshold was altered. The
truncation cause remains unestablished; the recovered evidence passes final
authentication and the new runner fails closed on missing/damaged payloads.

Source checkpoint `0f5439dc902937e6633a40cea47b9cdf54453aff` passed hosted
CI run `36253174020`. The completed work is published through
[draft PR #16](https://github.com/Gayles9/robust-quadrotor/pull/16), based on
merged main `b2f107545c80328cdd75b063f0aaa1e21a4db9f8`. Exact final commit,
hosted-CI status, archived evidence and session closeout are recorded in the
canonical engineering log. No unqualified controller is selected as the default.

Reproduction uses the unchanged lockfile, Python 3.12.14 and uv 0.12.3:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.geometric_correction_validation \
  --stage development --frequency 1 --stiffness 1.28 --output NEW_DIR --workers 3
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.geometric_correction_validation \
  --stage development --frequency 1 --strategy measured --output OTHER_NEW_DIR --workers 3
```

Development commands exit nonzero because their performance gates fail. The
retained scripts and protocols reproduce the remaining profiles, original
diagnosis, complete true-state regression and independent evidence audit.
