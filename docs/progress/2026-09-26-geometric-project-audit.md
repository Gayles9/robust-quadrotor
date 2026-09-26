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

Final full-gate, development, qualification and publication results are appended
at session closeout. No performance promotion is implied by this interim record.
