# ESKF Measurement-Only Sensor Replay

- Date: 2026-09-23
- Status: Implemented, audited, locally verified, published, and hosted code CI verified
- Baseline: [`1013f3de53cfea244d8ef154171b77f99a140a72`](https://github.com/Gayles9/robust-quadrotor/commit/1013f3de53cfea244d8ef154171b77f99a140a72)
- Code commit: [`2e26a26d2b4867e0a905e94fad68cb1385955b84`](https://github.com/Gayles9/robust-quadrotor/commit/2e26a26d2b4867e0a905e94fad68cb1385955b84)
- Tested/published code tree: `c324b95aef4e68a2ec745a2beae6744d1cb434a0`
- Review: [pull request #2](https://github.com/Gayles9/robust-quadrotor/pull/2)
- Hosted code CI: [run 35884309624](https://github.com/Gayles9/robust-quadrotor/actions/runs/35884309624), successful
- Contracts: [ADR 0006](../decisions/0006-eskf-sensor-replay.md)

## Completed scope and plan

The next execution gap after the prediction and measurement-update cores was the absence
of a defined filter epoch, IMU interval semantics, observation order and stale-data policy.
The baseline README and preceding measurement-update closeout identify this decision.
The implemented increment completes that bounded milestone:

1. Verify repository HEAD and all 82 tracked blobs; run the 1,675-test baseline.
2. Define measurement-only inputs, independent prior/configuration, supported recorded-run
   timing and an explicit non-rewinding stale-observation policy before implementation.
3. Add tests before the new interfaces, then implement the runner and adapters without
   modifying existing filter equations, plant, sensor generation or persistence.
4. Add analytic, temporal, ownership, numerical, corrupt-metadata and end-to-end regressions.
5. Audit the completed implementation and compatibility, run all checks and warnings-as-errors,
   publish the exact tested tree, and review the remote diff and hosted CI.
6. Complete README, architectural decisions and this verification record after code CI passed.

The two production modules are `eskf_replay.py` and `eskf_run_replay.py`. Their two test
files add 246 cases. No earlier tracked source or test file was changed. All 82 baseline
blobs remained identical through code publication; only the subsequent documentation
commit updates the applicable technical records. No dependency, tool version, CI workflow,
manifest version or artifact schema changed. No generated result or large log is committed.

## Execution and data-access review

The prior is explicitly supplied at the first available paired IMU sample. For generated
runs this is `dt`, not zero. Prediction to epoch `i` uses only sample `i-1` and the actual
elapsed interval; no terminal extrapolation is added. After prediction, fresh position and
then altitude are corrected through the unchanged public ESKF primitives. Post-update
histories and per-correction diagnostics have distinct, documented meanings.

The mathematical runner has no run artifact, truth configuration, sensor generator or RNG
interface. The artifact adapter reads only clock, sensor values and timing/delivery
metadata. A read-trap integration fixture fails if true position, velocity, quaternion,
angular velocity, IMU bias history or rotor command is accessed. It succeeds through the
complete adapter/runner composition. Nominal configuration supplies only model assumptions;
the caller's initial state, IMU bias estimates, prior covariance and continuous `Q_c` are
never replaced by truth or silently inferred.

Only full-rate paired zero-delay IMU is accepted by the recorded-run adapter. Position and
altitude may arrive later, but older acquisitions are explicitly `STALE` and skipped.
Pending records never fuse before delivery. Fresh disabled streams have a separate status
for same-input dead-reckoning comparison. Out-of-grid acquisition, missing/asynchronous IMU,
noncausal delivery, source-ID conflicts and inconsistent global delivery tables are errors,
not silently repaired data. The adapter checks arrival eligibility with the scheduler's
exact scale-aware tolerance and requires adjacent clock rows to remain distinguishable.

## Verification evidence

| Area | Independently checked behavior |
| --- | --- |
| Initial epoch | Single-row altitude example corrects NED down by -1 m (within solve roundoff) and reduces its variance from 1 to 0.5 m², without an initial prediction |
| IMU endpoint | Distinct accelerations `[1,2,3,999]` on successive rows yield x velocities `[0,0.1,0.3,0.6]` m/s; the last sample cannot leak into the preceding interval |
| Composition | Every state and covariance matches an explicit sequence of existing prediction, position update and altitude update calls |
| Frames and bias signs | Independent Rodrigues rotations for an arbitrary fixed body-rate axis and nonzero IMU biases recover analytical constant-world-acceleration position, velocity and attitude |
| Timing and causality | First/final/single/nonuniform epochs; shuffled input canonicalization; future-value changes leave prior history unchanged; no pending or stale measurement is fused |
| Dropout and recovery | Direct measurement-only input with a gap propagates growing vertical uncertainty and reduces it again on fresh position updates; this is not a new sensor dropout generator |
| Covariance and failures | Invalid shapes, nonfinite/complex data, local asymmetry, indefiniteness, negative subnormal variance, zero-diagonal coupling, singular innovation and finite arithmetic overflow reject explicitly |
| Failure atomicity | A later failure after a successful update returns no partial result and leaves input arrays and prior unchanged |
| Ownership | Frozen/slotted containers and independent read-only storage, including separation of caller, event and history arrays |
| Recorded delivery metadata | Wrong counts, shapes, integer types, source IDs, off-grid/noncausal times, false pending flags and global delivery inconsistencies reject; delay-boundary cases match the scheduler |
| Saved replay | Generate, save, authenticated load, adapt and estimate: tested state, covariance, innovation and gain bytes match the in-memory path exactly |
| Truth isolation | An attribute-read trap prohibits all true kinematics, bias histories and actuator commands while replay still succeeds |

Two independent noise-free generated scenarios use stationary and constant-velocity motion
for 101 truth steps at 0.01 s. Explicit experiment priors at the first sensor epoch, not
copied truth-history rows, produce position and velocity matching analytic references
within `3e-13` in their SI units.

A fixed regression ensemble uses seeds `0..11` in both stationary and constant-velocity
scenarios, with 151 truth steps at 0.01 s: 1.51 s of generated truth and 1.50 s of estimator
propagation after initialization. Each of the 24 cases uses the same measurements and prior
for fused and disabled-fusion replay. Each passes position RMSE below 50% of its corresponding
dead-reckoning RMSE. Each covariance history is finite and exactly symmetric, its eigenvalues
have no negative value below `-1e-12`, and quaternion norms stay within `2e-15` of one.
These bounds were specified as regression assertions before the ensemble run. They are
not fitted confidence intervals or a held-out consistency result. Their large initial
position error and short scenario duration are relevant to interpreting the RMSE ratio.

The vertical-bias fixture generates 801 truth steps at 0.01 s, yielding an eight-second
estimator span. It isolates a 0.15 m/s² true vertical accelerometer bias with zero initial
bias estimate and a vertical position/velocity prior error. The final fused vertical bias
error is below 0.01 m/s² and absolute down-position error below 0.03 m; the same-input
dead-reckoning down-position error exceeds 6 m. These are verified test inequalities,
not an assertion of general multi-axis bias observability. In particular, this fixture
does not establish calibrated uncertainty or a valid `Q_c` choice for all sensor models.

## Numerical audit and compatibility

Review of the new nominal adapter found that a positive standard deviation as small as
`1e-200` could square to zero in float64. Two regressions first failed, then passed after
adding explicit rejection of an unrepresentable positive variance. Overflow is also
rejected. The adapter does not silently turn a noisy measurement into an exact constraint,
add variance floors or change the existing PSD/innovation validation.

The rest of the review confirmed that left-held interval arithmetic, bias signs, covariance
ordering/reset, clock-index conversion and result ownership compose the established
contracts without changing their equations. The new result is deliberately in memory;
the existing manifest and 35-array artifact retain their previous meanings.

Remote review confirmed exactly four added code/test files and zero changes to all existing
tracked files at code publication. Every new remote blob and the complete Git tree matched
the locally tested content. Passing the retained 1,675 tests supplements that hash-based
compatibility check; it is not a universal proof against every possible regression.

## Exact verification record

The following commands ran on 2026-09-23 against the content published as
`2e26a26d2b4867e0a905e94fad68cb1385955b84`, using Python 3.12.14 and uv 0.12.3.

| Command | Verified outcome |
| --- | --- |
| `uvx --from uv==0.12.3 uv run make check` | **1,921 passed**; Ruff passed; 80 files already formatted at code publication; mypy passed over 21 source files |
| `uvx --from uv==0.12.3 uv run pytest -W error -q` | **1,921 passed**, no warnings |
| `uvx --from uv==0.12.3 uv run pytest --collect-only -q tests/unit/test_eskf_replay.py tests/unit/test_eskf_run_replay.py` | 246 cases: 117 core and 129 adapter/integration |
| `GIT_INDEX_FILE=/workspace/scratch/f2c7111463b5/evidence/estimator-replay/code.index git diff --cached --check 2a42e3d522363daccc81d7b9bbd623240d688548` | Passed against the verified baseline tree using an isolated review index |
| Hosted `uv sync --locked`, then `make check`, run 35884309624 | Successful; **1,921 passed**, Ruff/formatting/mypy clean |

Normal repository usage remains `uv sync --locked` and `uv run make check` with the pinned
uv version. The explicit launcher and isolated review index above describe this audit
environment, not a tooling or repository-workflow change. Test durations depend on the host.

Documentation closeout repeated `uvx --from uv==0.12.3 uv run make check`: 1,921 tests passed,
Ruff passed, 82 files were formatted and mypy passed over 21 source files. Ruff also checks
Markdown code blocks, so its file count is not a Python-source count. Both README ESKF
examples executed successfully; changed Markdown fences and relative links were checked.
The four published code/test blob hashes remained unchanged throughout documentation.

## Remaining scope and next bounded decision

This increment completes replay on its stated supported input domain. It does not close
the master plan's full estimation gate or its 100-seed nominal target. Broader motion/bias
excitation, held-out NIS/NEES consistency, observation outlier/dropout generation, estimator
result persistence and closed-loop performance remain unverified or unimplemented.

The next estimator decision is a pre-update innovation-diagnostics and outlier-gating
boundary with explicit thresholds and rejection records, chosen before held-out evaluation.
It must distinguish statistical rejection from numerical/model failure and preserve the
accepted epoch policy. A broader consistency campaign needs its own frozen configuration,
seed partition and reporting contract. Gate G2 baseline control remains open and must not
be bypassed when making a later closed-loop claim. No ROS 2, PX4 or controller work is
included here.
