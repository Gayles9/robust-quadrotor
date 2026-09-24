# 2026-09-24: Causal ESKF Feedback and Paired Mission Evidence

## Scope and preceding-milestone audit

Audited main commit: `23197e95feace859c4d7405c4e2574e06f2c33bb`
(merged position/velocity and mission PR #9). The repository takes precedence over
the older engineering-log checkpoint. The master plan's estimated-feedback
integration follows the now-implemented true-state cascade. Its bounded design,
information flow, timing and acceptance criteria were fixed in
[ADR 0013](../decisions/0013-estimated-state-mission-feedback.md) before implementation.

The source audit covered the inner/outer controller laws, attitude construction,
mission supervisor, motor/RK4 plant, sensor acquisition/delivery, named RNGs,
endpoint propagation and replay, and existing campaign/scoring/persistence tests.
It found no demonstrated defect requiring a changed control law, gain, reference,
plant equation or filter equation. Fresh baseline evidence:

- `.venv/bin/python -m pytest -q -W error`: **2859 passed in 92.39 s**.
- Previous fixed campaign, workers 4: **5/5 pass** under unchanged protocol
  `43f0deece9cbf4f9d7f2157d6f5426b7a5d032df3ee784d1fe6d155e124697fb`.
- After sharing the private mission loop, all five previous fixed/refinement
  histories compare **bit-for-bit**, for every NPZ array; all metrics are identical.
  The previous refinement differences remain 1.337793209544834e-10 m in position
  and 6.96858039838782e-10 degrees in attitude.
- An AST comparison verifies that every statement in the extracted ordered
  observation-correction body is identical to the previous replay body; only
  execution packaging changed.
- `AGENTS.md` now requires this audit-first procedure at every subsequent scoped step.

The implementation scope is a caller-driven online ESKF and a bounded numerical
estimated-feedback mission. No controller/filter mathematics, dependencies, tool
pins, historical manifest/artifact schemas, ROS/PX4 integration or physical
touchdown behavior changed. This is not a new controller or automatic alignment.

## Implemented boundaries and mathematical correspondence

| Component | Implemented responsibility | Principal evidence |
| --- | --- | --- |
| `eskf_online.EskfOnlineEstimator` | Transactional measurement-only epochs, explicit prior, first-order or endpoint continuation | Exact replay parity; first-call/nonuniform-clock checks; independent endpoint predict/update/rate calculation; rollback and ownership tests |
| `eskf_replay._correct_eskf_epoch` | Shared existing ordered correction, gates, stale/disabled rules | Existing replay suite plus both online propagation modes |
| `estimated_mission.MissionSensors` | Copied true distributions, slow schedules and root seed | Invalid type/seed/schedule tests; named-stream reconstruction |
| `estimated_mission.simulate_estimated_mission` | Current truth → actual-force IMU and scheduled slow sensors → ESKF → estimated feedback | Truth/prior separation, causality, force/drag/bias identities, offline replay and controller reconstruction |
| Private shared mission loop | Estimated supervision/commands with separate labeled truth safety oracle | Unchanged true-state histories; guard priority and inter-tick abort tests; no terminal command |
| `EstimatedMissionResult` | Owned full-grid clocks, truth, measurements, states/covariances, rates, noise means and dispositions | Cross-field rejection and independent-memory tests |
| Estimated-feedback campaign and archive | Paired separate closed loops, frozen seeds, complete failures, lossless covariance chunks and exact byte digests | Worker parity, round trips, corrupt schema/metric/history/digest/path/pickle rejection |
| Headless plotting | Validated full-history diagnostics and all-trial comparison | Input identity, no-overwrite and validation-before-render tests |

The [interface guide](../estimated-feedback.md) specifies units, frames and equations.
In particular, the true accelerometer is computed from actual rotor force plus drag
divided by true mass, not commanded thrust or world acceleration. Bias walks advance
once over each completed interval; their densities are not per-sample noise.
Position/velocity/attitude feedback uses the current posterior. Body-rate feedback is
the measured gyro minus posterior gyro bias **and** conditional endpoint gyro-noise
mean. The full 21-coordinate shared-sample covariance remains in continuation memory.

Plant and paired IMU are 400 Hz, inner loop 100 Hz, outer loop 50 Hz, local position
5 Hz and barometer 25 Hz. The explicit prior is at zero; slow acquisition begins at
the first positive sensor period. Delayed observations are stale or pending, never
silently fused as current. Every epoch, including the terminal epoch, is recorded.

## Local verification on 2026-09-24

All commands ran on the audited base plus the uncommitted scoped implementation.
The final execution-source digest is
`58e03e5e55e7ff093131c2effdfdc1d72b325135c84d93608f6f6f0ed4ad39f4`.
It covers ordered names, lengths and bytes of `src/quadrotor_math/*.py`,
`experiments/*.py`, `pyproject.toml` and `uv.lock` under the existing source-hash
definition; documentation-only changes do not change that digest.

| Exact command or check | Observed outcome |
| --- | --- |
| `.venv/bin/python -m pytest -q -W error tests/unit/test_eskf_online.py tests/unit/test_estimated_mission.py tests/unit/test_estimated_feedback_validation.py` | **93 passed in 52.16 s** |
| `make check` with the preexisting global launcher | Refused before checks: global uv 0.12.17 did not match required 0.12.3; no pin relaxed |
| `env PATH=/workspace/scratch/f2c7111463b5/estimated-feedback-tools/bin:$PATH PYTEST_ADDOPTS='-W error' make check` | Exit 0: Ruff lint passes, **148 files formatted**, mypy **46 files**, **2952 tests passed in 150.72 s** |
| Fresh previous fixed campaign and exact NPZ/metric comparison | **5/5 unchanged bit-for-bit** |
| Independent physical/sensor/control reconstruction, fixed/development/validation | **18/18 histories pass**, **288418 full-rate rows**; this verifies consistency, not each performance criterion |

A task-local install of the already-required `uv==0.12.3` resolved the launcher
mismatch. No global tool, dependency declaration, lockfile or project environment
pin was changed. CI continues to use the existing Python 3.12, uv 0.12.3,
`uv sync --locked` and `make check` workflow.

The 93 added tests include two seeded 10-second takeoff/translation/landing
regressions, independent full-grid bias/noise reconstruction, future-reference and
future-measurement causality, independent endpoint sample-noise corrections, atomic
failure after one successful same-epoch correction, frozen-array alias isolation,
off-grid stale/pending deliveries, and estimated-versus-true completion distinctions.

## Frozen fixed and development evidence

Commands (each destination new):

```bash
.venv/bin/python -m experiments.position_control_validation --partition fixed --workers 4 --output /workspace/scratch/f2c7111463b5/evidence/estimated-feedback/baseline-fixed
.venv/bin/python -m experiments.position_control_validation --partition fixed --workers 4 --output /workspace/scratch/f2c7111463b5/evidence/estimated-feedback/compatibility-fixed
.venv/bin/python -m experiments.estimated_feedback_validation --partition development --workers 3 --output /workspace/scratch/f2c7111463b5/evidence/estimated-feedback/development-v1
.venv/bin/python -m experiments.estimated_feedback_validation --partition fixed --workers 3 --output /workspace/scratch/f2c7111463b5/evidence/estimated-feedback/fixed-v1
```

The fixed command exits **1**, correctly reporting **one acceptance failure and
zero numerical failures**. Development exits 0, **3/3 pass**. Every paired
true-feedback mission passes; every estimated-feedback mission completes without
any inner or outer limiting. Completion alone is not the acceptance decision.

| Fixed case | Estimated-feedback true tracking RMSE (m) | Paired true-feedback RMSE (m) | Position estimation RMSE (m) | Acceptance |
| --- | ---: | ---: | ---: | --- |
| Noiseless square | .018762 | .018761 | .000002985 | Pass |
| Noisy 60-second hover, seed 30 | .044578 | .007091 | .015454 | **Fail: hold peak .107563 m > .08 m** |
| Noisy square, seed 31 | .058067 | .021274 | .017092 | Pass |
| Noisy vertical step, seed 32 | .115637 | .103360 | .018632 | Pass; settling 2.445 s |
| Noisy mild wind, seed 33 | .057421 | .032953 | .019476 | Pass |

Noiseless common-grid position and attitude differences are respectively
**1.1650170625498233e-5 m** and **3.2063552374987055e-6 degrees**,
below the frozen .005 m / .05 degree limits. The noiseless case has zero prior,
sample and walk covariance but positive nominal slow-sensor R: it is a deterministic
integration check, not evidence of stochastic calibration.

The hover shortfall peaks at **5.395 s**, during the early hold after takeoff.
Its full-mission tracking RMSE is .044578 m; final true position and speed errors
are .020268 m and .025612 m/s. Within the originally scored 5–65 s hold, 448 of
24001 sampled rows exceed .08 m. A descriptive 10–65 s subwindow has peak error
.051989 m; **that narrower window is not used to rescore or excuse the failure**.
The sensor/command/physical audit passes and no implementation inconsistency was
identified. A single trajectory does not isolate a unique physical cause or prove
a general performance bound. Gains, prior, noise, seeds, windows and thresholds
were not changed after observing the shortfall.

Development square seeds 8100–8102 give true tracking RMSE
**.051717–.059369 m**, paired true-feedback RMSE **.020005–.020991 m**,
position estimation RMSE **.017188–.018562 m**, and peak attitude estimation
error **2.773–3.810 degrees**. These results do not imply noisy estimates should
outperform true-state feedback.

Protocol SHA-256 values:

- Fixed: `dcc035d1589e82bb7f7d3490f38f6765a62015a44ef064bd6fa0a6cdd3d5ca15`
- Development: `d2dffe95e0ec54ed836dd3be6aea183e778318ef390ab388ba22bbaccf5b8b53`
- Validation: `e942a867424a6c4d1578c949286f90f99171ae183655808d276e9cd451e66927`

The prevalidation checkpoint was written at **2026-09-24 13:11:28 UTC** before
executing validation seeds 90000–90009. It retains the exact execution-file hashes,
targets, seed identities, baseline commit, full gate log hash, development/fixed
report hashes and the known hover failure. No held-out result was used to revise
the protocol.

## Held-out validation

```bash
.venv/bin/python -m experiments.estimated_feedback_validation --partition validation --workers 3 --output /workspace/scratch/f2c7111463b5/evidence/estimated-feedback/validation-v1
```

Exit 0: **10 planned, zero numerical failures, zero acceptance failures**.
All ten estimated-feedback and paired true-feedback square missions complete at
39.5 s with zero inner/outer limiting. The execution digest and protocol exactly
match the prevalidation freeze. Trial generation completed at
2026-09-24 13:17:55 UTC; the command returned only after the full-history replay,
controller/phase/metric audit and bundle publication also succeeded.

| Held-out seed | Estimated-feedback true tracking RMSE (m) | Paired true-feedback RMSE (m) | Position estimation RMSE (m) | Peak attitude estimation error (deg) |
| --- | ---: | ---: | ---: | ---: |
| 90000 | .056461 | .022184 | .019649 | 3.071 |
| 90001 | .063686 | .020976 | .018615 | 4.080 |
| 90002 | .053827 | .020908 | .018329 | 2.350 |
| 90003 | .047205 | .019002 | .017742 | 3.410 |
| 90004 | .070885 | .019499 | .019542 | 4.205 |
| 90005 | .056286 | .019909 | .017956 | 3.097 |
| 90006 | .057604 | .023249 | .015459 | 2.987 |
| 90007 | .050961 | .020558 | .020525 | 7.197 |
| 90008 | .055573 | .020335 | .017358 | 3.437 |
| 90009 | .057733 | .020299 | .017310 | 4.944 |

Mean true tracking RMSE is **.057022 m**; the per-trial range is
**.047205–.070885 m**, below the frozen .15 m limit. Position estimation RMSE
is **.015459–.020525 m**, below .10 m; peak attitude estimation error is
**2.350–7.197 degrees**, below 15 degrees. Final true position errors are
**.008726–.043436 m** and speeds **.006755–.028431 m/s**, below their .15-unit
limits. These are per-trial ranges, not confidence intervals or population
guarantees. This **10/10 validation result does not supersede the 4/5 fixed result**.

### Saved-evidence integrity recovery

The first post-generation reload correctly rejected one of 50 validation archives:
`trial-003-data.npz` had 9,347,818 bytes, an invalid ZIP ending and a digest different
from the generation-time report. The storage cause was not established; this was
not treated as a successful artifact load or silently accepted by changing a hash.

The damaged copy was preserved separately. With the execution source and all
configuration unchanged, seed 90003 was reproduced. Every metric and **all five
archive hashes** matched the original report exactly, including the complete
11,894,294-byte data archive. Only the damaged target was restored from those
verified bytes. A fresh check verifies all 50 original hashes and ZIP CRCs; the
original report is unchanged. This is deterministic evidence recovery, **not an
additional statistical trial, seed selection or acceptance revision**. The saved
proofs retain detection, original bytes, reproduction and restoration checks.

## Independent audit and technical limits

The external `audit_histories.py` imports only the authenticated archive reader.
It independently reconstructs rotor geometry/wrench, exact exponential motor
response, quaternion rotation, drag, accelerometer specific force, all six named
random streams, bias increments, every slow observation, the quaternion attitude
and body-rate feedback laws, outer PD acceleration and time-weighted tracking RMSE.
It also checks covariance symmetry/eigenvalues, quaternion norms and physical bounds
over every retained row. Exact algebraic identities are checked at 1e-10 absolute
tolerance in their stated units; smooth central-difference acceleration residuals
are separately bounded at .01 m/s² and .01 rad/s², not confused with exact identities.
Across all 18 histories the largest motor-step residual is 5.46e-12 rad/s,
accelerometer reconstruction residual 7.11e-15 m/s², and rate-law moment residual
5.56e-17 N m. Bias walks, gyro readings, slow-sensor readings, posterior rate feedback
and outer PD acceleration reconstruct exactly. Maximum smooth central-difference
residuals are .001534 m/s² and .008191 rad/s². Every physical covariance is symmetric
and positive semidefinite within the audited tolerance; the exact-zero noiseless
case accounts for the minimum eigenvalue of zero.
The campaign validator additionally reproduces every online state, physical
covariance and event by measurement-only offline replay and verifies all command,
phase, reference and metric correspondence.

All six fixed-case figures were rendered from the validated bundle and visually
inspected at full resolution. Labels, units, NED sign, legends and full time ranges
are readable. The ensemble retains the hover's overall `FAIL` marker even though
its displayed RMSE and limiting metrics individually pass; its failing condition
is the hold peak recorded above. Figure titles saying `completed=True` describe
mission termination, not all-target acceptance.

The held-out ensemble figure was subsequently rendered from the restored,
fully revalidated bundle and visually inspected. It includes all ten planned
seeds, paired tracking RMSE, position estimation RMSE and the zero-limiting result.
Both plot commands below exit 0 after independently reloading and auditing their
complete inputs:

```bash
.venv/bin/python -m experiments.plot_estimated_feedback --input /workspace/scratch/f2c7111463b5/evidence/estimated-feedback/fixed-v1 --output /workspace/scratch/f2c7111463b5/evidence/estimated-feedback/fixed-plots
.venv/bin/python -m experiments.plot_estimated_feedback --input /workspace/scratch/f2c7111463b5/evidence/estimated-feedback/validation-v1 --output /workspace/scratch/f2c7111463b5/evidence/estimated-feedback/validation-plots
```

Generated full-rate evidence and plots stay outside Git. The repository contains
the reproducible code, frozen definitions, tests and technical documentation.

This implementation is a bounded numerical integration, with an explicit modest-
error prior, illustrative matched models and a separately labeled truth safety
oracle. It is not automatic startup alignment, global convergence, a real-time
service, hardware safety, ground contact or fault-robust control. The existing
known-prior NIS/NEES calibration evidence cannot be transferred as a new consistency
claim for this feedback distribution. Descriptive NIS means here are not a new
calibration study.

The next scoped change must first audit this integration and reproduce the noisy
hover shortfall. Any startup-transient or feedback-performance improvement needs
an explicit design rationale, separate development evidence, frozen criteria and
new held-out seeds. The measured miss must not be erased by changing its scoring
window or claiming that every fixed target passed.
