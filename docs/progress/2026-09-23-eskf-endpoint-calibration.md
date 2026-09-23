# ESKF Endpoint Propagation and Calibration Verification

Date: 2026-09-23 (UTC).

- Audited baseline: `bf767d913967b7fa1a9ac11c32413658bf94fa1c`.
- Design and frozen evaluation: [ADR 0010](../decisions/0010-eskf-endpoint-propagation.md).
- Interfaces and mathematics: [estimator guide](../estimation.md).
- Original evidence: [completion record](2026-09-23-eskf-completion.md).

## Implemented correction

The original left-held nominal propagation produced a deterministic time-discretization
error on continuously changing force and attitude. A higher-order covariance alone cannot
remove that error. `eskf_endpoint.py` now integrates the two instantaneous IMU endpoints
with second-order smooth-motion accuracy, and differentiates that same discrete map to
propagate uncertainty. The prediction retains 15 physical errors plus six temporary
current-sample noise coordinates. Full Joseph conditioning updates the sample's conditional
mean and cross-covariance; the right-local attitude reset transports all cross terms.

This is an explicit option: `EskfReplayConfiguration.sampled_imu_noise` selects endpoint
propagation and requires zero in the unused continuous-covariance field. Its sample
covariance and bias-increment spectral density have separate units. The default original
method, original protocol hashes, raw input records and historical results remain intact.
The nominal run adapter supports the same option without inferring noise from truth.
No dependency, sensor model, plant dynamics, controller, artifact schema or CI change is made.

## Diagnostic isolation before held-out execution

All ablations use development seeds 4000–4004, identical measurements and priors within
each seed, original noise levels and unchanged gates. Each duration is 30 seconds at
100 Hz. The middle variant substitutes endpoint nominal propagation but retains old
covariance and discards sample memory. It is a diagnostic ablation, not a supported mode.

| Propagation | Noisy mean NEES | Noisy descriptive central95 coverage | Noiseless position RMSE [m] | Noiseless normalized discrepancy |
| --- | --- | --- | --- | --- |
| Original first order | 18.346906 | 90.0633% | .010675976 | 2.31377073 |
| Endpoint nominal only, old covariance | 15.846144 | 94.8684% | .0000156309 | .00000454899 |
| Matched endpoint and sample memory | 15.900795 | 94.7018% | .0000156826 | .00000459465 |

The deterministic probe has exact initialization and noise-free measurements but keeps
positive estimator P/Q/R. Its normalized discrepancy is not a chi-square calibration
experiment. The reduction isolates numerical integration as the principal contribution
in this probe. Nominal-only and matched variants have similar ensemble statistics at
100 Hz, but only the matched method satisfies the exact discrete sample-correlation
contract. The ablation is not a reason to discard those cross terms.

Independent tests show approximately fourfold reduction in position, velocity and
attitude error when a smooth-motion integration step is halved, consistent with second
order. The noiseless 200 Hz replay additionally reduces endpoint discrepancy. Neither
five diagnostic seeds nor repeated time samples establish population calibration.

The reproducible probe composes `make_eskf_synthetic_case(seed, number_of_steps=round(30/dt),
time_step_s=dt, stochastic=...)`, `endpoint_case` when selected, `replay_eskf` and
`evaluate_eskf_replay`. Compute each trajectory's position RMS vector-error norm and
mean NEES, then average the five seed summaries. The ablation composes the endpoint
nominal map with original `predict_eskf` covariance on the same left sample and zero
sample/state cross-covariance; it is isolated in the evidence archive's `probe.py`.
The complete probe script, records and source identity are retained in that archive.

## Mathematical and interface evidence

| Contract | Independent evidence |
| --- | --- |
| NED gravity, stationary motion and signed quaternion | Analytic zero-translation and constant-yaw solutions |
| Translation quadrature | Exact constant and linearly varying world acceleration |
| Discrete error/noise derivatives | Central differences of all 21 prior and 12 independent-driver columns at nontrivial attitude, bias, noise mean and changing inputs |
| Temporal sample correlation | Exact variance `h² sigma² (N-1/2)` across N trapezoidal velocity increments |
| Conditional sample mean and covariance | Multiple prediction/correction cycles compared with dense batch Gaussian conditioning over all independent samples |
| Attitude reset and cross terms | Joint posterior compared with independent Gaussian conditioning and block reset |
| Correlated PSD noise | Seeded 3,000-realization nonlinear local-moment test; entrywise covariance and mean standard-error checks |
| Local numerical order | Independent analytic motion; step refinement of position, velocity and attitude |
| Timing and observation policy | Future samples cannot change past output; final endpoint consumed; stale/pending/disabled/rejected events cannot condition sample memory |
| Storage and failures | Independent read-only arrays, shape/type/finite/PSD rejection, overflow, singular innovation and atomic failure |
| Recorded-run integration | Bit-identical generated versus saved/loaded replay in both modes; instrumented rejection of truth/RNG access |
| Experiment integrity | Worker determinism, unchanged original protocol hashes, fresh disjoint seeds, exact paired baseline, duplicate/missing/stale-summary rejection and headless plots |

The source audit checked all new production functions, every modified replay branch and
the adjacent correction/diagnostic contracts. Tests compare derivatives, analytical
solutions, batch conditioning and independent samples rather than only reproducing the
implementation's algebra. This is automated source review and numerical evidence, not
an independent human review or exhaustive proof.

## Frozen protocol and provenance

The version-2 evaluation retains the original trajectory/prior/noise/fault distributions.
It adds an original-method comparison using the exact same measured arrays and prior.
The 100 held-out seeds 50000–50099 are separate from original validation seeds 40000–40099,
development seeds 5000–5004, diagnostic seeds 4000–4004 and smoke seeds 20–21.
There are 380 planned replay variants. Every failure or finite divergence is retained.

| Partition | Replays | Protocol SHA-256 |
| --- | --- | --- |
| Smoke | 18 | `6ab9abde2adc5b9b21ff664b5134225f6cf24cc5f8656d7e8688b33dee4e151b` |
| Development | 75 | `56b270af8eac8681b7a8d0f8591718e592c8a76f7954df598fdfa2613c5f57f5` |
| Held-out | 380 | `f08c1b597c860095158fda068cf8d599dcdbeffdd35323d10cf3af25c972bbb6` |

The source/configuration SHA-256 fixed before held-out evaluation is
`d772e3d4a81dd1dd887f346178938038e85f97831f790328c966bd2a4e714888`.
The freeze was recorded at `2026-09-23T21:48:56.188864+00:00`, after the complete
2,517-test gate. The CLI checks source bytes before and after execution. Pre-commit
reports accurately record the baseline Git commit and `git_worktree_clean=False`;
publication verification relates those exact executable bytes to the published commit.
Documentation is outside the executable-source digest.

The complete development campaign finished at `2026-09-23T21:46:55.972102+00:00` with
zero numerical failure or nominal/gated divergence. Excited-motion endpoint mean NEES
was 16.661759 and descriptive coverage 94.5685%, versus 18.398705 and 87.5308% for
paired first order. Endpoint mean position RMSE was .0671241 m, versus .0675064 m.
Stationary and translating/yaw endpoint coverage was 95.9280% and 95.9480%, respectively.
The development bias/fault/recovery targets passed. No physical noise, prior or gate
parameter was adjusted from these results.

## Fresh held-out outcome

The campaign completed at `2026-09-23T22:01:33.685294+00:00`. All 100 planned trial
identities and 380 variants are present. Numerical failures: **0/380**; nominal
divergence: **0/100**; gated-fault divergence: **0/20**. The paired original-method
and ungated-fault variants also had no divergence. All 100 dead-reckoning cases exceed
the predeclared broad divergence threshold and remain included in the evidence.

| Statistic | Paired first order | Endpoint |
| --- | --- | --- |
| Mean full 15-state NEES (ideal reference 15) | 16.9643146 | **14.6030255** |
| Individual central95 NEES coverage | 91.994335% | **95.289570%** |
| Fraction of epoch means in pointwise reference band | 14.561813% | 90.169943% |
| Mean position RMSE [m] | .068086951 | **.067387623** |
| Mean velocity RMSE [m/s] | .091025475 | .089503485 |
| Mean attitude RMSE [rad] | .025158257 | .024432713 |
| Position NIS mean / central95 coverage | 3.00489 / 95.0266% | 2.99428 / 95.0598% |
| Altitude NIS mean / central95 coverage | 1.01200 / 94.7285% | 1.01059 / 94.8344% |

Endpoint position RMSE is 98.9729% of the paired original value, satisfying the frozen
maximum ratio 1.05. Mean final/initial bias-error norm ratios are .0938466 for the
accelerometer and .0402148 for the gyro, both below .5. Initial→final means are
.1333020→.0125099 m/s² and .0124539→.000500832 rad/s. Mean endpoint dead-reckoning
position RMSE is 155.8464 m under the declared initial bias distribution, versus
.0673876 m with fusion. This large comparison is scenario-specific.

The endpoint physical covariance remains finite and PSD; its minimum normalized
eigenvalue across nominal cases is .00051604305. The maximum unit-quaternion
squared-norm defect is 4.440892098500626e-16. Construction also checks each internal
joint covariance, so a materially indefinite or nonfinite joint result would fail.

For 100 independent seeds, the pointwise NEES-mean reference interval is
[13.9455503,16.0923322]. Both the individual-coverage and seed-mean evidence improve.
The 90.17% fraction of epochs inside that pointwise band is reported descriptively;
temporally correlated epochs are not independent trials or a simultaneous 95%
guarantee. The original 88.8367% campaign used different seeds; its result is retained,
not replaced by the new paired original-method value. Noise/gate values were fixed
before this evaluation and were not tuned against these outcomes.

| Gated fault quantity | Position | Altitude |
| --- | --- | --- |
| True positives / false negatives | 820 / 0 | 420 / 0 |
| False positives / true negatives | 43 / 4337 | 24 / 2356 |
| Precision | 95.0174% | 94.5946% |
| Recall | 100% | 100% |
| False-positive rate | .981735% | 1.008403% |
| Dropped / stale / pending | 400 / 400 / 20 | 0 / 200 / 20 |
| Burst detection delay | 0 s in all declared bursts | 0 s in all declared bursts |

These rates apply to the declared 2–3 m faults and prevalence. Every rejected NIS
remains in the report. Gated position RMSE is .1118259 m versus .3715369 m ungated.
Horizontal covariance grows through dropout in 20/20 cases and contracts after return
in 20/20. Mean variance at 7.9, 9.99 and 11.0 s is .00416673, .18496046 and
.00385243 m²; corresponding mean position-error norms are .0824138, .3936088 and
.0711081 m. Gated-fault NEES coverage is 96.0646%; selection and faults limit its
ideal Gaussian interpretation. Uncensored fault NIS is not expected to be nominal.

All 40 Q/R sensitivity variants completed without divergence. The first ten matched
nominal seeds have mean NEES 15.9750. Q multipliers .25/4 produce 33.6370/9.8563;
R multipliers .25/4 produce 32.7023/10.9119. These deliberately wrong assumptions
remain diagnostic evidence; they are not tuned candidates.

`validate_validation_report` reconstructs the complete summaries and assessments
successfully. The raw report SHA-256 is
`391a1e1d4fb52eb0aa93a04597427b938ffd839a5e81b8220c8ff9c841290660`.
The recorded source digest equals the prevalidation freeze. The original held-out
schema-1 report also revalidates with its original metrics and protocol hash.

The scoped correction meets the frozen implementation, accuracy, bias, fault and
consistency-investigation criteria. This supports improved covariance calibration
on the tested distribution. It does not prove exact Gaussianity or universal
consistency beyond the stated assumptions.

## Reproduction and verified commands

All local commands below ran on 2026-09-23. Environment: Python 3.12.14, uv 0.12.3,
NumPy 2.5.2, pytest 9.1.1, Ruff 0.16.2, mypy 1.20.2, Matplotlib 3.11.2.

| Actual command or experiment | Verified outcome |
| --- | --- |
| Baseline `.venv/bin/pytest -W error` | 2449 passed in 57.01 s |
| `uv tool run --from uv==0.12.3 uv run make check` | Ruff passed; 115 files formatted; mypy 32 source files; **2517 tests passed in 66.64 s** |
| Final `.venv/bin/pytest -W error` | **2517 passed in 113.60 s**, no failures/skips/warnings |
| `.venv/bin/pytest tests/unit/test_eskf_run_replay.py tests/unit/test_eskf_endpoint* -q -W error` | 212 passed, including both adapter modes |
| `OPENBLAS_NUM_THREADS=1 .venv/bin/python -m experiments.eskf_validation --propagation endpoint --partition development --workers 4 --output "$EVIDENCE/development.json"` | 75 replays; zero numerical failures or nominal/gated divergence |
| Same command with `--partition validation --output "$EVIDENCE/validation.json"` | 380 replays, 100 new independent seeds; all frozen targets met as qualified above |
| `validate_validation_report` on original and new held-out JSON | Original values preserved; new full ledger and summaries recompute |
| Plot CLI on development and held-out reports | Seven figures per report, including paired propagation comparison; all 14 figures visually inspected |
| `git diff --check`; relative documentation link check; protected-file comparison | Passed; original predictor, generator, dependencies, schemas, CI and instructions unchanged |

The 68 additional tests comprise 65 endpoint/core/experiment cases and three adapter
cases that retain the original tests and additionally exercise endpoint mode.

Use the locked Python 3.12 environment and uv 0.12.3. `$EVIDENCE` below is a new
output directory outside Git. Commands run from the repository root. Generated reports
and figures refuse overwrite. Workers change throughput, not job order or randomness.

```bash
uv sync --locked
make check
uv run pytest -W error
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.eskf_validation --propagation endpoint --partition development --workers 4 --output "$EVIDENCE/development.json"
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.eskf_validation --propagation endpoint --partition validation --workers 4 --output "$EVIDENCE/validation.json"
uv run python -m experiments.plot_eskf_validation --input "$EVIDENCE/validation.json" --output "$EVIDENCE/validation-plots"
```

## Technical limits

The corrected method is a second-order discrete approximation with local Gaussian error
and small-attitude-error assumptions. Its covariance matches the stated independent
instantaneous-sample and bias-endpoint model; it is not a generic colored-noise or
continuous-time IMU model. High angular acceleration, large coning motion, wrong sensor
assumptions, large initialization errors, weakly observable motion and persistent loss
of observations remain limitations. The first-order option retains its original
calibration qualification. This increment does not establish unknown-pose initialization,
live transport, delayed fusion/rewind, restart persistence, controller performance or
hardware flight readiness. Gate G2 remains separate.
