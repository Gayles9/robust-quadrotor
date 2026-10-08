# Basic ESKF Completion and Held-Out Engineering Evaluation

Date: 2026-09-23 (UTC).

- Audited baseline: `a95fbfcc2054a9a18151cb4c7d58d3b5817d4642`.
- Published code: `39e27daeceef755cc1b12a54a3571e935b9d9c76`.
- Published code tree: `0d57eed0030d6747611a8c4bea481619722b6f3b`.
- Pull request: [#5](https://github.com/Gayles9/robust-quadrotor/pull/5).
- Hosted code CI: [35904707795](https://github.com/Gayles9/robust-quadrotor/actions/runs/35904707795),
  job 107329339281, successful.
- Design fixed before implementation: [ADR 0009](../../decisions/0009-eskf-completion-validation.md).
- Mathematics, interfaces and reproduction: [estimator guide](../../design/estimation.md).

The subsequent [post-merge audit](eskf-post-merge-audit.md) strengthens experiment
and report validation while preserving this historical campaign and its NEES qualification.

## Outcome and qualification

The known-prior measurement-driven basic ESKF and the scoped Week 10–12 engineering
evaluation are implemented. Prediction, position/altitude correction, bias states,
Joseph covariance, quaternion injection/reset, explicit stale rejection and optional
NIS gates compose through the existing measurement-only runner. This increment adds the
independent motion, observation-fault and experiment infrastructure needed to evaluate
those components thoroughly rather than merely showing a smooth estimate.

All 280 held-out replay variants completed without numerical failure. All 100 nominal
trajectories and 20 gated fault variants avoided the declared divergence thresholds.
Position RMSE, excited-motion bias reduction, outlier precision/recall and dropout
uncertainty growth/recovery met their frozen targets.

**Full-state NEES central95 coverage was 88.8367%, outside the 90–98% investigation
band. This target is not passed.** The miss is retained in machine-readable assessment,
plots, README and this record. Independent development-only probes identify a deterministic
discretization contribution. No noise covariance or gate threshold was tuned against the
held-out data. The master plan calls for misses to be understood and documented, not
concealed by tuning. G3 engineering evidence is complete with this calibration
qualification; an unqualified statistical-consistency or flight-readiness claim is not made.

## Implemented scope and source mapping

| Requirement | Implementation and verification |
| --- | --- |
| IMU prediction, finite-difference Jacobians, covariance contracts | Existing `eskf.py`, `test_eskf.py`; unchanged |
| Position/altitude updates, Joseph form, injection/reset and gain diagnostics | Existing `eskf.py`, `test_eskf_update.py`; unchanged |
| Causal measurement-only execution and stale/pending handling | Existing `eskf_replay.py`, `eskf_run_replay.py` and their tests; unchanged |
| Fixed NIS gates and explicit rejection diagnostics | Existing `eskf_innovation.py`, replay and gating tests; unchanged |
| Analytic changing motion and known noisy/bias reference | New `eskf_synthetic.py`, `test_eskf_synthetic.py` |
| Single/burst outliers, dropouts and configurable additional delay | New `eskf_faults.py`, `test_eskf_faults.py`; explicit source identities and immutable labels |
| Bias convergence, dead reckoning, held-out errors/consistency and Q/R sensitivity | `experiments/eskf_validation.py`, `test_eskf_validation_experiment.py` |
| NIS/NEES, RMSE, bias, marginal uncertainty and fault plots | `experiments/plot_eskf_validation.py`; headless plot tests and visual inspection |
| Hand-computed scalar and 1D position/velocity examples | `experiments/kalman_sandbox.py`, `test_kalman_sandbox.py` |
| Reproducibility and failure accounting | Fixed separate streams/seeds, canonical protocol hash, actual Git/version provenance, source hash, exclusive output and all-seed summaries |

Core runtime remains NumPy-only. Matplotlib 3.11.2 and its locked transitive dependencies
are added to the development group for reproducible scientific plots. Existing locked
package versions are unchanged. Pytest's root import path permits testing the source
`experiments` package; it is excluded from the runtime wheel. No source/test behavior
from the baseline was modified. Existing CI still performs locked installation and
`make check`, now exercising the added tests and plotting dependency.

The scope does not include live transport, asynchronous/sparse IMU, automatic unknown-pose
initialization, estimator persistence/continuation, history rewind, new physical sensor
models, controller/mission logic or ROS/PX4 adapters. The accepted delay strategy explicitly
rejects every older acquisition. G2 baseline control remains open. These kinematic motion
fixtures do not establish rotor feasibility or closed-loop tracking performance.

## Frozen evaluation and provenance

Every full-duration trajectory spans 0–30 s at 100 Hz, with position at 10 Hz and altitude
at 5 Hz. Analytic position and roll/pitch/yaw amplitudes, frequencies and phases are
randomized independently of prior error, sensor noise, bias walks and fault offsets.
The prior is a declared known-pose distribution; the estimator never receives true
state/bias histories or fault labels. All model, noise, initialization and fault constants
are recorded in ADR 0009 and each report's canonical protocol.

| Partition | Seeds | Motion and replay count | Protocol SHA-256 |
| --- | --- | --- | --- |
| Smoke | 10–11 | Excited, .4 s, 16 variants | `7cf471729803d0ef11ffb126a9b2e90e767d10afccc71180f74778721c4f0ce6` |
| Development | 4000–4004 | Stationary, translating/yaw, excited; 60 variants | `998813847853b3cad682bf1346dda7dcd1b358c7bda456a5243148ed5dbf1651` |
| Held-out validation | 40000–40099 | Excited, 280 variants | `aec7acf36a2facb36cb6406ae6ab23106b159e79dd4400b0853268e1f9fddd12` |

The validation partition has 100 paired nominal/dead-reckoning trials, 20 additional
paired gated/ungated fault trials and four covariance sensitivity variants on the first
10 paired seeds. Variants are correlated comparisons, not extra independent seeds.
Numerical failure suppresses the corresponding complete-group ensemble. Finite divergent
trials remain included. Every scored NIS, including rejected observations, is retained.

The full held-out report was generated at `2026-09-23T18:39:58.228694+00:00`. Its actual
Git provenance is the baseline commit with `git_worktree_clean=False`, because the code
had not yet been committed. Its source/configuration digest is
`ffa1cc568de5c53f3157f8246aef5c29f4dda32ea599d090e7340f539a9919dc`.
All 113 published code-tree files were subsequently verified by their Git blob hashes.
A smoke run from the exact published commit, with `git_worktree_clean=True`, produced
the **same source digest** and completed all 16 variants. Thus the full campaign's
executed source bytes are identified with the published code without falsely claiming
that the earlier run had clean Git provenance.

Development ran before one additional input-domain guard for unrepresentably small
sample intervals. Its source digest is
`3c5fbbe432b0689625baf5652fa2e29f8c06e9b8a7c0e36018aa57d5795a3fe3`.
The guard changes only rejection of invalid inputs; model equations and protocol
parameters are unchanged. The held-out campaign and final tests use the final source.

## Held-out nominal performance

All values below include all 100 planned seeds. RMSE is the time RMS of the physical
three-vector error norm, with each physical block kept separate. The reported aggregate
is the arithmetic mean of per-seed RMSE values, not a mixed-unit state norm.

| Quantity | Measured result |
| --- | --- |
| Numerical failures across all variants | 0/280 |
| Nominal divergence | 0/100 |
| Mean / median / p95 / max position RMSE | .069264 / .069906 / .077082 / .081118 m |
| Paired mean dead-reckoning position RMSE | 167.647204 m |
| Reduction in mean position RMSE | 99.9587% (target ≥50%) |
| Mean initial → final accelerometer-bias error norm | .133856 → .014232 m/s² |
| Mean initial → final gyro-bias error norm | .0126571 → .000565708 rad/s |
| Final/initial bias norm ratios | .106321 accelerometer; .044695 gyro (both target ≤.5) |
| Minimum normalized covariance eigenvalue | .000384599648 |
| Maximum unit-quaternion squared-norm defect | 4.440892098500626e-16 |

All 100 dead-reckoning cases exceed the predeclared 10 m position or pi/2 attitude-error
divergence limit. They are retained, not discarded or mistaken for fused-filter failures.
Their large drift is expected under the declared .08 m/s² and .008 rad/s initial bias
standard deviations over 30 s. This comparison demonstrates drift correction under those
conditions; it is not a universal accuracy improvement factor.

Development controls share the same seed/prior design. Their final mean accelerometer/
gyro bias norms were .072696 m/s² / .006815 rad/s (stationary), .062957 / .004914
(translating/yaw), and .015702 / .000606 (excited), from initial means .129453 / .011389.
The contrast supports the stated excitation dependence; it does not prove global
observability for arbitrary motion.

## Consistency finding and investigation

| Statistic | Nominal mean | Individual central95 coverage |
| --- | --- | --- |
| Full 15-state NEES | 18.358807 | **88.8367% — below investigation band** |
| Position NIS, 3 DOF | 3.036568 | 94.9568% |
| Altitude NIS, 1 DOF | .992371 | 94.7682% |

For 100 independent seeds, the pointwise NEES-mean reference band is
`[13.9455503,16.0923322]`; only 12.5625% of epoch means lie within it. This is a material
full-state covariance-calibration finding, despite small position RMSE and near-nominal
innovation coverage. Temporal percentages are descriptive: repeated epochs are not
independent Bernoulli samples, and pointwise bands are not simultaneous guarantees.

The analytic generator is independent of prediction. Its derivatives satisfy central-
difference checks, and noise-free prediction has approximately first-order convergence
under step halving. The filter holds the left IMU sample, whereas these continuous
trajectories have changing force and angular rate inside every interval. The covariance
uses `Phi=I+F dt` and omits higher-order position/noise cross terms. Deterministic
integration discrepancy is not included as a random process in the declared Q.

Two additional **development-only** investigations use seeds 4000–4004 and retain all
noise assumptions and gate settings:

| Probe | dt=.01 s | dt=.005 s |
| --- | --- | --- |
| Noisy mean NEES | 18.346906 | 16.980935 |
| Noisy central95 coverage, descriptive five-seed result | 90.0633% | 98.4869% |
| Exact/noiseless mean normalized deterministic discrepancy | 2.313771 | .961650 |
| Exact/noiseless mean position RMSE | .010676 m | .005992 m |

The exact/noiseless probe sets the initial estimate and measurements to their analytic
values and removes actual bias/noise/walk realizations while retaining the estimator's
declared P/Q/R. Its normalized score is **not** a chi-square calibration experiment;
it quantifies systematic numerical discrepancy relative to the declared uncertainty.
Both that discrepancy and position error fall when the step is halved. This isolates a
discretization contribution. The noisy half-step probe also changes noise sampling and
is only supporting evidence, not a paired-noise causal proof or replacement acceptance
test. Five seeds and temporal correlation cannot establish a new calibrated operating point.

These observations explain a limitation of the documented first-order model, without
establishing that all residual contributions have been uniquely separated. The full
held-out result remains unchanged. Inflating Q would change the statistical result but
would not remove deterministic integration error; no such adjustment was made. Any
future change to numerical propagation or calibration requires a new design, protocol
and independent held-out seeds before asserting improved consistency.

The probes can be reproduced from the final source with `make_eskf_synthetic_case(seed,
number_of_steps=round(30/dt), time_step_s=dt, stochastic=...)`, `replay_eskf`, then
`evaluate_eskf_replay`, for each seed and dt above. The noisy call uses `stochastic=True`;
the deterministic call uses `False`. Average `history.nees.mean()` over the five seeds;
for position RMSE use `sqrt(mean(sum(history.error_states[:,:3]**2, axis=1)))` per seed.

## Held-out faults and recovery

The fixed 99% gates were selected before evaluation. Single outliers at 5 s and bursts
at [12,14) and [22,24) s have independently drawn directions/signs and magnitude 2–3 m.
Position drops out on [8,10) s. Both sensors are delayed .1 s on [15,17) s and at their
last acquisition; these become stale or pending, never fused at the wrong epoch.

| Metric | Position | Altitude |
| --- | --- | --- |
| True positives / false negatives | 820 / 0 | 420 / 0 |
| False positives / true negatives | 47 / 4333 | 24 / 2356 |
| Precision | 94.5790% | 94.5946% |
| Recall | 100% | 100% |
| False-positive rate | 1.0731% | 1.0084% |
| Dropped / stale / pending observations | 400 / 400 / 20 | 0 / 200 / 20 |
| First labeled burst rejection delay | 0 s in all 40 bursts | 0 s in all 40 bursts |

All labels remain outside replay. Missing/unscored values are excluded from confusion
denominators and recorded separately. The precision >.90 and recall >.85 targets pass
for this distribution; the empirical 100% recall is not a guarantee for small or subtle
faults. All 20 gated and ungated fault cases remained finite and below divergence limits.
Mean position RMSE was .119491 m gated and .392396 m ungated. The two arms use exactly
the same measurements, initial state and covariance.

Across the 20 gated cases, horizontal position covariance trace `P_NN+P_EE` increased in
20/20 dropouts and decreased after return in 20/20. Mean values at 7.9, 9.99 and 11.0 s
were .00413490, .20026199 and .00394818 m². Mean position-error norms at those epochs
were .080921, .423628 and .068593 m. Recovery is thus measured after fresh observations
resume; no truth-dependent reset or retrospective fusion is applied.

Gated-fault NEES coverage is 87.4309%. Selection and the changing observation pattern
further limit its ideal Gaussian interpretation. This value is retained, not hidden by
reporting accepted-only innovations. Ungated faults have NEES mean 246.885212 and
coverage 23.2239%, despite remaining below the broad divergence threshold.

## Q/R sensitivity

Each variant uses the same first 10 held-out seeds as its matched nominal comparison.
Multipliers act on covariance, not standard deviation; only one assumption changes at a
time. No gate is active. All 40 sensitivity variants completed without divergence.

| Variant | Mean position RMSE [m] | Mean NEES |
| --- | --- | --- |
| Matched nominal, same 10 seeds | .068047 | 18.905310 |
| Q × .25 | .070804 | 44.634450 |
| Q × 4 | .068370 | 10.254964 |
| R × .25 | .069864 | 35.677558 |
| R × 4 | .075272 | 13.572634 |

Position error alone would obscure the strong covariance sensitivity. The increased-Q
case's central95 coverage is 83.8854%, illustrating that simply increasing uncertainty
does not establish consistency. These are declared diagnostic mismatches, not tuned
alternatives or acceptance demonstrations.

## Commands and verification

All commands ran on 2026-09-23. The working source is identified above; after publication,
the clean smoke explicitly records code commit `39e27daeceef755cc1b12a54a3571e935b9d9c76`.
Environment: Python 3.12.14, uv 0.12.3, NumPy 2.5.2, pytest 9.1.1, Ruff .16.2,
mypy 1.20.2, Matplotlib 3.11.2. The host's other uv installation was not substituted for
the repository pin: `uv tool run --from uv==0.12.3` provided an isolated pinned runner.

Here `$REPO` denotes the absolute checkout root and `$EVIDENCE` the external result
directory; the experiment commands below ran from `$REPO`. Paths have been normalized
for reproduction. Raw results/figures/logs are excluded from Git by repository policy.

| Exact command or normalized invocation | Verified result |
| --- | --- |
| `uv tool run --from uv==0.12.3 uv run --project "$REPO" make -C "$REPO" check` | Baseline: 2358 tests passed in 45.48 s; final code: 2427 in 54.83 s; lint/format/type gates pass |
| `.venv/bin/pytest -W error` | Final: 2427 passed in 55.05 s; no skipped/failed tests or warnings |
| `.venv/bin/pytest tests/unit/test_eskf_faults.py tests/unit/test_eskf_synthetic.py -W error -q` | 43 passed, including ledger forgery and unrepresentable-interval guards |
| `.venv/bin/pytest tests/unit/test_eskf_validation_experiment.py tests/unit/test_kalman_sandbox.py -W error -q` | 26 passed, including worker-count determinism, failure retention, exclusive output and plot checks |
| `OPENBLAS_NUM_THREADS=1 .venv/bin/python -m experiments.eskf_validation --partition development --workers 4 --output "$EVIDENCE/development.json"` | 60 variants; zero numerical failures or nominal/gated divergence |
| `OPENBLAS_NUM_THREADS=1 .venv/bin/python -m experiments.eskf_validation --partition validation --workers 4 --output "$EVIDENCE/validation.json"` | 280 variants; zero numerical failures or nominal/gated divergence; NEES miss retained |
| `.venv/bin/python -m experiments.eskf_validation --partition smoke --workers 2 --output "$EVIDENCE/published-smoke.json"` | 16 variants from the clean published commit; source digest matches validation |
| `.venv/bin/python -m experiments.plot_eskf_validation --input "$EVIDENCE/validation.json" --output "$EVIDENCE/validation-plots"` | Six figures generated; each visually inspected |
| `.venv/bin/python -m experiments.plot_eskf_validation --input "$EVIDENCE/development.json" --output "$EVIDENCE/development-plots"` | Six development figures generated and visually inspected |
| `.venv/bin/python -m experiments.kalman_sandbox --output "$EVIDENCE/kalman-sandbox"` | Fixed example data and figure; visual inspection passed |
| `git diff --check` and Git blob/tree comparison | Clean diff; all 113 published blobs match local files |
| Hosted `uv sync --locked` then `make check` | 2427 passed in 67.63 s; 107 files reported formatted; 31 source files type-checked |

The additions contribute 69 tests without altering existing tests. Analytic derivative
checks, step refinement, seeded sample moments, initial bias/epoch identities, immutable
ownership, malformed-input rejection, additive delay/pending/dropout behavior, event-label
alignment, all-seed accounting, covariance checks and source-change detection are exercised.
The CI run verifies the same locked environment and code commit. This record distinguishes
passing implementation checks from the explicitly missed statistical investigation target.
