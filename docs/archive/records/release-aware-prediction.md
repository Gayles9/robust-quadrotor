# Release-aware prediction and uncertainty

Date: 2026-09-29. Scope: ADR 0035, first-interval component and uncertainty gate.

## Audit and fixed protocol

Audit [PR 33](https://github.com/Gayles9/robust-quadrotor/pull/33), merge
`7d47303feeb9544cbf7fc750576a9c8016aeba12`, tree
`aadf749ed190cb8ec61d8ad3ddd160b50b1195bd`. The local workspace initially contains
an older PR 23 snapshot. Ordinary remote fetch fails without GitHub credentials;
restore the 64 changed files through authenticated repository access and require
Git blob hashes and the complete tree hash to match before advancing local main.
Restore matching commit objects, preserve the old branches and do not rerun old
implementation steps. Both preceding evidence archives authenticate.

Fresh warning-strict endpoint/support/prior tests pass 94 tests in 4.75 s.
The independent ADR 0034 verifier authenticates 65 payloads and reconstructs all
17 rejections. The original ADR 0031 campaign authenticates and all 34 saved
flight scores reproduce. The documented force discontinuity remains the only
demonstrated preceding defect. No current flight result is overwritten.

[ADR 0035](../../decisions/0035-release-aware-prediction.md) freezes one-sided force
integration, complete first-order derivatives, finite nonlinear checks and the
prior decision before implementation. It explicitly limits this step to the
component gate and freezes the subsequent isolated 25-flight comparison. It does
not execute that comparison or combine velocity conditioning with flight feedback.

## Implementation and independent checks

`experiments/release_prediction.py` implements the first post-release acceleration
rule and complete A21x21/B21x12 derivatives. Its opt-in experiment context validates
support/sample/interval/noise ownership, changes exactly the first prediction,
restores on exit and rejects reuse. No production source changes. The old gyro
conditional mean and all joint cross terms remain; old accelerometer force does
not cross the support discontinuity. The context never conditions velocity.

Twenty-seven new tests cover analytic free fall and constant acceleration,
quaternion sign, all 33 independent finite-difference columns, full correlated
covariance, smooth-force error bounds and step-halving, rotating force, shared
noise across subsequent predictions/corrections, singular initialization,
invalid vectors/intervals, restoration/rejection and short online/saved-replay
agreement. The independent finite map uses batched quaternion-vector operations,
not the implementation's matrices. Combined fresh relevant checks pass
**121 tests in 7.83 s**. A short simulation in the software test is not counted
as a scientific campaign flight.

Initial formatting/import findings and a NumPy save keyword type annotation are
fixed before evaluation. A partial mypy invocation omitted the configured source
roots and reported untyped imports; the prescribed `mypy src experiments scripts`
passes all 93 files. No numerical failure prompted parameter or gate changes.

The runner authenticates/reconstructs all preceding releases, then retains all
5,000 paired Gaussian draws per configuration: 85,000 paired draws and 170,000
nonlinear outputs, with paired linear errors and latent perturbations. Saved
verification rederives every array and complete report exactly. An independent
NumPy-only artifact verifier checks all covariance entries, both endpoints, ranks,
noise identities, original support records and all Gaussian outputs. Its maximum
independent nonlinear error difference is **2.776e-17**. No project code is imported
by that verifier. The Gaussian pushforward is conditional on the declared local
prior; it is not an independent physical alignment or mission population.

## Numerical outcome

| Quantity | Previous release rule | One-sided release rule |
| --- | --- | --- |
| Analytic down-velocity bias | -0.0122625 m/s | 0 |
| Analytic down-position bias | -0.0000204375 m | 0 |
| Necessary ballistic screen with exact velocity prior | 0/17 pass | 17/17 pass |
| Conditioned down-velocity sigma | 0.0001030783 m/s | 0.0001250005 m/s |

The new sigma follows the derived noise weights; it is not an inflation factor.
The conditioned first-order joint covariance has rank 18. In the ballistic case
it asserts `delta_v + h*R*(delta_ba + delta_new_accel_noise) = 0`. Its paired linear
outputs satisfy that relation within 1.58e-19 m/s. Nonlinear outputs have standard
deviation **6.057e-7 to 6.744e-6 m/s** in those three relations. Their marginal
velocity variance ratios are 0.9507–1.0365 and maximum absolute standardized mean
is 0.03071. Passing-looking marginals do not validate exact joint constraints.

An independent Gaussian yaw/force counterexample derives strictly positive
omitted variances 4.3061e-14 and 2.0933e-11 (m/s)². Fixed positive order-nine
quadrature agrees within 4.34e-25 (m/s)². This is an analytic local-model finding,
not a stochastic threshold selected after seeing the samples. The original broad
prior stays rank 21; its marginal velocity ratios range 0.9523–1.0499, without a
claim that those marginals establish complete joint calibration.

## Acceptance and limits

The bounded implementation/equation/ownership step **passes**. Exact zero-velocity
conditioning with this first-order joint covariance remains **no-go** under the
frozen uncertainty criterion. Removing the deterministic bias does not repair
nonlinear covariance by itself. No pseudoinverse, floor, Q/R or gain tuning is used.
Zero new scientific flights run; the ordinary 10.18 cm hover failure and unchanged
8 cm requirement remain. Cascade stays default. No hardware support or full
flight improvement is claimed.

Next derive nonlinear joint moments for the corrected first interval, preserving
bias and new sample correlations. Verify analytic limits, integration accuracy
and full covariance with a fixed fresh population. Only after that gate should
the frozen boundary-only 25-flight regression run; a separately frozen comparison
may then add velocity conditioning. Fresh validation still precedes integration.

## Reproduction and identities

Commands from the repository root (evidence paths are supplied externally):

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_release_prediction.py tests/unit/test_supported_velocity_prior.py tests/unit/test_eskf_endpoint.py tests/unit/test_supported_start.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.release_prediction_validation --campaign ../release-recovered/campaign --output ../release-prediction-results
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.release_prediction_validation --campaign ../release-recovered/campaign --verify ../release-prediction-results
.venv/bin/python scripts/check_docs.py
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q
```

Evaluation and saved verification exit zero for successful execution, while the
report explicitly retains `zero_velocity_prior_accepted=false`. The independent
archive command is `python verify_evidence.py`, with NumPy installed.

| Identity | SHA-256 |
| --- | --- |
| Execution source | `d3ce6ffd267c9cdf5e25b990a5810c62aaed785f1e1b12459af7fdc8f22f4d33` |
| Frozen ADR 0035 | `ef7756704cb95be66d5634568ea79036f9bcac45e2d250cb863e812875730585` |
| Input campaign report | `b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63` |
| Complete result report | `bc5be3e461f39eaeefb8c2361389c8ef15073dd4cb85005e723a154b4644d2db` |

The full warning-strict suite passes **3,850 tests in 362.29 s**. Documentation
checks pass 136 files, 606 local links and 23 Python/JSON examples. Ruff lint and
format (313 files), strict mypy (93 files) and staged whitespace checks pass.

Execution occurs on the audited base with declared uncommitted candidate source;
the source fingerprint binds the published implementation. Full gate and hosted
publication receipts accompany the evidence archive and pull-request closeout.
