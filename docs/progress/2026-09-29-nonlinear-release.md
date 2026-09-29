# Nonlinear release uncertainty and isolated flight comparison

Date: 2026-09-29. Scope: [ADR 0036](../decisions/0036-nonlinear-release-flight-comparison.md).

## Audit and bounded implementation

Audit PR 34 merge `85f0d78d867122500c9f988c9daef60967aca4ce`, tree
`f85f5fd305e02f63a22f549002e50b5095ba2736`, against live GitHub and clean local
main. Fresh warning-strict relevant checks pass **78 tests in 2.09 s**. The
preceding independent verifier authenticates **99 payloads** and reproduces its
nonlinear errors within **2.776e-17**. The earlier rank-18 first-order omission
is real; the prior milestone is not reimplemented or relabelled as a flight gain.

Freeze the complete mathematical and 25-flight protocol before implementation.
Reduce the 33-dimensional Gaussian input to six nonlinear rotation variables
and 18 conditionally affine variables. Analytically integrate the latter and
use positive order-five quadrature over the former. Intrinsic attitude centering
and the law of total covariance retain all position/velocity/attitude/bias/fresh
sample cross terms. [The guide](../nonlinear-release.md) gives the derivation.

The implementation is confined to experimental modules. Exactly the first
prediction changes. All subsequent intervals, controller settings, sensor/noise
parameters and the original velocity prior in flight remain intact. No simulated
truth, covariance floor, pseudoinverse, null-space deletion or empirical tuning
enters the predictor. Exact-zero-prior checks are mathematics only.

Nineteen new mathematical/routing/decision tests cover analytic omitted variance,
correlations, positive covariance sums, limiting cases, sign invariance, first-only
ownership, online/replay equality and frozen acceptance. The initial relevant
combination passes **77 tests in 4.18 s**. Three evidence-durability tests added
after a recording failure bring the new-test total to 22; all 22 pass in 2.09 s.

## Mathematical outcome and evidence integrity

Reconstruct all 17 genuine supported releases and causal time-zero posteriors.
Retain 20,000 fixed paired Gaussian draws per configuration, or **680,000 nonlinear
outcomes across both priors**. All 17 cases pass full rank, full covariance
calibration, mean calibration and order-five/order-seven accuracy under both
priors. The whitened covariance eigenvalue ranges are **0.9280067–1.0721111**
(original) and **0.9281937–1.0752919** (exact velocity), within [0.90,1.10].
Maximum absolute whitened means are **0.0192471** and **0.0203668**, below 0.05.
Measured full covariance integration discrepancies are at most **2.014e-12**
and **1.069e-8**, well below 0.001. Ballistic discrepancies are at most
**1.404e-12** and **1.196e-8**. The output rank is 21 in every tested case.

An independent NumPy-only verifier regenerates all fixed Gaussian perturbations,
evaluates the finite nonlinear map with quaternion-vector rotations and checks
every saved error vector and full covariance statistic. Its maximum discrepancy
is **1.388e-16**. It does not import project code. This is separate from the
repository runner's exact full report/array reconstruction. Neither check claims
that the local Gaussian approximation is an exact physical posterior.

The first mathematical execution passed, but its saved-data replay found one
truncated NPZ (`wind_tracking-47003.npz`, 3,228,143 bytes). Sixteen other payloads
retained their recorded hashes. This happened before any comparison flight.
The underlying storage cause is unknown. Add experiment-local staged output:
close and fsync each payload, publish through a no-clobber link, and fsync the
directory. Unit tests check unchanged JSON bytes, array round-trip, no overwrite
and failed-write cleanup. Existing repository-wide writers remain unchanged.

Repeat the **identical fixed calculation**, not a new random population, under
the new execution-source fingerprint. Every numerical report row and all
**17 originally expected NPZ hashes reproduce exactly**. The new files survive
the next tool boundary and complete saved replay. Retain the failed recording
receipt and original report rather than silently rebinding old provenance.
No acceptance threshold, equation or scientific outcome changes.

The first full software-suite run overlapped that source edit; its integrity
guard correctly reported `execution source changed during the campaign`
(3,854 passed, 15 fixture errors). Restart the software gate with fixed source.
Those fixture errors are not failed scientific flights or grounds for tuning.
The next software run exits successfully but its redirected log is truncated
after completion. Repeat the fixed-source gate with a synced completion receipt:
**3,872 tests pass in 409.24 s**, including all 22 new tests.

## Flight result and decision

Execute **exactly 25 scientific flights**, with no retries: nine clean aligned
flights and off/on supervision pairs for eight fault cases. The original velocity
prior remains unchanged in every first-prediction trace. The mathematical
exact-prior variant never enters a flight. Reuse authenticated original controls;
do not rerun baseline flights.

| Hover seed | Baseline peak, cm | Candidate peak, cm | Candidate within 8 cm | No regression |
| --- | --- | --- | --- | --- |
| 47001 | 10.180776 | 10.198128 | No | No |
| 47002 | 5.463810 | 5.478334 | Yes | No |
| 47003 | 5.241372 | 5.245494 | Yes | No |

All nine clean whole-flight RMSEs improve, by **0.42665–0.90981 mm**. This is a
small measured benefit, not a hover fix. All three hover peaks rise, by
**0.04122–0.17352 mm**. The six nominal/wind tracking comparisons pass, but all
three hover comparisons fail the frozen acceptance: one still exceeds 8 cm and
all three violate no-regression. Thus **6/9 clean comparisons pass**, with
**2/3 hover flights** passing their absolute flight limits. No averaging hides
the failing seed or trades the peak requirement against RMSE.

All **8/8 fault-response comparisons pass**: position and altitude dropout,
rejection and delay; position recovery; and landing position dropout. The seven
persistent supervised cases produce the required numerical abort, not a claim of
safe physical landing. The landing dropout terminates at 14.005 s with true speed
0.258428 m/s, so its ordinary completion/final-speed conditions are false even
though its specified fault response passes. Both recovery arms complete.

Each flight is saved and reloaded before complete estimator, command, health,
supervision, plant and motor reconstruction. First-prediction online/replay
traces agree exactly. All plant/motor residuals are zero. Maximum reconstructed
random-draw error is **6.433e-15**; paired baseline/candidate IMU/slow-sensor
noise differences are at most **8.882e-15**, **1.388e-17** and **8.882e-16** for
accelerometer, gyro and slow sensors respectively. Full histories and every score
remain available outside Git.
The independent NumPy-only artifact checker authenticates all 25 histories,
rescoring their flight metrics and checking initial-prior identity and signed
changes. All independent checks pass; the performance decision remains failed.

The bounded derivation, implementation and comparison are **completed**.
The mathematical acceptance **passes**. The frozen flight comparison **fails**,
so adoption and flight qualification remain **no-go**. No production source or
controller default changes. This experiment isolates the complete release
prediction package from velocity conditioning; it does not separate the force
rule's contribution from the nonlinear mean/covariance contributions.

Next separately freeze a combined supported-velocity-prior experiment using the
boundary-only histories as controls and retaining the original uncorrected
comparison. No combined flight has run, and success is not presumed. Fresh
validation still precedes integration; geometric tuning and mass compensation
are not reopened by this result.

## Reproduction and identities

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_durable_release_evidence.py tests/unit/test_nonlinear_release.py tests/unit/test_nonlinear_release_validation.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.nonlinear_release_validation --campaign ../release-recovered/campaign --output ../nonlinear-release-uncertainty-durable
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.nonlinear_release_validation --campaign ../release-recovered/campaign --verify ../nonlinear-release-uncertainty-durable
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.nonlinear_release_campaign --campaign ../release-recovered/campaign --uncertainty ../nonlinear-release-uncertainty-durable --output ../nonlinear-release-flights --workers 2
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.nonlinear_release_campaign --campaign ../release-recovered/campaign --uncertainty ../nonlinear-release-uncertainty-durable --verify ../nonlinear-release-flights --workers 2
.venv/bin/python scripts/check_docs.py
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q
```

| Identity | SHA-256 |
| --- | --- |
| Original pre-durability source | `e1a79756a0e6f1b50eec558cec9a24cc78f5f8957262f35766ccc7183e1604a0` |
| Final execution source | `cd190a70ccae59e9d437e9c44555902f0ce28912eae8dff8fa5c7d4a1a09b215` |
| Frozen ADR 0036 | `2a98e254e1505ab19e0ec98731dfe6151417779a0169083628bbd9ff33e82265` |
| Original baseline campaign report | `b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63` |
| Original mathematical report | `b2d39c35e0712271c13472a6c703913178f8fea3ccb544b2f0da3f2a534be9ea` |
| Durable mathematical report | `8daddf91b2bbfc446a98a431e974303f58837057684ac3a7e11118d378384c81` |
| Complete 25-flight report | `54f848a0593cef3abe705bb713db97eb584299d3d74fedac91ab309df4ff9cf2` |

The execution occurs on the audited base with uncommitted candidate source,
bound by the execution fingerprint. Generated evidence remains outside Git.
Documentation checks pass **139 files, 619 local links and 23 Python/JSON
examples**. Ruff lint/format pass (**323 files**); strict mypy passes **97 source
files**. Whitespace checks pass. The warning-strict full suite is **3,872 passed**.
Complete software, saved-replay and hosted publication receipts accompany closeout.
