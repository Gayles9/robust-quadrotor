# Nonlinear pre-arm uncertainty verification

Date: 2026-09-28. Scope: one uncertainty derivation and offline calibration.

## Audit and frozen decision

Audited merged PR 26 at main `2f92c9ece0d88c4fdff95bca2de1f10158d6d2ba`.
Main matches GitHub and the starting worktree is clean. Fresh warning-strict
pre-arm/documentation tests pass 33 tests in 0.28 s. Five preserved payloads
authenticate and reproduce the previous failed 1.112163609 covariance maximum.

[ADR 0028](../decisions/0028-nonlinear-prearm-uncertainty.md) freezes one positive
Gaussian quadrature model and independent acceptance before execution. The
original 0.5 s/201-sample interval, sensor/bias/heading priors, rejection limits
and support contract remain fixed. No production algorithm or flight changes.

The [derivation](../prearm-nonlinear-uncertainty.md) identifies a mixed
heading/inclination term in the nonlinear rotation error. At level, its leading
variance is about 7.811% of the small attitude variance conditional on terminal
accelerometer bias. Finite differences confirm the mixed derivative. This
explains why correct first-order cross-covariance can still underpredict a
joint direction; it is not an empirical noise adjustment.

## Candidate and all retained outcomes

Use 625 fixed positive quadrature nodes over mean-accelerometer and heading
errors. Propagate the exact inverse gravity direction, solve the local rotation
mean, and retain joint covariance through an exact Gaussian conditional-bias
decomposition. The gyro-bias mean-to-terminal formula remains unchanged. A fixed
order-seven accuracy reference checks integration error only.

| Frozen gate | Observed | Result |
| --- | --- | --- |
| Original Gaussian regression maximum variance <=1.10 | 1.112164 before; 1.061394 after | Pass |
| Fresh Gaussian maximum variance <=1.10 | 1.142382 paired first order; 1.082222 candidate | Pass |
| Fresh Gaussian maximum absolute whitened mean <=0.05 | 0.020453 | Pass |
| Nominal rejection <=1% | 0/5,000 | Pass |
| Modeled rotation mean norm <=1e-13 rad | Maximum 9.242e-14 | Pass |
| Quadrature relative covariance error <=0.001 | Maximum 1.917e-12 | Pass |
| Quadrature rotation-mean difference <=1e-8 rad | Maximum 7.087e-18 | Pass |
| Independent identities, correlations and rejection tests | All targeted checks pass | Pass |

Fresh nominal axis-error median/p95/p99/max are
0.096012/0.160244/0.186317/0.237789 degrees. Reported local 99% radius ranges
0.533317–0.700221 degrees. Gaussian marginal radius coverage is 99.26%.
The new nominal rejection count is not compared as a noise-gate improvement
against the different old nominal random population.

The original 5,000 Gaussian errors, whitened errors, covariances and gate
statistics reconstruct exactly before applying the candidate. A separate
5,000 nominal and 5,000 Gaussian trials use the frozen new streams. All
15,000 outcomes and paired first-order comparisons are preserved, with no
selection of accepted windows and no retries with alternate seeds or models.

The largest fresh nominal mean-rotation correction is 1.737e-7 rad. The benefit
is more accurate uncertainty accounting; a useful change in pointing or flight
performance is not demonstrated. No flight was executed in this step.

## Verification and identities

Python 3.12.14, NumPy 2.5.2; dependency pins unchanged. The twenty new tests cover
quadrature moments independently, quaternion/matrix/ESKF agreement, mixed
finite differences, explicit latent conditioning, the linear covariance limit,
heading-prior retention, nonlinear mean and full covariance, accuracy reference,
invalid inputs/nonconvergence, inherited motion/support rejection, the known
IMU ambiguity, fresh-sample handoff and evidence authentication.

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_prearm_nonlinear_uncertainty.py tests/unit/test_prearm_alignment_feasibility.py tests/unit/test_documentation.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.prearm_nonlinear_uncertainty --prior-evidence ../prearm-alignment-final --output ../prearm-nonlinear-evidence
.venv/bin/python scripts/check_docs.py
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q
```

Targeted checks pass **53 tests in 0.48 s**. Lint and formatting pass; strict mypy
passes 79 source files. Two mechanical typing issues were corrected before the
single full population execution. Full repository results and publication
identities are recorded in the closing pull request.

| Identity | SHA-256 |
| --- | --- |
| Frozen ADR 0028 | `08ed3e88af2e5c4335f8194b930026d85a0fadd6c95c6ed7c75745b98d3c954f` |
| Executed source fingerprint | `9a344c76211ef00adfebb0f39435634afb39dd693fd1d7691590291f7fa452df` |
| Original report | `260532b86c36cb7cd31c3d2fd079f6ff9abd4bb975b350522984cc1dafb4dfba` |
| Original trial payload | `953adb6dfa156abffaa726fab58fc4a803efb3c13acba6c960a2dba2c7c90451` |

The report records execution from the audited base with a dirty working tree.
Its source fingerprint binds the subsequently published implementation, without
claiming a clean post-merge execution. Generated numerical evidence stays
outside Git, with an integrity manifest and independent summary reconstruction.

## Completion and limitations

**The bounded derivation step met all frozen acceptance criteria.** It is go
for the next standalone alignment component. No empirical inflation, duration
adjustment, threshold relaxation, controller change or reserved flight seed was
used. The preceding first-order failure remains preserved as historical evidence.

The pushforward is a local uncertainty approximation, not the exact nonlinear
gravity-constrained posterior. The calibration margin is finite, and the study
does not qualify hardware, prove all-pose coverage, or establish physical support.
Gravity still supplies no heading information; a constant acceleration can still
pass the IMU gates. The externally guaranteed stationary interval is mandatory.

Next, audit this closeout and implement the standalone supported-acquisition
component with explicit support provenance, motor status, sample/clock ownership,
restricted priors, latched rejection, diagnostics and one-time fresh release.
Freeze those component tests first. Keep flight integration and arming outside
that step; later full-flight evaluation must retain the original scoring limits.
