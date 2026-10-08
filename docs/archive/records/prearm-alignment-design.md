# Stationary pre-arm alignment design and feasibility

Date: 2026-09-28. Scope: contract, mathematics and offline feasibility only.

## Prior audit and frozen scope

Audited main `cdc27707ce40d50eb3be8066e53f15ea46b5b258` (PR 25), with
post-merge CI `36479669755` successful. Fresh warning-strict startup and
documentation tests passed 32 tests in 0.29 s. Both existing diagnostic evidence
packages authenticated all 27 payloads. No estimator defect was demonstrated.

[ADR 0027](../../decisions/0027-stationary-prearm-alignment-design.md) freezes one
mean-based alignment design, duration allocation, uncertainty budgets, rejection
tests, two populations and a stopping rule before execution. The explicitly
authorized supported-stationary operating mode is conditional; it does not
establish a physical support guarantee for hardware or the existing flight plant.

The [contract](../../design/prearm-alignment.md) defines the required support evidence,
proposed production interface, gravity/gyro information, uncertainty, rejection,
latched failure behavior and disjoint endpoint handoff. The reference remains
under `experiments`; no production algorithm, controller, estimator, dependency,
noise profile, mission, threshold or reserved flight seed changed.

## Design and result

The analytic minimum span is 0.34 s: 137 accelerometer samples and 132 gyro
samples satisfy the respective budgets. Block rounding selects 201 paired
samples spanning 0.5 s, then one fresh release sample at 0.5025 s. The white
inclination sigma is 0.0164784 degree per axis and terminal gyro-bias three sigma
is 0.0242885 degree/s. The level accelerometer-bias floor remains 0.1752165
degree per axis; heading prior sigma remains 3 degrees.

| Frozen check | Result | Decision |
| --- | --- | --- |
| Noise allocation | 0.01648 degree <=0.02; gyro 0.02429 degree/s <=0.03 | Pass |
| Nominal rejection <=1% | 1/5,000 = 0.02%, due to variation gate | Pass |
| Nominal axis error | Median 0.09604; p95 0.16403; p99 0.18693; maximum 0.26035 degrees | Descriptive, new supported population |
| Nominal reported local 99% radius | 0.53330–0.69828 degrees; empirical coverage 100% | Below 0.75-degree budget |
| Gaussian reported local radius coverage | 99.4%; axis-error p99 0.55348 degrees | Descriptive marginal check |
| Gaussian nonlinear full joint covariance | Largest normalized variance 1.112163609 versus 1.10 maximum | **Fail** |
| Gaussian linearized covariance control | Largest normalized variance 1.062171150 | Consistent with covariance algebra |
| Motion/invalid-input fixtures | Required examples rejected | Pass |
| Indistinguishable acceleration fixture | IMU gates pass as predicted | Required limitation retained |

Both populations retain all 5,000 outcomes, including rejected windows. The
Gaussian check whitens actual nine-dimensional nonlinear errors by each
reported Cholesky factor, then examines centered empirical covariance. The
worst direction has 11.2164% excess variance versus the allowed 10%; this is
about 5.46% excess standard deviation, not 11.2% angular error. The miss is
narrow but remains a failure. No confidence claim beyond the frozen finite-study
decision is inferred, and no seeds or covariance inflation were fitted afterward.

Independent latent-increment assembly agrees with the first-order covariance;
finite differences confirm the frame/Jacobian signs. Nonlinear rotation/bias
coupling is a plausible explanation for the difference from the linearized
control, not a uniquely proven cause. Good marginal tilt results do not permit
discarding correlations or overlooking the joint uncertainty failure.

The impulse, vibration and changing-gravity variation statistics are 1788.15,
3756.04 and 4090.87. A 0.05 rad/s mean-rate fixture gives gyro statistic 99.94.
The constant-acceleration fixture [0.1712081, 0, 0.0014941] m/s² remains
indistinguishable from stationary one-degree pitch. This is why support cannot
be inferred from the existing IMU alone.

## Verification and reproducibility

Environment: Python 3.12.14, NumPy 2.5.2, Ruff 0.16.2; dependency pins unchanged.

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_attitude_startup_audit.py tests/unit/test_documentation.py
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_prearm_alignment_feasibility.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.prearm_alignment_feasibility --output ../prearm-alignment-final
.venv/bin/python scripts/check_docs.py
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q
```

The new independent test module passes 22 tests in 0.22 s. Initial formatting
and type issues were corrected without algorithm changes. A repeat of the same
frozen 10,000 trials after those mechanical corrections has exact equality in
all 16 saved trial arrays and every numerical report field. This is a source
verification repeat, not an additional candidate or population. Full repository
checks and publication evidence are recorded with the closing pull request.

| Identity | SHA-256 |
| --- | --- |
| Frozen protocol | `0be5cc1d3ca50135ce1903c1e136fd8cf3abcc841433e81e4d50f5e734b42645` |
| Executed source fingerprint | `2445466e6cdafe09107856b888d453618d0c099b0dfd0815c13be0a8dbdd4e1f` |
| Initial pre-type-cleanup source fingerprint | `94279cf4402217ce1eb545c7adfe0927bc89b70754bb87985dc5ec0cbb3bd0a6` |

The report records the audited base commit and a dirty working tree during
execution. The fingerprint binds the executed code; publication does not
relabel this as a clean post-merge run. Generated report and all per-trial
numerical evidence are preserved outside Git with integrity verification.

## Completion and exact next step

The requested design/feasibility step is complete, with an explicit contract
and a justified stop. **Acceptance for production implementation was not met.**
The supported alignment signal is promising; its first-order joint uncertainty
requires more work. This study contains no new flight result, qualified hover
improvement, geometric promotion or verified hardware support procedure.

Next, independently reconstruct the joint covariance miss and derive one
nonlinear uncertainty representation at the estimated attitude, including
leading heading/rotation/bias coupling and any mean correction. Freeze its
independent calibration tests before execution; retain the original noise,
0.5 s interval, <=10% criterion, support contract and stopping rule. Do not
implement the production initializer or integrate flight until that gate passes.
