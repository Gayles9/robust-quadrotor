# Combined supported velocity and nonlinear release comparison

Date: 2026-09-29. Scope: [ADR 0037](../../decisions/0037-combined-supported-velocity-comparison.md).

## Audit and implementation

The preceding milestone was audited at commit
`9dad5d79229f1b8b194bc21c0e1cd9e988646fc8`, tree
`e21e946a284e21073275d5f08036b776788a675f`. Its 80 focused tests passed in
4.25 s. All 17 mathematical configurations reconstructed, including both
velocity priors and independent moment checks to 1.388e-16. No preceding
implementation defect was demonstrated.

One new experiment module combines the existing supported velocity conditioner
with the unchanged nonlinear first-prediction adapter. No production source,
dependency, gain, noise model, default or later prediction interval changes.
Eleven new tests check one-time conditioning, prior/configuration ownership,
missing/moving/revoked/stale support, complete online/replay equality, recovery
of prediction routing on exceptions, and comparison against both controls.
The first historical test run failed on the missing new module; two test
assertions were corrected for sample-copy ownership and the existing states API.
The complete focused group then passed 91 tests in 6.86 s.

The warning-strict full software suite passed 3,883 tests in 421.07 s, with
Ruff, formatting, type checking and documentation checks passing. The campaign
completed in 1,269.69 s with exactly 25 new scientific flights and no retries.
Both saved control histories were also reconstructed before comparison.

## Clean results and acceptance

All nine clean flights pass their absolute limits. The three hover peaks are:

| Seed | Original aligned (cm) | Boundary only (cm) | Combined (cm) |
| --- | ---: | ---: | ---: |
| 47001 | 10.180776 | 10.198128 | 6.753928 |
| 47002 | 5.463810 | 5.478334 | 5.694340 |
| 47003 | 5.241372 | 5.245494 | 5.564383 |

The failed seed47001 peak drops 3.4442 cm (33.77%) against the boundary control
and 3.4268 cm against the original. Seed47002 and seed47003 increase by
2.1601 and 3.1889 mm against the boundary control. Four of nine clean
no-regression comparisons pass: all three seed47001 cases and wind seed47002.
RMSE changes range from -8.3018 to +4.2800 mm against the boundary control,
and -8.8999 to +3.4208 mm against the original. All seed47003 RMSE comparisons
regress; seed47002 hover and nominal also regress against the boundary control.

The full frozen comparison fails. This is a useful measured tradeoff under a
physically supported prior, not a default adoption or fresh-seed qualification.

## Faults and reconstruction

All eight fault-response comparisons pass. Both recovery flights complete.
Seven persistent supervised cases reach the specified numerical abort; that
response does not establish a physical safe landing. The landing-dropout
candidate stops at 14.005 s with speed 0.258758 m/s and fails ordinary completion
and landing-speed requirements while passing its prescribed fault response.

All 25 candidate histories pass complete saved reconstruction, including the
ESKF, controller, guards, health/supervision, plant/motors and noise draws.
Plant/motor reconstruction residual is zero. Maximum draw error is 6.9181e-15;
maximum paired accelerometer, gyro and slow-sensor errors are 8.8818e-15,
1.3878e-17 and 8.8818e-16 respectively.

The separate NumPy-only verifier authenticates every archived payload, recomputes
scores and decisions, checks the isolated covariance change, and confirms that
the complete first-prediction input and output equal the exact-prior mathematical
gate. It does not independently reconstruct the full fault supervisor; that is
checked by the source-based replay above.

## Reproduction and identities

The [guide](../../results/combined-supported-prior.md) gives campaign and saved replay
commands. Software checks used:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m pytest -q tests/unit/test_combined_supported_prior.py tests/unit/test_supported_velocity_prior.py tests/unit/test_release_prediction.py tests/unit/test_nonlinear_release.py tests/unit/test_nonlinear_release_validation.py tests/unit/test_durable_release_evidence.py
.venv/bin/python scripts/check_docs.py
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m pytest -q
```

| Item | SHA-256 |
| --- | --- |
| Preceding execution source | `cd190a70ccae59e9d437e9c44555902f0ce28912eae8dff8fa5c7d4a1a09b215` |
| Combined execution source | `0073d6024a5fea509fe3aaf2dca06e3c5b835b708eb637136c0043833926a12b` |
| Frozen ADR 0037 | `728969273ab54651868b8b0e9f568249a8db40143cbd040497b3eb844ed97b5d` |
| Original aligned report | `b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63` |
| Boundary-only report | `54f848a0593cef3abe705bb713db97eb584299d3d74fedac91ab309df4ff9cf2` |
| Mathematical report | `8daddf91b2bbfc446a98a431e974303f58837057684ac3a7e11118d378384c81` |
| Combined report | `6d44ab913156fafe3846a630065cdaad9eee2dc9c64c1ea50234b82fbfa314fa` |
| Complete evidence ZIP | `0198ec4bc88443cbf49fd093cca61b68f78571db4aa1254236d71f558812a24f` |

Execution occurred on the audited base with uncommitted experiment code and
results outside Git. Excluding exactly the new module reproduces the preceding
execution fingerprint. The evidence archive retains the original publication
block and staged tree `c2b11e24e1c2714021b3d5d7254218116b7126fb` as historical
receipts. After explicit publication authorization, a cleared checkout was
restored from the authenticated base and conversation. Execution source and the
frozen ADR match their saved hashes exactly. Tests and explanatory documentation
were reconstructed and checked again; the publication tree is a new identity,
not a claim that the deleted staged tree was recovered byte for byte.

## Next decision

Use the saved improved and regressing histories to reconstruct observation
corrections, navigation-error feedback and commanded acceleration. Run no new
flights and fit no gains. If there is no specific correctable defect or
independently justified next hypothesis, close this startup candidate while
preserving the failed gate. Mass-offset and geometric work remain separate.
