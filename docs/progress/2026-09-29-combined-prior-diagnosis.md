# Saved combined-prior tradeoff diagnosis

Date: 2026-09-29. Scope: [ADR 0038](../decisions/0038-combined-prior-tradeoff-diagnosis.md).
The [guide](../combined-prior-diagnosis.md) explains the equations and reproduction.

## Preceding milestone and implementation

PR 36 published the combined experiment at
`5bf719f6b10eafcf736de9956c96e9d035d23726`. Hosted CI run `36601370908`
passed all 3,883 tests in 265.42 s and merged at
`250f7343f9fbb663e165cb2872ef543027419428`, retaining tree
`30e080436a33195c6c513ea6ffc81ca64f97892b`. The execution source and frozen
protocol match the original experiment after checkout recovery. The dated
[experiment record](2026-09-29-combined-supported-prior.md) preserves that recovery.

Before this implementation, 91 warning-strict campaign tests passed in 8.23 s
and 31 existing diagnostic tests passed in 1.55 s. The independent archived
checker passed all 25 combined flights. ADR 0038 was frozen while the preceding
PR's CI was running; its merge completed before diagnostic execution.

One new experiment module authenticates the original 34, boundary-only 25 and
combined 25 saved outcomes, rescores every flight, and reconstructs all 27 clean
histories. It independently checks the full joint measurement correction,
covariance reset and sample-noise mean, records effective gains and innovations,
reconstructs requested acceleration, and retains signed sampled-response budgets.
The existing unfitted cascade model is applied to all nine hover histories.
No production file, gain, noise model, dependency or default changed. No new
scientific flight, oracle or fitted coefficient was introduced.

Nine new tests cover hand-computable covariance/gain changes, rejection, ordered
updates, NED feedback signs, cancellation, different durations, malformed inputs
and report tampering. Initial collection failed because the module did not yet
exist. The first implementation passed eight tests; an extreme 1,000 m rejection
fixture exceeded the frozen absolute NIS arithmetic bound. A still-rejected 1 m
fixture tests the same rejection behavior within the diagnostic's numerical
scale. No acceptance bound changed. Type checking also caught an incorrect
noise-pairing call signature, repaired before any diagnostic execution.

The combined focused group passed 51 tests in 3.61 s. The full warning-strict
suite passed 3,892 tests in 409.14 s. Ruff, formatting, mypy and documentation
checks passed. Complete diagnostic execution took 449.86 s using two processes.

## Independent numerical checks

All 84 source outcomes authenticate and rescore. All 27 clean histories match
their complete saved replay records, including state/covariance/events,
controller, guards, plant/motors, noise and first-prediction traces. Across
13,608 observations, maximum independent differences are:

| Check | Maximum absolute difference | Frozen limit |
| --- | ---: | ---: |
| Joint correction, first 15 coordinates | 1.041e-17 | 1e-10 |
| Full joint covariance after reset | 2.602e-18 | 1e-10 |
| Sample-noise posterior mean | 1.323e-22 | 1e-10 |
| NIS | 5.329e-15 | 1e-9 |
| Reconstructed controller command | 0 | 1e-12 |
| Acceleration identity | 3.187e-15 | 1e-11 |
| Position/velocity response identity | 4.663e-15 | 1e-10 |

Gain/innovation and requested-acceleration differences close within 1e-12.
Signed MSE shares and full-duration score changes close within 1e-12 m².
An additional NumPy-only artifact checker independently recovers the saved
gain/correction identities, response sums, MSE changes and model discrepancies
for all nine jobs. It does not replace the complete source-based replay above.

## What changes and why the outcomes differ

For hover seed47001, the first position observation at 0.2 s has horizontal
velocity-gain diagonal entries 0.13536 and 0.13506 per second in the boundary
control. The combined prior reduces these to 0.000652 and 0.000347 per second.
The known zero release velocity reduces early measurement-noise corrections to
velocity. Changed cross-covariances and subsequent innovations also redistribute
corrections among velocity, attitude and bias coordinates.

This includes a discrete gate change. At 1.6 s the hover control NIS is 10.5778
(10.5309 in the original) and the combined NIS is 14.8480, against the unchanged
position threshold 11.345. The controls fuse the observation and the combined
candidate rejects it. The same disposition difference occurs once in each of
the three seed47001 clean missions: three unique events, six pair comparisons.
There are no other paired disposition changes. The experiment therefore cannot
be described solely as a smooth gain change. No gate was retuned.

The table decomposes the change in full-flight tracking MSE against the
boundary control. Units are cm². Negative values improve that signed budget;
they are correlated pathwise shares, not independent intervention effects.

| Mission / seed | Navigation position + velocity | Attitude estimation | All other channels | Total MSE change |
| --- | ---: | ---: | ---: | ---: |
| Hover 47001 | -6.3633 | -3.3874 | -0.2351 | -9.9858 |
| Nominal 47001 | -5.6362 | -3.0271 | -0.1143 | -8.7777 |
| Wind 47001 | -5.5245 | -4.1594 | -1.1822 | -10.8661 |
| Hover 47002 | -0.4187 | +1.1521 | -0.4034 | +0.3300 |
| Nominal 47002 | -0.3624 | +0.9299 | -0.2958 | +0.2717 |
| Wind 47002 | -0.3758 | +0.4418 | -0.7389 | -0.6729 |
| Hover 47003 | +2.5697 | +0.7651 | +0.2103 | +3.5452 |
| Nominal 47003 | +1.8785 | +0.6537 | +0.0218 | +2.5541 |
| Wind 47003 | +1.4858 | +1.4575 | +0.4734 | +3.4166 |

Seed47001 benefits in both principal channels. Seed47002's smaller navigation
contribution is outweighed by increased attitude-estimation contribution in
hover and nominal tracking. Seed47003 increases both. The preserved original
comparison is also retained in full; an intermediate control cannot hide an
earlier regression. The original 8 cm peak and full-flight limits are unchanged.

## Unfitted model and limits

The nine hover histories have horizontal RMS model discrepancies of
0.145844..0.768575 mm, all below the frozen 2 mm explanatory bound. The model's
combined-minus-boundary trajectory discrepancy is 0.261436, 0.137803 and
0.106543 mm for seeds 47001, 47002 and 47003 respectively. This agreement supports
the explanation through the existing sampled feedback dynamics, without fitting.

The model consumes saved estimation errors. It is not an independent prediction
of future sensor noise, a nonlinear stability proof or a new controller. The
exact response budget also retains physical forcing histories and integration
remainders. Neither method licenses subtracting a component to claim an
achievable flight. These finite known seeds do not establish population
optimality or fresh qualification. Normalized innovation rejection is a valid
branch under the unchanged policy, not evidence by itself of a sensor defect.

## Decision and next exact action

Both exact diagnostic acceptance and the unfitted model check pass. No specific
correctable implementation defect or independently justified further startup
variant was demonstrated. Close the startup study with the failed ADR 0037
comparison preserved. Keep the implemented combined option experimental and
retain current production defaults. Do not run another startup gain/seed search.

Next audit the remaining whole-flight estimated-feedback requirements and derive
an estimation/control error budget from the existing sensor model and sampled
cascade sensitivities. Separate navigation, attitude and bias contributions,
correlations and gate discontinuities. Freeze that design review before any
implementation; a new flight candidate requires a specific, independently
justified measurement or feedback hypothesis. Fresh qualification, mass-offset,
geometric-control and integration requirements remain open.

## Commands and evidence identities

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m pytest -q tests/unit/test_combined_prior_diagnostic.py tests/unit/test_attitude_startup_audit.py tests/unit/test_residual_hover_diagnostic.py tests/unit/test_combined_supported_prior.py
.venv/bin/python scripts/check_docs.py
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m pytest -q
```

The guide supplies the diagnostic execution command. The full saved replay
mode is available for reproduction; the publication run used complete source
reconstruction once and then the independent artifact checker, without repeating
the 27 histories solely to exercise that command-line mode.

| Item | SHA-256 |
| --- | --- |
| Preceding execution source | `0073d6024a5fea509fe3aaf2dca06e3c5b835b708eb637136c0043833926a12b` |
| Diagnostic execution source | `01d663368ed67174c9b300fdd2ef7450781da5f6ec3947e7f9a9b2d8473f2b0e` |
| Frozen ADR 0038 | `60cfde1c06e504f390133093f7fd3753b4cc31c3b5667d648f1eeee05fb843a5` |
| Complete diagnostic report | `09c53c8e86f94f16b685fa6f297e54ecd4bcc3a7e78cb7c806217ed020910276` |

The evidence bundle retains all update/response arrays, comparisons, complete
source reports, test/CI receipts, figure, independent verifier and payload
manifest outside Git. The original full flight archives remain authoritative
inputs rather than duplicated or regenerated trials.
