# Final geometric improvement and comparison — 2026-09-30

Audited baseline: `5eb932766db6fc3ef5f465bf551e390633561d43`, merged PR #41.
The local tree was clean and remote main agreed. The existing supported-start
study remains intact, including its failed strict wind comparison and unopened
conditional validation. The new request authorizes one final bounded improvement
attempt, followed by a pause; professor-report preparation remains a later task.

## Audit, diagnosis and frozen scope

Fresh baseline verification:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_supported_geometric_comparison.py tests/unit/test_geometric_control.py tests/unit/test_geometric_filter.py tests/unit/test_geometric_transient_study.py
```

Result: **73 passed in 20.15 s**. Source, saved metrics and documentation agree.
The baseline development report has SHA-256
`fa93de167c3b788e711655023906bf056c0a26b9caf91820eb8511b4ac4aa0e3`.

Saved-array phase decomposition locates the nominal geometric peak at 0.7175 s:
15.5949 cm total error, including 15.1012 cm vertical error. Cascade's peak is
9.0226 cm. Geometric nominal horizontal RMS over 5..15 s is 2.2031/1.0680 cm,
against cascade 5.1176/2.4745 cm. This identifies delayed vertical release
correction as a targeted opportunity while retaining the useful trajectory
feedforward. It does not establish an estimator implementation defect.

[ADR 0042](../../decisions/0042-final-geometric-comparison.md) was frozen before
new implementation or scientific flights. Its exact SHA-256 is
`df78a8e97b863bf956bb42b3a50bf45f0dcb9e4e607790c49fcaa254e0cf9189`.
The six-profile search, independently selected cascade, absolute limits,
balanced metrics, effort budget and later seed ledger are prospective.
ADR 0041's result is not reclassified.

## Implemented change and mathematical verification

The new experimental builder uses third-order force filters with a fixed
30 rad/s vertical pole and horizontal poles 10, 15 or 20 rad/s. It constructs
the force value and both derivatives from the same axis-dependent filter state.
The original analytic force-to-rotation map and geometric moment law remain
unchanged. Two declared outer-loop frequencies, 1 and 1.25 rad/s, are offered to
both controller families. Original vertical/inner gains, noise, mission timing,
limits and all production defaults remain unchanged.

An independent discrete transfer-function calculation checks the shaped force
and both derivative jets using the bilinear substitution. Isotropic compatibility,
constant-prehistory reset, invalid poles, sample order, domain rejection and
transactional memory are checked. Central differences of the full nonlinear
motor/rigid-body plant independently check the horizontal and vertical sampled
maps. Horizontal spectral radii range from 0.97419 to 0.98256; the vertical
radius is 0.97839. These are local deterministic maps with estimator errors as
external forcing, not poles or a stability certificate of the complete ESKF loop.

New targeted verification:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_axis_shaped_geometric.py tests/unit/test_final_geometric_comparison.py
```

Result: **35 passed in 17.41 s**. Short seed-40 software fixtures additionally
save and authenticate histories, replay every estimated/control/plant epoch,
verify release quadrature and test tamper rejection. Synthetic ledgers check
independent selection, category weighting, dictionary-order independence,
fail-closed decisions and blocking fresh fixtures after a failed parent gate.
Short fixtures do not count as scientific performance outcomes.

Preflight lint and formatting pass; strict typing checks 104 source files.
An initial narrow mypy invocation omitted the project's source root and produced
import-context errors; the prescribed `mypy src experiments scripts` passes.
Formatting and type-annotation issues were resolved before scientific execution.
There were no failing mathematical or flight-pipeline tests.

The frozen execution-source SHA-256 is
`18af27137bd56d4328bf3227d901d84d119e77e341b72f3951d5a0352ce8fbe2`.

The complete warning-strict software regression runs concurrently with the
scientific campaign, without source changes:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
```

Result: **4,000 passed in 408.70 s**; lint, formatting and typing pass. The
source fingerprint is unchanged after the full suite. Software tests verify
implementation behavior; scientific flight metrics determine performance.

## Complete development results

All 36 declared flights complete, meet the original absolute conditions, and pass complete saved replay. The original geometric and cascade controls reproduce the preceding study to numerical precision. No configuration, gain, scoring rule or source file changed during the campaign.

| Profile | Hover 30 RMSE [cm] | Hover 93012 RMSE [cm] | Nominal RMSE [cm] | Wind RMSE [cm] | Balanced RMSE / selected cascade | Maximum effort / selected cascade |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| historical-geometric | 3.4762 | 3.1370 | 4.1150 | 6.8739 | 1.193434 | 0.783391 |
| axis-h10-f1 | 3.2979 | 2.9694 | 3.7599 | 6.6371 | 1.124281 | 0.792812 |
| axis-h15-f1 | 3.2421 | 2.9121 | 3.6932 | 6.5813 | 1.107682 | 1.446177 |
| axis-h20-f1 | 3.2223 | 2.8894 | 3.6686 | 6.5576 | 1.101327 | 2.669382 |
| axis-h10-f1.25 | 2.7790 | 2.7048 | 3.1836 | 5.0525 | 0.929297 | 1.122467 |
| axis-h15-f1.25 | 2.7189 | 2.6475 | 3.1165 | 5.0049 | 0.913202 | 2.514150 |
| axis-h20-f1.25 | 2.6976 | 2.6261 | 3.0941 | 4.9865 | 0.907474 | 4.951279 |
| cascade-f1 | 3.2159 | 2.8506 | 5.0824 | 6.6268 | 1.228858 | 0.873086 |
| cascade-f1.25 | 2.7187 | 2.6259 | 4.0498 | 5.0784 | 1.000000 | 1.000000 |

The fixed selection chooses **axis-h10-f1.25** and **cascade-f1.25**. Both use horizontal Kp=1.5625 s^-2 and Kv=2.25 s^-1; vertical gains remain 2.25 s^-2 and 3 s^-1. The geometric filter poles are (10,10,30) rad/s, with unchanged kR=0.64 and kOmega=0.32. Faster filters at the higher gains reduce error slightly more but exceed the per-flight effort budget; they remain visible in the table.

Against the independently selected cascade, balanced development RMSE is **7.0703% lower**, with balanced effort **1.6425% lower**. Nominal trajectory RMSE improves **21.3885%**; wind improves **0.5091%**; average hover RMSE is **2.6112% higher** under the declared geometric-mean ratio. Whole-flight peaks remain **14.1684% higher** on the balanced ratio. This is an accuracy/effort tradeoff, not universal metric dominance.

Against the original cascade, balanced development RMSE is **24.3772% lower**. Independently increasing the cascade gains improves its balanced score by **18.6236%**. Giving both controller families the gain choice materially strengthens the comparison.

The ablation separates the implemented mechanism from gain tuning. Changing
only the vertical filter from 10 to 30 rad/s (`axis-h10-f1`) lowers RMSE in all
four known cases, with a **5.7945% balanced improvement over historical geometric**.
The selected profile improves balanced RMSE by **22.1325% over historical
geometric**; its four whole-flight peaks fall by 30.34..34.02%. These are paired
development results for an actual controller change. The later fresh comparison
is still necessary to assess the selected profile independently.

The 5% development opportunity gate passes. These are known development seeds and cannot establish fresh performance. The complete development report SHA-256 is `b2dc09a00206fbc1f6092c78ed6504a15879a17e8c4546fa1b5504371c0ed86a`.

The tested implementation and frozen protocol are published before later checks in commit `c97d4786f6bb729cddd2b1af95d65410f4d12eca`, tree `be0e68caf005a589faaceae37360ae2816bc976d`. A subsequent adapter correction is recorded below; the force-shaping and production algorithms remain unchanged.

## Corrected regression adapter, without retuning

The first true-state regression attempt exposes a driver defect. Its seven
geometric flights complete and pass physical conditions. Its seven cascade
calls raise `TypeError` before simulation because the estimated-only
`allow_minimum_snap` keyword was forwarded to `simulate_mission`. The resulting
report fails closed; no fresh configuration is constructed. Preserve this
failed ledger, SHA-256
`85d27d58853f605c334acda9411b3cb49a7c05180ba7bc18c1d4c8ee2565b7ab`.

[ADR 0043](../../decisions/0043-geometric-regression-adapter.md) freezes the narrow
correction before implementation, with SHA-256
`7031505e65738c826a5082c50d670e893ae903de63f2a84b3a2f17f0e73f1440`.
Only estimated configurations receive the permission. The controller laws,
filter, gains, selection, acceptance criteria, planned cases and stopping rule
do not change. A new short true-state pipeline test covers both controller
families; compatibility tests reject changed report bytes or changed algorithms.
All **37 targeted tests pass in 18.02 s**; lint, format and strict typing pass.

The exact historical development report remains a valid parent because that
estimated execution path is unaffected. The corrected adapter authenticates all
296 referenced payloads, recomputes the same selection and reconstructs all 36
saved configurations identically. A separate hash binds 103 unchanged execution
algorithm/dependency files, excluding the corrected driver:
`77a2c1b140111142412e316b74cd3b1a508ce375e04d9ec1d9226f35be28b75d`.
No generic source-mismatch exception is introduced; only the exact recorded
historical report is admitted under this correction.

The corrected driver is published in commit
`6e79406b8bc75be50ce4304dd7d403f3c751c5a2`, tree
`87ac9bc744fc2ebd97296c22176002ca7132463a`. Its execution-source fingerprint is
`41bb1bc1868e5d0f1c34dfb76dfa731aba4ab8f6c5d87b32251c0aa700f59a3b`.
Repeat all fourteen true-state flights in a separate corrected directory.
The earlier seven geometric flights are repetitions, not additional independent
trials, and the seven rejected API calls are not flights. The failed first
regression report cannot authorize fresh validation.

## Corrected true-state regression

All fourteen corrected flights complete and pass the original physical limits.
The seven repeated geometric scores match the first attempt exactly. Both
controllers retain the selected development gains throughout.

| Case | Geometric RMSE [cm] | Selected cascade RMSE [cm] |
| --- | ---: | ---: |
| Nominal | 0.222473 | 2.335559 |
| Refined integration | 0.222473 | 2.335559 |
| Positive initial offset | 0.841686 | 2.430243 |
| Negative initial offset | 0.835085 | 2.428886 |
| Mild wind | 2.401784 | 2.668002 |
| Reversed wind | 2.492201 | 2.530374 |
| Wind and initial offset | 2.562606 | 2.775275 |

Common-clock refinement differs by at most `2.08253e-10 m` position and
`2.41484e-6 degrees` attitude, below the frozen 5 mm and 0.05 degree caps.
The corrected regression report SHA-256 is
`eec1f5238492512de99fec6434da02ce9cc6d604e95739f72a899c5b0c214dde`.
This accepted parent opens the reserved validation cases. The earlier failed
report is never used as that parent.

The compatibility unit test uses a synthetic algorithm-hash fixture so future
unrelated repository changes do not fail a historical boundary test. The actual
scientific parent check still computes and verifies the complete hash. The final
fixture adjustment passes its targeted test in 0.19 s; executable source remains
unchanged. The earlier 4,000-test full run predates the adapter correction;
the final published tree requires its own complete CI check.

Saved software provenance is Python 3.12.14, NumPy 2.5.2 and package 0.1.0.
Development records baseline commit `5eb9327` with a dirty worktree containing
the new implementation; its exact execution fingerprint is `18af2713...` and
that code was subsequently published as `c97d478`. Corrected regression records
`6e79406` with a dirty worktree because documentation is being completed. Its
execution fingerprint is `41bb1bc1...`. The protocol hashes bind executable
content independently of documentation and the dirty-worktree flag.

The complete warning-strict suite was rerun after the adapter correction and
final compatibility-test fixture adjustment, using the same full-suite command
above. Result: **4,002 passed in 421.37 s**. The log is
`corrected-full-tests.log`; the earlier 4,000-test log is retained separately.
This closes local full-suite verification of the corrected execution source.
Publication still requires the repository's full CI gate on its final tree.

## Reserved hover results

All twelve hover flights pass their absolute conditions and complete saved
reconstruction. Every hold window includes all samples from 5 through 65 s.

| Seed | Geometric hold peak [cm] | Selected cascade hold peak [cm] | Original cascade hold peak [cm] |
| --- | ---: | ---: | ---: |
| 95000 | 5.0791 | 5.1015 | 5.7738 |
| 95001 | 5.4252 | 5.4730 | 6.2808 |
| 95002 | 4.8540 | 4.8892 | 5.4384 |
| 95003 | 5.7295 | 5.7604 | 6.2296 |

Geometric hold peaks are slightly lower than the selected cascade in all four
cases, while whole-flight RMSE is 1.6100% higher on the paired geometric mean.
Whole-flight peaks also remain higher. Distinguishing the hold window from the
complete takeoff/hold/landing history is necessary to describe this tradeoff.

## Complete reserved comparison and decision

All 52 declared reserved executions finish and pass saved reconstruction: 36 clean controller-arm flights and sixteen supervisor off/on fault flights. Both selected controllers pass all twelve clean absolute conditions. The original cascade passes eleven; its wind seed 96005 final position error is 8.01836 cm, 0.1836 mm above the 8 cm limit. Preserve this small secondary-comparator miss without overstating it.

| Case | Seed | Geometric RMSE [cm] | Selected cascade RMSE [cm] | RMSE improvement [%] | Geometric peak [cm] | Cascade peak [cm] | Effort ratio |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| hover | 95000 | 2.7979 | 2.7520 | -1.6658 | 10.8884 | 9.2477 | 1.133097 |
| hover | 95001 | 3.1885 | 3.1181 | -2.2591 | 12.6944 | 10.8840 | 1.001487 |
| hover | 95002 | 3.0475 | 2.9836 | -2.1429 | 13.1376 | 11.5851 | 1.100322 |
| hover | 95003 | 3.2362 | 3.2239 | -0.3832 | 10.2376 | 8.6406 | 1.196242 |
| spline | 96000 | 3.2622 | 4.0106 | 18.6607 | 10.8297 | 9.4665 | 0.978828 |
| wind_spline | 96001 | 4.5857 | 4.5319 | -1.1878 | 10.8941 | 9.7331 | 0.977940 |
| spline | 96002 | 4.0343 | 4.9609 | 18.6789 | 11.4290 | 9.8330 | 1.050060 |
| wind_spline | 96003 | 3.4685 | 3.3912 | -2.2809 | 10.6803 | 9.2860 | 1.013705 |
| spline | 96004 | 4.6468 | 5.0283 | 7.5874 | 12.1933 | 10.8204 | 0.979459 |
| wind_spline | 96005 | 4.9405 | 5.3549 | 7.7384 | 12.0718 | 10.3979 | 0.979048 |
| spline | 96006 | 3.5809 | 3.9635 | 9.6538 | 9.6418 | 8.3179 | 0.892293 |
| wind_spline | 96007 | 4.8399 | 4.8920 | 1.0634 | 12.8368 | 11.1522 | 0.937145 |

Positive improvement percentages favor geometric control. The category-balanced RMSE ratio is **0.9522828816553321**, a **4.7717% improvement** over selected cascade. Nominal tracking improves **13.7942%**, wind improves **1.4117%**, and hover whole-flight RMSE is **1.6100% higher**. Every nominal case improves; two wind cases improve and two regress. Maximum individual RMSE regression is **2.2809%**.

Balanced whole-flight peaks are **15.2880% higher**, while actual squared-moment effort is **1.6736% higher**. The largest individual effort ratio is **1.196242**. Peak and effort dominance therefore do not hold. Candidate whole-flight peak never exceeds **13.1376 cm**, and final error never exceeds **6.67695 cm**. Every clean final error, speed, hold peak, effort and limit flag is retained in the report and readable evidence index.

Against the original cascade, balanced RMSE is **20.2899% lower**, balanced peaks **7.1598% higher**, and effort **23.8136% higher**. The independently selected cascade is the primary benchmark; the larger secondary gain must not replace it.

The fixed 20,000-resample stratified bootstrap yields RMSE-ratio interval **[0.9297889076, 0.9740108282]**, or **2.60..7.02% improvement**. This is descriptive sensitivity within four sampled cases per category. It is not another gate, a broad qualification statement or a theorem of controller superiority.

| Frozen primary condition | Outcome |
| --- | --- |
| balanced_accuracy | Fail |
| balanced_effort | Pass |
| cascade_physical | Pass |
| category_accuracy | Pass |
| faults | Pass |
| geometric_physical | Pass |
| individual_accuracy | Pass |
| individual_effort | Pass |

The complete frozen performance gate **fails only its 10% balanced-improvement target**. It does not fail because the measured gain is zero, because one selected clean flight violates an absolute limit, or because a fault/audit failed. The implemented controller demonstrates a modest average accuracy advantage with peak-error and effort costs. Its development advantage was 7.0703%; the fresh advantage is smaller.

## Complete fault-response results

| Fault | Supervisor off terminal time [s] | Supervisor on terminal time [s] | Response comparison |
| --- | ---: | ---: | --- |
| position_dropout | 17.5000 | 6.8025 | Pass |
| position_rejection | 17.5000 | 6.8025 | Pass |
| position_delay | 17.5000 | 6.8025 | Pass |
| altitude_dropout | 17.5000 | 6.2450 | Pass |
| altitude_rejection | 17.5000 | 6.2450 | Pass |
| altitude_delay | 17.5000 | 6.2450 | Pass |
| position_recovery | 17.5000 | 17.5000 | Pass |
| landing_position_dropout | 17.5000 | 14.0050 | Pass |

Persistent faults abort at their first required expiry with identical causal prefixes and no later command. Brief position recovery completes in both modes. All eight comparisons pass. A supervised abort is the intended numerical response, not mission completion or a demonstrated physical emergency maneuver.

## Execution, authentication and reproducibility

In the commands below, `$EVIDENCE` denotes the external evidence directory; the executed absolute path has been factored out for reuse. Every output directory was new. The first regression used `regression`; the corrected run used `regression-corrected`.

```bash
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error .venv/bin/python -u -m experiments.final_geometric_comparison --stage development --workers 2 --output "$EVIDENCE/development"
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error .venv/bin/python -u -m experiments.final_geometric_comparison --stage regression --workers 2 --development "$EVIDENCE/development" --output "$EVIDENCE/regression"
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error .venv/bin/python -u -m experiments.final_geometric_comparison --stage regression --workers 2 --development "$EVIDENCE/development" --output "$EVIDENCE/regression-corrected"
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error .venv/bin/python -u -m experiments.final_geometric_comparison --stage validation --workers 2 --development "$EVIDENCE/development" --regression "$EVIDENCE/regression-corrected" --output "$EVIDENCE/validation"
OPENBLAS_NUM_THREADS=1 .venv/bin/python "$EVIDENCE/independent_rescore.py" "$EVIDENCE"
```

Development and corrected regression return exit 0. The first regression returns 1 because of the documented adapter defect; fresh validation returns 1 because of its preserved performance decision. The complete fresh ledger has zero execution or reconstruction errors. No gain, filter, limit, seed, scoring rule or execution source changes during corrected regression or validation. The final execution fingerprint remains `41bb1bc1868e5d0f1c34dfb76dfa731aba4ab8f6c5d87b32251c0aa700f59a3b`.

Independent saved-array arithmetic authenticates **723 payload references** and rescores **109 scientific executions**, agreeing to a maximum of **1.3877787807814457e-17** in SI units. The total is 36 development + 14 corrected regression + 52 fresh + seven repeated geometric flights from the first regression attempt. Seven rejected API calls are preserved separately and are not flights. There are **twelve independent fresh performance cases**, not 109 independent trials.

All **88 estimated executions** receive complete saved estimator/noise/controller/plant reconstruction. The **21 true-state executions**, including seven repeats, receive payload authentication and independent saved-array rescoring. The separate `source-provenance.json` reconstructs both historical execution fingerprints from the published Git objects.

| Evidence | SHA-256 |
| --- | --- |
| `development/report.json` | `b2dc09a00206fbc1f6092c78ed6504a15879a17e8c4546fa1b5504371c0ed86a` |
| `regression/report.json` | `85d27d58853f605c334acda9411b3cb49a7c05180ba7bc18c1d4c8ee2565b7ab` |
| `regression-corrected/report.json` | `eec1f5238492512de99fec6434da02ce9cc6d604e95739f72a899c5b0c214dde` |
| `validation/report.json` | `6d08f21ee80453d26728544e4f803f5fdab8cc49bf416b7665d4c589ad4612bd` |
| `independent-scores.json` | `27d98f7fb59913f322046bd1ade214c4b9f0a56910d0e2eb0b746d28a74791eb` |
| `corrected-full-tests.log` | `8eab4156cc5df6e55933e664be2091c416e99ecf651d2af53f3304a27a5c5302` |

The evidence uses `Quadrotor_Final_Geometric_Results_2026-09-30.zip` plus independently extractable development, regression, corrected-regression and validation data ZIP parts. Extract them into the same directory to restore `final-geometric/`. The results archive contains the readable complete metric index, all reports/protocols/configurations, logs, independent rescoring tools and an archive/member SHA-256 manifest. Data parts retain every saved history and large diagnostic. CRC and member-path checks precede persistence. Generated scientific data and repository source copies are excluded from Git.

Final local checks: **4,002 tests pass in 421.37 s**, ruff lint passes, formatting checks **358 files**, and strict typing checks **104 source files**. Production `src`, `pyproject.toml` and `uv.lock` remain byte-identical to the audited baseline. Final documentation/link verification and the repository CI `make check` gate accompany publication.

## Closeout and next exact action

The implementation, bounded comparison and evidence-preservation scope is complete. The stronger frozen 10% performance claim is not met. Retain the new axis-dependent controller as an explicit reproducible experiment, with its real measured gain and full tradeoffs. No controller is established as universally superior, and this finite study cannot establish that further improvement is impossible.

Pause controller improvement now. The next requested task, for later, is preparing the professor-facing report. Start by auditing the published source and these complete records, then update the existing editable report to explain the new mathematical mechanism, development selection, fair cascade comparison, fresh gains, failed stronger claim, assumptions and remaining questions. Report revision 3.0 is unchanged here. No new tuning, sensor, mass-integral combination, middleware integration, report revision or message to the professor is part of this closeout.
