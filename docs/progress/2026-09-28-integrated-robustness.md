# Integrated robustness evaluation — 2026-09-28

## Scope and audit

I audited merged `90fae0b1cc082f4061c89394adb0d15765a484a5`, tree
`677f23b2166fb02e21d49231d8b9e3d27bfdc149`, before implementing this step.
The preceding [supervisor](2026-09-28-observation-loss-supervision.md) passed
its post-merge CI and 136 fresh relevant tests in 63.93 seconds. No demonstrated
in-scope defect required a repair. I froze
[ADR 0023](../decisions/0023-integrated-robustness-evaluation.md) before
implementation and execution.

The new optional live delivery adapter supplies measured-value dropout, offset
and integer-epoch delay before the online ESKF. Its source ledger is reconciled
against the existing offline injector. Existing controller, estimator, health
and supervisor laws are unchanged. The [guide](../integrated-robustness.md)
explains pairing, evidence reconstruction and interpretation.

## Software checks

The 36 live-boundary tests passed in 7.21 seconds. The combined 57 new boundary
and campaign tests passed in 59.27 seconds with warnings treated as errors.
They check empty-fault byte parity, causal/out-of-order delivery, invalid-call
atomicity, pending/dropout accounting, exact ESKF replay, complete planned-case
retention and rejection of altered claims and reauthenticated diagnostics or
controller commands. Strict mypy passes all 72 source files; lint and formatting
pass. The complete warning-strict gate passed **3,523 tests in 873.96 seconds**
(57 more than the audited baseline), including documentation, lint, formatting
and strict typing. Exact full-gate command:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
```

Exact focused commands, with pinned uv 0.12.3:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run pytest -q tests/unit/test_observation_health.py tests/unit/test_observation_supervision.py tests/unit/test_observation_supervision_mission.py tests/unit/test_eskf_faults.py
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run pytest -q tests/unit/test_eskf_live_faults.py
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run pytest -q tests/unit/test_eskf_live_faults.py tests/unit/test_robustness_validation.py
```

The preliminary smoke campaign retained four normal numerical executions,
two passing response cases and zero passing flight cases out of one required.
The stationary nominal fixture reached `landing_timeout`; final true speed
was 0.09430 m/s, below the report's 0.15 m/s limit but above the mission's
unchanged 0.08 m/s completion tolerance. This valid failed outcome is exercised
by the saved-report tests. It is not maneuver qualification. The first smoke
archive predates the final source-fingerprint check; the focused tests created
and verified new smoke evidence against the final source.

## Maneuver protocol and evidence

```bash
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.robustness_validation --partition campaign --workers 2 --output results/robustness-campaign
```

The runner uses original cascade parameters, seeds 30/31, declared original
priors, 400/100/50 Hz plant/attitude/position clocks, 5 Hz position and 25 Hz
altitude observations. Each supervision-off comparator is an actual simulation
with the same fault plan and noise as its supervision-on partner. No source,
policy or threshold is changed while the campaign runs. All histories are
saved before their independent estimator/control/decision reconstruction.

The execution-source SHA-256 is
`93660aba17673c62f8e0d6086ad70438cc291e252e7e4cbedad85175bef58b20`.
It covers core/experiment Python sources, `pyproject.toml` and `uv.lock`.
Only documentation changes follow the campaign's source freeze. The Git
provenance records the audited base with a dirty working tree; the final
published tree contains the fingerprinted implementation.

The nominal pairs preserve complete saved payloads. Both complete their virtual
flights, at 15.5 s for hover and 17.5 s for tracking. Hover RMSE is 0.076605 m,
final error 0.039518 m and final true speed 0.015916 m/s. Its 5..11 s hold peak
is **0.107563 m at 5.395 s**, exceeding the frozen 0.08 m limit. Tracking RMSE
is 0.062445 m, final error 0.057363 m and final speed 0.018551 m/s; it passes
its declared flight criteria. Neither case produces an observation abort.

All six persistent tracking-fault combinations pass their declared response
criteria. Position dropout, rejection and delay each detect unhealthy data at
6.202500000000001 s and abort at 6.8025 s, 0.8025 s after onset. Their last
inner and outer command is at 6.8 s. Altitude dropout, rejection and delay each
detect degradation at 6.0425 s and abort at 6.245 s, 0.245 s after onset; their
last commands are at 6.24 s. These are the first actual sampled epochs at/after
the corresponding floating-point deadlines, within the frozen 1.0075/0.2875 s
ceilings. No terminal command is issued. The full saved pre-abort state and
command prefixes match their unsupervised comparators exactly.

The supervised position cases reach only five of ten planned fault sources
before termination; the ledger retains the unacquired count rather than
pretending the whole two-second fault window was flown. A correct numerical
abort is not a successful flight completion or a demonstrated physical recovery.

The brief position dropout degrades at 6.202500000000001 s and confirms health
at 6.6000000000000005 s, before expiry. Both modes remain payload-identical and
complete at 17.5 s, with RMSE 0.066013 m, final error 0.057608 m and final speed
0.018274 m/s. It passes both response and flight criteria.

The landing dropout begins at 13.2 s, degrades at 13.405000000000001 s and
aborts at 14.005 s, a 0.805 s onset-to-response delay. Its unsupervised
comparator completes at 17.5 s. The supervised vehicle still has true speed
0.271286 m/s when numerical execution stops. This is evidence of the stated
abort policy during LAND, not a safe physical landing demonstration.

## Complete campaign outcome

All **12/12 response cases pass**, with **24/24 numerical executions** and
**zero numerical failures**. Only **3/5 applicable flight cases pass**. The
campaign therefore returns the documented failed-result exit status 1, with
its complete report saved. There was no infrastructure or evidence exception.
All 24 histories were authenticated and reconstructed by the runner before
report publication; the separate report-verification CLI is covered by the
smoke and tampering tests, not an additional repeat of this full campaign.

| Case | Supervision off | Supervision on | Common-horizon tracking RMSE (m), both modes | Response | Applicable flight gate |
| --- | --- | --- | ---: | --- | --- |
| nominal_hover | `complete` at 15.5 s | `complete` at 15.5 s | 0.076605 | pass | FAIL |
| nominal_tracking | `complete` at 17.5 s | `complete` at 17.5 s | 0.062445 | pass | pass |
| position_dropout | `complete` at 17.5 s | `observation_local_position_timeout` at 6.8025 s | 0.085657 | pass | expected abort |
| position_rejection | `complete` at 17.5 s | `observation_local_position_timeout` at 6.8025 s | 0.085657 | pass | expected abort |
| position_delay | `complete` at 17.5 s | `observation_local_position_timeout` at 6.8025 s | 0.085657 | pass | expected abort |
| altitude_dropout | `complete` at 17.5 s | `observation_barometric_altitude_timeout` at 6.245 s | 0.087561 | pass | expected abort |
| altitude_rejection | `complete` at 17.5 s | `observation_barometric_altitude_timeout` at 6.245 s | 0.087561 | pass | expected abort |
| altitude_delay | `complete` at 17.5 s | `observation_barometric_altitude_timeout` at 6.245 s | 0.087561 | pass | expected abort |
| position_recovery | `complete` at 17.5 s | `complete` at 17.5 s | 0.066013 | pass | pass |
| landing_position_dropout | `complete` at 17.5 s | `observation_local_position_timeout` at 14.005 s | 0.067546 | pass | expected abort |
| wind_tracking | `complete` at 17.5 s | `complete` at 17.5 s | 0.067174 | pass | pass |
| mass_tracking | `landing_timeout` at 25 s | `landing_timeout` at 25 s | 0.423933 | pass | FAIL |

All five nonpersistent pairs, including recovery and both model-mismatch cases,
have byte-identical corresponding saved NPZ parts with supervision off/on.
Every persistent pair has an exact shared state/command prefix. These results
support correct intervention and absence of unintended nominal behavior; they
do not show a tracking improvement from supervision itself.

Wind passes: full-run RMSE is **0.067174 m**, final true error **0.080216 m** and
final true speed **0.019628 m/s**. The 10% mass mismatch fails in both modes:
`landing_timeout` at **25 s**, RMSE **0.423933 m**, final error **0.434634 m** and
final speed **0.023486 m/s**. Its final NED vertical error is **+0.433019 m**;
common-horizon position-estimation RMSE is only **0.018065 m**. Observations
remain usable and the supervisor produces no false abort. The unchanged
position controller is the next diagnostic target.

For level stationary unsaturated flight with exact estimates, the existing
nominal-mass PD law predicts

$$
e_z = \left(\frac{m_{true}}{m_{nominal}}-1\right)\frac{g}{K_{p,z}}
    = \frac{0.1\times9.81}{2.25}=0.436\ \mathrm{m}.
$$

This equilibrium calculation is consistent with the measured vertical offset;
it is not a complete explanation of every noisy transient. The controller
needs a position error to supply the additional steady force because it has no
integral disturbance state. The separate next scope must check the complete
estimator/actuator history and earlier integral probes before changing that law.

## Evidence and publication

The campaign uses Python 3.12.14, NumPy 2.5.2 and pinned uv 0.12.3 without any
dependency or tool-version change. Its protocol digest remains the pre-run
`7a686a341305a6a58a0afcf5d20cb4929844165165f7aa70bc11679d99843c30`.
The smoke/campaign protocol digests also match when calculated under the tested
Haswell and Sandybridge OpenBLAS kernels. Production replay remains exact and
requires a matching numerical environment; this check does not loosen it.

The full campaign report, frozen protocol, all sensor/estimate/covariance/
command/event histories, exhaustive source/fault ledgers and diagnostic traces
are retained outside Git with the execution logs. The evidence archive records
the tested source commit and fingerprint. Generated payloads are not committed.
The guides, design record and this numerical summary are versioned with source.

Before merge, hosted CI must pass on the reviewed pull-request head, and the
merged tree must equal the tested tree. The pull request retains the hosted
check and publication identifiers. Final documentation and whitespace checks
also run after the numerical results are recorded.

## Assessment and next action

The **bounded engineering/evaluation step meets its acceptance criteria**:
causal integration and evidence checks pass, the complete software gate passes,
and every planned outcome is retained and scored against the frozen protocol.
The **evaluated campaign fails its overall flight-performance criterion**.
Response correctness does not close the hover gap, compensate for model mass,
qualify geometric control or establish a physical fallback maneuver. The
cascade stays the default; geometric control remains experimental with noisy
estimated feedback. Reserved qualification seeds remain unopened.

Next: audit this evidence and investigate one bounded vertical disturbance-
compensation candidate for the mass-mismatch offset, with explicit anti-windup,
initialization/reset and observation-loss behavior. Keep the original default,
all comparison cases and all performance limits. The
[next-step plan](../next-steps.md) defines acceptance boundaries and keeps the
remaining hover and hardware work separate.
