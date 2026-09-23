# ESKF Measurement Updates and Contract Audit

- Date: 2026-09-23
- Status: Implemented, locally verified, published, and hosted code CI verified
- Audited baseline: `20f2a7a15c8f446124edf81dbc0041c26775d69b`
- Code commit: [`a751fe843178440e035a6ad375a3abec828a2b1a`](https://github.com/Gayles9/robust-quadrotor/commit/a751fe843178440e035a6ad375a3abec828a2b1a)
- Tested and published code tree: `63945ab61aa2f217793b7afa77c4a83f30a7ce29`
- Review: [pull request #1](https://github.com/Gayles9/robust-quadrotor/pull/1)
- Hosted code CI: [run 35870476786](https://github.com/Gayles9/robust-quadrotor/actions/runs/35870476786), successful

## Scope and completed plan

The baseline README and the September 22 ESKF prediction closeout identify the bounded next
milestone: local-position and barometric-altitude models, innovation and gain, Joseph
covariance update, right-local injection, and covariance reset. That mathematical core is
now implemented in full. The exact models, units, assumptions, algorithms, public interfaces,
and limitations are specified in [ADR 0005](../decisions/0005-eskf-measurement-updates.md).

The implementation plan was executed in four bounded parts:

1. Reproduce the report audit's contract gaps and establish the unchanged 1,470-test baseline.
2. Correct validation gaps with regressions while retaining historical schemas and valid-run
   outputs. Preserve deliberate motor/plant splitting and prediction approximations.
3. Implement measurement models, independently verified reset mathematics, generic stable
   correction, immutable diagnostics, and the two sensor-specific update boundaries.
4. Audit the completed changes, compare original and revised outputs, run the established
   checks, publish the code, and inspect remote contents and hosted CI before updating docs.

No gating, delayed fusion, estimator runner, closed-loop controller, trajectory planner,
Monte Carlo consistency campaign, ROS/PX4 integration, new artifact schema, or dependency
was added. Gate G2 baseline control remains open.

## Disposition of source-audit findings

| Finding | Implemented decision | Evidence |
| --- | --- | --- |
| Structural configuration accepts a broader domain than numerical execution | Preserve historical configuration decoding; preflight positive truth gravity/rotor coefficients, strict inertia symmetry, and downstream initial-attitude validity before histories or RNGs are created | `test_audit_generation_preflights_truth_before_creating_random_streams`; `test_generation_preflights_initial_rotation_before_random_streams`; nominal-zero-domain regression |
| Redundant schedule metadata could disagree with its inputs | Require non-Boolean integer `sample_stride` and exact numeric `effective_sample_period_s` matching configured stride and truth step in every decoded schema | `test_audit_decoder_rejects_inconsistent_derived_schedule_values`; existing v1–v6 round trips |
| Nonfinite floating artifact payloads could survive construction/persistence | Validate every floating payload on construction and recheck finiteness at save; loading validates archive arrays through the same constructor | `test_audit_artifact_rejects_nonfinite_payloads`; fault-injected save/load regressions |
| Scheduler and artifact delivery tolerances disagreed | Reuse `16*eps*max(1,abs(t1),abs(t2))` and the scheduler's due-time inequality in persistence | Generated-run boundary round trips; scheduler/persistence checks at 0.1-second and million-second scales |
| Some rotor/quaternion arithmetic could overflow despite finite inputs | Reject nonfinite rotor allocation/force/moment and quaternion-derivative arithmetic; scale-protect quaternion normalization; reject non-unit rotation inputs without overflow warnings | Warning-as-error overflow and extreme-scale normalization regressions |
| Nominal ESKF quaternion acceptance differed from downstream rotation acceptance | Use the rotation boundary's squared-norm contract in `EskfNominalState` | Two formerly accepted, noncomposable near-unit cases now reject at construction |
| Motor exactness does not imply coupled fourth-order accuracy | Retain existing operator splitting and the stated scope of the convergence study | No integrator or motor arithmetic changed |
| Nominal ESKF translation uses start-of-step world acceleration | Retain the documented high-rate approximation | Prediction regressions and known-motion composition tests |
| Ownership, hashing, and publication have bounded guarantees | Retain read-only ownership, SHA-256 byte binding, and Linux atomic no-replace publication; do not claim adversarial immutability or authenticated authorship | Existing persistence/ownership tests remain active |

The decoder remains permissive about harmless JSON whitespace/key order; canonicality is an
encoder property. Persistence still validates a stored trajectory structurally rather than
reintegrating its dynamics. Finite-output hardening is specific to the listed boundaries;
it is not a blanket claim about every legacy mathematical or sensor function.

## Measurement-update evidence

`tests/unit/test_eskf_update.py` contains 110 cases. Their independent oracles include:

- a scalar barometric example with prior down-position variance 4 m², measurement variance
  1 m², innovation 2 m, gain -0.8, and posterior position variance 0.8 m²;
- three independent position axes with gain 0.25 and posterior observed variances equal to
  0.75 of their prior values;
- conditional-Gaussian covariance calculations for correlated cases, independent of the
  Joseph implementation;
- 15-column measurement-model central differences and a Rodrigues/logarithm finite-difference
  reset Jacobian at zero, small, and moderate correction angles;
- independent quaternion-to-rotation calculations and 24 deterministic seeded correction
  cases across observation dimensions 1 and 3;
- exact zero-correction state ownership, cross-covariance reset, singular innovation rejection,
  zero-noise observations, local PSD/asymmetry failures, subnormal preservation, extreme
  diagonal scaling, and finite overflow;
- two 800-step, 0.01-second stationary/constant-velocity regressions isolating vertical
  accelerometer bias. The no-update vertical error has analytical final magnitude
  `0.5 + 0.1*8 + 0.5*0.15*8² = 6.1 m`. Each corrected case passes final position error
  below 0.005 m, vertical bias error below 0.001 m/s², and position RMS error below one tenth
  of its dead-reckoning comparator. These are scenario-specific test bounds, not a general
  estimator-performance result;
- 400 steps of rotating known motion under constant world acceleration, plus a sequential
  versus joint same-epoch independent-observation comparison.

The full prediction file now contains 110 cases as well. Existing prediction mathematics,
including first-order covariance discretization, remains unchanged.

## Compatibility review

All 79 baseline tracked blobs were verified against the original GitHub tree before editing.
The six-case compatibility probe used the original and revised implementations in separate
Python processes under the same pinned environment. Cases covered Euler and projected RK4,
historical ideal actuation and motorized actuation, and environmental ideal/motorized runs.
Stochastic fixtures retained identical explicit seeds.

For every case, all 35 arrays matched byte for byte; canonical manifest bytes and NPZ bytes
also matched exactly. This supplements the retained save/load/replay tests. The result is
specific to those fixtures and that environment, not proof of universal cross-platform
bitwise reproducibility. Invalid data may now fail earlier, and contradictory schedule or
nonfinite payloads that previously slipped through are deliberately rejected.

The remote code commit has exactly the reviewed 13 changed code/test files, no dependency or
workflow changes, and the identical locally computed Git tree hash. No generated artifacts
or large logs are committed.

## Verification commands and outcomes

All local commands below were run on 2026-09-23 with Python 3.12.14 and pinned uv 0.12.3.
The final working tree was subsequently published as code commit `a751fe843178440e035a6ad375a3abec828a2b1a`.

| Command | Outcome |
| --- | --- |
| `uvx --from uv==0.12.3 uv run make check` | 1,675 tests passed; Ruff passed; 74 files already formatted; mypy passed over 19 source files |
| `uvx --from uv==0.12.3 uv run pytest -W error -q` | 1,675 passed, no warnings |
| `uvx --from uv==0.12.3 uv run pytest -W error -q tests/unit/test_eskf.py tests/unit/test_eskf_update.py tests/unit/test_run_generation.py tests/unit/test_run_artifact.py` | 579 passed |
| `git diff --check` | Passed |
| Hosted `uv sync --locked`, then `make check` in run 35870476786 | Both passed for the code revision |

The normal repository commands remain `uv sync --locked` and `uv run make check` when uv
0.12.3 is already selected. The `uvx --from` prefix records the explicit launcher used in the
review environment; it does not alter repository tooling.

## Remaining integration decision

The mathematical measurement-update milestone is complete. Before connecting it to saved
sensor deliveries, an explicit estimator-execution contract must establish the filter epoch,
IMU sample pairing/interval semantics, observation ordering, and policy for delayed samples.
Gating and NIS/NEES evaluation also need their own bounded decisions. None of those choices
is silently embedded in these timestamp-free primitives. General bias observability and
flight performance remain unverified. Gate G2 baseline control must remain visible before a
later closed-loop estimator-integration claim.

## Subsequent execution closeout

The timing and data-access decision above is now implemented by
[ADR 0006](../decisions/0006-eskf-sensor-replay.md) and the
[sensor-replay milestone](2026-09-23-eskf-sensor-replay.md), published as code commit
`2e26a26d2b4867e0a905e94fad68cb1385955b84`. The runner starts at the first IMU sample,
uses left-held paired IMU, and logs stale/pending observations without fusing them as
current. It does not alter the mathematical core documented here. Gating, consistency
evaluation, estimator-result persistence and Gate G2 baseline control remain open.
