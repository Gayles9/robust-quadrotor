# ESKF Pre-Update Innovation Diagnostics and Outlier Gating

- Date: 2026-09-23
- Status: Implemented, audited, locally verified, published and hosted code CI verified
- Baseline: [`5309f9d6f03c857ebbdbc9e5ce402c405a569541`](https://github.com/Gayles9/robust-quadrotor/commit/5309f9d6f03c857ebbdbc9e5ce402c405a569541)
- Code commit: [`b674c716b4d2fff345e457b6f0ff247b40acd88c`](https://github.com/Gayles9/robust-quadrotor/commit/b674c716b4d2fff345e457b6f0ff247b40acd88c)
- Tested/published code tree: `f6e9bc08740311f77eba7f678453a12328d35553`
- Review: [pull request #3](https://github.com/Gayles9/robust-quadrotor/pull/3)
- Hosted code CI: [run 35890398654](https://github.com/Gayles9/robust-quadrotor/actions/runs/35890398654), successful
- Contract: [ADR 0007](../decisions/0007-eskf-innovation-gating.md)

## Completed scope and implementation plan

The baseline replay corrects every fresh enabled observation. Its README and preceding
verification record identify pre-update innovation diagnostics and explicit fixed gating
as the next bounded estimator milestone. This increment completes that boundary:

1. Verify current main and all 88 tracked blobs against the working snapshot; run the
   complete 1,921-test baseline and inspect the mathematical, execution and test contracts.
2. Specify scoring, finite numeric domain, opt-in policy, gate placement and event
   invariants before implementation; fix statistical thresholds before evaluation.
3. Add failing tests for innovation mathematics and replay integration, then implement
   the pure scoring module and appended optional configuration/event fields.
4. Verify analytic and correlated cases, boundary values, rejection ordering, ownership,
   truth isolation, deterministic generated runs and saved/loaded input compatibility.
5. Audit source and tests, compare baseline/revised default execution in independent
   processes, run repository checks and warnings-as-errors, publish the exact verified tree.
6. Review the remote diff and hosted CI, then complete architecture, usage and evidence
   documentation with runnable examples and explicit limits.

`eskf_innovation.py` adds owned pre-update residual/covariance, whitened residual and NIS,
plus `EskfInnovationPolicy` and an explicit rounded 99% chi-square preset. The replay runner
scores only fresh enabled observations when a policy is supplied. NIS strictly above the
per-sensor threshold produces `REJECTED` before correction; equality fuses. Each rejected
event retains its source identity, observation, full diagnostics and applied threshold.

The configuration default remains `innovation_policy=None`, which avoids scoring entirely.
An explicit policy with absent thresholds provides diagnostics only. The nominal adapter
forwards this independent policy without deriving it from truth or observation noise.
The correction primitives and all existing time/frame conventions remain unchanged.

## Mathematical and execution review

For each observation the implementation computes $r=z-h$, $S=HPH^T+R$, then diagonal
scaling and Cholesky whitening to obtain $\epsilon_\nu=r^TS^{-1}r$. No inverse is formed.
The generic score accepts positive observation dimensions; replay uses 3D NED position
and scalar positive-up altitude, with their existing bias, datum and Jacobian signs.
All scoring quantities must remain finite, and $S$ must be numerically positive definite.
The stored NIS and whitened residual are derived from owned $r/S$, never supplied separately.

The opt-in preset fixes position threshold 11.345 and altitude threshold 6.635, taken from
the rounded NIST table cited in ADR 0007. Neither threshold is fitted to the test data.
A Gaussian reference is necessary for a chi-square probability interpretation. The preset
is not an empirical estimator-consistency result or a guarantee of outlier identification.

Pending/stale/disabled disposition precedes scoring. Rejected observations do not call the
correction, inject error, reduce covariance or add noise. Same-epoch altitude sees the
actual posterior after accepted position, or the unchanged prior after rejected position.
Covariance/model/numeric errors remain exceptions and never become statistical rejections.
Nested event/result arrays own read-only copies; a later error returns no partial result
and changes neither caller inputs nor random state.

## Verification evidence

| Area | Verified behavior |
| --- | --- |
| Analytic score | Scalar and diagonal formulas; positive observation dimensions 1, 3, 5 and 17 |
| Correlated score | 36 independent seeded cases in dimensions 1, 3 and 7 agree with a direct solve reference and the existing correction's exact residual/covariance bytes |
| Coordinate changes | NIS is invariant within floating roundoff under permutation, rotation and extreme consistent unit scaling |
| Numeric domain | Finite real arrays, exact shapes, local covariance symmetry/PSD, singular innovation, overflow, smallest positive variance and a representable sum of individually underflowing squares |
| Statistical reference | Closed-form one/three-degree CDFs confirm the preset's three-decimal rounding; known-Gaussian samples check whitening and predeclared moment/acceptance bounds |
| Threshold boundary | Both sensors accept equality and the next larger threshold; the next smaller representable threshold rejects |
| Gate placement | Correction-call traps prove rejection precedes correction; rejected history bytes equal the prior |
| Sequential sensors | Accepted/rejected position changes the prior used to score same-epoch altitude exactly as specified |
| Eligibility | Stale, pending and disabled observations have no innovation score or threshold and cannot be relabeled as statistical outliers |
| Event contracts | Reject contradictory status, missing diagnostics/threshold, mismatched dimensions and diagnostics inconsistent with the correction |
| Compatibility | None policy bypasses scoring; diagnostics-only preserves valid correction bytes and existing numerical failure outcomes |
| Integration | Fixed signed single/burst outliers, fresh recovery, saved/loaded diagnostics equality, nominal policy forwarding, truth/RNG access traps |
| Atomicity/ownership | Independent read-only buffers, copied policy, immutable objects and unchanged inputs after a scoring failure following a successful update |

The known-Gaussian checks use 4,096 samples in dimension one (seed 6021) and dimension three
(seed 6023), with fixed nontrivial covariance factors. Before execution, tests specify a
maximum absolute whitened sample mean below 0.06, mean NIS within 0.18 of its dimension,
NIS variance within 0.9 of twice its dimension, and acceptance fraction between 0.98 and
0.998. These assertions all pass. They validate the statistic on its assumed input
distribution; they do not establish calibrated ESKF covariance on generated sensor runs.

Eight deterministic outlier regressions use the existing constant-velocity generator,
seed 713, 61 truth steps of 0.01 s and nonzero sensor noise. Each test changes only a
measurement-only fixture: either position or altitude, source row 3 alone or rows 3--5,
with an offset of +1,000 or -1,000 m (applied to each position axis for position cases).
Each corrupted value is rejected, the immediately following clean observation fuses,
and the complete state/covariance output matches omission of those corrupt values byte
for byte. This is 0.61 s of truth and 0.60 s of estimator propagation. It establishes
the gate contract in a controlled regression, not realistic fault-detection performance.

The saved-input checks use 41 steps and seed 281. Clean input covers diagnostics-only;
the active-gate fixture corrupts source row 4 in each sensor after adaptation. In-memory
and authenticated saved/loaded input paths produce identical state, covariance, status,
threshold, residual, innovation covariance, whitening and NIS values. No corruption or
estimator result is written into a new artifact schema.

## Compatibility and audit findings

Only three prior files change at code publication: `eskf_replay.py`, `eskf_run_replay.py`
and `test_eskf_run_replay.py`. Three files are added: the innovation module and two test
modules. All other **85 baseline blobs** are identical. All 1,921 existing tests remain;
the new coverage is **161 innovation tests, 60 gate tests and 15 adapter/integration tests**,
for **236 additions** and **2,157 total cases**. The adapter test file now contains 144 cases.

Separate Python processes import the original and revised source trees for six generated
noisy stationary/constant-velocity cases: seeds 0, 7 and 19, 31 truth steps of 0.01 s.
All **2,394 compared array payloads** and all compared event/time metadata match exactly.
This includes nominal state fields, history covariances, observations, gains, corrections,
residuals and innovation covariances. It is bounded tested-environment compatibility
evidence, not a claim of universal cross-platform bit reproducibility.

The numeric review confirms why scoring is optional: a finite residual of `1e200` with
zero gain can yield a valid unscored correction while its NIS is unrepresentable. Default
replay retains that domain; opted-in scoring rejects the calculation with `ValueError`.
Separately, a gross ungated 1,000 m corruption fixture reaches the existing covariance
validator's numerical limit in both baseline and revised execution on the tested host.
Diagnostics-only preserves that error rather than claiming to prevent it. The regression
compares default/scored outcomes without requiring roundoff-triggered failure on every
BLAS/CPU. Active gating rejects these fixture values before they can enter correction.

No production plant, sensor generator, correction equation, test tolerance, dependency,
tool version, CI workflow, package version, manifest version or artifact schema is changed.
No generated results or large logs are committed. Remote review verifies all six changed
file hashes and the complete Git tree against the locally tested content.

## Exact verification record

Commands below ran on 2026-09-23 with Python 3.12.14 and uv 0.12.3 against the content
published as `b674c716b4d2fff345e457b6f0ff247b40acd88c`.

| Command | Outcome |
| --- | --- |
| `uvx --from uv==0.12.3 uv run make check` | **2,157 passed**; Ruff clean; 85 files already formatted; mypy clean over 22 source files |
| `uvx --from uv==0.12.3 uv run pytest -W error -q` | **2,157 passed**, no warnings |
| `uvx --from uv==0.12.3 uv run pytest --collect-only -q tests/unit/test_eskf_innovation.py tests/unit/test_eskf_gating.py tests/unit/test_eskf_run_replay.py` | 365 cases: 161 + 60 + 144; 129 adapter cases predate this increment |
| `GIT_INDEX_FILE=/workspace/scratch/f2c7111463b5/evidence/innovation-gating/code.index git diff --cached --check 45f37491141d35b26e7a77abf6f9331d76615a8a` | Clean diff against the verified baseline tree using an isolated review index |
| Hosted `uv sync --locked`, then `make check`, run 35890398654 | Successful; **2,157 passed**, Ruff clean, 85 formatted files, mypy clean over 22 source files |

The explicit launcher/index describe this audit environment. Normal repository setup is
still `uv sync --locked` followed by `uv run make check`. Ruff's formatted-file count
includes checked Markdown code blocks and is not a Python-source count. Test durations
depend on the host.

Documentation closeout repeated `uvx --from uv==0.12.3 uv run make check`: **2,157 tests
passed**, Ruff passed, **87 files** were formatted and mypy passed over **22 source files**.
All three README ESKF code examples (correction, replay and the gating continuation)
executed successfully. The eight changed documentation files have balanced code fences,
46 valid relative links and no unresolved drafting markers. All six published code/test
blob hashes remained unchanged through documentation closeout.

## Remaining scope and next bounded decision

This milestone completes pre-update diagnostics and optional rejection on the supported
replay domain. Scoring/rejection records remain in memory, and saved runs do not persist
the estimator policy. Delayed-state fusion, sparse/asynchronous IMU, a production fault
generator, estimator-result persistence and a live service remain outside this boundary.

The next bounded estimator step is a **frozen consistency-evaluation contract and nominal
measurement-only evaluation harness**. Before implementation, specify truth-to-estimate
epoch alignment, right-local error extraction, valid NEES covariance domain, the relationship
between sampled IMU noise and supplied continuous `Q_c`, and a fixed seed/configuration
partition. Record both accepted and rejected innovation scores so selection cannot bias
coverage reporting. Then evaluate the declared nominal cases without tuning gates on the
evaluation data. This is a scope decision for subsequent work, not completed evidence.

The master plan's broader held-out NIS/NEES, 100-seed nominal target, fault precision/recall
and bias-excitation results remain open. Gate G2 baseline control and full G3 validation
remain unclosed. No closed-loop, controller, ROS 2, PX4 or flight-robustness claim follows
from this increment.
