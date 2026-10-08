# Attitude-estimation startup audit — 2026-09-28

## Preceding audit and frozen scope

Audited main is `7ee788091bb24fefb0e31a9e17754429365e1343`, tree
`e08f895bdf5a0ef312b2a1d703eaeb3bc290b838`. Its post-merge CI run 36463274850
passed. Fresh warning-strict early-flight/documentation checks pass 35 tests
in 1.95 s. Endpoint/online checks pass 77 tests in 5.48 s; update, endpoint-replay
and consistency checks pass another 271 tests in 6.14 s.

[ADR 0026](../../decisions/0026-attitude-startup-audit.md) froze this audit before
implementation. Its SHA-256 is
`0dae747fb8c45580ad9fa6bb70047b1b0cde69d5e250470a1765006bf2d57f9c`.
No production source, controller, prior, sensor, gain, gate, scoring rule,
dependency or tool pin changes. Local Python 3.12.14 / NumPy 2.5.2 is retained.
The restored Ruff executable was truncated and crashed; reinstalling the same
locked Ruff 0.16.2 restored the local formatting/lint tool. Global uv remains
0.12.18; local checks invoke the existing environment directly, while hosted CI
uses repository-pinned uv 0.12.3.

## Demonstrated evidence defect and exact recovery

The local original campaign copy was incomplete; rematerializing it restored
the exact published SHA-256
`139518b7d6356a1cdd2ba14fca73704af21f7d1c720a3701b7adcd39927241f4`.
The persisted early-flight ZIP itself was incomplete: 15,035,145 bytes, SHA-256
`1746a3ae6a6cc6eae3334a973d320d272da2099439996f688e724797ca462774`.
There was no ZIP central directory; its final history member was incomplete.
This is an evidence-packaging defect, not an estimator or flight failure.

Complete local-header members pass CRC, length and surviving-manifest hashes.
The original analysis and oracle reports match their published digests. A
single exact repetition of the original ADR 0025 oracle, at unchanged source
`cbda2444c76898476487dd7891f2fda3e3dfd6c9b92338d5db6d46b9e75ddaa6`,
recovers the missing data member with its **original** digest
`4765988eee5822d2b58aaba1fc2a49e97e49afe02c4883720e8277cbc9ce535c`.
All 15 available/recovered historical payloads match, including both covariance
chunks. The recovered execution passes the old full verifier; its report is
identical except for honestly recorded software provenance. The old reports
are retained, not relabelled as new executions. No new experimental condition
or seed is opened. The missing packaging helper scripts are not claimed to have
been recovered; refreshed packaging documents that limit and verifies the
complete numerical evidence. The repaired package has a new outer ZIP identity.

## Estimator findings

Both complete 15.5 s histories reconstruct exactly at all 6,201 epochs each:
physical states, covariances, measurement events and conditional sample-noise
means. There are 464 delivered measurement events per flight. Separate
21-state conditioning uses a direct linear solve, Joseph covariance and an
independent Gauss-Legendre integral for the SO(3) reset derivative.

| Maximum independent residual | Original | Oracle |
| --- | ---: | ---: |
| Physical correction | 8.33e-17 | 4.17e-17 |
| Joint covariance | 3.47e-18 | 3.47e-18 |
| Conditional noise mean | 7.94e-23 | 7.94e-23 |
| NIS | 3.55e-15 | 3.55e-15 |

Minimum physical covariance eigenvalues remain positive: about 2.1226e-9.
The existing all-column finite-difference measurement, endpoint-map and reset
tests pass independently of this saved-data reconstruction.

Original initial thrust-axis error is 2.2940763608704455 degrees. Through
0.2 s, prediction/true-motion evolution contributes +0.0248008246583975 degrees,
position correction +0.8442653773883539 and altitude corrections
-0.0017437152286610, yielding 3.161398847688536. Each correction is separated
from propagation, and the scalar-angle budget telescopes to roundoff.

At the first position observation, innovation is
`[-0.02495258, -0.03161390, -0.00734287] m`; the sampled sensor-noise component
is `[-0.01828121, -0.01974007, -0.00441956] m`. The roll/pitch correction is
`[-0.67537370, +0.53430614] degrees`; approximately
`[-0.42167846, +0.39140529] degrees` comes from sensor noise and
`[-0.25369523, +0.14290085] degrees` from the predicted-position residual.
These are linear gain-times-innovation components, not separate flight effects.
The coupled velocity correction is `[-0.02146013, -0.02717279, -0.00129269] m/s`;
accelerometer-bias correction is `[0.00031163, 0.00039443, 0.00016794] m/s²`.
Gyro-bias correction is about `[7.17e-6, -5.66e-6, 1.14e-9] rad/s`.

NIS 1.215812 is below 11.345. Roll/pitch sigma falls from about 3.000548 to
2.888449 degrees despite the realized error increase. Through the first second,
full-state NEES ranges 3.08684..6.51101 and the largest componentwise standardized
error is 1.48485. These descriptive values neither show a startup covariance
collapse nor establish population calibration. The prior is explicitly broader
than the uniform truth population (variance ratios 6.75 and 8.33333).

| Original epoch [s] | Thrust-axis error [degrees] |
| --- | ---: |
| 0.0 | 2.294076 |
| 0.2 | 3.161399 |
| 0.4 | 1.132386 |
| 1.0 | 0.499222 |
| 5.0 | 0.038467 |
| 5.395, hover peak | 0.055712 |

The oracle's filter similarly reaches 3.156732 degrees at 0.2 s; its controller
sees a different, truth-substituted attitude. The earlier 32.79% peak reduction
is still only channel-level causal headroom. No deployable performance gain is
claimed by this audit.

## Information and decision

The [guide](../../results/attitude-startup-audit.md) derives the local constant-hover
position-output rank of 11/15 and exhibits all four nullspace directions.
Two are tilt/accelerometer-bias combinations; the other two concern yaw.
This does not prove global unobservability on an excited trajectory. Ordinary
2 cm position noise at 5 Hz supplies limited early curvature information;
the illustrative three-point acceleration-noise standard deviation is
1.224745 m/s², equivalent to 7.153182 degrees, not an ESKF accuracy bound.

**No-go:** no demonstrated propagation, measurement, injection or reset defect
justifies a new correction law under the fixed free-flight initialization.
Do not inject a false stationary-gravity observation, suppress a valid first
position update, tune Q/R, silently tighten the prior or reopen the geometric
study. Cascade and endpoint ESKF remain unchanged. The separate mass transient
is still open. This audit succeeds as a diagnosis and justified stopping decision;
it does not close the original 8 cm hover requirement.

**Next exact step:** freeze a stationary pre-arm alignment contract and
feasibility design with the existing IMU, contingent on an explicit supported,
nonaccelerating interval. Specify inputs, timing, motion rejection, uncertainty,
yaw/bias limits and independent tests before implementation. If that operating
guarantee is unavailable, request a different independent attitude-information
source or explicit startup redesign rather than fabricating extra information.
This is a new operating scope, not a retroactive pass for the free-flight case.

## Verification commands and publication

The 21 new tests pass in 0.56 s. They cover independent joint conditioning and
noise memory, reset integration/signs, malformed inputs, quaternion invariance,
analytic nullspace/noise amplification, unchanged prior, digest failure, a valid
update with worsening realized error, and tampered saved-state rejection.

Final audit source SHA-256 is
`5e10ee310ebb96b8abe3628c52b7750ae2d7906c0c0030393a1ac4a4226c938b`;
report SHA-256 is
`4849a6fcbf309530e16775d86ec31a15db4e9f892290a0ab50c3fa236a83fe0f`.
Repeating the offline audit after formatting/type-only refinements reproduces
every numerical finding exactly. Lint and formatting pass for 261 files, strict
typing passes 77 source files, and documentation checks pass 109 files,
473 local links and 23 executable-syntax examples.

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m experiments.early_flight_oracle \
  --baseline ORIGINAL/campaign --output NEW_RECOVERY
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m experiments.attitude_startup_audit \
  --baseline ORIGINAL/campaign --oracle EARLY_FLIGHT/oracle --output NEW_AUDIT
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/python -m mypy src experiments scripts
.venv/bin/python scripts/check_docs.py
```

The first command is evidence recovery at the preceding source revision only,
not a routine audit requirement or another candidate. Frozen reports and both
exact numerical histories remain outside Git. The publication PR records the
final complete-suite and hosted-CI results; neither is inferred from focused
checks. The separate PDF remains source-pinned revision 2.0.
