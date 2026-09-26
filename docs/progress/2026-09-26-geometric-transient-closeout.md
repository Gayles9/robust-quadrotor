# Geometric startup/hover investigation — 2026-09-26

I closed the bounded follow-up with a negative result. One coherent-force
candidate passes the observed spline comparison and greatly reduces effort,
but worsens both required full hovers. I retained the cascade default and
original geometric controller, left estimated-feedback qualification open,
and stopped before the reserved validation cases. No production source changed.

## Baseline audit and fixed scope

The starting commit was `60ca2a81b61e1def8aebc41cf84c00f633700fba`, the merged
documentation review. The original execution-source fingerprint was
`cc4f982a40f90bc775c03703aa92007549a21f4509e296eb83743d63ef5aa9fb`.
Python 3.12.14, NumPy 2.5.2, uv 0.12.3 and the existing lockfile were retained.
The first targeted mathematical and campaign checks passed 62 tests.

The earlier baseline archive authenticated all 61 manifest entries. Its spline
pair and seed-93012 hover were available as full histories. The earlier rebase
archive could not be opened as a valid ZIP, so I did not use it for new numerical
claims. Its previously recorded failures remain in the preceding study.

I freshly reproduced the original seed-30 spline pair and both complete
geometric hovers at seeds 30 and 93012. The three overlapping flights matched
the earlier mission, measurement and covariance hashes exactly; the seed-30
hover also reproduced its earlier reported maximum. The baseline source stayed
unchanged throughout those runs. The new study uses only these already observed
development cases, the original 5..65 s hover window, and the existing limits.

## Diagnosis and command reconstruction

The diagnostic aligns truth and estimated-state errors, accepted position and
velocity corrections, desired lift and derivatives, desired thrust axis, all
six moment-law terms, allocated moment, rotor speeds and actual moment. It uses
the actual 400 Hz plant timestamps, 50 Hz outer ticks and 100 Hz inner ticks.
Reconstruction uses the saved posterior and measured-gyro estimate. Truth is
used only for evaluation and plant-wrench checks.

On all three original geometric flights, reconstructed requested moments match
within `1.12e-16 N m`; allocated moments, thrust and actual moments match exactly.
This verifies the command path before interpreting its terms. The aligned
histories retain individual correction events instead of differentiating a
resampled or interpolated estimate.

For the failing original seed-93012 hover:

| Time [s] | True tracking error [cm] | Position-estimation error [cm] | True horizontal speed [m/s] |
| ---: | ---: | ---: | ---: |
| 0.0 | 2.260 | 2.260 | 0.0216 |
| 0.2 | 2.795 | 3.305 | 0.0801 |
| 1.0 | 12.782 | 2.045 | 0.1876 |
| 5.0 | 5.458 | 6.835 | 0.1333 |
| 6.2275 | 13.686 | 0.768 | 0.0098 |

Initial attitude-estimation error and subsequent revisions precede the large
position excursion. At the hover peak the position estimate is already close
to truth, while the vehicle has reached a turning point after accumulating
motion. The first scored 8 cm crossing is at 5.4825 s. This is a coupled transient,
not simply a large instantaneous position-estimation error at the maximum.
Initial position, velocity and attitude errors coexist; this timeline does not
identify unique additive causal contributions or prove global estimator stability.

The original correction-force filter has approximately 0.1 s low-frequency
delay, while its raw force value enters the attitude target directly. Motor lag
adds 0.025 s and the command holds add sampling effects. Differentiated estimator
revisions increase feedforward effort, but suppressing them can also remove a
fast corrective response. The earlier rebasing results and the new shaped-force
result both demonstrate that reducing effort alone does not resolve hover.

## Local sampled model and frozen candidate

[ADR 0020](../decisions/0020-coherent-geometric-force.md) was written before
candidate execution. Its original bytes have SHA-256
`bfbb04357cf6da545eeb1187cf1737703573dc9545ec7ca3ba8aec32080beeae`.
It selects exactly one filter cutoff and makes no gain, estimator, prior,
sensor, plant, scoring-window or threshold change.

For each horizontal direction, the local model uses
`[p,v,eta,r,a,previous,s1,s2,s3]`. North uses `eta=-pitch`; east uses `eta=roll`.
The physical part satisfies `p'=v`, `v'=g*eta`, `eta'=r`, `r'=a` and
`tau*a'=u-a`. Three bilinear sections update every 20 ms; two separate 10 ms
moment commands act through the exact local motor/plant hold map. The command is

```text
eta_d = -(kp*p + kv*v)/g
u = (kR/J)*(eta_target-eta) + (kOmega/J)*(eta_rate-r) + eta_acceleration
```

The original uses raw `eta_d` and filtered derivative jets. The candidate uses
the third section as `eta_target` and the corresponding derivative jets. Its
cutoff is 10 rad/s, one quarter of the motor pole, trading about 0.3 s filter
delay for attenuation of rapid estimate revisions. Reference trajectory jerk
and snap remain analytic. Constant-prehistory initialization and immutable
memory/reset are preserved. Both raw and shaped commands must satisfy the
original acceleration, tilt and thrust domain.

| Local model | North spectral radius | East spectral radius | Slowest decay [1/s] |
| --- | ---: | ---: | ---: |
| Original, 30 rad/s derivatives | 0.983317 | 0.983330 | about 0.841 |
| Coherent force, 10 rad/s | 0.982553 | 0.982548 | about 0.880 |

Independent central differences of the nonlinear geometric controller,
allocator, motor and RK4 plant agree with both nine-state maps in both frame
directions, within `3e-6` absolute and `3e-5` relative tolerance. This checks
signs, derivatives, sample/hold timing and motor dynamics. The model treats
estimation errors as external inputs. Its poles are not those of the full ESKF
and do not predict a noisy full-flight pass.

The candidate is confined to an experiment adapter restored on exit. It is not
a new supported controller option. Twelve added tests check fixture identity,
independent bilinear step response, force direction, immutable continuation,+channel/domain rejection and the sampled-model comparisons. Together with the
existing focused controller suite, 73 tests pass.

## Complete development result

| Metric | Original geometric | Coherent-force candidate | Requirement |
| --- | ---: | ---: | ---: |
| Seed-30 spline RMSE [cm] | 6.051717 | 6.768787 | <=7.405886 (paired cascade) |
| Spline squared-moment effort / cascade | 10.859867 | 0.854508 | <=2 |
| Full hover maximum, seed 30 [cm] | 8.764895 | 10.868718 | <=8 |
| Full hover maximum, seed 93012 [cm] | 13.685526 | 17.325067 | <=8 |

Both candidate hovers cover every required sample through 65 s. All four
candidate/comparator flights complete without inner/outer limiting or rotor
bound contact. The candidate passes the paired spline RMSE and effort checks,
but fails both hover checks. Its spline effort is about 92.1% lower than the
original geometric controller, while spline RMSE is about 11.9% higher than
that original controller. It is not an overall replacement.

The full development ledger is complete and the candidate is rejected. There
was no second cutoff, gain change or candidate. The true-state promotion gate
and 40-case qualification matrix were not run, as required after a development
failure. Reserved seeds 95000..95003 and 96000..96003 remain unopened. Historical
true-state results remain applicable to the unchanged original implementation,
not to the rejected candidate.

## Evidence and checks

The execution source stayed fixed throughout the candidate flights at
`9640d17e07c12a000fe55dc3c2021b010f0431927a98e95b3a9e3da2793235ee`.
Protocols record complete configurations before execution. Reports retain
individual scores, failed gates and the original payload hashes. Independent
raw-array scoring checks RMSE, peak error, full hover coverage, moment effort,+limiting and completion without importing the production scorer.

Saved-payload damage was detected after original hashes had been recorded.
One empty baseline covariance chunk was restored from the authenticated earlier
identical flight. The candidate spline mission archive was rebuilt from its
authenticated duplicate mission arrays, matching its original hash exactly.
One later damaged candidate hover history required an identical-fixture repeat.
That repeat matched every original metric and all nine payload hashes; the
damaged history was restored to its original expected hash. All **56 primary
flight payloads** authenticate, and independent scoring verifies all eight
primary executions. Including the recovery repeat, nine flights were executed;
these are development/reproduction runs, not nine held-out trials.
Recovery records retain expected hashes and the negative checks; no expected
digest, measurement, score or flight requirement was changed. The cause of the
file damage is not established.

Reproduction from the repository root uses the pinned environment:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.geometric_transient_study --output NEW_DIR --workers 3
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.geometric_transient_diagnostic --input NEW_DIR --output DIAGNOSTIC_DIR --shaped
```

The study command exits 1 because the unchanged development requirements fail.
Generated full histories, aligned signal arrays, figures, authentication
manifests, recovery records and exact commands are retained outside Git.
The warning-strict `make check` passed **3,338 tests in 217.84 s**, Ruff lint
and formatting (219 Python files), and strict mypy (66 source files). The final
documentation check covers 89 Markdown files, 355 local links and 20 executable
Python/JSON examples. No code or test
expectation was changed after this full gate.

## Decision and next action

This bounded controller study is complete; noisy-feedback qualification is not.
Keep the cascade default and original geometric implementation as the existing
limited references. Move to the [observation health-monitoring task](../next-steps.md),
without implementing a degraded-flight response or middleware ahead of its scope.
