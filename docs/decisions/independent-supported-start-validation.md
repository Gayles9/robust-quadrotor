# Independent supported-start validation

A benefit at a known seed does not establish repeatability. This experiment
checks supported alignment at three separate seeds and tests the existing
observation-loss supervisor under the same supported startup. The controller,
sensor model, plans and limits remain fixed.

## Flight ledger and comparisons

There are 34 complete scheduled flights in 17 pairs:

| Cases | Seeds | Paired conditions | Flights |
| --- | --- | --- | --- |
| Nominal hover, nominal tracking, wind tracking | 47001, 47002, 47003 | Supported unaligned/aligned, supervision on | 18 |
| Eight observation faults | 47004 | Aligned, supervision off/on | 16 |

The fault order is position dropout, position rejection, position delay, altitude
dropout, altitude rejection, altitude delay, position recovery, and landing
position dropout. All pose, bias and noise draws use the original configuration
factory and generators with these seeds. The navigation prior is independent
of truth. Acquisition rejection remains in the denominator and is not replaced
by another seed.

The clean pairs isolate alignment under identical support and random draws.
The fault pairs isolate supervision under identical aligned priors and faulted
measurements; they do not compare aligned and unaligned fault responses.
Unrelated archived free-flight seeds are not causal no-regression comparators.
The seeds are distinct from the geometric comparison seeds 95000..95003 and
96000..96003, whose use belongs to a separate study.

## Physical and response contracts

The [supported-start fixture](supported-start-flight-evaluation.md) retains
constant true pose, zero velocity/rate/rotors, gravity/drag balance, acquisition
samples 0..200 and fresh supported sample 201 at 0.5025 s. All terminal-bias and
sample-noise correlations and the full 21×21 endpoint are preserved. Release is
continuous from zero motor speed. The original integration error across the
support-force discontinuity and its complete transient remain in the scores.
No ground-contact or hardware-support model is assumed.

The eight existing fault definitions keep their channels, acquisition indices,
onset/duration, 5 m offsets, 100-step delays and dropouts. Health policies,
recovery counts and supervision budgets of 0.6/0.2 s are unchanged. Persistent
loss is expected to cause a latched numerical abort, not a physically landed
vehicle. Supervision-off flights are numerical comparisons, not demonstrated
safe fallback behavior.

The campaign permits four isolated worker processes, one mission per process
and no shared mission threads. Worker count affects scheduling, not the defined
scientific cases or the earlier two-worker campaign.

## Evidence and acceptance

Every acquisition and scheduled flight is accounted for. A rejected acquisition
never flies and makes this finite validation inconclusive or unsuccessful; it
does not alone establish a component defect.

Authenticated histories reconstruct plant/motor intervals, estimator states,
covariances and events, controller commands, guards, health and supervision.
An exhaustive pre-fault acquisition ledger separates original sensor draws from
the subsequent dropout, offset or delay mapping. Common random draws agree over
the acquired common prefix.

Each clean aligned flight must complete, satisfy 0.15 m whole-flight RMSE and
final-position limits, and end at no more than 0.15 m/s. Every hover covers all
of 5..11 s with peak at most 0.08 m. Conditions passed by its same-seed unaligned
comparator must remain passed, and aligned hover peak cannot exceed the
unaligned peak by more than 1e-12 m. Individual failures cannot be hidden by
averaging.

Every fault pair must have exact common-prefix parity, established health
before the fault, actual fault exposure, and the original abort reason and
onset bound at the first expired epoch. Persistent faults produce no
terminal-epoch command. Recovery must complete without observation abort and
with complete on/off payload parity; both recovery flights must also meet the
ordinary flight limits. A successful persistent-fault response is not a
completed-flight pass.

All nine clean and eight response comparisons must pass for a supported
integration case. Valid failures retain their original thresholds and seeds.
Three clean seeds cannot establish a high-reliability probability or a general
robustness guarantee. Mass mismatch remains a separate failed requirement;
these cases do not fix it or demonstrate geometric-control or hardware
readiness. Software smoke runs are separate from scientific evidence.

The residual failure is examined in
[the supported-hover diagnosis](residual-supported-hover-diagnosis.md).
