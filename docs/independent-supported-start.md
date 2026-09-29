# Independent supported-start validation

The known-seed [flight comparison](supported-start-flight.md) showed that supported
pre-arm alignment could improve flight. This separate campaign asks whether the
benefit repeats and whether the existing observation supervisor still responds
correctly with the new prior. [ADR 0031](decisions/0031-independent-supported-start-validation.md)
freezes the jobs, pairing, operating boundary and acceptance before execution.

## What the pairs establish

| Group | Fresh seeds | Compared arms | Question |
| --- | --- | --- | --- |
| Hover, nominal tracking and wind tracking | 47001, 47002, 47003 | Supported unaligned / supported aligned, both supervised | Does alignment improve each hover and preserve the original flight limits? |
| Eight existing observation faults | 47004 | Aligned, supervision disabled / enabled | Does supervision preserve the common trajectory until the required response, with correct abort or recovery? |

There are 17 pairs and 34 planned full flights. The fault pairs isolate
supervision, not alignment. This distinction avoids interpreting different
priors as a failed on/off trajectory-parity check. No new free-flight baseline
is introduced; old seeds remain historical evidence, not causal comparators
for new random draws.

The ideal fixture, 201 alignment samples, fresh sample at 0.5025 s, complete
endpoint covariance and continuous zero-speed motor release are unchanged.
Each clean pair shares its physical initialization and every noise stream.
Each fault pair shares its aligned prior, injected faults and sensor streams.
The original controller, ESKF, support model, noise scales, health rules and
supervision time budgets remain fixed. No stationarity is assumed during flight.

## Acceptance and interpretation

Each aligned clean flight must complete, remain below 15 cm whole-flight RMSE
and final position error, and below 15 cm/s final speed. Each hover must cover
the complete inclusive 5..11 s window with peak <=8 cm and no increase against
its same-seed unaligned comparator. A good average cannot replace a failed
individual condition. Startup stays in whole-flight scoring.

Faults retain the original dropouts, 5 m offsets and 100-step delays, including
brief position recovery and position loss during landing. Persistent faults
require the original observation-timeout reason, first expired sample, onset
bound and command cutoff. Brief recovery requires complete on/off payload
parity, no observation abort and passing full-flight limits. A correct numerical
abort is a response success, not a safely landed vehicle.

## Decision

**No-go for normal mission integration.** Alignment improves all three fresh
hover peaks, but one remains above the unchanged limit. This independently
confirms a useful benefit without establishing sufficient repeatability to
promote the initializer.

All 34 planned flights complete their numerical execution and evidence checks.
Eight of nine clean comparisons pass, and all eight fault-response comparisons
pass. Recovery completes with identical supervised/unsupervised histories;
persistent loss triggers the required numerical abort. The complete software
suite passes 3,769 tests. These results keep the failed hover condition visible.

| Seed | Unaligned hover peak, cm | Aligned hover peak, cm | 8 cm condition |
| --- | ---: | ---: | --- |
| 47001 | 11.220063 | 10.180776 | FAIL |
| 47002 | 6.823515 | 5.463810 | PASS |
| 47003 | 6.868656 | 5.241372 | PASS |

The [verification record](progress/2026-09-28-independent-supported-start.md)
contains every flight score, response result, source identity and software gate.
The failed seed's aligned thrust-axis estimation error stays below 0.134 degrees
throughout its hover mission. Its remaining tracking peak is mainly horizontal.
Those observations are not a causal proof or evidence of a new estimator defect.

Keep alignment experiment-only and the cascade controller as default. The
previous mass-mismatch failure remains open; this campaign does not combine the
rejected integral candidate or open geometric qualification seeds. Three fresh
seeds are a small independent check, not a high-reliability probability estimate.
No ground-contact or hardware support procedure is qualified.

## How the evidence is checked

Each flight is saved, authenticated and decoded before scoring. The verifier
reconstructs all plant/motor intervals, ESKF states, covariances and events,
control commands, mission guards, health transitions and supervision decisions.
The original slow-sensor acquisition ledger is reconstructed before applying
dropouts, offsets or delays; otherwise a correctly injected fault would be
mistaken for changed sensor noise. The fault mapping is checked independently.

The saved support window is reproduced exactly. Fresh sample ownership,
the full covariance handoff, initial navigation-prior isolation and paired bias
walks are explicit checks. Support rejection prevents both paired flights,
remains in the denominator and cannot be replaced by another seed. Process-local
adapters restore on exit; four isolated worker processes do not share mission
threads. Default behavior of the original eight-flight runner is retained.

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.supported_repeatability --output results/supported-independent --workers 4
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.supported_repeatability --verify results/supported-independent
```

Use a new output directory. A completed experiment that fails acceptance exits
1 and retains its report; implementation/evidence errors raise separately.
Full archived replay requires the recorded source fingerprint and protocol.
The evidence bundle's NumPy-only verifier independently authenticates payloads,
recomputes scores, checks handoffs and reconstructs response/comparison decisions.

The subsequent [residual diagnosis](residual-supported-hover.md) now separates
physical tracking, estimation error and the retained release response. Its
quantitative explanation led to the [navigation-feedback isolation](navigation-feedback-isolation.md),
which reduces the failed hover peak to 1.99 cm using true outer position/velocity.
That diagnostic headroom led to the [supported velocity prior screen](supported-velocity-prior.md).
The prior-only candidate is rejected before flight because its release uncertainty
does not cover the known first-interval integration error. The next step addresses
that boundary; ordinary flight remains unqualified.
