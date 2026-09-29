# ADR 0031: independent supported-start repeatability and fault validation

Status: protocol and acceptance frozen before implementation or flight execution,
2026-09-28.

## Audit and scope

Audit PR 29 implementation `72f7311a8b97425c837f0c507ac1fe6b4a6cf235`, tree
`63c59159445f6641a47a94e2c2cd74d6e6cb0b98`. Fresh warning-strict supported-start,
pre-arm, early-flight and documentation checks pass 130 tests in 6.57 s. The
independent evidence verifier authenticates 66 files and reconstructs all eight
new and four original scores. Review finds no demonstrated implementation defect.
The prior full suite passed 3,757 tests; PR 29 records its hosted CI and merge.

ADR 0030's known-seed benefit needs an independent check before ordinary mission
integration is justified. This is one fixed experiment using the same component,
physical support, controller, noise, plans and limits. It is not a new controller
candidate, a gain search or a population-level reliability qualification.

## Frozen jobs and pairing

Run exactly 34 full flights in 17 pairs, preserving their defined order:

1. For each seed 47001, 47002 and 47003, run nominal_hover, nominal_tracking and
   wind_tracking. Each has supported unaligned and supported aligned arms, both
   with supervision enabled: nine pairs, 18 flights.
2. At seed 47004 run position_dropout, position_rejection, position_delay,
   altitude_dropout, altitude_rejection, altitude_delay, position_recovery and
   landing_position_dropout, in that order. Each has aligned initialization with
   supervision off and on: eight pairs, 16 flights.

The fresh seeds do not occur in existing source/test/protocol text and are
distinct from geometric qualification seeds 95000..95003 and 96000..96003,
which remain unopened. All initialization and noise draws use the existing
generators/distributions with the new seed, including true pose and bias.
Generate the fresh seeded base using the original configuration factory; retain
the frozen robustness mission, wind modification and schedules. The initial
navigation prior remains independent of truth. Do not screen seeds using
acquisition quality or a preliminary full flight, substitute rejected seeds,
or repeat a valid failed candidate.

Clean pairs isolate alignment under identical physical support and named random
streams. Fault pairs isolate supervision under the identical aligned prior and
faulted measurements. They do not compare aligned and unaligned fault responses.
The archived original free-flight runs remain historical evidence; comparing a
fresh seed against an unrelated old seed is not a causal no-regression test.

## Unchanged physical boundary and response contract

Use ADR 0030's complete fixture: constant true pose, zero velocity/rate/rotors,
external gravity/drag balance, acquisition samples 0..200 and fresh supported
sample 201 at 0.5025 s. Preserve all sample-noise and terminal-bias correlations,
the complete 21x21 endpoint, continuous zero-speed motor release, and supported
left-limit endpoint at flight zero. Retain the original integration error across
the force discontinuity and the entire startup transient in scoring. No in-flight
stationarity, ground-contact model or hardware support is assumed.

Use the existing eight ADR 0023 fault definitions unchanged: same channels,
acquisition indices, onset/duration, 5 m offsets, 100-step delays and dropouts.
Use the same health policies, recovery counts and 0.6/0.2 s supervision budgets.
Persistent loss is expected to produce a latched numerical abort, not a landed
vehicle. Disabled-supervision executions remain complete numerical comparisons;
their outcomes must not be called safe fallback behavior.

Four isolated worker processes are allowed for this independent campaign, with
one mission at a time per process and no shared mission threads. This changes
execution scheduling only; it does not modify ADR 0030's recorded two-worker run.

## Frozen acceptance

1. All 17 planned acquisitions must be accounted for and all 34 scheduled flights
   must have valid evidence for an integration go. A rejected acquisition never
   flies, remains in the denominator and is not resampled. It makes this small
   validation inconclusive/no-go; it does not by itself establish a component bug.
2. Authenticate and decode every saved history before scoring. Reconstruct every
   plant/motor interval, ESKF state/covariance/event, controller, guard, health and
   supervision transition. For faults, reconstruct original slow-sensor draws
   from the exhaustive pre-fault acquisition ledger, then verify the fault mapping
   separately. Dropped, offset and delayed data must not be mistaken for changed
   sensor noise. Shared draws must agree on the acquired common prefix.
3. In every clean pair the aligned flight must complete and pass original
   whole-flight RMSE/final-position limits of 0.15 m and final-speed limit of
   0.15 m/s. Each hover must include all of 5..11 s and have peak <=0.08 m.
   Preserve every passing unaligned flight condition; each aligned hover peak
   must be <= its same-seed unaligned peak +1e-12 m. No averaging can hide a
   failed individual condition. Report every metric, including small regressions.
4. Every fault pair must pass the existing ADR 0023 response conditions: exact
   common-prefix parity, established health before the injected fault, actual
   exposure, first-expired-epoch abort with the original reason and onset bound,
   and no terminal-epoch command for persistent faults. Recovery must complete
   without an observation abort and have complete on/off payload parity. Both
   recovery flights must also pass the unchanged full-flight limits. Persistent
   numerical aborts are response successes, not completed-flight passes.
5. Go for a separately scoped opt-in mission integration only if all evidence,
   nine clean comparisons and eight response comparisons pass. Any valid failure
   is retained and produces no-go without tuning, seed replacement or relaxed
   thresholds. Report observed counts and individual ranges; three seeds cannot
   establish a high-reliability probability or general robustness guarantee.
6. Mass mismatch remains an explicit failed requirement from ADR 0030. Do not
   rerun or claim to fix it here, combine the rejected integral candidate, change
   production algorithms/defaults, qualify geometric control or claim hardware
   readiness. Even a go is limited to modeled support, nominal mass and these
   nominal/wind/fault conditions; normal integration itself is a later step.
7. Preserve all outcomes outside Git, pass relevant and full software gates and
   hosted CI, publish the decision and stop. Tests may use distinct smoke seeds
   and shortened plans; smoke runs never count as the fixed campaign evidence.

Implementation/evidence defects may be fixed at their source, with invalid runs
preserved and the correction disclosed. A performance failure is not a defect
just because it prevents acceptance.
