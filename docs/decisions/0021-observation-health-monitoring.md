# Decision 0021: causal observation health monitoring

Observation health describes whether usable measurements are arriving on time.
It is deliberately separate from whether those measurements are accurate.

## Purpose and boundary

Track the availability of accepted local-position and barometric-altitude
observations independently. The monitor consumes the current estimator clock
and its delivered event dispositions. It receives no truth, state estimate,
covariance, measurement value, controller command or future delivery queue.
Fresh observations do not establish accuracy, observability or flight readiness.

The existing ESKF fuses same-epoch observations only. Delayed deliveries are
`STALE`, even when fusion is disabled; undelivered observations are `PENDING`.
Only `FUSED` refreshes accepted-observation age. Rejection is statistical;
estimator arithmetic errors raise and are not health events. The existing
truth and estimate mission guards remain separate from this diagnostic.

## Availability policy

Each stream explicitly supplies its acquisition period T, configured delivery
delay D, monitoring clock interval h, warning and loss period counts w and l,
consecutive rejection limit r, recovery acceptance count c and recent delivery
window size n. Require T,h > 0, D >= 0, positive non-Boolean integer counts,
l > w, c >= 2, n >= max(r,c), finite derived ages and an explicit enable flag.
The warning age is w*T+D+h; the loss age is l*T+D+h. The additional h covers
delivery quantization onto the monitoring clock. These are availability-policy
parameters, not estimator tuning or demonstrated safe-flight thresholds.

Age uses the last accepted acquisition timestamp; before the first acceptance,
the elapsed time since the monitor's initial epoch controls the startup grace.
An age exactly equal to its threshold is still within that threshold. State
changes occur on the first observed clock strictly beyond it. Current deliveries
are processed before reporting the state; there is no invented transition at an
unobserved time. A gap beyond the warning age resets the acceptance streak before
new deliveries, so a late recovery cannot inherit an old streak.

State priority is DISABLED, LOST (age above loss), DEGRADED (age above warning or
at least r consecutive rejected deliveries), RECOVERING (a previous degradation
requires c consecutive accepted deliveries; with zero new acceptances it remains
DEGRADED awaiting recovery), WAITING (no acceptance yet), then
HEALTHY. The first acceptance within startup grace establishes HEALTHY. After
degradation, c accepted deliveries are required; any intervening nonaccepted
delivery resets recovery. A rejected delivery resets the acceptance streak;
any other disposition resets the consecutive rejection streak. Silence ages
information without inventing rejection events. Recent evidence comprises the
last n delivered dispositions, not a time window; timestamps expose silence.

Disabled streams report DISABLED and retain delivered evidence. An enabled flag
must match the estimator's fusion setting. A fresh disabled event cannot be
accepted by an enabled policy, or a fused/rejected event by a disabled policy.

## API and integration

An immutable configuration supplies one policy per stream. A transactional
monitor starts at an explicit nonnegative time and consumes every estimator
epoch in order, beginning with index zero. Times strictly increase thereafter.
Each event must belong to the current delivery epoch and a known acquisition
epoch, with no reused sensor/observation identity or sensor/acquisition pair.
Canonical same-epoch ordering is required; legitimately out-of-order stale
deliveries remain valid. PENDING is rejected at this live boundary.

Snapshots expose per-stream state/reason, accepted acquisition/delivery times,
accepted age, latest delivery/acquisition times, consecutive counts, recent
dispositions, and ordered state transitions at the observed clock. Frozen
scalar-only records cannot modify continuation memory. Reset clears all history
and identities, and requires a new initial epoch. Invalid calls are atomic.

Estimated missions optionally accept a fresh caller-owned monitor after
validating its initial epoch and policies against sensor schedules, fusion
flags and the plant/IMU clock. Every epoch, including abort/completion, records
a passive snapshot after ESKF correction. The monitor retains an immutable
history; a failed mission leaves only the observed prefix, and reuse requires
reset. Existing mission results and saved flight schemas stay unchanged.

The evidence writer serializes every mission-result field. Keeping diagnostics
in a caller-owned monitor therefore preserves the flight archive schema instead
of adding availability state to every saved flight.

## Acceptance

Deterministic checks cover normal delivery, isolated/repeated rejection,
missing/stale/pending observations, disabled streams, independent stream loss,
recovery with interruption, exact boundaries, irregular clocks, canonical and
out-of-order delivery, identity reuse, invalid values, ownership, reset and
atomic retry. Real ESKF events exercise the adapter. Enabled/disabled monitoring
must produce exactly equal complete mission, sensor, estimate and command
histories for noisy cascade and geometric cases, delayed/disabled fusion and
an immediate abort. Full documentation, lint, formatting, typing and test gates
remain required. No performance qualification follows from these checks.
