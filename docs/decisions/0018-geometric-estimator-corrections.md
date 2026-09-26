# ADR 0018: Separate estimator corrections from physical force derivatives

Date: 2026-09-26. Status: bounded experiment; promotion requires measured gates.

## Preceding audit

Audited draft PR #15 at `e7783dfffae221c7f463eec5e34ad2a3f8a09eca`,
tree `da4441495d55121c5bf0a237855e072560db17a5`. Fresh focused gate:
154 tests passed in 139.56 s, warnings treated as errors. The source implements
the intended geometric signs, moving-frame transport, causal filter recurrence,
owned transactional memory and truth-isolated feedback. Existing full-gate CI
passed 3,282 tests; a new full gate is required after this change.

Reproduced the noisy nominal spline pair (seed 30) and the failed full hover
(seed 93012) without changing source. The latter again peaks at 13.6855 cm.
The nominal geometric moments reconstruct to 1.12e-16 N m from saved estimates.
Offline removal of measurement-update jumps from derivative memory reduces the
desired-acceleration moment RMS from 0.016614 to 0.002702 N m on that same
history. This is a causal-path diagnosis, not a closed-loop performance result.

## Bounded correction

The ESKF's accepted update injects additive world position/velocity corrections
delta_p [m] and delta_v [m/s]. These are revisions of an estimate, not measured
physical impulses. Their jump in the PD correction force is

    delta_c = m * (Kp * delta_p + Kv * delta_v)  [N].

Accumulate these jumps between outer ticks using accepted measurement events
only. Before the next derivative step, translate the previous input and all
three low-pass section values by delta_c. Differences between the sections, hence
the existing first/second derivatives, are invariant under this translation.
The current unfiltered PD force still responds fully to the posterior estimate.
Physical force variations still use the original 30 rad/s transfer functions.
Rejected/stale/missing measurements contribute no correction. No estimator,
prior, sensor, truth plant, timing, limit, scoring window or cascade default changes.

Add an explicit `rebase_estimator_corrections=False` geometric option. Keep the
original configuration and ADR 0017 campaign reproducible. True-state execution
has no measurement correction to rebase. This remains approximate reference
feedforward; it does not restore the ideal continuous-time stability proof.

## Frozen implementation and performance acceptance

1. RED/GREEN tests for derivative invariance under coordinate jumps, retained
   response to physical ramps, immutable ownership, overflow failure, and index
   continuity. Invalid option types must fail explicitly.
2. Both modes must reconstruct estimated commands from measurement-derived
   history and reproduce offline ESKF replay; the default stays identical.
3. Compare three predeclared candidates: rebasing with horizontal natural
   frequencies 1, 1.5, 2 rad/s and damping ratio 0.9, hence Kp=1,2.25,4 and
   Kv=1.8,2.7,3.6. Vertical and attitude gains stay fixed. This bounded sweep
   follows the low-frequency position model, not an unconstrained seed search.
4. Development cases are the already observed nominal spline seed 30 and full
   hover seeds 30/93012. Keep every result. Selection minimizes the number of
   failed unchanged gates, then worst normalized violation, then mean tracking
   error. A diagnostic improvement does not establish qualification.
5. Freeze one candidate before the full ADR 0017 matrix and fresh hover seeds
   95000..95003 / spline seeds 96000..96003 (alternating nominal/mild wind).
   Preserve all original 15 cm RMSE, 25 cm peak, 8 cm terminal position/speed,
   full 5..65 s hover peak <=8 cm, paired RMSE <=1, paired actual squared-moment
   effort <=2, refinement, no limiting and exact-repeat requirements. Report
   known baseline failures separately from candidate qualification without
   rewriting the historical all-cases outcome. Fresh cases are evaluated once.
6. Stop this bounded study after the declared candidates and validation. If
   gates still fail, retain an experimental result and identify the remaining
   cause; do not claim a fundamental optimum or alter the acceptance criteria.
7. Run the complete warning-strict software gate, verify saved histories and
   hashes independently, push tested source on the existing draft, and update
   current documentation plus the canonical engineering log. No automatic merge
   or default-controller promotion of an unqualified result.

## Diagnostic amendment before opening fresh seeds

All three development candidates failed both original hover gates. Increasing
horizontal frequency from 1 to 2 rad/s reduces spline RMSE from 8.3764 to
4.6956 cm, but raises paired effort from 1.1357 to 5.2126; hover seed 93012
still peaks at 15.5239 cm. Rebasing alone is therefore not a promotion.

The existing scalar inner gains are overdamped: with roll inertia Jxx=.02,
`Jxx*s²+.32*s+.64` has dominant pole -2.3431/s. Suppressing derivative kicks
exposes this slow response to corrected attitude targets. One final bounded
extension is justified before qualification: kR=1.28 N m, derived from critical
roll damping `kR=kOmega²/(4*Jxx)`, at horizontal frequencies 1 and 1.5 rad/s.
The corresponding roll double pole is -8/s before motor lag. Keep kOmega=.32,
all previous limits, and all existing failed candidates. The same three observed
flights and fixed cascade comparator are used; no fresh seed has been opened.
This is a local model rationale, not a sampled-system stability proof. No further
gain extension is planned after these two cases; characterize one frozen choice
and preserve any unmet gates as limitations.

The independent artifact audit also found covariance payloads that were empty
after successful original hashing. Exact sensor-only replay restored two to
their original SHA-256 values; a third is being checked. The cause of truncation
is not established. Add explicit saved-payload verification to the campaign gate,
so an apparently successful flight can never qualify from damaged evidence.
