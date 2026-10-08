# ADR 0018: Separate estimator corrections from physical force derivatives

## Motivation

A measurement correction changes the filter's estimate immediately, even though
the physical vehicle has not jumped. Differentiating that change as if it were
physical motion can create an artificial feedforward moment. This study tests
whether translating the derivative filter's memory with the estimate correction
reduces that effect.

The noisy nominal spline pair uses seed 30; the full hover uses seed 93012
and peaks at 13.6855 cm. Geometric moments reconstruct to 1.12e-16 N m from
the saved estimates. Removing measurement-update jumps from derivative memory
on that same history reduces desired-acceleration moment RMS from 0.016614 to
0.002702 N m. This offline diagnosis is not a closed-loop performance result.

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

The explicit `rebase_estimator_corrections=False` geometric option keeps the
original configuration and ADR 0017 campaign reproducible. True-state execution
has no measurement correction to rebase. This remains approximate reference
feedforward; it does not restore the ideal continuous-time stability proof.

## Verification and performance criteria

1. Tests for derivative invariance under coordinate jumps, retained
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
5. The proposed validation fixes one candidate before the full ADR 0017 matrix
   and independent hover seeds 95000..95003 / spline seeds 96000..96003
   (alternating nominal/mild wind).
   Preserve all original 15 cm RMSE, 25 cm peak, 8 cm terminal position/speed,
   full 5..65 s hover peak <=8 cm, paired RMSE <=1, paired actual squared-moment
   effort <=2, refinement, no limiting and exact-repeat requirements. Report
   known baseline failures separately from candidate qualification without
   rewriting the historical all-cases outcome. Fresh cases are evaluated once.
6. A failed required case disqualifies the candidate under these criteria.
   That result does not establish a fundamental performance optimum.
7. Warning-strict software checks and independent saved-history/hash
   verification are required in addition to the flight-performance criteria.
   Passing software checks alone does not qualify a controller.

## Gain alternatives and measured outcome

All three development candidates failed both original hover gates. Increasing
horizontal frequency from 1 to 2 rad/s reduces spline RMSE from 8.3764 to
4.6956 cm, but raises paired effort from 1.1357 to 5.2126; hover seed 93012
still peaks at 15.5239 cm. Rebasing alone is therefore not a promotion.

The existing scalar inner gains are overdamped: with roll inertia Jxx=.02,
`Jxx*s²+.32*s+.64` has dominant pole -2.3431/s. Suppressing derivative kicks
exposes this slow response to corrected attitude targets. One final bounded
extension uses kR=1.28 N m, derived from critical
roll damping `kR=kOmega²/(4*Jxx)`, at horizontal frequencies 1 and 1.5 rad/s.
The corresponding roll double pole is -8/s before motor lag. Keep kOmega=.32,
all previous limits, and all failed candidates in the comparison. The same
three observed flights and fixed cascade comparator define the development
data. This is a local model rationale, not a sampled-system stability proof.
The study includes only these two additional gain profiles.

Saved-payload verification is part of the campaign gate: a successful flight
cannot qualify from damaged evidence. Some covariance payloads were found empty
after their original hashing; exact sensor-only replay recovered the affected
bytes. The cause of truncation is not established.

## Final disposition

All five rebasing profiles failed development. ADR 0019 records one separate
measured-derivative design, its failure, and early disqualification before fresh
validation. No candidate or default gain change is promoted. The artifact audit
authenticates 202 payloads across 42 executions after seven exact-hash recoveries;
the cause of the original truncations remains unknown.
