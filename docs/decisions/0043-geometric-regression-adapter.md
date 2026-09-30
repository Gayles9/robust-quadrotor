# Decision 0043: correct the true-state regression adapter

Frozen on 2026-09-30 UTC before the corrective implementation or any fresh
validation configuration. Audited implementation:
`c97d4786f6bb729cddd2b1af95d65410f4d12eca`, from ADR 0042.

## Demonstrated defect

All 36 development flights complete, pass full saved replay and meet absolute
limits. The fixed selection is `axis-h10-f1.25` against `cascade-f1.25`, with
7.0703% lower balanced RMSE and 1.6425% lower balanced moment effort. Independent
array scoring authenticates 296 payload references and matches all 36 scores
within 1.39e-17 SI. The tested control algorithms have not changed.

The subsequent true-state regression adapter mistakenly adds
`allow_minimum_snap=True` to cascade configurations. That permission belongs
to `simulate_estimated_mission`; `simulate_mission` accepts minimum-snap plans
directly and has no such argument. All seven geometric flights execute and
meet physical limits, but all seven cascade calls raise TypeError before
simulation. The regression report fails closed and no fresh seeds are opened.
Preserve that entire failed report and its seven saved geometric outcomes.

## Narrow correction and evidence compatibility

Add the permission only when an estimator configuration is present. Change no
controller law, force filter, gain, fixture, clock, noise, reference, criterion,
candidate, selected profile, seed or stopping rule. Add a short true-state
save/load regression covering both selected controller families and a test of
the exact archived-development compatibility boundary.

The failed true-state regression ledger is unusable as a passing parent. Repeat
all fourteen declared true-state flights in a new directory after the fix.
Retain the seven earlier geometric executions as repeated development evidence,
not seven new independent trials. Do not substitute a different candidate if
the corrected physical or refinement criteria fail.

The estimated development evidence is unaffected: its required option remains
present, and its complete estimator/controller/plant audits already pass. Permit
only the exact development report with SHA-256
`b2dc09a00206fbc1f6092c78ed6504a15879a17e8c4546fa1b5504371c0ed86a`
and execution-source SHA-256
`18af27137bd56d4328bf3227d901d84d119e77e341b72f3951d5a0352ce8fbe2`
to serve as the legacy parent. Authenticate all of its existing payloads and
recompute its selection, as before. Additionally reconstruct and compare every
saved estimated-flight configuration under the corrected adapter, and verify
that the force-shaping and production algorithms are byte-identical to the
published implementation. This verifies the unchanged execution path without
resampling flights or pretending that historical data used a newer source.

Bind this correction's hash into new stage protocols. Preserve the historical
development report and its original source identity unchanged; new regression
and validation reports identify the corrected executable source and the exact
historical parent. All other source/protocol mismatches remain errors. A damaged,
incomplete or failing parent cannot open later stages.

## Unchanged decision and stopping rule

ADR 0042's selection, 5% development screen, 10% fresh accuracy target, individual
and category tradeoff bounds, effort limits, absolute physical requirements,
four fresh cases per category, eight fault pairs and final pause all remain
unchanged. This is an adapter correction, not another tuning experiment or a
revision of a performance outcome. The professor-facing report remains deferred.
