# Regression adapter and evidence compatibility

The true-state and estimated-state simulation APIs accept different options.
The geometric comparison adapter preserves that distinction so both controller
families can use the same declared minimum-snap regression plans.

## API behavior

`allow_minimum_snap=True` belongs to `simulate_estimated_mission`.
`simulate_mission` accepts minimum-snap plans directly and has no such argument.
The adapter adds the permission only when an estimator configuration is present.
A true-state save/load regression covers both selected controller families,
alongside a test of the exact development-evidence compatibility boundary.

This correction changes no control law, force filter, gains, fixtures, clocks,
noise, reference, criteria, profile selection or seeds.

## Why evidence compatibility is narrow

The development ledger contains 36 completed flights that pass full saved
replay and absolute limits. Selection is `axis-h10-f1.25` against
`cascade-f1.25`, with 7.0703% lower balanced RMSE and 1.6425% lower balanced
moment effort. Independent scoring authenticates 296 payload references and
matches the 36 scores within 1.39e-17 SI.

The original true-state adapter supplied the estimated-only option to cascade
calls. Seven geometric flights ran and met physical limits, but all seven
cascade calls raised `TypeError` before simulation. That incomplete ledger
cannot serve as a passing regression parent. The complete fourteen-flight
regression is repeated with the corrected adapter. The seven earlier geometric
runs remain repeated development evidence, not seven additional independent
trials.

The estimated development path is unaffected: it retains its required option
and already has complete estimator, controller and plant audits. Reuse is
limited to these exact identities:

| Item | SHA-256 |
| --- | --- |
| Development report | `b2dc09a00206fbc1f6092c78ed6504a15879a17e8c4546fa1b5504371c0ed86a` |
| Development execution source | `18af27137bd56d4328bf3227d901d84d119e77e341b72f3951d5a0352ce8fbe2` |

Authentication includes every payload and recomputation of profile selection.
Each saved estimated-flight configuration is reconstructed under the corrected
adapter; force shaping and production algorithms must remain byte-identical.
This establishes unchanged behavior on the reused execution path without
resampling flights or claiming that old data used newer source.

New stage protocols bind the correction's hash. The development report retains
its actual source identity; regression and validation reports identify the
corrected executable source and exact earlier parent. All other source/protocol
mismatches remain errors. A damaged, incomplete or failing parent cannot make
later stages eligible.

## Unchanged comparison criteria

The [final comparison](final-geometric-comparison.md) retains its selection,
5% development screen, 10% fresh accuracy target, individual/category tradeoff
bounds, effort limits, absolute physical requirements, four fresh cases per
category and eight fault pairs. Correcting an API adapter does not retune the
experiment or revise any measured performance outcome.
