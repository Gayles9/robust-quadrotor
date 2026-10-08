# Supported-start geometric comparison

This experiment asks whether the existing supported initialization and corrected
release can rescue the coherent-force geometric controller's hover performance.
It combines existing implemented mechanisms; it does not search for new gains.
[ADR 0041](../decisions/0041-supported-geometric-comparison.md) freezes the scope,
acceptance tradeoffs and conditional validation before scientific execution.
The [verification record](../archive/records/supported-geometric-comparison.md)
retains the audited baseline, source identity, commands and measured outcomes.

## Measured result

The startup package improves the geometric controller on all four development
clean cases. Both full hover peaks now pass 8 cm:

| Case | Matched unaligned geometric | Combined geometric | Combined cascade |
| --- | ---: | ---: | ---: |
| Hover peak, seed 30 [cm] | 8.7284 | 5.2101 | 5.2313 |
| Hover peak, seed 93012 [cm] | 18.1010 | 4.6250 | 4.7610 |
| Nominal spline RMSE [cm] | 6.7963 | 4.1150 | 5.0824 |
| Wind spline RMSE [cm] | 8.1083 | 6.8739 | 6.6268 |

Every candidate clean flight passes its absolute physical/accuracy limits and
effort comparison. The nominal spline improves RMSE by 19.03% against combined
cascade, with effort ratio 0.8412. Wind RMSE is 3.73% worse than cascade, with
effort ratio 0.9187. That wind comparison fails the frozen trajectory gate.
Overall development acceptance therefore fails and fresh validation stays closed.
All eight fault-response comparisons pass, and all 28 planned flights are saved
and fully audited. Zero of the 40 conditional fresh-validation flights run.

This is a useful implemented experimental option with measured improvements,
not a demonstration of superiority in every metric. Geometric average hover
error is 8.09% and 10.05% higher than the matched cascade; its whole-flight spline
peaks are also higher, while remaining inside 25 cm. Defaults stay unchanged.
The result does not establish that geometric control cannot meet future goals.

## What changes

The candidate retains the coherent geometric force and its derivative jets at
10 rad/s. Pre-arm alignment estimates inclination and gyro bias during the
explicit stationary fixture. Exact velocity conditioning acts once before
observations, and the nonlinear predictor handles the first release interval.
All later ESKF predictions, gains, noise distributions and reference timings
remain fixed. No true state enters the estimator or controller.

The matched unaligned geometric arm shares the same supported physical start
and random draws. It isolates the complete initialization package, rather than
comparing a stationary motors-off candidate with a moving, spinning baseline.
The matched cascade uses the same combined initialization and original gains;
it provides the controller comparison. All three arms keep full histories.

## Prospective acceptance

The two development hovers use the full inclusive 5..65 s scoring window and
the existing 8 cm limit. Spline accuracy must match or beat the cascade, and
each candidate clean flight's squared actual moment integral must stay within
twice its matched cascade. The candidate must also satisfy completion, 15 cm
RMSE, 25 cm peak, 8 cm final error and 8 cm/s final speed limits without limiting
or saturation. Only the fixture's initial zero rotor speeds are exempted.

The worst hover must improve against the unaligned geometric arm. Individual
metrics may regress while remaining inside these prospective limits; this does
not reinterpret the earlier strict no-regression study. Eight existing fault
patterns retain their original response and recovery criteria.

The development ledger contains 28 flights. Only a complete passing result
permits the 40 frozen fresh-validation flights. A failed development gate stops
the study before reserved seeds are opened. Neither stage automatically promotes
a default or establishes general flight or hardware qualification.

## Implementation and reproduction

- [Runner and gates](../../experiments/supported_geometric_comparison.py) compose the
  existing support, conditioner, predictor and shaped-reference functions.
- [Saved command audit](../../experiments/robustness_evidence.py) now reconstructs
  geometric correction-force commands as well as the existing cascade path.
- [Tests](../../tests/unit/test_supported_geometric_comparison.py) check composition,
  saved replay, domain aborts, tampering, full-window scoring and seed isolation.

```bash
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.supported_geometric_comparison \
  --stage development --workers 2 --output /tmp/new-supported-geometric
```

Each flight is saved before scoring, then authenticated and reconstructed through
the estimator, sample-noise memory, supervisor, controller, nonlinear plant,
motors and original sensor draws. First-prediction order-five moments are checked
against order seven. `--verify /tmp/new-supported-geometric` replays saved data
without new scientific flights. Exit code 1 denotes failed performance; preserve
the complete report and individual outcomes rather than rerunning a seed.

The [final axis-dependent geometric study](final-geometric.md) is complete under
a separate frozen protocol and is covered by the [official report](../report.md).
This earlier study's failed gate and unopened conditional stage retain their
original status. Controller improvement pauses; middleware, an additional sensor
and combined integral compensation remain outside this study.
