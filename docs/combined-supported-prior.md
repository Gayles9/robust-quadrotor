# Supported velocity conditioning with nonlinear release prediction

The boundary-only study repaired the first prediction's joint uncertainty but
did not fix the failed hover case. This experiment adds the known zero release
velocity from the existing mechanical support fixture. It is an explicit research
option; it does not change the normal estimator or controller defaults.

## What changes

The existing [supported velocity conditioner](supported-velocity-prior.md) is
applied exactly once before the initial observations. For the 21-dimensional
joint endpoint covariance C, H selects the three world-velocity coordinates:

```text
K = C H^T (H C H^T)^-1
C_supported = (I - K H) C (I - K H)^T.
```

The implementation solves the innovation system rather than forming an inverse.
The original mean velocity is already zero and independent of the other prior
coordinates, so only the three velocity covariance rows and columns change.
Genuine simulated support, motors off and sample ownership remain prerequisites.
There is no stationarity constraint after release.

The verified [nonlinear first prediction](nonlinear-release.md) propagates this
singular prior to a full positive 21-dimensional joint covariance, retaining
new-sample correlations. Every subsequent interval uses the ordinary ESKF.
The initial sensor observations, noise draws, gains, mission limits and fault
patterns are unchanged. Simulated truth is not supplied to the controller.

## Comparison and limits

[ADR 0037](decisions/0037-combined-supported-velocity-comparison.md) freezes
25 candidate flights: nine clean flights and eight off/on fault pairs. Every
candidate is compared with both the original aligned and boundary-only saved
controls. The complete saved histories are authenticated and reconstructed.
The original 8 cm hover, 15 cm tracking and 15 cm/s landing limits remain, along
with strict peak/RMSE no-regression requirements and existing fault timing rules.

All nine clean flights pass their absolute conditions. Hover peaks are 6.75,
5.69 and 5.56 cm. The first improves substantially, while the other two increase
slightly. Four of nine clean comparisons and all eight fault-response comparisons
pass. The overall comparison fails; the candidate remains experimental and
fresh validation is incomplete. The [dated record](progress/2026-09-29-combined-supported-prior.md)
preserves every outcome and the exact evidence identities.

## Source and reproduction

The [runner](../experiments/combined_supported_prior.py) checks the isolated prior
change and restores prediction routing even on failure. The
[tests](../tests/unit/test_combined_supported_prior.py) cover ownership, invalid
support, complete online/replay equality, later interval routing and both controls.

With the matching source fingerprint and authenticated evidence directories:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.combined_supported_prior --campaign ../release-recovered/campaign --boundary ../nonlinear-release-flights --uncertainty ../nonlinear-release-uncertainty-durable --output ../combined-prior-flights --workers 2
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.combined_supported_prior --campaign ../release-recovered/campaign --boundary ../nonlinear-release-flights --uncertainty ../nonlinear-release-uncertainty-durable --verify ../combined-prior-flights --workers 2
```

The verification mode uses saved data and runs no new scientific flights. A
completed comparison that fails acceptance exits with status 1; inspect the
complete report to distinguish that result from an interrupted software error.
Valid flights must not be retried or used to tune the frozen design.
