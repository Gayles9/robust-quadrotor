# Whole-flight estimation/control error-budget review

Date: 2026-09-29. Scope: [ADR 0039](../decisions/0039-whole-flight-error-budget.md).
The [guide](../whole-flight-error-budget.md) gives the derivation, assumptions,
measurement decision and reproduction command.

## Audit, scope and implementation

The audited preceding milestone is merged PR 37 at
`b4fc5821fe41474bc69edf6d92d9a97286da8824`, tree
`7ab1b0b87c106063e30cb83299d2d98ace6904e2`. Remote main and the clean checkout
agree. Before implementation, 51 warning-strict relevant tests pass in 3.47 s.
The preceding independent NumPy verifier authenticates the complete archive
and recovers all 27 saved algebra records. No defect is demonstrated. The scope
and acceptance were written and hashed before the new analysis was implemented.

One experiment module computes uncentred response Gram matrices for all 27
existing clean flights and their mission windows. It retains all eleven physical
channels, all signed cross terms, nonzero means, original scores and full durations.
It also derives the periodic lifted horizontal cascade, its phase-sensitive
frequency response and finite-horizon impulse bounds, audits sensor assumptions,
compares local observation ranks, and preserves all gate decisions. The fixed
preceding manifest authenticates all 101 payloads before analysis. The old
full replay is reused as authenticated evidence, not claimed as rerun here.

No production source, original experiment runner, gain, gate, seed, noise model,
dependency or controller default changed. No new scientific flight was run.

## Numerical results

All full-flight Gram matrices recover the original squared RMSE within
1e-12 m², with response sums within 1e-10 m. Channel-energy changes alone are
insufficient to explain a changed score. The following table uses the combined
arm minus the boundary-only control, in cm²; all original-arm comparisons remain
in the complete generated record.

| Mission / seed | Diagonal-energy change | Cross-term change | Total MSE change |
| --- | ---: | ---: | ---: |
| Hover 47001 | -3.5471 | -6.4387 | -9.9858 |
| Nominal 47001 | -3.1610 | -5.6166 | -8.7777 |
| Wind 47001 | -3.1844 | -7.6817 | -10.8661 |
| Hover 47002 | -0.0236 | +0.3536 | +0.3300 |
| Nominal 47002 | +0.1730 | +0.0988 | +0.2717 |
| Wind 47002 | +0.1774 | -0.8502 | -0.6729 |
| Hover 47003 | -0.0205 | +3.5657 | +3.5452 |
| Nominal 47003 | -0.3588 | +2.9128 | +2.5541 |
| Wind 47003 | -0.3813 | +3.7979 | +3.4166 |

In all three seed47003 missions, diagonal energy decreases while cross terms
increase enough to worsen the total. In seed47002 wind, diagonal energy increases
while cancellation improves the score. These are pathwise second-moment
identities, not independent-source variance estimates or causal interventions.

The one-period local model has spectral radius 0.9831523666. Its analytic DC
gains `[-1,-1.8,-9.81,-3.27]` agree within 1.546e-13. Independent real time-domain
sinusoidal evolution agrees within 7.994e-15. Every sampled frequency's maximum
phase amplitude is at most its DC value on the frozen grid; no continuous-band
maximum is inferred from that grid.

The zero-initial-state coefficients over the inclusive 5..11 s window are
`[0.9998664150,1.7997595470,9.8086895313,3.2695631771]` for position, velocity,
inclination and inclination-rate vector-norm error. Each single-input ceiling
using the entire 8 cm allowance is respectively 8.0011 cm, 4.4450 cm/s,
0.467306 degrees and 1.401919 degrees/s. Those values are not simultaneous
budgets. No bound for vertical/physical residuals or future estimation errors
is established by the old RMS model discrepancy.

Local observability rank is 11 with the existing position/altitude sensors,
11 after adding ideal direct velocity, and 13 after adding two independent
inclination coordinates. Analytic null directions verify the distinction.
The current 5 Hz, 2 cm position noise would produce 14.142 cm/s per-axis
adjacent-difference noise, with explicit same-sample and adjacent-difference
correlations. This is not an independent velocity sensor proposal.

All 13,608 observation decisions are retained, including 120 rejections. The
same three unique seed47001 events change branch relative to both controls:
six paired differences. No gate is changed or linearized across rejection.

## Decision and next exact action

The bounded review meets its mathematical and diagnostic acceptance. It supplies
the prospective joint inequality and identifies missing information; it does
not establish a passing new controller or estimator. No-go for implementation
under the current sensor assumptions. Go for a standalone independent-inclination
measurement contract and feasibility study, with no scientific flights.

The next step must specify a physically justified body-down direction source,
frames/calibration, tangent-space uncertainty, bias, timestamps/latency, outages
and position/attitude correlation. Its expected benefit is removal of the two
local tilt/accelerometer-bias ambiguities and possible reduction of the dominant
low-frequency error terms. Derive the measurement/Jacobian and a feasible joint
budget before proposing an update component. No source availability, noise bound
or achieved flight benefit is assumed. Stop if those physical assumptions cannot
be supported; do not manufacture a stationary-gravity observation in flight.

The startup study remains closed, ADR 0037 remains failed, and the combined
option remains experimental. Fresh estimated-feedback qualification, the
mass-offset requirement, geometric qualification and external integration stay
open. Neither a local rank increase nor a software test count closes those gates.

## Verification and identities

Twenty-six new tests pass in 0.20 s. They compare arbitrary lifted inputs and
initial states with the separate existing cascade, independently evolve complex
phasors as two real axes, attain each impulse bound with adversarial vector
signs, preserve nonzero means and correlated cancellation, verify analytic null
directions, and reject bad clocks, physics, data and evidence tampering. Initial
static checks caught formatting and array-return annotations; these were fixed
before execution without changing a mathematical tolerance.

The full warning-strict suite passes 3,918 tests in 359.04 s. Ruff, formatting
(338 files), mypy (100 source files) and documentation checks (148 files,
661 local links, 23 Python/JSON examples) pass. An additional NumPy-only saved
artifact checker verifies all 144 window Gram matrices and independently
reconstructs the lifted flow from analytic exponential integrals; its maximum
impulse-kernel difference is 1.319e-16. Publication CI results are retained with
the evidence. Exact commands are:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m pytest -q tests/unit/test_whole_flight_error_budget.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m pytest -q
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
.venv/bin/python scripts/check_docs.py
```

| Item | SHA-256 |
| --- | --- |
| Audited preceding execution source | `01d663368ed67174c9b300fdd2ef7450781da5f6ec3947e7f9a9b2d8473f2b0e` |
| Error-budget execution source | `d83889b7189ab898aadb87bfa258259ec30e381b14f0d550d207343835216f95` |
| Frozen ADR 0039 | `8675fdad2ac8943342747ca9bb66284b81a53afbabc8145ffe79608930507cd9` |
| Complete report | `b93b1fb050bab01b06b790b9bac7d795469cb99942572ea476ecfe47775798cb` |
| Preceding manifest | `1a67b992e5daa5a299bbe5cf4c31927386b71637c14172e673c82316ff768d0f` |

Generated evidence outside Git retains all Gram matrices, the complete gate
ledger, lifted matrices, phase responses, impulse coefficients and bounds,
an independent saved-artifact checker, figure and test/publication receipts.
The unchanged preceding archive supplies the full response/update inputs.
