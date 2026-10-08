# Early-flight diagnosis

The current mass and hover failures have different mechanisms. The bounded
[ADR 0025 investigation](../decisions/0025-causal-early-flight-diagnosis.md) separates
them without changing a production controller, seed, scoring window or limit.
The [dated evidence](../archive/records/early-flight-diagnosis.md) retains the
measured results and their limits.

## Mass transient

With 1.1 kg truth, 1.0 kg nominal mass and nominal hover thrust, the initial
downward acceleration deficit is `9.81*(1-1/1.1) = 0.891818 m/s²`.
The vertical integral eventually supplies the required negative nominal
acceleration, approximately -0.981 m/s². Starting from zero, it first needs
position error to accumulate. Its healthy learning begins at 0.2 s, but 90%
of the steady correction is not applied until 8.42 s in the saved case.

An independent scalar model retains the original vertical gains, reference,
health timing, mass, clocks and 25 ms motor time constant. It integrates squared
rotor speed analytically and uses no fitted parameter. Despite omitting attitude
and estimator noise, it matches both saved vertical trajectories to about 8 mm
RMS. This identifies slow disturbance rejection as the main vertical mechanism;
it does not turn the failed 17.57 cm whole-flight result into a pass.

## Hover transient

The hover error is predominantly horizontal. Early attitude-estimation error
changes the direction of actual thrust relative to the controller's intended
direction. Motion accumulates before the scored hover maximum, so inspecting
only the instantaneous attitude error at the position peak is misleading.

One full original-cascade hover counterfactual substitutes true attitude only
at the inner controller call. The estimator, its prior, position/velocity/rate
feedback, guards, sensor model, random draws and mission remain fixed. The
changed trajectory naturally changes sensor values and estimator revisions.
The peak over the original 5..11 s window falls from 10.7563 to 7.2291 cm;
whole-flight RMSE falls from 7.6605 to 4.4700 cm.

This establishes performance headroom in that feedback channel for this known
case. It does not show that a sensor-only correction will achieve it, distinguish
startup alignment from later attitude revisions, or generalize to other seeds.
The older startup study used different gains/fixtures and remains separately
valid; its conclusions cannot be replaced with this single result.

## Reproduction and interpretation

From the pinned repository environment, with both original campaign archives
extracted so their `campaign` directories contain `report.json`:

```bash
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.early_flight_diagnostic \
  --baseline BASELINE/campaign --candidate VERTICAL/campaign --output NEW_ANALYSIS
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.early_flight_oracle \
  --baseline BASELINE/campaign --output NEW_ORACLE
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.early_flight_oracle \
  --baseline BASELINE/campaign --verify NEW_ORACLE
```

The first command authenticates both reports and every referenced payload,
independently reproduces all 48 archived full-flight scores, reconstructs four
saved supervised histories, and writes axis/phase/window diagnostics. The
second executes exactly one new diagnostic flight; the third replays its saved
evidence without flying again. Temporary oracle adapters restore on all exits
and must not share a process with concurrent mission threads.

The force budget uses `a = g*e3 - T*b3/m` and an explicitly ordered exact split
into the feasible request, mass mismatch, estimated-to-true direction error,
estimated-to-target direction error and motor thrust mismatch. The term named
`attitude_tracking` in the archive always means estimated-to-target direction;
in the oracle run that is not the substituted inner-feedback tracking error.
RMS values of these terms are descriptive and are not additive percentages of
closed-loop causation. Endpoint diagnostics hold the last command; they do not
represent a new terminal command.

The default remains cascade. Geometric control and vertical compensation keep
their existing research status. The subsequent [startup audit](attitude-startup-audit.md)
and supported-start studies are complete; [current results](README.md) explain
their outcomes. This diagnosis does not authorize gain tuning or controller promotion.
