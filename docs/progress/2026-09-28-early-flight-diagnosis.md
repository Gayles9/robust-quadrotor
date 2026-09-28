# Causal early-flight diagnosis — 2026-09-28

## Audit and frozen scope

Audited main is `ca0f226ce57006dc705b49a5e1324af871fadb97`, tree
`5cd8868211f7bedd89eb65ae040be5930e49a2f0`. Its post-merge CI run
36452258490 passed. Fresh vertical and documentation tests pass 51 tests in
5.81 s; the report audit authenticates two reports and 182 payload references
and reproduces all 48 archived flight scores. No preceding defect was found.

[ADR 0025](../decisions/0025-causal-early-flight-diagnosis.md) froze the questions,
windows, scalar-model adequacy check and one permitted oracle before execution.
Its original SHA-256 is
`57f19049cc4b74ba060c0c739c55505f6ad791bda9af3e0336c917fbf475e9cf`.
Production source is unchanged. The diagnostic execution source is
`cbda2444c76898476487dd7891f2fda3e3dfd6c9b92338d5db6d46b9e75ddaa6`.
Original flight fingerprints remain distinct; adding diagnostic scripts does not
rewrite their provenance or relax the historical verifiers.

## Vertical mechanism

NED tracking error is truth minus reference; estimation error is estimate minus
truth. The uncompensated 1.1 kg / 1.0 kg case has 41.8942 cm vertical RMS error
and only 0.6174 cm vertical estimation RMS error. The earlier PD equilibrium
calculation correctly explained the persistent offset. The new question is why
the bounded integral improves landing but still misses whole-flight RMSE.

At nominal hover thrust the mass mismatch produces 0.891818 m/s² downward
acceleration. The integral must learn -0.981 m/s² in nominal acceleration units.
Learning is health-gated until 0.2 s; the ten initial frozen outer epochs are
0..0.18 s. It then integrates continuously, without inner/outer limiting or
integral clipping. Applied correction is -0.042317 m/s² at 1 s and
-0.653459 m/s² at 5 s. The first crossings of 50%, 90% and 95% of the required
steady correction are 3.82, 8.42 and 10.44 s. Motor lag is 0.025 s, much shorter
than these learning times. That timing explains a late landing benefit with a
large retained early error.

The independently integrated scalar model uses the original sampled PD/integral
law, known mass, initial vertical state, rotor energy, saved reference and health
timing. Four equal rotors respond as `omega(t)=u+(omega0-u)*exp(-t/tau)`;
the model integrates `g-4*kf*omega(t)^2/m` analytically over each plant interval.
There is no parameter fit, attitude error, estimator noise or new controller.

| Quantity | Original mass case | Vertical candidate |
| --- | ---: | ---: |
| Actual whole-flight 3-D RMSE [cm] | 42.3933 | 17.5684 |
| Actual vertical RMS error [cm] | 41.8942 | 16.3001 |
| Scalar-model vertical RMS error [cm] | 42.0073 | 16.3875 |
| Model-to-actual vertical trajectory RMS discrepancy [mm] | 8.0938 | 8.0768 |

Both discrepancies meet the predeclared 1 cm adequacy check. This supports
vertical load learning as the main mass-case mechanism without claiming that a
scalar model predicts all axes, estimator covariance or a qualified mission.
The unchanged 15 cm whole-flight requirement still fails.
Even eliminating all horizontal error would leave 16.3001 cm vertical RMS;
an attitude-only improvement cannot be assumed to close this mass requirement.

The scalar candidate predicts a 36.2928 cm vertical peak at 2.58 s; the saved
candidate has 36.3429 cm at 2.7975 s. The timing difference is 0.2175 s.
These vertical peaks are distinct from its 37.2568 cm three-dimensional peak
at 2.28 s. The model explains scale and timing approximately, not sample exactly.

## Hover mechanism and one intervention

Original hover whole-flight axis RMS errors are north 6.2285 cm, east 4.3729 cm,
down 0.8755 cm. The peak at 5.395 s is almost entirely horizontal:
approximately `[-0.1003, 0.0389, 0.0001] m`. Initial thrust-axis estimation
error is 2.2941 degrees, grows to 3.1614 degrees at 0.2 s, then falls to
0.4992 degrees at 1 s and 0.0557 degrees at the scored peak. At 1 s, the true
vehicle is already about 9.68 cm north and 11.38 cm east of its reference.
The peak records accumulated motion, not a contemporaneous large tilt estimate.

Exactly one new full hover flight replaces only the quaternion supplied to the
inner controller with true attitude. The ESKF still runs on fresh measurements
from the changed physical trajectory. Its states, prior, position/velocity/rate
feedback and all guards keep their original contracts. The original random
streams, sensor model, reference, gains, clocks, health and supervision are fixed.

| Metric | Saved original | Attitude-only oracle |
| --- | ---: | ---: |
| Peak over inclusive 5..11 s [cm] | 10.7563 | 7.2291 |
| Whole-flight RMSE [cm] | 7.6605 | 4.4700 |
| Final position error [cm] | 3.9518 | 2.5448 |
| Completion time [s] | 15.5 | 15.5 |
| Inner / outer / rotor-bound contacts | 0 / 0 / 0 | 0 / 0 / 0 |

The peak reduction is 32.7920%; RMSE falls by about 41.65%. The oracle's peak
occurs at 5.5125 s. Its 7.2291 cm result is below the unchanged 8 cm requirement,
but **cannot count as qualification** because truth feedback is unavailable to
the real controller. Position/velocity estimation errors remain, and the
counterfactual does not partition nonlinear causes into additive percentages.
It also does not isolate initial alignment from later attitude-estimation error.

After removing each trajectory's physical signal, accelerometer, gyro and slow
sensor random draws match within respectively `7.11e-15`, `4.17e-17` and
`2.23e-16` in their native units. Bias walks match exactly. Both streams become
healthy at the original times: altitude 0.04 s, local position 0.2 s, without a
later unhealthy transition. No fault response or actuator limit explains this
hover failure.

The older [startup diagnostic](2026-09-24-feedback-startup-diagnostic.md) used
different gains and observed seeds, and found a larger effect from position and
velocity interventions. This result does not supersede that fixture-specific
finding or reopen the geometric study.

## Verification and evidence

The new software checks independent quaternion algebra, exact motor integrals
against numerical quadrature, NED force signs, hold boundaries, original scoring
windows, hash rejection, channel isolation, adapter restoration after failure,
actual short-run oracle commands, and positive/negative random-draw pairing.
The 24 new tests pass in 4.04 s. No production behavior or test threshold changed.

The archived analysis independently scores all 48 original executions, then
reconstructs ESKF state/covariance/events, health, guards, outer/inner commands
and integral traces for the four supervised mass/hover histories. The oracle
also authenticates and reconstructs every command with its declared substituted
input. Saved nonlinear plant and exact motor histories are checked interval by
interval. The frozen protocol, full oracle history, input trace, derived signal
arrays, plots and digests are preserved outside Git.

Exact local commands use the existing pinned-dependency Python environment
directly because global uv is 0.12.18 rather than the repository's pinned 0.12.3:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q
.venv/bin/python -m ruff check .
.venv/bin/python -m ruff format --check .
.venv/bin/python -m mypy src experiments scripts
.venv/bin/python scripts/check_docs.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m experiments.early_flight_diagnostic \
  --baseline BASELINE/campaign --candidate VERTICAL/campaign --output NEW_ANALYSIS
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m experiments.early_flight_oracle \
  --baseline BASELINE/campaign --output NEW_ORACLE
```

Ruff passes with 256 files formatted; strict mypy passes all 76 source files.
Documentation checks pass 106 Markdown files, 460 local links and 23 executable
Python/JSON examples.
The complete warning-strict test gate and hosted CI are required before merge;
their exact results and publication identities are recorded in the associated
pull request. The source fingerprint above binds the diagnostic executions to
that published implementation.

## Decision and next step

The data-reconstruction and attribution acceptance criteria are met. All five
saved original/candidate/oracle plant and motor reconstructions have exactly
zero residual, covering 35,600 plant intervals. The scope establishes two
testable mechanisms and real improvement headroom; it does not deliver a new
flight-qualified controller. Software publication remains subject to the gate
above.

**Go:** one bounded audit of the attitude-estimation startup path, using the saved
original and oracle data. Reconstruct which propagation and observation updates
produce the early roll/pitch error, check its covariance/observability, and decide
whether one causal measurement-only correction can be justified. Freeze its
interface and unchanged full-flight acceptance before implementation.

**No-go:** production changes or another gain/cutoff trial based on this oracle
alone. Do not assume a stationary accelerometer alignment while this simulation
is freely flying, replace the prior with truth, or treat an availability-health
flag as an estimator-accuracy guarantee. A negative observability/design result
must be retained without adding sensors or retuning noise silently.

Mass compensation still needs a separate future load-estimation/transient design.
Improving attitude estimation alone cannot be assumed to remove its force deficit.
Keep cascade default, keep vertical compensation research-only, and leave the
closed geometric study and reserved seeds unchanged.
