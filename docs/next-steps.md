# Next-step plan

The next task is a bounded diagnosis of the remaining estimated-feedback
controller problem. The six-profile study is closed. I will use its retained
failures to decide whether one further change has a defensible physical basis,
then either validate that change or keep the existing limited reference and
move on. Repeating a broad gain search is not the next milestone.

This plan follows the [project review](project-review.md) and
[controller tradeoff analysis](controller-tradeoffs.md). It covers the existing
Python simulation stack. It does not include new sensors, a different estimator,
automatic startup alignment, middleware, hardware or relaxed flight criteria.

## 1. Establish the comparison and evidence

Start from the merged controller implementation in
`29a37b017114b58810ea8f7535448934365e39bc` and the current documentation changes.
Record the actual checkout commit, source fingerprint, dependency versions and
protocol before executing anything. Confirm that relevant mathematical tests
and saved-payload authentication pass.

Use the existing seed-30 spline and full hovers at seeds 30 and 93012. These are
development cases. Retain the fixed cascade comparator, original geometric
controller and failed candidate results. Retrieve the archived full histories;
if they are unavailable or fail authentication, reproduce the declared original
fixtures before drawing new conclusions. Do not treat a summary table as a raw
trajectory.

Deliverable: one authenticated comparison manifest tying each trace to its
configuration, source and original score. No new validation seeds are opened.

## 2. Explain the coupled transient

For the startup and first failing hover excursion, align the following signals
on their actual timestamps:

- true tracking error and estimated-state error;
- accepted estimator position/velocity corrections;
- desired force, its filtered derivatives and desired thrust axis;
- requested feedback and feedforward moments;
- allocated moment, motor response and actual moment.

Use truth only to evaluate and diagnose the simulation. Any proposed runtime
signal must be available causally from measurements and estimator state.
Reconstruct commands from those inputs and check them against the recorded
commands before interpreting a decomposition.

Extend the existing local coupled-loop reasoning to include the selected
geometric reference dynamics, sample/hold timing and motor lag. Check a local
prediction against small-perturbation simulations; do not transfer a
continuous-time damping result directly to the sampled noisy system.

Deliverable: an explanation of which error is generated first, which mechanism
prolongs it, and what a specific change is expected to improve or worsen.
If the evidence does not justify a candidate, stop here and record that result.

## 3. Freeze at most one candidate

Write a short decision record before implementation. State the mathematical
change, causal inputs, memory/reset behavior, expected benefit and likely cost.
Choose one parameter set from that reasoning. Do not open another multidimensional
gain sweep or combine several untested mechanisms.

Preserve sensor distributions, prior, plant, scoring windows, failure accounting
and all original thresholds. Add independent tests for the changed equation or
timing behavior, including its relevant frame/sign and invalid-input cases.
Keep the existing default available for exact comparison.

Deliverable: one optional, reproducible candidate with its assumptions and tests.

## 4. Apply the existing gates in order

| Gate | Required result | If it fails |
| --- | --- | --- |
| Known development cases | Both full hovers ≤8 cm over every sample from 5 to 65 s; spline RMSE no worse than the paired cascade; actual squared-moment effort ≤2 times cascade; all original physical and limiting requirements pass | Reject the candidate and close this study |
| Original true-state regression | Preserve all seven comparisons, original-five mean RMSE ratio ≤0.80, timestep refinement and exact repeat conditions | Reject the candidate; investigate only a demonstrated implementation defect |
| Frozen qualification matrix | Run the declared 40-case matrix once, including reserved hover seeds 95000–95003 and paired spline seeds 96000–96003; retain comparator failures separately and satisfy every candidate requirement | Record the complete negative result; do not tune against those newly observed seeds |
| Publication | Software and documentation checks pass; all saved payloads authenticate; results and limitations agree with the code | Correct an identified defect before publication |

The qualification runner is already implemented. It is not evidence that its
40 cases have been executed for a new candidate. The reserved seeds remain
unopened at the time of this plan.

## 5. Close the decision and continue the project

If the candidate passes all applicable gates, publish the evidence and make an
explicit controller-selection decision. If it fails, retain the cascade and
original geometric implementation as limited references, document the operating
boundary, and end this tuning effort. Do not relabel the open flight gate as
complete.

The following separate task should define a small health-monitoring interface
around the existing estimator event diagnostics and mission guards. Its design
must distinguish a single rejected observation from persistent loss of usable
information before adding any degraded-flight policy. Broad evaluation, custom
ROS 2/PX4 integration and hardware work follow their own acceptance criteria.
The technical report can then be updated from the current guides and verified
results, including the controller limitations.
