# ADR 0025: bounded causal early-flight diagnosis

Status: frozen before diagnostic implementation and counterfactual execution,
2026-09-28. This is an investigation, not a controller candidate.

## Preceding audit

Audited main is `ca0f226ce57006dc705b49a5e1324af871fadb97`, tree
`5cd8868211f7bedd89eb65ae040be5930e49a2f0`. Its post-merge CI passed
(run 36452258490). Fresh warning-strict vertical and documentation tests pass
51 tests in 5.81 s. Both archived campaign ZIP digests match their published
identities; the report audit authenticates two reports and 182 payload references,
recomputes all 48 full-grid execution metrics and the declared mass windows.
No preceding implementation defect is demonstrated.

The existing Python 3.12.14 / NumPy 2.5.2 environment is used directly for local
checks because the workspace's global uv is 0.12.18, while the repository pins
0.12.3. No dependency, pin or system configuration is changed. Hosted CI retains
the pinned uv environment.

## Fixed inputs and questions

Use the original ADR 0023 and vertical ADR 0024 campaigns, authenticating their
reports and every referenced payload before analysis. Retain their original
execution fingerprints separately from the diagnostic source fingerprint.
Analyze supervised mass and hover histories in both campaigns; their off/on
parity is already established. Keep full-flight scoring and the inclusive
5..11 s hover window exactly as recorded.

Diagnostic windows are full flight, 0..1 s, 1..5 s, 5..11 s and the last five
seconds, plus each saved mission phase. These overlapping windows explain
timing; they do not replace any flight score. Use NED true-minus-reference
tracking error and estimated-minus-true estimation error, reporting all axes.

Reconstruct nominal requested force, actual rotor thrust and world direction,
commanded direction, true/estimated attitude, motor speed response, limit flags,
health transitions and integral state. Check force and controller signs against
independent algebra. Distinguish exact force-budget identities from additive
closed-loop causal claims.

The mass hypothesis is a positive-down acceleration deficit from 1.1 kg truth
and 1.0 kg nominal mass, reduced only after the initially zero integral learns
negative acceleration. Test its transient magnitude and timing with an
independent scalar vertical model using the original gains, clocks, reference,
health gating and motor time constant. Compare both original and compensated
histories. Do not fit parameters. Model discrepancy must be reported; use a
1 cm vertical trajectory RMS discrepancy as the explanatory adequacy check.

The separate hover hypothesis is that attitude-estimation error changes actual
thrust direction before the scored position excursion. If saved reconstruction
does not rule this out, run exactly one original-cascade hover counterfactual:
replace only the quaternion delivered to the inner attitude controller with
instantaneous true attitude. Keep the ESKF, its prior, position/velocity/rate
feedback, noisy sensor streams, reference, clocks, limits and supervision fixed.
The attitude-domain guard must keep its original estimated input. This oracle
is diagnostic only: it is not implementable flight feedback or qualification.
Its changed physical path naturally changes measurements and ESKF revisions;
the random draws and sensor model remain fixed. Save the full execution and
oracle input trace, verify the substitution and restore adapters on exit.
No other oracle, gain, cutoff, prior or seed trial is authorized by this record.

## Acceptance and decision

1. Authenticate inputs and reproduce original full-flight metrics independently.
2. Validate force/motor reconstruction and diagnostic model with independent
   analytic oracles, and test adapter restoration and channel isolation.
3. Report signs, magnitudes, timing, competing explanations and model limits.
   Retain a failed model or oracle outcome without another trial.
4. Give an explicit go/no-go and scope at most one justified next candidate or
   a further bounded identification step. A diagnostic pass is not a flight pass.
5. Preserve reproducible evidence outside Git and pass focused checks, complete
   software checks and hosted CI before publishing the closeout.

Production controllers stay unchanged. Cascade remains default; the rejected
vertical option and geometric controller retain their current research limits.
No qualification seeds, shortened scoring, threshold relaxation, controller
implementation or reopening of the closed geometric study belongs to this step.
