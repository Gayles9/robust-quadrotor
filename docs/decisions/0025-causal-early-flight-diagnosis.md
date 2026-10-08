# ADR 0025: bounded causal early-flight diagnosis

This diagnostic study separates two problems: persistent vertical error under
mass mismatch and early horizontal error during hover. It reconstructs saved
forces and estimation errors, then uses a limited model and one controlled
simulation change to test possible causes. It does not introduce a controller.

## Evidence boundary

The two baseline campaign reports authenticate 182 payload references. The
diagnostic recomputes all 48 full-grid execution metrics and the declared mass
windows before interpreting them. The original campaign identities remain
separate from the diagnostic source identity.

## Fixed inputs and questions

The analysis uses the original ADR 0023 and vertical ADR 0024 campaigns after
authenticating their reports and every referenced payload. Original execution
fingerprints remain separate from the diagnostic source fingerprint. It examines
supervised mass and hover histories, whose off/on parity is established, using
the recorded full-flight scoring and inclusive 5..11 s hover window.

Diagnostic windows are full flight, 0..1 s, 1..5 s, 5..11 s and the last five
seconds, plus each saved mission phase. These overlapping windows explain
timing; they do not replace any flight score. Use NED true-minus-reference
tracking error and estimated-minus-true estimation error, reporting all axes.

The diagnostic reconstructs nominal requested force, actual rotor thrust and
world direction, commanded direction, true/estimated attitude, motor speed
response, limit flags, health transitions and integral state. Independent
algebra checks force and controller signs. Exact force-budget identities describe
how forces combine; they do not by themselves establish additive closed-loop
causes.

The mass hypothesis is a positive-down acceleration deficit from 1.1 kg truth
and 1.0 kg nominal mass, reduced only after the initially zero integral learns
negative acceleration. An independent scalar vertical model tests its transient
magnitude and timing using the original gains, clocks, reference, health gating
and motor time constant. The comparison includes both original and compensated
histories without fitted parameters. A 1 cm vertical trajectory RMS discrepancy
is the explanatory adequacy criterion, and model discrepancies remain visible.

The separate hover hypothesis is that attitude-estimation error changes actual
thrust direction before the scored position excursion. One original-cascade
hover counterfactual replaces only the quaternion delivered to the inner
attitude controller with instantaneous true attitude. The ESKF, its prior,
position/velocity/rate feedback, noisy sensor streams, reference, clocks,
limits and supervision remain fixed.
The attitude-domain guard keeps its original estimated input. This oracle
is diagnostic only: it is not implementable flight feedback or qualification.
Its changed physical path naturally changes measurements and ESKF revisions;
the random draws and sensor model remain fixed. The saved evidence includes the
full execution and oracle input trace. Verification checks the substitution,
and adapters are restored on exit.
The comparison changes only this attitude input; gains, filter cutoff, prior
and seed are not additional experimental variables.

## Verification and interpretation

1. Input authentication and independent recomputation check the original
   full-flight metrics.
2. Independent analytic references check force/motor reconstruction and the
   diagnostic model; tests cover adapter restoration and channel isolation.
3. Results include signs, magnitudes, timing, competing explanations and model
   limits. A failed model or oracle outcome is retained without another trial.
4. Evidence for a mechanism is separate from evidence of improved flight. A
   successful diagnosis is not a controller-performance pass.
5. Complete evidence and focused reconstruction tests complement the full
   software checks.

Production controllers stay unchanged. Cascade remains default; the rejected
vertical option and geometric controller retain their current research limits.
The [early-flight results](../results/early-flight-diagnosis.md) explain the
measured mechanisms and link the resulting startup investigation.
