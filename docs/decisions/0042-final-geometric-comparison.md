# Decision 0042: final bounded geometric improvement and fresh comparison

Frozen on 2026-09-30 UTC before implementation or new scientific flights.
Audited baseline: `5eb932766db6fc3ef5f465bf551e390633561d43` (PR #41).

## Audit and mathematical target

The preceding implementation and its 28 saved outcomes are consistent. Fresh
warning-strict checks of the supported geometric composition, geometric law,
filter and coherent shaping pass 73 tests. The working tree is clean. ADR 0041
remains a failed historical comparison; its criteria and result will not change.

Saved histories identify a specific opportunity. Combined geometric nominal
startup reaches 15.59 cm error at 0.7175 s, of which 15.10 cm is vertical. Cascade
reaches 9.02 cm. After 5 s geometric nominal trajectory tracking is substantially
better. The isotropic third-order force filter delays every feedback axis by
approximately 3/w seconds at low frequency. Vertical collective control does
not pass through the horizontal attitude dynamics and need not use the same
filter bandwidth. Faster vertical shaping may reduce the release transient.
Modestly stronger horizontal feedback may reduce the wind equilibrium error,
while faster horizontal shaping trades delay against correction noise.

Use diagonal filters H_i(s)=(w_i/(s+w_i))^3 on the feedback correction
c=m(Kp ep+Kv ev). Construct one coherent force jet:
L=m(g e3-a_ref)+Hc, Ldot=-m j_ref+sHc, Lddot=-m s_ref+s^2 Hc.
Implement each section by the existing bilinear rule and initialize all three
sections to the first correction. Planned jerk/snap remain analytic. Keep raw
and shaped force-domain checks. No clipping, future measurements, true-state
feedback in estimated flights, correction rebasing, integral, new sensor,
measured-acceleration mode, inner gain change or trajectory retiming is allowed.
The sampled local maps must be stable and independently checked; they are not
a stability theorem for the noisy nonlinear estimator/controller.

## Fixed development budget and fair comparator

Run exactly four known cases: full 5..65 s hover at seeds 30 and 93012,
nominal minimum-snap at 30, mild-wind minimum-snap at 31. Every case has nine
arms (36 flights total):

- Historical coherent geometric w=(10,10,10), horizontal frequency f=1.
- Six new geometric profiles: w=(h,h,30), h in {10,15,20} rad/s and
  f in {1,1.25} rad/s. Horizontal Kp=f^2, Kv=1.8 f; original vertical gains.
- Two cascade profiles with the same f values and original vertical/inner gains.

Both algorithms retain the existing explicit 0.5 s supported alignment, exact
one-time zero-world-velocity condition, order-five nonlinear first release
prediction and ordinary subsequent ESKF. All arms share physical support,
motors initially off, sensor distributions, clocks, limits, reference and named
random streams. Keep every outcome, including disqualified profiles. Use at
most two worker processes. Short seed-40 fixtures are software checks only.

Select geometric and cascade independently on these development cases. Eligible
profiles must pass all original absolute requirements on every case: completion,
RMSE <=15 cm, full peak <=25 cm, final error <=8 cm, final speed <=8 cm/s,
full hover peak <=8 cm, no command limiting or rotor saturation. Only the initial
supported actual rotor value may be zero; all later actual speeds and all
commands must lie strictly in (0,900) rad/s. All saved audits must pass.

For each profile, divide its per-flight RMSE by the original cascade's paired
RMSE. Average log ratios within each of hover, nominal spline and wind spline,
then average those three categories equally and exponentiate. Select the
eligible cascade with the lowest balanced ratio, breaking ties by declaration
order. Select the new geometric profile with the lowest balanced ratio among
eligible profiles whose squared actual moment effort is <=2 times that selected
cascade on every case. This compares whole flights without trimming release or
landing. The historical geometric arm is an ablation, not a selectable candidate.

Require at least a 5% balanced development RMSE improvement against the selected
cascade before opening later scientific stages. This is an opportunity gate,
not a fresh-data result. No adaptive extension of the six-profile search, new
gain, extra development seed or post-validation retuning is permitted.

## Locked checks and fresh ledger

Bind the selected profiles to the full authenticated development report, source
and this decision's hash. First run the existing seven true-state regression
cases (nominal, refined integration, positive/negative offsets, mild/reversed
wind, wind plus offset), each with the selected geometric and cascade profiles:
14 flights. Require original absolute flight limits and geometric coarse/fine
agreement <=5 mm position and <=0.05 deg attitude at common epochs. This checks
the new force shaping on the existing true-state fixtures, including their
original freely flying initial conditions; it is not hardware qualification.

Only passing development and true-state regression permit construction of fresh
seed-dependent configurations. Run full hovers at 95000..95003, nominal splines
at 96000,96002,96004,96006 and mild-wind splines at 96001,96003,96005,96007.
Each has the selected geometric, selected cascade and original cascade, with
the original cascade deduplicated when it is selected: 24 or 36 clean flights.
Then run all eight existing candidate supervisor off/on pairs at seed 97000:
16 fault flights. Complete this entire fresh ledger even if a comparison fails.

The primary comparison is against the independently selected cascade; the
original cascade is a declared secondary comparison. The primary bounded
accuracy-advantage claim requires all of:

- Every candidate and primary comparator clean flight meets the original
  absolute conditions, all saved audits pass, and all eight fault-response
  comparisons pass the original causal/timing/recovery conditions.
- Balanced fresh RMSE ratio <=0.90 (at least 10% lower), each category's
  geometric-mean ratio <=1.05, and no individual RMSE ratio >1.10.
- Per-flight actual moment effort ratio <=2 and balanced effort ratio <=1.25.

The 5% category tolerance and 10% individual tolerance acknowledge small
stochastic tradeoffs while requiring a larger 10% overall improvement. They are
prospective engineering preferences, not revised interpretations of ADR 0041
or significance thresholds. The effort bounds distinguish an accuracy benefit
at an acceptable declared cost from universal metric dominance. Report all
peak, final error, speed, effort and individual paired regressions regardless
of the primary decision. Separately report whether all paired accuracy, peak
and effort values dominate cascade; never conflate these claims.

Report a descriptive paired, category-stratified percentile bootstrap interval
for the balanced RMSE ratio (20,000 resamples, analysis seed 424200). It quantifies
sensitivity within these twelve sampled cases, not general operating-envelope
coverage; it is not an additional acceptance gate. No inference from local poles
or this small simulated set establishes universal superiority or a new theorem.

## Evidence and stopping

Retain complete flight histories, configurations, conditional IMU memory,
support data, fault/health decisions and source/protocol digests. Authenticate
and reconstruct ESKF, controls, plant, random draws and first-release quadrature
for every estimated flight. Independently rescore saved arrays. Check the new
filter algebra, causality, reset, invalid-domain behavior, isotropic compatibility
and sampled maps before scientific execution. Do not modify executable source
during any stage; a software defect requires a documented amendment and makes
affected evidence unusable for this frozen study.

Publish the implementation and all outcomes. Default promotion remains a
separate integration decision. Stop controller improvement after this bounded
study regardless of outcome. The next requested work is the professor-facing
report, in a later task; do not prepare or send it in this step.
