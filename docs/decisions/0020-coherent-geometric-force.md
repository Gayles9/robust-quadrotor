# Decision 0020: one coherent feedback-force shaping experiment

This experiment asks whether using one filtered feedback signal for the force
and both of its derivatives improves consistency of the commanded motion.

The original reference combines raw feedback force with derivatives from a
three-section bilinear filter. Accepted estimator corrections therefore enter
the attitude target directly and enter its derivative feedforward through a
different temporal response. Earlier correction rebasing reduced effort but
worsened hover; it is not selected here.

For one experiment, use the third filter section as the feedback-force value
as well as its existing first and second derivatives. With c=m(Kp ep+Kv ev),
H(s)=(w/(s+w))^3, and bilinear implementation, use
L=m(g e3-a_ref)+Hc, Ldot=-m j_ref+sHc, Lddot=-m s_ref+s²Hc.
This is one command-shaping mechanism. Planned derivatives remain analytic.
The discrete jets approximate a continuous shaped reference; holding all jets
for 20 ms is still an approximation and is included in the local model.

The fixed cutoff is w=10 rad/s, one quarter of the 40 rad/s motor pole, with all original
position/attitude gains, sensor distributions, estimator prior, limits and
mission times unchanged. This trades approximately 0.3 s low-frequency delay
for strong attenuation near the 5 Hz local-position update frequency. It is
not an optimized cutoff or a stability theorem. The two sampled horizontal
models have spectral radii below 0.983; their slowest decay is about 0.88/s.
The raw 30 rad/s reference has a slowest decay about 0.84/s. Independent
nonlinear small-perturbation runs check the local maps.

Only causal estimated position/velocity and reference derivatives are inputs.
All sections start at the first correction, corresponding to constant prehistory.
Memory is immutable across outer ticks, and each flight uses a new instance.
This mode uses neither correction-event rebasing nor measured acceleration.
It is a process-local experiment, not a supported production-controller option.
The shaped force passes the original acceleration, tilt and thrust domain checks
before being issued, in addition to the raw-command checks.

Development uses precisely the original seed-30 spline pair and full geometric
hovers at 30 and 93012. Require all original physical conditions, both full
5..65 s hover maxima <=0.08 m, paired spline RMSE ratio <=1 and squared actual
moment effort ratio <=2. A failure ends this study, without another cutoff or
gain change. Only a passing development gate permits the original true-state
regression and frozen 40-case qualification, with unchanged limits and reserved
seeds 95000..95003 and 96000..96003. These identify the proposed protocol, not
currently unused seeds. No qualification claim follows from local poles, short
diagnostics or software tests. See the [geometric control guide](../design/geometric-control.md)
for the current evidence and supported configurations.

## Outcome

Spline RMSE is 6.77 cm and squared actual-moment effort is 0.85 times the
cascade value, but full-hover maxima are 10.87 and 17.33 cm. Both exceed the
unchanged 8 cm condition, so the proposed validation stage is not executed.
Filtering reduces sharp commands while delaying physical recovery; improving
one metric does not satisfy the combined flight requirement. The
[controller tradeoffs](../results/controller-tradeoffs.md) place this result
alongside the other derivative designs.
