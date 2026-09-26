# Decision 0020: one coherent feedback-force shaping experiment

Frozen before candidate flight execution on 2026-09-26. Baseline: `60ca2a81b61e1def8aebc41cf84c00f633700fba`.

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

Freeze w=10 rad/s, one quarter of the 40 rad/s motor pole, with all original
position/attitude gains, sensor distributions, estimator prior, limits and
mission times unchanged. This trades approximately 0.3 s low-frequency delay
for strong attenuation near the 5 Hz local-position update frequency. It is
not an optimized cutoff or a stability theorem. The two sampled horizontal
models have spectral radii below 0.983; their slowest decay is about 0.88/s.
The raw 30 rad/s reference has a slowest decay about 0.84/s. Verify those maps
against independent nonlinear small-perturbation runs before flight evaluation.

Only causal estimated position/velocity and reference derivatives are inputs.
Initialize all sections to the first correction (constant prehistory), retain
immutable memory across outer ticks, and reset with a new instance per flight.
Do not rebase correction events or use measured-acceleration mode. Keep the
experiment process-local; do not add a supported production-controller option.
Check the shaped force against the original acceleration, tilt and thrust
domain before issuing it, in addition to the existing raw-command checks.

Development uses precisely the original seed-30 spline pair and full geometric
hovers at 30 and 93012. Require all original physical conditions, both full
5..65 s hover maxima <=0.08 m, paired spline RMSE ratio <=1 and squared actual
moment effort ratio <=2. A failure ends this study, without another cutoff or
gain change. Only a passing development gate permits the original true-state
regression and frozen 40-case qualification, with unchanged limits and reserved
seeds 95000..95003 and 96000..96003. No qualification claim follows from local
poles, short diagnostics or software tests.
