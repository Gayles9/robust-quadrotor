# Why the combined prior changes flight tradeoffs

The combined startup treatment meets the 8 cm limit on all three hover peaks,
yet five of nine clean comparisons regress against at least one control. This
diagnosis asks how a changed initial covariance redistributes estimator
corrections and controller feedback. Exact replay establishes reproducibility;
it does not imply that better prior information improves every noisy trajectory.

## Inputs and scope

The analysis uses all nine clean jobs—hover, nominal tracking and wind at seeds
47001..47003—in original aligned, boundary-only and combined form: 27 complete
histories. Full campaign ledgers of 34/25/25 flights and their fault outcomes
remain part of authenticated evidence. Each selected history uses its own
initial covariance and first-prediction rule.

| Input report | SHA-256 |
| --- | --- |
| Original | `b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63` |
| Boundary-only | `54f848a0593cef3abe705bb713db97eb584299d3d74fedac91ab309df4ff9cf2` |
| Combined | `6d44ab913156fafe3846a630065cdaad9eee2dc9c64c1ea50234b82fbfa314fa` |

This is saved-data analysis, with no new scientific flights, oracle intervention,
gain search, covariance inflation or measurement-gate changes. Truth is used
only for offline physical/error accounting. Support assumptions, controller,
noise, defaults and earlier acceptance decisions are unchanged.

## Estimator and feedback accounting

At every fresh observation, an independent calculation solves the full 21-state
Gaussian gain, correction and Joseph covariance, followed by the existing
independent rotation-reset quadrature. It compares correction, covariance,
sample-noise mean and normalized innovation squared (NIS) with the actual
update. Rejected observations have zero effective gain. Position and altitude
updates retain their order.

Each innovation is split into saved sensor noise and physical prediction error.
For paired effective gains and innovations, the ordered identity is
$`\Delta\mathbf c=(K_1-K_0)\boldsymbol\nu_0+K_1\Delta\boldsymbol\nu`$ (subscripts 0 and 1 denote control and candidate; $`\Delta\boldsymbol\nu=\boldsymbol\nu_1-\boldsymbol\nu_0`$).
This separates a change in gain from a change in the innovation it acts on.
It is an accounting identity in each update's coordinates, not a prediction of
a filter with a noise source removed. Matched events use the common time prefix,
with differing dispositions and durations reported explicitly.

Requested acceleration is decomposed into reference forcing, true-state PD
feedback, and negative position-error and velocity-error feedback. The
sample-held response also retains attitude estimation/tracking, motor response,
drag, limits, mass and integration remainder.

For true tracking error `e` and additive response channels `e_i`, signed
mean-square-error contributions are `mean(e dot e_i)`. Their sum is
`mean(e dot e)`. Complete flight durations remain in the comparison. Signed
contributions can cancel, so they are neither additive norms nor causal
percentages.

For the nine hover histories, the near-level sampled cascade model uses original
gains, inertia, motor lag and clocks without fitting. It reports horizontal
RMS/peak discrepancy and paired trajectory changes. Its explanatory agreement
criterion is RMS discrepancy at most 0.002 m in every hover. Failure of this
approximation is distinct from failure of the exact algebra or of flight limits.

## Reconstruction requirements

All 84 saved campaign outcomes are rescored, and all 27 clean histories are
reconstructed, including commands, guards, plant/motors, paired noise and
first-prediction traces.

| Quantity | Maximum discrepancy |
| --- | --- |
| Independent corrections, covariance and sample means | 1e-10 |
| NIS | 1e-9 |
| Gain/innovation and requested-acceleration identities | 1e-12 |
| Force identity | 1e-11 |
| Full response | 1e-10 |
| Signed MSE sum | 1e-12 m² |

Independent fixtures check hand-computable signs, covariance-induced gain
changes, noise/error decomposition, rejection, ordered updates, correlated
cancellation, malformed dimensions/clocks and evidence tampering.

The conclusion separates an explained tradeoff from population optimality.
Finite saved trajectories cannot establish the latter. A successful diagnosis
can find no justified further startup modification while preserving the failed
no-regression gate. The [whole-flight error budget](whole-flight-error-budget.md)
examines what additional information the remaining error would require.
