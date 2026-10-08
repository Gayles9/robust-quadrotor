# Final bounded geometric comparison

This study compares geometric control with the nested position/attitude
controller (the cascade) under matched simulation conditions. It smooths the
geometric controller's requested force more slowly in the horizontal plane than
vertically, allowing a faster vertical response after release while limiting
horizontal sensitivity to noisy estimates. This is called axis-dependent force
shaping.

The [supported-start comparison](supported-geometric.md) explains the
configuration using the same filter on every axis. The
[study protocol](../decisions/final-geometric-comparison.md) defines the
search, independent comparator selection, reserved evaluation cases, metrics
and acceptance rules.

## Tracking result on reserved cases

The selected geometric controller has **4.7717% lower balanced whole-flight
RMSE** than the independently selected cascade on twelve reserved cases.
Here RMSE means root-mean-square position error over the complete flight;
"balanced" gives hover, nominal tracking and wind tracking equal weight.
All twenty-four selected-controller clean flights pass the original absolute
conditions. This is a measured average accuracy benefit, with retained
tradeoffs; the predeclared 10% balanced-improvement target is not met.

| Metric / category | Geometric change versus selected cascade |
| --- | ---: |
| Balanced whole-flight RMSE | 4.7717% lower |
| Nominal spline RMSE | 13.7942% lower |
| Mild-wind spline RMSE | 1.4117% lower |
| Full-hover whole-flight RMSE | 1.6100% higher |
| Balanced whole-flight peak error | 15.2880% higher |
| Balanced actual squared-moment effort | 1.6736% higher |

Every nominal trajectory improves; wind improves in two cases and regresses in
two. The largest individual RMSE regression is 2.2809%, within the prospectively
allowed 10%. Each geometric full-hold peak is slightly lower than its matched
cascade peak; all four lie between 4.8540 and 5.7295 cm, below the unchanged 8 cm
limit. Hold maxima and whole-flight maxima cover different intervals and must
not be interchanged. Maximum individual effort ratio is 1.196242, below two.

The selected settings are horizontal frequency 1.25 rad/s for both controllers
and geometric filter poles `(10,10,30)` rad/s. Cascade gains were selected from
the same two outer-gain choices; this is not an exhaustive optimization of
either controller family. Development's 7.0703% gain falls to 4.7717% on fresh
cases, reinforcing the need to separate selection results from validation.

Against the original cascade settings, balanced RMSE is 20.2899% lower, with
23.8136% higher moment effort and 7.1598% higher balanced peak error. That
secondary comparator passes eleven of twelve absolute checks: wind seed 96005
ends 8.01836 cm from its target, 0.1836 mm above the 8 cm cap. Both selected
controllers pass that case. This small miss is retained at its actual scale.

The predeclared descriptive stratified bootstrap gives a balanced RMSE-ratio
interval of `[0.929789, 0.974011]`, corresponding to 2.60..7.02% improvement.
It describes resampling sensitivity within these twelve cases, with only four
per category. It is neither an additional acceptance gate nor a general
operating-envelope guarantee. The data support a modest bounded accuracy gain;
they do not meet the stronger predeclared 10% claim or dominate every metric.

The complete reserved ledger contains 36 clean executions and sixteen fault
executions. All eight fault-response comparisons and all saved-history audits
pass. Of the eight primary acceptance conditions, only balanced accuracy fails.
The original cascade's single small absolute miss is confined to the declared
secondary comparison. Both selected profiles pass every clean absolute limit.

The [complete numerical record (ZIP)](../../evidence/development-records.zip)
retains every seed and metric, the original-cascade comparison and all fault
responses. The results apply to these tested conditions and do not change the
default controller.

## Why separate the axes

For the feedback force $`\mathbf c=m(K_p\mathbf e_p+K_v\mathbf e_v)`$, each axis has three states:

```math
\begin{aligned}
\dot{\mathbf x}_1&=\omega_f(\mathbf c-\mathbf x_1),\\
\dot{\mathbf x}_2&=\omega_f(\mathbf x_1-\mathbf x_2),\\
\dot{\mathbf x}_3&=\omega_f(\mathbf x_2-\mathbf x_3).
\end{aligned}
```

Here $`\omega_f`$ is the filter pole for the axis being filtered and $`\mathbf x_1,\mathbf x_2,\mathbf x_3`$ are the three filter states.

Use `x3`, `w(x2-x3)` and `w²(x1-2x2+x3)` as the feedback force and its first
two derivative jets. The planned acceleration, jerk and snap remain analytic.
Applying different positive poles to different axes preserves these identities
axis by axis. The resulting lift determines the desired body-down direction;
the existing analytic normalization/cross-product derivatives supply the
rotation, angular rate and angular acceleration to the geometric moment law.

The three-section low-frequency delay is approximately `3/w`. The previous
10 rad/s pole introduces about 0.3 s on every axis. A 30 rad/s vertical pole
reduces that value to about 0.1 s. Vertical collective control has a different
response from horizontal motion through the attitude dynamics. Increasing
horizontal bandwidth also exposes more estimator-correction noise, so this
experiment tests only three declared horizontal poles: 10, 15 and 20 rad/s.

The equal horizontal poles also give a coordinate interpretation. If `n` is
the unit gravity direction, the transfer operator is
$`H=H_h(I-\mathbf n\mathbf n^T)+H_v\mathbf n\mathbf n^T`$. It filters the horizontal plane identically and
the gravity direction separately. It therefore commutes with rotations about
gravity. Under a general static coordinate change, rotating `n` and the force
together transforms this operator consistently. The implementation specializes
that expression to the repository's fixed NED frame; it does not alter the
rotation-based attitude error or establish a new global stability theorem.

There is also a quantitative noise cost. For one continuous section cascade,
$`\lvert(j\omega)^2 H(j\omega)\rvert=\frac{\omega_f^2 r^2}{(1+r^2)^{3/2}}`$, where $`r=\omega/\omega_f`$. Its maximum is
`2 w²/(3 sqrt(3))`, attained at $`\omega=\sqrt2\,\omega_f`$. Doubling a horizontal pole
therefore quadruples the largest correction-to-second-derivative gain. The
bilinear frequency map retains that maximum while changing its digital
frequency. Actual nonlinear moment effort also depends on correction spectra,
attitude and feedback, so this is a mechanism for the cost rather than an exact
prediction of a flight's effort ratio. The fixed effort budget prevents a
small accuracy gain from concealing a large increase in moment demand.

The bilinear discretization uses the original 20 ms outer period. Force and
derivatives use the same states, initialized with constant prehistory. The
held jets are still an approximation to a continuously evolving reference.
Both raw and shaped commands retain the original smooth-domain checks. No
true state enters an estimated controller, and an invalid update cannot commit
new filter memory. This is an experimental implementation, not a new default.

The other declared choice is horizontal frequency $`f=1`$ or `1.25 rad/s`, with
$`K_p=f^2`$ and $`K_v=1.8f`$. Vertical and attitude gains remain unchanged. In an ideal
PD equilibrium, a constant disturbance's position offset scales inversely with
Kp; the higher setting predicts a 36% reduction in that component. Estimation
error and transient coupling prevent that simple calculation from predicting
the complete flight result.

## What makes the comparison fair

Six new geometric profiles are evaluated on four known development cases.
Cascade receives both outer-loop gain settings and is selected independently.
The original geometric profile is retained as an ablation. Every arm uses
identical support physics, supported alignment, velocity conditioning, nonlinear
release prediction, sensor distributions, reference timing, clocks and limits.
Every named random draw is paired and reconstructed.

The modeled vehicle has mass 1 kg, diagonal inertia `(0.02,0.025,0.04)` kg m²,
25 ms motor time constant and a 2.5 ms plant/estimator step. Attitude commands
update every 10 ms and position commands every 20 ms. The mild-wind vector is
`(0.5,-0.3,0)` m/s in NED, acting through the unchanged quadratic drag model.
These specific simulated conditions bound the interpretation of the results.

Selection weights hover, nominal tracking and wind tracking equally, averaging
log RMSE ratios within each category. This prevents two development hovers from
receiving twice the weight of either trajectory category. Profiles must first
pass all original absolute physical conditions. Geometric effort must remain
within twice the selected cascade on every development case. A 5% balanced
development improvement is required to open subsequent checks.

The selected profiles then remain locked. Seven existing true-state regression
cases check original flight limits and numerical refinement. Conditional fresh
evaluation contains four hovers, four nominal splines, four wind splines and
eight supervisor off/on fault pairs. The primary opponent is the independently
selected cascade; the original cascade is also retained when different.

The predeclared fresh accuracy-advantage criterion requires at least 10% lower
balanced RMSE, no category more than 5% worse and no individual case more than
10% worse. All original physical conditions, fault responses and evidence audits
must pass. Balanced squared-moment effort may increase by at most 25%, with no
individual ratio above two. These are explicit engineering tradeoffs. Passing
this criterion would not imply lower peak error or lower effort everywhere.
The full record reports every paired metric and any regressions.

Actual squared-moment integral is a control-effort proxy, not electrical energy
or battery consumption. The twelve fresh cases sample this simulation setup;
they do not establish a general mathematical superiority theorem or hardware
qualification. The supported start is an explicit stationary fixture assumption.

## Implementation and reproduction

- [Axis shaping and sampled maps](../../experiments/axis_shaped_geometric.py).
- [Fixed study runner and selection](../../experiments/final_geometric_comparison.py).
- [Independent filter/plant tests](../../tests/unit/test_axis_shaped_geometric.py).
- [Selection, replay and seed-gate tests](../../tests/unit/test_final_geometric_comparison.py).
- [Verification record (ZIP)](../../evidence/development-records.zip).

Use new output directories. Later stages authenticate all parent payloads and
the fixed source/decision before constructing their seed-dependent fixtures:

```bash
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error python -m experiments.final_geometric_comparison --stage development --workers 2 --output results/final-geometric-development
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error python -m experiments.final_geometric_comparison --stage regression --workers 2 --development results/final-geometric-development --output results/final-geometric-regression
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error python -m experiments.final_geometric_comparison --stage validation --workers 2 --development results/final-geometric-development --regression results/final-geometric-regression --output results/final-geometric-validation
```

Replace `--output` with `--verify` to authenticate saved payloads and reconstruct
the corresponding report. Verification of estimated flights replays estimator,
sample-noise memory, controller, plant, supervision and support. True-state
verification authenticates and rescores the saved histories. A failing gate
returns exit code 1 while preserving the complete stage ledger.

Saved reports require matching execution source identities:

| Evidence | Required source commit |
| --- | --- |
| Saved development report | `c97d4786f6bb729cddd2b1af95d65410f4d12eca` |
| Corrected regression and reserved validation | `6e79406b8bc75be50ce4304dd7d403f3c751c5a2` |

A complete new reproduction can run all stages at the corrected commit. The
[adapter compatibility record](../decisions/geometric-regression-adapter.md)
identifies the unchanged development report accepted by the corrected runner.
The retained failed regression attempt is a call-interface error, not a valid
flight outcome; it does not alter the scientific settings or reported results.
