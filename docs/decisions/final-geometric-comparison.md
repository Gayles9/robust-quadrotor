# Final geometric and cascade comparison

The final study tests a specific tradeoff: faster vertical force shaping may
reduce startup delay, while modestly stronger horizontal feedback may reduce
wind-tracking error. Geometric and cascade profiles are selected independently
on development cases, then compared on separate cases. This avoids giving one
controller a tuning advantage by fixing the other at a weaker profile.

## Motivation and controller model

Saved nominal geometric startup reaches 15.59 cm position error at 0.7175 s,
including 15.10 cm vertically; the cascade reaches 9.02 cm. After 5 s, geometric
nominal trajectory tracking is substantially better. An isotropic third-order
force filter delays each feedback axis by approximately `3/w` seconds at low
frequency. Vertical collective control does not pass through horizontal
attitude dynamics, so it need not share their filter bandwidth. Faster
horizontal shaping trades reduced delay against greater correction noise.

Diagonal filters $`H_i(s)=\left(\frac{\omega_i}{s+\omega_i}\right)^3`$ act on feedback correction
$`\mathbf c=m(K_p\mathbf e_p+K_v\mathbf e_v)`$. They form one coherent force jet:

```math
\begin{aligned}
\mathbf L&=m(g\mathbf e_3-\mathbf a_r)+H\mathbf c,\\
\dot{\mathbf L}&=-m\mathbf j_r+sH\mathbf c,\\
\ddot{\mathbf L}&=-m\mathbf s_r+s^2H\mathbf c.
\end{aligned}
```

Here $`\mathbf a_r,\mathbf j_r,\mathbf s_r`$ are reference acceleration, jerk, and snap. The scalar $`s`$ is the Laplace variable; $`\mathbf s_r`$ is a trajectory derivative.

Each section uses the existing bilinear rule and initializes to the first
correction. Planned jerk and snap remain analytic, with raw and shaped
force-domain checks. The design includes no clipping, future measurements,
true-state feedback in estimated flights, correction rebasing, integral,
new sensor, measured-acceleration mode, changed inner gains or trajectory
retiming. Sampled local maps must be stable and independently checked; that
is not a stability theorem for the nonlinear noisy closed loop.

## Development cases and selection

Four known cases are used: full 5..65 s hovers at seeds 30 and 93012, nominal
minimum-snap at seed 30 and mild-wind minimum-snap at seed 31. Each has nine
arms, for 36 flights:

| Profiles | Parameters |
| --- | --- |
| Historical coherent geometric reference | $`\boldsymbol\omega_f=(10,10,10)`$, horizontal frequency $`f=1`$ |
| Six new geometric profiles | $`\boldsymbol\omega_f=(h,h,30)`$, `h in {10,15,20}` rad/s, `f in {1,1.25}` rad/s |
| Two cascade profiles | The same two `f` values |

Horizontal gains are $`K_p=f^2`$ and $`K_v=1.8f`$; original vertical and inner gains
remain unchanged. Every arm uses the same 0.5 s supported alignment, exact
one-time velocity condition, nonlinear first-release prediction and ordinary
later estimator. Physical support, stopped initial motors, sensor distributions,
clocks, limits, reference and named random streams are paired. All profiles,
including disqualified ones, remain in the evidence. Execution uses at most
two processes; short seed-40 runs are software checks only.

An eligible profile passes every absolute requirement on every case: completion,
RMSE at most 15 cm, full peak at most 25 cm, final error at most 8 cm, final
speed at most 8 cm/s, full-hover peak at most 8 cm, and no command limiting or
rotor saturation. Actual rotor speed may be zero only at the initial supported
sample; all later actual speeds and all commands lie strictly in (0,900) rad/s.
All saved audits must pass.

Selection uses a balanced RMSE ratio. Each profile's per-flight RMSE is divided
by the original cascade's paired value. Log ratios are averaged within hover,
nominal spline and wind spline; the three category averages receive equal
weight and are exponentiated. Thus a category with more cases does not dominate
the score merely by count.

The eligible cascade with the smallest balanced ratio is selected, with ties
broken by declaration order. The new geometric profile with the smallest ratio
is selected among eligible profiles whose per-flight integrated squared actual
moment is at most twice that selected cascade's. Complete release and landing
intervals stay in the scores. The historical geometric reference is an ablation
comparison, not a selectable candidate.

A balanced development improvement of at least 5% against the selected cascade
is required before the next
scientific stages. This is an opportunity screen on known cases, not a result
on fresh data. The six-profile search, seeds and gains are fixed without
adaptive extension or validation-driven retuning.

## True-state regression and independent cases

The selected profiles bind to the authenticated development report, executable
source and protocol. Seven existing true-state regression cases run both
selected controllers, for 14 flights: nominal, refined integration, positive
and negative offsets, mild and reversed wind, and wind plus offset. They retain
their freely flying initial conditions. Original absolute limits apply, with
geometric coarse/fine agreement at most 5 mm position and 0.05 degree attitude
at common epochs.

Passing development and true-state regression makes the independent ledger
eligible. It contains full hovers at 95000..95003, nominal splines at
96000/96002/96004/96006 and mild-wind splines at 96001/96003/96005/96007.
Each uses selected geometric, selected cascade and original cascade. The
original cascade is deduplicated if it was selected, yielding 24 or 36 clean
flights. Eight candidate supervisor off/on fault pairs at seed 97000 add
16 flights. The complete eligible ledger is retained even if performance fails.

The primary comparison is against the independently selected cascade. The
original cascade is a declared secondary comparison. The bounded primary
accuracy-advantage claim requires all of:

- Candidate and primary comparator clean flights meet every absolute condition,
  all saved audits pass, and all eight fault pairs satisfy their original
  causal, timing and recovery conditions.
- Balanced fresh RMSE ratio is at most 0.90: at least 10% lower RMSE overall.
  Each category's geometric-mean ratio is at most 1.05, and no individual ratio
  exceeds 1.10.
- Each flight's actual-moment effort ratio is at most 2, and the balanced
  effort ratio is at most 1.25.

The 5% category and 10% individual tolerances allow small stochastic tradeoffs
while demanding a larger overall gain. They are engineering preferences, not
statistical significance thresholds or revised interpretations of another
study. Accuracy advantage at a bounded effort cost is distinct from dominance
on every metric. Peaks, final error, speed, effort and individual regressions
remain reported regardless of the primary result.

A paired, category-stratified percentile bootstrap interval uses 20,000
resamples and analysis seed 424200. It describes sensitivity within these
twelve sampled cases; it is not an extra acceptance gate or evidence of broad
operating-envelope coverage.

## Evidence boundary

Complete histories retain configurations, conditional IMU memory, support,
fault/health decisions and source/protocol digests. Independent reconstruction
checks estimator, controls, plant, random draws and first-release quadrature;
saved arrays are rescored independently. Filter algebra, causality, reset,
invalid-domain behavior, isotropic compatibility and sampled maps are checked
separately.

A stage's evidence belongs to its actual executable source. The
[regression adapter compatibility rule](geometric-regression-adapter.md) explains
the narrowly permitted reuse of development evidence after correcting the
true-state interface. Default promotion and production integration remain
separate from measured comparison results. Neither local poles nor a small
simulated population establishes universal controller superiority.
