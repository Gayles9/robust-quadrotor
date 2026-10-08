# ADR 0019: Measurement-derived physical feedback derivatives

## Motivation

All five gain/rebasing profiles in [ADR 0018](0018-geometric-estimator-corrections.md)
missed the required hover gates. Rebasing suppresses correction impulses but
leaves a delayed estimate of physical force derivatives.

An IMU measures specific force, so the controller can estimate physical
acceleration without reading the simulated true state or inferring force from
a nominal rotor/drag model. This alternative uses that estimate to construct
the force derivatives at the original position and geometric gains. The
acceptance criteria and filter frequency are unchanged.

## Equations and measurement boundary

Let $`\mathbf f_m`$ [m/s²] be current measured body specific force, $`\mathbf b_a`$ its posterior bias,
$`\bar{\mathbf n}_a`$ the endpoint ESKF's conditional current accelerometer-noise mean, $`\hat R`$ the
posterior active $`R_{WB}`$ and $`g`$ nominal gravity. All are available causally:

```math
\hat{\mathbf a}_W=\hat R(\mathbf f_m-\mathbf b_a-\bar{\mathbf n}_a)+g\mathbf e_3.
```

Here $`\bar{\mathbf n}_a`$ is the posterior conditional mean of accelerometer sample noise.

At each outer tick feed $`\hat{\mathbf F}=m\hat{\mathbf a}_W`$ [N] into the unchanged 30 rad/s
three-section filter. Its first derivative estimates $`m\hat{\mathbf j}_W`$ [N/s]; the
second output is unused. For $`\mathbf e_v=\hat{\mathbf v}-\mathbf v_d`$, planned $`\mathbf a_d,\mathbf j_d,\mathbf s_d`$, and the
unchanged raw PD correction $`\mathbf c=m(K_p\mathbf e_p+K_v\mathbf e_v)`$:

```math
\begin{aligned}
\mathbf u&=m(g\mathbf e_3-\mathbf a_d)+\mathbf c,\\
\dot{\mathbf u}&=-m\mathbf j_d+mK_p\mathbf e_v+K_v(\hat{\mathbf F}-m\mathbf a_d),\\
\ddot{\mathbf u}&=-m\mathbf s_d+K_p(\hat{\mathbf F}-m\mathbf a_d)+K_v(D_1(\hat{\mathbf F})-m\mathbf j_d).
\end{aligned}
```

These are physical kinematics with a filtered measured jerk. They ignore
instantaneous posterior-state revisions as physical derivatives; raw feedback
still uses the posterior. Constant wind load/drag is present in measured
acceleration, so the rejected nominal-model force closure is not resurrected.
No actual rotor speeds, wind, true acceleration, true biases or future sample
are supplied to the controller. No ideal continuous-time proof is claimed.

## Interfaces and frozen acceptance

The explicit `use_measured_acceleration=False` geometric selection is mutually
exclusive with estimator-correction rebasing. It requires the estimated-state
adapter; true-state requests must fail clearly rather than receive a hidden
truth-acceleration oracle. The original true-state filtered-force design remains
available and is used for the true-state comparison matrix.

Before flight, test independent force-jet kinematics, constant biased-position
hover, NED hover acceleration sign, sensor-only command reconstruction, ESKF
replay, reset/ownership, invalid/missing mode inputs and default compatibility.
The development comparison uses the same observed spline pair and two full
hovers at original gains. Passing them is a prerequisite for the proposed
40-case characterization/qualification matrix in ADR 0018, with its original
tracking, effort, hover-window, refinement and repeat conditions. Each
independent validation seed is evaluated once, with all failures retained.

Software verification and saved-history checks establish implementation
correctness separately from measured flight performance. The experiment makes
no claim of a global performance optimum.

## Outcome and limits

The measured derivative design also fails development: nominal spline RMSE is
8.5360 cm versus 7.4059 cm for the cascade, although effort falls to 1.8242 times
cascade. Full-hover peaks are 11.9179 and 17.4527 cm. The unchanged 8 cm condition
is missed. Across all six declared profiles, no candidate passes the combined
tracking/effort/hover conditions. None is promoted; original defaults remain.

**The proposed 40-case validation did not run because the required development
cases failed.** New random cases cannot erase those failures. The proposed seed
identities 95000..95003 and 96000..96003 identify this unexecuted protocol;
the [final comparison](../results/final-geometric.md) documents their
use in a separate study. The implemented runner and failed evidence remain
available for reproduction.

The original true-state configuration has a separate 15-flight regression
(seven pairs and an exact repeat). Neither that regression nor these failed
profiles proves that further controller improvement is mathematically impossible.
