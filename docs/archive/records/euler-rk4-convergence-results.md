# 2026-08-23: Euler-versus-RK4 Convergence Results

## Milestone summary

The project now has a reproducible numerical experiment that compares the constant-input
explicit-Euler and projected-RK4 simulators across four nested fixed-step grids. Reusable
metrics measure position, velocity, attitude, and angular-velocity errors independently, and
the experiment reports both final-time and maximum-trajectory errors with observed orders.

The study measured approximately fourth-order convergence for projected RK4 and orders
approaching one for explicit Euler over the tested range. These results are measured relative
to a fine numerical reference. They are empirical evidence for this scenario, not a formal
proof or a comparison with an exact solution.

## Engineering question

The simulation-parity milestone made the numerical method the controlled difference between
two otherwise matching propagation interfaces. The next question was quantitative: as the
fixed time step is halved, how quickly do the complete-state errors decrease for explicit
Euler and for the quaternion-projected RK4 implementation?

Answering that question required physical error definitions, exact alignment between coarse
and reference samples, numerical safeguards for observed-order calculations, and a
deterministic experiment that could be rerun without generated artifacts.

**Think about it this way.** The experiment keeps the vehicle, input, duration, and comparison
times fixed, then changes only the integrator and its time-step resolution.

## Architecture

The convergence capability is split by responsibility:

- `src/quadrotor_math/metrics.py` contains reusable, validated physical trajectory-error and
  observed-order metrics.
- `src/quadrotor_math/simulation.py` remains responsible for deterministic sequential Euler
  and projected-RK4 histories.
- `experiments/euler_rk4_convergence.py` defines the scenario, runs the grids, samples the
  numerical reference, calculates errors and orders, and prints Markdown tables.

The experiment is not part of the production mathematical API. It composes public simulation
and metric functions and keeps scenario-specific tabulation local. NumPy remains sufficient;
no plotting, tabular-data, or scientific-computing dependency was added.

```text
constant asymmetric rotor input
    -> Euler and projected-RK4 state histories
    -> integer-stride samples from a fine projected-RK4 reference
    -> independent physical trajectory errors
    -> final and maximum errors
    -> adjacent-grid observed orders
    -> reproducible Markdown tables
```

## Deterministic scenario and nested grids

The established asymmetric scenario starts from:

- NED position `[10, 20, 30]` m;
- NED velocity `[1, -2, 3]` m/s;
- identity Hamilton scalar-first body-to-world quaternion;
- FRD angular velocity `[0, 0, 2]` rad/s; and
- one active rotor at FRD position `[2, 0, 0]` m with angular speed `2` rad/s.

The mass is `1 kg`, body inertia is `diag(2, 3, 4) kg·m²`, gravity magnitude is
`9.81 m/s²`, thrust coefficient is `0.75`, and moment coefficient is `0.5`. Rotor speeds,
geometry, spin directions, and physical parameters remain constant for the complete 1-second
simulation.

The comparison grids are:

| Time step (s) | Transitions |
| ---: | ---: |
| 0.1 | 10 |
| 0.05 | 20 |
| 0.025 | 40 |
| 0.0125 | 80 |

The numerical reference uses projected RK4 with `1 / 2560 s` steps and 2560 transitions.
Every comparison grid divides 2560 exactly. The reference is therefore sampled using an
integer stride, including the shared initial and final times. No floating-point time equality,
nearest-neighbour match, rounding, or interpolation is used.

**Think about it this way.** Each coarse timestamp already exists as an exact row index in the
fine history, so the study compares aligned rows rather than searching approximately equal
floating-point times.

## Physical trajectory errors

Four quantities remain separate because their dimensions and frames differ:

1. Euclidean NED position error in metres;
2. Euclidean NED velocity error in m/s;
3. sign-invariant geodesic body-to-world attitude error in radians; and
4. Euclidean FRD angular-velocity error in rad/s.

The generic Euclidean metric calculates one row-wise norm for position, velocity, or angular
velocity. It is called independently for each quantity; no mixed-unit state norm is formed.
For attitude, corresponding unit quaternions are sign-aligned before the chord distance is
converted to the physical rotation angle. Consequently, `q_WB` and `-q_WB` produce zero error
because they represent the same orientation.

For each method, resolution, and physical quantity, the experiment records the error at the
final sample and the maximum error over all matching trajectory samples. The maximum is taken
within one physical quantity only.

**Think about it this way.** Metres, metres per second, radians, and radians per second answer
different questions. Keeping four error columns makes every number interpretable.

## Observed orders and numerical safeguards

For adjacent time steps `h_i > h_(i+1)` and errors above a scale-aware numerical floor, the
observed order is the logarithmic change in error divided by the logarithmic change in time
step. A pair touching or falling below its error floor produces `NaN` instead of an unstable
or meaningless order. Negative orders remain valid measurements when error grows under
refinement.

Two floating-point defects were found through boundary-focused unit tests. The initial direct
ratio formula could overflow or underflow for finite positive values even though the final
mathematical order was representable. It was replaced with differences of logarithms. A
subsequent test showed that separately evaluated logarithms of distinct adjacent large floats
can round to the same value, causing cancellation and a zero denominator. Targeted `log1p`
fallbacks now recover the relative change only when the log difference is zero but the
original positive values differ. Equal errors retain an exact zero numerator.

These were real numerical defects because valid finite inputs could raise floating-point
exceptions or lose a distinguishable refinement interval. Ordinary convergence data did not
expose them because its magnitudes and refinement ratios were far from the floating-point
extremes.

**Think about it this way.** The usual formula worked for ordinary numbers, but the boundary
tests checked whether the same mathematics survived the full range of valid float64 inputs.

## Measured results

The experiment printed the following values. The 2560-step projected-RK4 trajectory is a
numerical reference, not an exact solution.

### Final-time errors

| Method | Time step (s) | Steps | Position (m) | Velocity (m/s) | Attitude (rad) | Angular velocity (rad/s) |
|---|---:|---:|---:|---:|---:|---:|
| Euler | 1.0000000000e-01 | 10 | 3.7898843195e-01 | 2.1333333720e-01 | 6.1419959406e-02 | 9.6625488934e-02 |
| Euler | 5.0000000000e-02 | 20 | 1.9240040225e-01 | 1.0781157542e-01 | 2.8827045783e-02 | 4.8163034469e-02 |
| Euler | 2.5000000000e-02 | 40 | 9.6960580961e-02 | 5.4123698463e-02 | 1.3938647805e-02 | 2.4030730648e-02 |
| Euler | 1.2500000000e-02 | 80 | 4.8673997501e-02 | 2.7108099933e-02 | 6.8506604118e-03 | 1.2000996758e-02 |
| RK4 | 1.0000000000e-01 | 10 | 8.0402004144e-06 | 1.3739558037e-05 | 5.3505724132e-06 | 1.8640418690e-06 |
| RK4 | 5.0000000000e-02 | 20 | 5.1167307180e-07 | 8.5522467574e-07 | 3.3114899315e-07 | 1.1736332260e-07 |
| RK4 | 2.5000000000e-02 | 40 | 3.2276438706e-08 | 5.3372642778e-08 | 2.0602700684e-08 | 7.3637772587e-09 |
| RK4 | 1.2500000000e-02 | 80 | 2.0267431351e-09 | 3.3337959962e-09 | 1.2848461243e-09 | 4.6115746682e-10 |

### Maximum-trajectory errors

| Method | Time step (s) | Steps | Position (m) | Velocity (m/s) | Attitude (rad) | Angular velocity (rad/s) |
|---|---:|---:|---:|---:|---:|---:|
| Euler | 1.0000000000e-01 | 10 | 3.7898843195e-01 | 2.1333333720e-01 | 6.1419959406e-02 | 9.6625488934e-02 |
| Euler | 5.0000000000e-02 | 20 | 1.9240040225e-01 | 1.0781157542e-01 | 2.9047281644e-02 | 4.8163034469e-02 |
| Euler | 2.5000000000e-02 | 40 | 9.6960580961e-02 | 5.4123698463e-02 | 1.4163031839e-02 | 2.4030730648e-02 |
| Euler | 1.2500000000e-02 | 80 | 4.8673997501e-02 | 2.7108099933e-02 | 6.9969418401e-03 | 1.2000996758e-02 |
| RK4 | 1.0000000000e-01 | 10 | 8.0402004144e-06 | 1.3739558037e-05 | 5.3505724132e-06 | 1.8640418690e-06 |
| RK4 | 5.0000000000e-02 | 20 | 5.1167307180e-07 | 8.5552244654e-07 | 3.3114899315e-07 | 1.1736332260e-07 |
| RK4 | 2.5000000000e-02 | 40 | 3.2276438706e-08 | 5.3430727380e-08 | 2.0602700684e-08 | 7.3637772587e-09 |
| RK4 | 1.2500000000e-02 | 80 | 2.0267431351e-09 | 3.3377349208e-09 | 1.2848461243e-09 | 4.6115746682e-10 |

### Final-time observed orders

| Method | Time-step pair (s) | Position (m) order | Velocity (m/s) order | Attitude (rad) order | Angular velocity (rad/s) order |
|---|---|---:|---:|---:|---:|
| Euler | 1.0000000000e-01 → 5.0000000000e-02 | 9.7804199763e-01 | 9.8459734607e-01 | 1.0912845628e+00 | 1.0044775186e+00 |
| Euler | 5.0000000000e-02 → 2.5000000000e-02 | 9.8864156715e-01 | 9.9417975223e-01 | 1.0483323845e+00 | 1.0030457768e+00 |
| Euler | 2.5000000000e-02 → 1.2500000000e-02 | 9.9424707786e-01 | 9.9753643209e-01 | 1.0247756341e+00 | 1.0017262783e+00 |
| RK4 | 1.0000000000e-01 → 5.0000000000e-02 | 3.9739372486e+00 | 4.0058883076e+00 | 4.0141408648e+00 | 3.9893807402e+00 |
| RK4 | 5.0000000000e-02 → 2.5000000000e-02 | 3.9866690001e+00 | 4.0021311255e+00 | 4.0065751026e+00 | 3.9943918214e+00 |
| RK4 | 2.5000000000e-02 → 1.2500000000e-02 | 3.9932462428e+00 | 4.0008627192e+00 | 4.0031659692e+00 | 3.9971146258e+00 |

### Maximum-error observed orders

| Method | Time-step pair (s) | Position (m) order | Velocity (m/s) order | Attitude (rad) order | Angular velocity (rad/s) order |
|---|---|---:|---:|---:|---:|
| Euler | 1.0000000000e-01 → 5.0000000000e-02 | 9.7804199763e-01 | 9.8459734607e-01 | 1.0803044018e+00 | 1.0044775186e+00 |
| Euler | 5.0000000000e-02 → 2.5000000000e-02 | 9.8864156715e-01 | 9.9417975223e-01 | 1.0362730251e+00 | 1.0030457768e+00 |
| Euler | 2.5000000000e-02 → 1.2500000000e-02 | 9.9424707786e-01 | 9.9753643209e-01 | 1.0173337271e+00 | 1.0017262783e+00 |
| RK4 | 1.0000000000e-01 → 5.0000000000e-02 | 3.9739372486e+00 | 4.0053860797e+00 | 4.0141408648e+00 | 3.9893807402e+00 |
| RK4 | 5.0000000000e-02 → 2.5000000000e-02 | 3.9866690001e+00 | 4.0010641450e+00 | 4.0065751026e+00 | 3.9943918214e+00 |
| RK4 | 2.5000000000e-02 → 1.2500000000e-02 | 3.9932462428e+00 | 4.0007283703e+00 | 4.0031659692e+00 | 3.9971146258e+00 |

## Interpretation

Euler's final-time and maximum-trajectory orders move toward one for all four physical
quantities. Projected RK4's corresponding orders remain between approximately `3.97` and
`4.01`. The finest-grid final position errors are `4.8673997501e-02 m` for Euler and
`2.0267431351e-09 m` for RK4, compared with `3.7898843195e-01 m` and `8.0402004144e-06 m`
on the coarsest grid. The final and maximum tables show consistent refinement behavior, and
no reported order is `NaN`, negative, or nonmonotonic in this experiment.

The measured approximately fourth-order convergence supports the expected RK4 behavior over
the tested range, including the projected intermediate and final quaternion handling. It does
not prove fourth-order convergence for every state, scenario, duration, parameter set, or
step-size regime. The fine projected-RK4 reference also carries numerical error of its own, so
the reported values do not establish absolute error or long-horizon accuracy.

**Think about it this way.** Halving the tested time step reduces Euler error by about two and
RK4 error by about sixteen, but the comparison ruler is itself a very fine numerical solution.

## What this milestone demonstrates

- Both simulators can be compared on identical deterministic NED/FRD histories.
- The reusable metrics preserve physical units and quaternion sign equivalence.
- Errors decrease consistently on all tested grids for all four state quantities.
- Euler exhibits measured behavior approaching first order.
- Projected RK4 exhibits measured approximately fourth-order convergence and supports the
  expected RK4 behavior over the tested range.
- The experiment is reproducible with
  `uv run python experiments/euler_rk4_convergence.py`.

## What this milestone does not demonstrate

- It is not a formal convergence proof.
- The numerical reference is not an exact trajectory.
- It does not establish behavior beyond the one active-rotor scenario, 1-second duration, or
  tested fixed-step range.
- It does not establish long-horizon stability or absolute physical-model accuracy.
- It does not verify physical invariants, closed-loop behavior, scheduled inputs, adaptive
  integration, estimation, or robustness under uncertainty.

## Verification

At commit `8191c55`:

- the convergence experiment ran successfully and reproduced all four tables above;
- Ruff lint passed;
- Ruff formatting verification passed;
- strict mypy passed over `src` and `experiments`;
- 199 pytest tests passed; and
- `git diff --check` passed.

## Current limitations

- The scenario uses constant rotor speeds and physical parameters with fixed time steps.
- The fine reference is projected RK4 rather than an exact or independently derived solution.
- The study covers one asymmetric initial condition and input over 1 second.
- No state-validity or physically conditional invariant-monitoring framework exists.
- No controller, scheduled input, callback, event handling, or adaptive step size exists.
- No long-horizon, uncertainty, Monte Carlo, estimator, or flight-stack study has been run.

## Next exact milestone

Add state-validity and physically conditional invariant monitoring. The next work should
separate unconditional state checks, such as quaternion norm and finite histories, from
quantities that are conserved only under specific force-and-moment conditions. It must not
label energy, momentum, position, velocity, or angular velocity as invariants while gravity,
thrust, or applied moments are changing them.
