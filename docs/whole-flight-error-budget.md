# Whole-flight estimation/control error budget

The [frozen review](decisions/0039-whole-flight-error-budget.md) adds a quantitative
measurement requirement to the closed startup study. It preserves all 27 saved
clean flights and their failed comparisons. It does not run another flight or
change a production algorithm. The [dated record](progress/2026-09-29-whole-flight-error-budget.md)
contains numerical results and verification identities.

## Two different budgets

The whole-flight budget is exact accounting of a saved trajectory. Let `e_c[k]`
be each of the eleven existing physical response channels, with reference
position subtracted from the reference channel. Then

\[
e[k]=\sum_c e_c[k],\qquad
G_{cd}=\frac1N\sum_k e_c[k]^T e_d[k],\qquad
\mathrm{MSE}=\mathbf1^T G\mathbf1.
\]

`G` is an **uncentred** Gram matrix: it retains mean offsets as well as varying
errors. The diagonal sum is not generally the total MSE. All signed cross terms
are retained, and the earlier signed shares are the row sums of `G`. The triangle
bound `sum(sqrt(G_cc))` is a conservative RMS bound for these fixed channel
histories, not an achievable score after removing one source. Each flight keeps
its full duration. Separate initialize, takeoff, active and landing windows
partition it; the scored hover window also includes both 5 s and 11 s endpoints.

The prospective budget instead describes the existing **local horizontal hover
model**, with its exact sampled clocks. It does not assume that known saved
estimation errors predict future errors. The physical state is
`x=[p,v,u,w,alpha]`, where `u` is inclination-induced horizontal acceleration,
`w=du/dt`, and `alpha=dw/dt`. Between controller updates:

\[
\dot p=v,\quad\dot v=u,\quad\dot u=w,\quad\dot w=\alpha,
\quad\tau\dot\alpha=\alpha_h-\alpha.
\]

At each outer update `u_h=-Kp(p+e_p)-Kv(v+e_v)`. At each inner update
`alpha_h=Kr[Ka(u_h-u-g eta)-(w+g eta_dot)]`. Here `eta` is the difference in
the horizontal acceleration axes divided by gravity; near level it is a signed
rotation of roll/pitch error. It is not yaw. `eta_dot` is the corresponding
inclination-rate error, including gyro/attitude estimation effects.

The source fixes `h=2.5 ms`, outer period 20 ms, inner period 10 ms, motor
time constant 25 ms, `Kp=1`, `Kv=1.8`, `Ka=3`, `Kr=12`, `g=9.81` in SI units.
Every outer period has six input slots: one position sample, one velocity sample,
and two pairs of inclination/rate samples. Exact lifting gives

\[
x_{m+1}=Ax_m+Bu_m,\qquad p_{m,j}=C_jx_m+D_ju_m.
\]

The matrices retain the original update order and all eight intersample output
phases. For sinusoidal inputs, phase the two inner samples at their actual
acquisition times. With that input map `W(omega)`, the frequency response is

\[
H_j(\omega)=\left[C_j(e^{i\omega T}I-A)^{-1}B+D_j\right]W(\omega).
\]

The grid contains DC and 241 fixed frequencies from 0.01 to 25 Hz. It is an
analysis grid, not a gain search or a continuous-frequency maximum proof.
The DC sensitivities, also obtained directly from the equilibrium equations, are
`[-1, -Kv/Kp, -g/Kp, -g/(Ka Kp)] = [-1, -1.8, -9.81, -3.27]`.

## A usable prospective inequality

For zero initial state, past input coefficients are `C_j A^(lag-1) B` and the
current-period coefficient is `D_j`. Sum their absolute values by input family
to obtain `b_c(m,j)`. For a bound `epsilon_c` on the **horizontal vector norm**
of every sample of input family `c`, triangle inequality gives

\[
\|p_{m,j}\|_2\leq\sum_c b_c(m,j)\epsilon_c.
\]

No independence assumption or extra factor of square root of two is needed.
Adversarial signs aligned with one fixed horizontal direction attain each
single-input bound. Initial-state response is separately `C_j A^m x_0`.

Across the inclusive 5..11 s hover window, the maximum coefficients are:

| Error input | Finite-horizon coefficient | Single-input ceiling using the entire 8 cm |
| --- | ---: | ---: |
| Position | 0.999866 | 8.0011 cm |
| Velocity | 1.799760 s | 4.4450 cm/s |
| Inclination | 9.808690 m/rad | 0.4673 degrees |
| Inclination rate | 3.269563 m s/rad | 1.4019 degrees/s |

These ceilings are **not simultaneous allowances**. A prospective joint screen
must include all four terms, the initial response, and a justified allowance
`rho` for vertical error, physical disturbances and local-model discrepancy:

\[
0.999866\epsilon_p+1.799760\epsilon_v+9.808690\epsilon_\eta
+3.269563\epsilon_{\dot\eta}+b_{\rm initial}+\rho\leq0.08\;\mathrm m.
\]

Use unrounded saved coefficients for numerical evaluation. This is a sufficient
local-model screen only when its input bounds and residual allowance are actually
established. The previously observed sub-2-mm **RMS** model agreement supplies no
deterministic peak allowance for `rho`. Gaussian standard deviations are not
bounded errors. No numeric joint allocation is claimed from the available data.
The whole-flight 15 cm RMS/final-position, 15 cm/s final-speed, completion,
fault-response and strict no-regression conditions remain separate checks.

For stochastic analysis, the corresponding output spectrum uses the **full**
joint input spectral matrix: `S_y=H S_u H*`. Marginal noise levels alone cannot
discard cross spectra. Equivalently, a finite-horizon covariance calculation
must retain all cross-time and cross-channel covariances and nonzero means.
The present review does not estimate a stationary spectrum from nonstationary
takeoff/landing records or assign a probability of future flight success.

## Sensor and bias information

The existing model has 400 Hz IMU samples with per-axis standard deviations
0.04 m/s² and 0.002 rad/s; bias-walk densities are 0.0002 m/s²/sqrt(s) and
0.00002 rad/s/sqrt(s). Position arrives at 5 Hz with 2 cm per-axis standard
deviation; barometric altitude arrives at 25 Hz with 3 cm standard deviation.
There is no independent velocity or in-flight inclination observation.

Differencing adjacent position samples gives 14.142 cm/s per-axis noise, already
above the single-input deterministic velocity ceiling as a scale comparison,
not a probability claim. The difference shares a sample with current position:
`Cov(n_k,(n_k-n_(k-1))/dt)=sigma_p²/dt=0.002 m²/s`. Adjacent velocity differences
have covariance `-sigma_p²/dt²=-0.01 m²/s²`. Treating either difference as an
independent sensor double counts information. A three-point acceleration
difference has 1.224745 m/s² noise. No differentiated observation is proposed.

In the local constant-level-thrust error model, position/altitude observations
have rank 11 of 15. The four unobservable directions are two coupled tilt and
accelerometer-bias directions, yaw, and yaw gyro bias. An ideal direct velocity
observation leaves rank 11: it may improve finite-noise performance but does not
resolve those directions. Two independent inclination observations increase the
rank to 13 and remove the tilt/accelerometer-bias ambiguity, while yaw and yaw
gyro bias remain unobservable. This is a local rank result, not proof of global
observability or of improved flight tracking.

Accelerometer bias drives navigation error jointly with inclination. Gyro bias
drives inclination and rate error. These effects are already inside the four
controller inputs and must not be added again as independent acceleration
channels. Faster sensing cannot by itself create a missing measurement direction.

## Gate discontinuities and design decision

All 13,608 recorded observation decisions are retained, including 120 rejections.
Three seed47001 events at 1.6 s change branch in the combined experiment relative
to both controls, producing six paired differences. A response model conditional
on its inputs does not predict how a proposed estimator changes those decisions.
Any future design must preserve the original NIS thresholds and explicitly
evaluate changed gate sequences; no smooth transfer function bridges rejection.

**No-go for implementation with the present sensor assumptions. Go for a bounded
independent-inclination measurement contract and feasibility study.** The specific
hypothesis is that a genuinely independent observation of the body-down direction
in the world frame can constrain the two ambiguous directions throughout flight.
Its expected benefit is structural information and reduced low-frequency
inclination/navigation error, not a demonstrated numerical flight improvement.

The next contract must identify a physically justified source, such as an external
orientation measurement, and specify its world/body calibration, acquisition
time, latency, angular noise, bias, outages and correlations with existing
position measurements. A body-down observation lives on the unit sphere; its
two tangent coordinates leave a twist about the body-down axis unobserved.
That is not generally a world-vertical yaw rotation when tilted; the subsequent
[measurement contract](independent-inclination-feasibility.md) makes this
distinction explicit and completes the feasibility decision.
An orientation derived from the same accelerometer under arbitrary acceleration
does not satisfy the independent-source assumption. A co-produced external pose
must retain position/orientation correlation rather than claiming independence.

Before any component or flight proposal, that study must derive the tangent
measurement/Jacobian, account for joint noise and timing, establish a feasible
joint error allocation with residual allowance, and freeze rejection and
acceptance criteria. If no physical source or defensible noise/bias bounds can
be established, stop at that finding. No sensor hardware is asserted to exist.
Fresh complete nonlinear flight qualification would still be required after a
separately accepted implementation; startup tuning stays closed.

## Reproduction

Use the intact preceding `combined-prior-diagnosis-evidence-2026-09-29.zip`
contents, including its fixed manifest. From the repository root:

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.whole_flight_error_budget \
  --evidence /path/to/combined-prior-diagnosis-evidence \
  --output /path/to/new-whole-flight-error-budget
```

The runner authenticates every source payload, binds all preceding execution
files, and writes without clobbering existing evidence. It saves all Gram
matrices, phase-sensitive transfer functions, impulse coefficients/bounds,
sensor assumptions, the complete gate ledger and the decision. It uses no
new scientific flight or fitted coefficient. Reproducing the older full
ESKF/controller/plant replay remains the preceding diagnosis's separate command.
