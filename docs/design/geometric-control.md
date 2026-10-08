# Geometric control and causal reference derivatives

The geometric controller works directly with rotation error. It also uses the
desired angular velocity and acceleration to anticipate the turning motion
needed to follow a changing thrust direction. This feedforward helps in the
tested true-state flights, but requires reliable derivatives of the desired
force. A derivative describes how quickly a quantity changes. Computing it
from noisy estimates can magnify noise, so derivative filtering is a central
design concern.

I keep this controller opt-in because its measured tracking benefits come
with higher peak errors and moment effort in the final comparison. Read
[controller problems and tradeoffs](../results/controller-tradeoffs.md) and the
[final bounded comparison](../results/final-geometric.md) for the current evidence before
the equations below.
The [geometric protocol](../decisions/0017-geometric-reimplementation.md)
defines the baseline evaluation. The [supported-start comparison](../results/supported-geometric.md)
combines a filtered-force profile with supported alignment, exact initial
velocity and nonlinear release prediction. These are separate experimental
configurations with their own evaluation criteria.

The [final axis-dependent shaping study](../results/final-geometric.md) implements separate
horizontal and vertical force-filter poles. Its force value and derivatives
come from the same filtered signal, so they describe a consistent reference.
The study compares independently selected geometric and cascade gains, then locks both
profiles for regression and reserved validation. The experiment remains outside
the default mission path and preserves the original rotation-based moment law.

| Responsibility | Source |
| --- | --- |
| Rotation derivatives and moment law | [geometric_control.py](../../src/quadrotor_math/geometric_control.py) |
| Desired force and reference construction | [geometric_reference.py](../../src/quadrotor_math/geometric_reference.py) |
| Causal derivative-filter memory | [geometric_filter.py](../../src/quadrotor_math/geometric_filter.py) |
| Measurements and accepted-correction inputs | [estimated_mission.py](../../src/quadrotor_math/estimated_mission.py) |
| Declared comparison profiles and evidence gate | [geometric_correction_validation.py](../../experiments/geometric_correction_validation.py) |
| Experimental axis-dependent force shaping | [axis_shaped_geometric.py](../../experiments/axis_shaped_geometric.py) |
| Final selection and reserved validation | [final_geometric_comparison.py](../../experiments/final_geometric_comparison.py) |

## Frames and force reference

Use NED world and FRD body coordinates, Hamilton scalar-first $`q_{WB}`$, and active
$`R_{WB}`$ mapping body vectors to world vectors. Position and velocity errors are
$`\mathbf e_p=\mathbf p-\mathbf p_d`$, $`\mathbf e_v=\mathbf v-\mathbf v_d`$, in world coordinates. Nominal mass is $`m`$ kg,
position gains $`K_p`$ have units 1/s² and velocity gains $`K_v`$ have units 1/s.
For fixed-heading planned acceleration, jerk and snap $`\mathbf a_d`$, $`\mathbf j_d`$, $`\mathbf s_d`$:

```math
\begin{aligned}
\mathbf c&=m(K_p\mathbf e_p+K_v\mathbf e_v),\\
\mathbf u&=m(g\mathbf e_3-\mathbf a_d)+\mathbf c,\\
\dot{\mathbf u}&=-m\mathbf j_d+\widehat{\dot{\mathbf c}},\\
\ddot{\mathbf u}&=-m\mathbf s_d+\widehat{\ddot{\mathbf c}}.
\end{aligned}
```

The matrices $`K_p,K_v`$ contain the per-axis gains on their diagonals. The terms $`\widehat{\dot{\mathbf c}},\widehat{\ddot{\mathbf c}}`$ are `correction_rate` and `correction_acceleration`. Hats distinguish derivative estimates from exact derivatives of the raw feedback force.

$`\mathbf u`$ [N] points along the desired body-down axis. Applied thrust is opposite
body-down. Current/estimated attitude supplies the projected collective
$`f=\mathbf u^T R_{WB}\mathbf e_3`$ at each inner tick. This preserves the established thrust
sign; the controller receives no true acceleration, wind, drag, actual motors,
true mass or true biases. Nominal parameters remain explicit inputs.

The desired axes use the existing heading convention:

```math
\begin{aligned}
\mathbf b_{3d}&=\frac{\mathbf u}{\lVert\mathbf u\rVert},\\
\mathbf y_\psi&=[-\sin\psi,\cos\psi,0]^T,\\
\mathbf b_{1d}&=\frac{\mathbf y_\psi\times\mathbf b_{3d}}{\lVert\mathbf y_\psi\times\mathbf b_{3d}\rVert},\\
\mathbf b_{2d}&=\mathbf b_{3d}\times\mathbf b_{1d},\\
R_d&=[\mathbf b_{1d}\ \mathbf b_{2d}\ \mathbf b_{3d}].
\end{aligned}
```

The heading angle is $`\psi`$ (`yaw` in code).

Normalize and differentiate twice analytically. For any vector $`\mathbf x`$,
$`n=\lVert\mathbf x\rVert`$, $`\mathbf b=\mathbf x/n`$, the derivative identities are
$`\dot n=\mathbf b^T\dot{\mathbf x}`$, $`\dot{\mathbf b}=(\dot{\mathbf x}-\mathbf b\dot n)/n`$,
$`\ddot n=\mathbf b^T\ddot{\mathbf x}+\dot{\mathbf b}^T\dot{\mathbf x}`$, and
$`\ddot{\mathbf b}=(\ddot{\mathbf x}-\mathbf b\ddot n-2\dot{\mathbf b}\dot n)/n`$.
The cross-product product rule gives all three rotation-matrix derivatives.
Desired-frame rate and angular acceleration follow from $`R_d^T\dot R_d`$ and
$`R_d^T\ddot R_d-(R_d^T\dot R_d)^2`$, taking their skew-vector parts.

`RotationReference` owns its quaternion and rate/acceleration arrays. Quaternion
sign has no effect on the moment. Near-zero force or heading cross product is
rejected. In missions, outer acceleration/tilt/thrust limiting is an explicit
abort because clipped references require different derivative equations.

## Causal filter

A causal filter uses only measurements already available. It smooths the
feedback signal before estimating its derivatives, trading sensitivity to
noise against response delay.

By default, `FeedbackDerivativeFilter(period_s, pole_rad_s=30)` estimates the
derivatives of $`\mathbf c`$. The force value itself remains unfiltered; planned derivatives remain
analytic. Three cascaded sections implement the bilinear images of

```math
L(s)=\frac{\omega_f}{s+\omega_f},\qquad D_1(s)=sL(s)^3,\qquad D_2(s)=s^2L(s)^3.
```

The filter pole is $`\omega_f`$, distinct from vehicle angular velocity.

With $`a=\omega_f h/2`$, $`b=a/(1+a)`$, $`r=(1-a)/(1+a)`$, each section has
$`y[k]=r\,y[k-1]+b(x[k]+x[k-1])`$. Production evaluates the equivalent difference
form to preserve constant signals exactly. Outputs are
$`\omega_f(y_2-y_3)`$ and $`\omega_f^2(y_1-2y_2+y_3)`$.
For $`h=0.02\;\mathrm s`$, the three discrete poles equal `7/13` and low-frequency delay
is $`3/\omega_f=0.10\;\mathrm s`$. This is not the three-section -3 dB cutoff.

The first correction initializes all three sections, giving zero initial
correction derivatives. Every `step(c, index)` returns both derivative vectors
and a new immutable memory; invalid arithmetic cannot corrupt old state.
Only consecutive non-Boolean integer indices are accepted. Create a new instance
for a new run or period. Require resolved coefficients and $`0<\omega_f h\leq1`$.

This removes the artificial derivative feedforward at constant feedback error.
It does not remove steady wind position error or estimator bias. Since the force
is unfiltered while its derivatives are filtered, the derivative estimates
approximate the raw feedback reference; they are not its exact continuous derivatives.

### Optional estimator-correction rebasing

`GeometricControllerParameters(rebase_estimator_corrections=True)` enables
[estimator-correction rebasing](../decisions/0018-geometric-estimator-corrections.md).
It separates accepted ESKF position/velocity revisions from physical motion.
For an injected world-frame correction $`(\Delta\mathbf p,\Delta\mathbf v)`$, accumulate
$`\Delta\mathbf c=m(K_p\Delta\mathbf p+K_v\Delta\mathbf v)`$ between outer ticks. Translate the filter's
previous input and all three section states by that force revision before
stepping. Their differences, hence existing derivatives, remain unchanged.

The controller still uses the full posterior position and velocity for its raw
PD force. Only derivative feedforward avoids the instantaneous estimator jump.
No rejected event, true state, acceleration, rotor state or future observation
enters this calculation. Multiple corrections between outer ticks accumulate;
`rebase()` neither advances the sample index nor mutates existing memory.
The option defaults to `False` to preserve the original experiment. With true
state feedback it has no effect. This is an approximation for discontinuous
posterior estimates, not an exact derivative or a new stability proof.

## Moment and bounds

For desired-to-current transport $`A=R_{WB}^T R_d`$, define
$`\mathbf e_R=\tfrac12(R_d^T R_{WB}-R_{WB}^T R_d)^\vee`$ and $`\mathbf e_\Omega=\boldsymbol\Omega-A\boldsymbol\Omega_d`$. The body angular rate is $`\boldsymbol\Omega`$, the desired-frame rate is
$`\boldsymbol\Omega_d`$, and its derivative is $`\boldsymbol\alpha_d`$.
The vee map $`(\cdot)^\vee`$ converts a skew-symmetric matrix to a vector.
The moment is

```math
\begin{aligned}
\mathbf M={}&-k_R\mathbf e_R-k_\Omega\mathbf e_\Omega+\boldsymbol\Omega\times(J\boldsymbol\Omega)\\
 &-J\bigl(\boldsymbol\Omega\times(A\boldsymbol\Omega_d)-A\boldsymbol\alpha_d\bigr).
\end{aligned}
```

$`J`$ is full symmetric positive definite nominal inertia [kg m²]. Defaults are
$`k_R=0.64\;\mathrm{N\,m}`$, $`k_\Omega=0.32\;\mathrm{N\,m\,s}`$. This is the moment law of
[Lee, Leok and McClamroch, CDC 2010](https://doi.org/10.1109/CDC.2010.5717652).
Independent tests check the ideal identity
$`J\dot{\mathbf e}_\Omega=-k_R\mathbf e_R-k_\Omega\mathbf e_\Omega`$ and energy derivative
$`\dot V=-k_\Omega\lVert\mathbf e_\Omega\rVert^2`$, including moving references and off-diagonal inertia.
These ideal checks do not prove sampled, filtered, motor-lag flight stability.

Reference rate and local attitude violations abort. Requested moments are
component-limited using the existing bounds, then passed to the existing
collective-preserving allocator. Both actions remain visible in the histories.
The geometric `attitude_error_B` diagnostic is the sine-axis error $`\mathbf e_R`$;
`desired_omega_B` is $`A\boldsymbol\Omega_d`$, not the cascade's P-generated rate target.

## Interfaces and execution

```python
from quadrotor_math.geometric_control import GeometricControllerParameters

geometric = GeometricControllerParameters()
```

Pass this object as `geometric_controller=geometric` to `simulate_mission` or
`simulate_estimated_mission`, alongside that runner's explicit plant, mission,
sensor and estimator inputs. This snippet constructs the controller selection;
the complete experiment commands below construct and execute a flight.

Use the existing `AttitudeControllerParameters` for nominal inertia, rotors and
limits, and `PositionControllerParameters` for translational gains. The geometric
parameters specify scalar gains and the derivative pole. Defaults
continue to select the cascade. For an explicit estimated cascade minimum-snap
comparison, use `allow_minimum_snap=True`.

Fixed-yaw HOLD, SMOOTH quintic and minimum-snap missions are supported. STEP
references are rejected. Derivatives are right-sided at knots; quintic motion
can have jerk/snap jumps at joins. The geometric adapter recomputes the reference
at outer ticks and holds its rotation/rates until the next outer tick.

The shared execution order stays sensor/ESKF, guards, supervisor, control, plant.
ESKF control uses posterior nominal state and posterior-corrected gyro, with
separate labelled truth safety guards. No terminal command is recorded on abort.
Mission startup creates fresh filter memory. Legacy diagnostic calls retain
exactly their original positional arguments when the option is absent.

Run the fixed baseline campaign with:

```bash
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.geometric_reimplementation_validation \
  --output /tmp/new-geometric-evidence --workers 3
```

The command records every configuration, source fingerprint, flight, failure,
score and file hash. Estimated cases retain full covariance, sensor and event
histories. Nonzero exit on failed performance is an expected, preserved outcome;
passing software tests must not relabel a failed campaign as qualified.

The bounded correction study uses the same physical and effort gates:

```bash
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.geometric_correction_validation \
  --stage development --frequency 1.5 --output /tmp/new-correction-evidence --workers 3
```

Declared horizontal natural frequencies $`f`$ are 1, 1.5 and 2 rad/s, with damping
ratio 0.9 ($`K_p=f^2`$, $`K_v=1.8f`$). Vertical/attitude gains,
estimator prior, sensor distributions and the filter pole stay fixed.
The implemented `--stage qualification` would run the original 28-case matrix
plus twelve reserved cases after a candidate clears development. That stage
was not run for these six rejected profiles, so it has no reported results.
The [final comparison](../results/final-geometric.md) uses a separate protocol.
The runner reports candidate qualification separately from known comparator
hover failures.

### Measured physical derivatives

The [measured-derivative experiment](../decisions/0019-measured-geometric-derivatives.md)
selects `use_measured_acceleration=True`, mutually exclusive with
rebasing. The estimated adapter computes
$`\hat{\mathbf a}=\hat R(\mathbf f_m-\mathbf b_a-\bar{\mathbf n}_a)+g\mathbf e_3`$ from available IMU
and posterior ESKF quantities. At outer ticks, the filter takes $`\mathbf F=m\hat{\mathbf a}`$
and estimates $`\dot{\mathbf F}`$. It then constructs

```math
\begin{aligned}
\widehat{\dot{\mathbf c}}&=mK_p(\hat{\mathbf v}-\mathbf v_d)+K_v(\mathbf F-m\mathbf a_d),\\
\widehat{\ddot{\mathbf c}}&=K_p(\mathbf F-m\mathbf a_d)+K_v(\widehat{\dot{\mathbf F}}-m\mathbf j_d).
\end{aligned}
```

Here $`\widehat{\dot{\mathbf F}}`$ is the filtered derivative of $`\mathbf F=m\hat{\mathbf a}`$.

This avoids a second differentiation of corrected position/velocity. It uses
the original gains and preserves raw PD force. True-state-only use is rejected
because it has no measurement-derived acceleration input. Changing derivative
channels requires fresh filter memory. This remains approximate feedforward.

Use `--strategy measured --frequency 1` for its development reproduction.
For the two additional rebasing profiles, `--stiffness 1.28` selects the gain
derived from critical roll damping at frequencies 1 or 1.5. The original
three profiles keep stiffness 0.64.

**All six correction-study profiles failed their development performance
criteria.** Their conditional qualification stage was not run. The
[controller tradeoffs](../results/controller-tradeoffs.md) explain these limits;
the [final comparison](../results/final-geometric.md) reports a separate
axis-dependent force-shaping study. The cascade remains the default controller.
