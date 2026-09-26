# Geometric control and causal reference derivatives

The geometric controller works directly with rotation error. It also uses the
desired angular velocity and acceleration to anticipate the turning motion
needed to follow a changing thrust direction. This feedforward helps in the
tested true-state flights, but requires reliable derivatives of the desired
force. With noisy estimated feedback, that derivative calculation is the main
design difficulty.

I keep this controller opt-in because it still fails the combined noisy hover,
tracking and effort requirements. Read [controller problems and tradeoffs](controller-tradeoffs.md)
for the explanation and measured comparison before the equations below.
The [current implementation record](progress/2026-09-26-geometric-reimplementation.md)
and [tuning audit](progress/2026-09-26-geometric-project-audit.md) document its
fresh verification under [ADR 0017](decisions/0017-geometric-reimplementation.md).

| Responsibility | Source |
| --- | --- |
| Rotation derivatives and moment law | [geometric_control.py](../src/quadrotor_math/geometric_control.py) |
| Desired force and reference construction | [geometric_reference.py](../src/quadrotor_math/geometric_reference.py) |
| Causal derivative-filter memory | [geometric_filter.py](../src/quadrotor_math/geometric_filter.py) |
| Measurements and accepted-correction inputs | [estimated_mission.py](../src/quadrotor_math/estimated_mission.py) |
| Declared comparison profiles and evidence gate | [geometric_correction_validation.py](../experiments/geometric_correction_validation.py) |

## Frames and force reference

Use NED world and FRD body coordinates, Hamilton scalar-first `q_WB`, and active
`R_WB` mapping body vectors to world vectors. Position and velocity errors are
`ep=p-pd`, `ev=v-vd`, in world coordinates. Nominal mass is `m` kg,
position gains `Kp` have units 1/s² and velocity gains `Kv` have units 1/s.
For fixed-heading planned acceleration, jerk and snap `ad`, `jd`, `sd`:

```text
c      = m (Kp ep + Kv ev)
u      = m (g e3 - ad) + c
u_dot  = -m jd + correction_rate
u_ddot = -m sd + correction_acceleration
```

`u` [N] points along the desired body-down axis. Applied thrust is opposite
body-down. Current/estimated attitude supplies the projected collective
`f = u · (R_WB e3)` at each inner tick. This preserves the established thrust
sign; the controller receives no true acceleration, wind, drag, actual motors,
true mass or true biases. Nominal parameters remain explicit inputs.

The desired axes use the existing heading convention:

```text
b3d = u / ||u||
b1d = normalize([-sin(yaw), cos(yaw), 0] cross b3d)
b2d = b3d cross b1d
Rd = [b1d b2d b3d]
```

Normalize and differentiate twice analytically. For any vector `x`,
`n=||x||`, `b=x/n`, the derivative identities are
`n_dot=b·x_dot`, `b_dot=(x_dot-b*n_dot)/n`,
`n_ddot=b·x_ddot+b_dot·x_dot`, and
`b_ddot=(x_ddot-b*n_ddot-2*b_dot*n_dot)/n`.
The cross-product product rule gives all three rotation-matrix derivatives.
Desired-frame rate and angular acceleration follow from `Rd.T Rd_dot` and
`Rd.T Rd_ddot - (Rd.T Rd_dot)^2`, taking their skew-vector parts.

`RotationReference` owns its quaternion and rate/acceleration arrays. Quaternion
sign has no effect on the moment. Near-zero force or heading cross product is
rejected. In missions, outer acceleration/tilt/thrust limiting is an explicit
abort because clipped references require different derivative equations.

## Causal filter

By default, `FeedbackDerivativeFilter(period_s, pole_rad_s=30)` estimates the
derivatives of `c`. The force value itself remains unfiltered; planned derivatives remain
analytic. Three cascaded sections implement the bilinear images of

```text
L(s) = w/(s+w), D1(s)=s L(s)^3, D2(s)=s² L(s)^3
```

With `a=w*h/2`, `b=a/(1+a)`, `r=(1-a)/(1+a)`, each section has
`y[k]=r*y[k-1]+b*(x[k]+x[k-1])`. Production evaluates the equivalent difference
form to preserve constant signals exactly. Outputs are
`w*(y2-y3)` and `w²*((y1-y2)-(y2-y3))`.
For `h=0.02 s`, the three discrete poles equal `7/13` and low-frequency delay
is `3/w=0.10 s`. This is not the three-section -3 dB cutoff.

The first correction initializes all three sections, giving zero initial
correction derivatives. Every `step(c, index)` returns both derivative vectors
and a new immutable memory; invalid arithmetic cannot corrupt old state.
Only consecutive non-Boolean integer indices are accepted. Create a new instance
for a new run or period. Require resolved coefficients and `0<w*h<=1`.

This removes the artificial derivative feedforward at constant feedback error.
It does not remove steady wind position error or estimator bias. Since the force
is unfiltered while its derivatives are filtered, the jets approximate the raw
feedback reference; they are not its exact continuous derivatives.

### Optional estimator-correction rebasing

[ADR 0018](decisions/0018-geometric-estimator-corrections.md) adds
`GeometricControllerParameters(rebase_estimator_corrections=True)`.
It separates accepted ESKF position/velocity revisions from physical motion.
For an injected world-frame correction `(delta_p, delta_v)`, accumulate
`delta_c=m*(Kp*delta_p+Kv*delta_v)` between outer ticks. Translate the filter's
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

For desired-to-current transport `A=R_WB.T Rd`, define
`eR=vee(Rd.T R_WB-R_WB.T Rd)/2` and `eOmega=Omega-A Omega_d`. The moment is

```text
M = -kR eR - kOmega eOmega + Omega cross (J Omega)
    - J (Omega cross (A Omega_d) - A alpha_d)
```

`J` is full symmetric positive definite nominal inertia [kg m²]. Defaults are
`kR=0.64 N m`, `kOmega=0.32 N m s`. This is the moment law of
[Lee, Leok and McClamroch, CDC 2010](https://doi.org/10.1109/CDC.2010.5717652).
Independent tests check the ideal identity
`J eOmega_dot = -kR eR-kOmega eOmega` and energy derivative
`V_dot=-kOmega ||eOmega||²`, including moving references and off-diagonal inertia.
These ideal checks do not prove sampled, filtered, motor-lag flight stability.

Reference rate and local attitude violations abort. Requested moments are
component-limited using the existing bounds, then passed to the existing
collective-preserving allocator. Both actions remain visible in the histories.
The geometric `attitude_error_B` diagnostic is the sine-axis error `eR`;
`desired_omega_B` is `A Omega_d`, not the cascade's P-generated rate target.

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
limits, and `PositionControllerParameters` for translational gains. The new
parameters specify scalar geometric gains and the derivative pole. Defaults
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

Run the fresh, fixed campaign with:

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

Declared horizontal natural frequencies are 1, 1.5 and 2 rad/s, with damping
ratio 0.9 (`Kp=frequency**2`, `Kv=1.8*frequency`). Vertical/attitude gains,
estimator prior, sensor distributions and the filter pole stay fixed.
The implemented `--stage qualification` would run the original 28-case matrix
plus twelve reserved cases after a candidate clears development. That stage
was not run for these six rejected profiles. Keep the reserved cases unopened
until the conditions in the [next-step plan](next-steps.md) are met. The runner
reports candidate qualification separately from known comparator hover failures.

### Measured physical derivatives

[ADR 0019](decisions/0019-measured-geometric-derivatives.md) provides another
explicit experiment: `use_measured_acceleration=True`, mutually exclusive with
rebasing. The estimated adapter computes
`a_hat=R_hat*(f_measured-bias_a-conditional_noise_a)+g*e3` from available IMU
and posterior ESKF quantities. At outer ticks, the filter takes `F=m*a_hat`
and estimates `F_dot`. It then constructs

```text
correction_rate         = m*Kp*(v_hat-vd) + Kv*(F-m*ad)
correction_acceleration = Kp*(F-m*ad) + Kv*(F_dot-m*jd)
```

This avoids a second differentiation of corrected position/velocity. It uses
the original gains and preserves raw PD force. True-state-only use is rejected
because it has no measurement-derived acceleration input. Changing derivative
channels requires fresh filter memory. This remains approximate feedforward.

Use `--strategy measured --frequency 1` for its development reproduction.
For the two additional rebasing profiles, `--stiffness 1.28` selects the gain
derived from critical roll damping at frequencies 1 or 1.5. The original
three profiles keep stiffness 0.64.

**All six profiles were rejected for performance promotion.** The
[project audit](progress/2026-09-26-geometric-project-audit.md) retains the
complete results and explains the tradeoffs. Fresh validation cases remain
unopened because no candidate clears the known development conditions.
The cascade default and the original geometric defaults are unchanged.
