# Frame and State Contract

This contract defines the frames, orientation conventions, and initial simulator
truth state used by the mathematical core.

Equations use mathematical symbols; code names appear in backticks. Bold
lowercase letters denote three-dimensional vectors, except that force uses
uppercase $`\mathbf F`$ to distinguish it from specific force $`\mathbf f`$
(force per unit mass). Subscripts identify the quantity and its coordinate frame.

## Coordinate Frames

The world frame, `W`, uses North-East-Down (NED) axes:

- `+x`: north
- `+y`: east
- `+z`: down

The body frame, `B`, uses Forward-Right-Down (FRD) axes:

- `+x`: forward
- `+y`: right
- `+z`: down

All positive rotations follow the right-hand rule.

Increasing world `z` means descending. Increasing altitude means decreasing
world `z`.

## Orientation

`R_WB` is an active rotation that maps components expressed in the body frame
into components expressed in the world frame:

```math
\mathbf v_W = R_{WB}\mathbf v_B
```

Its transpose performs the inverse mapping:

```math
\mathbf v_B = R_{WB}^{T}\mathbf v_W
```

Rotation composition follows

```math
R_{WC} = R_{WB} R_{BC},
```

where the rightmost rotation acts first.

`q_WB` represents the same body-to-world orientation. It uses the Hamilton
quaternion convention, scalar-first ordering `[w, x, y, z]`, and unit norm.
The quaternions `q_WB` and `-q_WB` represent the same physical orientation.
Its action on a body vector is

```math
[0,\mathbf v_W] = q_{WB}\otimes[0,\mathbf v_B]\otimes q_{WB}^{*},
```

where $`\otimes`$ is the Hamilton product and $`*`$ is quaternion conjugation.

## Initial Simulator Truth State

| Symbol | Code name | Meaning | Shape | Units |
| --- | --- | --- | --- | --- |
| $`\mathbf p_W`$ | `p_W` | Body-origin position in the NED world frame | `(3,)` | m |
| $`\mathbf v_W`$ | `v_W` | Translational velocity in the world frame | `(3,)` | m/s |
| $`q_{WB}`$ | `q_WB` | Body-to-world unit quaternion | `(4,)` | dimensionless |
| $`\boldsymbol\omega_B`$ | `omega_B` | Angular velocity in the body frame | `(3,)` | rad/s |
| $`\boldsymbol\Omega`$ | `rotor_omega` | Four nonnegative rotor angular speeds | `(4,)` | rad/s |

Gravity expressed in world coordinates is

```math
\mathbf g_W = \begin{bmatrix}0\\0\\g\end{bmatrix}\ \mathrm{m/s^2}.
```

Collective thrust force expressed in body coordinates is

```math
\mathbf F_B = \begin{bmatrix}0\\0\\-T\end{bmatrix}\ \mathrm{N}.
```

Here $`g`$ is the positive gravity magnitude and $`T`$ is collective thrust
magnitude. The body moment $`\boldsymbol\tau_B`$ is `moment_B`, expressed in
`B`, with shape `(3,)` and units N·m.

## Level-Hover Sanity Check

For a level vehicle whose body axes are fully aligned with the world axes,
including zero yaw relative to north, $`R_{WB}=I`$. A vehicle can be level without
$`R_{WB}=I`$ if it has nonzero yaw. The gravity force is `[0, 0, mg]`, and the
collective thrust force is `[0, 0, -T]`. Force equilibrium therefore requires
$`T=mg`$.

## Environmental Wind and Quadratic Drag

The environmental model uses constant wind in the NED world frame and lumped anisotropic
quadratic coefficients along the FRD body axes.

| Symbol / code name | Meaning | Frame | Shape | Units |
| --- | --- | --- | --- | --- |
| $`\mathbf w_W`$ / `wind_velocity_W` | Velocity of the air relative to the world | `W` (NED) | `(3,)` | m/s |
| $`\mathbf v_{a,W}`$ / `velocity_air_W` | Vehicle velocity relative to the air | `W` (NED) | `(3,)` | m/s |
| $`\mathbf v_{a,B}`$ / `velocity_air_B` | Relative-air velocity on body axes | `B` (FRD) | `(3,)` | m/s |
| $`\mathbf c_B`$ / `quadratic_drag_coefficient_B` | Nonnegative quadratic drag coefficient per body axis | `B` (FRD) | `(3,)` | kg/m |
| $`\mathbf F_{d,B}`$ / `force_drag_B` | Drag force at the modelled centre of mass | `B` (FRD) | `(3,)` | N |

The exact equations are

```math
\mathbf v_{a,W} = \mathbf v_W-\mathbf w_W,
```

```math
\mathbf v_{a,B} = R_{WB}^{T}\mathbf v_{a,W},
```

and, element by element,

```math
\mathbf F_{d,B} = -\mathbf c_B\odot\lvert\mathbf v_{a,B}\rvert\odot\mathbf v_{a,B}.
```

Here $`\odot`$ and the absolute value both act elementwise. Because every coefficient is nonnegative,
the force cannot add power relative to the air:

```math
\mathbf F_{d,B}^{T}\mathbf v_{a,B}
= -\sum_{i=1}^{3}c_{B,i}\lvert v_{a,B,i}\rvert^3\le 0.
```

For the identity attitude, a vehicle moving north at `+2 m/s` in calm air has positive
forward relative-air velocity and therefore negative-forward drag. A stationary vehicle in
a northward wind of `+1 m/s` has negative-forward relative-air velocity and therefore
positive-forward drag: the wind pushes it north. These signs follow from defining relative
air velocity as vehicle velocity minus wind velocity.

The drag force acts at the modelled centre of mass and introduces no aerodynamic moment.
It is recomputed from the current velocity and attitude at every derivative evaluation;
projected RK4 therefore uses the separate velocity and projected attitude of each of its four
stages.

## ESKF Evaluation Coordinates

Estimator error order is `[delta_p_W, delta_v_W, delta_theta_B, delta_b_a_B,
delta_b_g_B]`, with three coordinates per block. Additive errors are reference
minus estimate. The attitude block is the principal rotation vector
$`\mathrm{Log}(\hat R_{WB}^TR_{WB}^{\mathrm{ref}})^\vee`$, expressed in the estimate's local body
coordinates. Thus $`R_{WB}^{\mathrm{ref}}=\hat R_{WB}\mathrm{Exp}([\delta\boldsymbol\theta_B]_\times)`$.
This is the same right-local convention used by injection and covariance reset.

The evaluation reference pairs artifact truth/bias row `j` with replay row `j-1`,
for completed rows $`j=1,\ldots,N`$. It compares the final post-update estimate and its
reset covariance with truth at the exactly equal epoch. Truth never supplies
the estimator prior through the measurement adapter. See
[ADR 0008](../decisions/0008-eskf-consistency-evaluation.md) for the finite NEES
domain, quaternion sign/pi convention and statistical assumptions.

## Boundaries and Naming

ROS ENU/FLU conversions will be isolated in adapter functions when integration
is introduced. ENU/FLU conventions must never leak into the mathematical core.

Public rotation and quaternion names must include their source and destination
frames. Vector names must include their expression frame. Ambiguous public names
such as `q`, `R`, `velocity`, `force`, or `omega` are prohibited.
