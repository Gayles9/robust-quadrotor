# Frame and State Contract

This contract defines the frames, orientation conventions, and initial simulator
truth state used by the mathematical core.

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

$$
v_W = R_{WB} v_B
$$

Its transpose performs the inverse mapping:

$$
v_B = R_{WB}^{T} v_W
$$

Rotation composition follows

$$
R_{WC} = R_{WB} R_{BC},
$$

where the rightmost rotation acts first.

`q_WB` represents the same body-to-world orientation. It uses the Hamilton
quaternion convention, scalar-first ordering `[w, x, y, z]`, and unit norm.
The quaternions `q_WB` and `-q_WB` represent the same physical orientation.
Its action on a body vector is

$$
[0, v_W] = q_{WB} \otimes [0, v_B] \otimes q_{WB}^{*},
$$

where $\otimes$ is the Hamilton product and $*$ is quaternion conjugation.

## Initial Simulator Truth State

| Symbol | Meaning | Frame | Shape | Units |
| --- | --- | --- | --- | --- |
| `p_W` | Position of the body-frame origin relative to the world-frame origin, expressed in `W` | `W` | `(3,)` | metres |
| `v_W` | Translational velocity expressed in `W` | `W` | `(3,)` | m/s |
| `q_WB` | Body-to-world unit quaternion | `B` to `W` | `(4,)` | dimensionless |
| `omega_B` | Angular velocity expressed in `B` | `B` | `(3,)` | rad/s |
| `rotor_omega` | Four nonnegative rotor angular speeds | Rotor axes | `(4,)` | rad/s |

Gravity expressed in world coordinates is

$$
gravity_W = [0, 0, g]\ \mathrm{m/s^2}.
$$

Collective thrust force expressed in body coordinates is

$$
force_B = [0, 0, -T]\ \mathrm{N}.
$$

The body moment is `moment_B`, expressed in `B`, with shape `(3,)` and units
N·m.

## Level-Hover Sanity Check

For a level vehicle whose body axes are fully aligned with the world axes,
including zero yaw relative to north, `R_WB = I`. A vehicle can be level without
`R_WB = I` if it has nonzero yaw. The gravity force is `[0, 0, mg]`, and the
collective thrust force is `[0, 0, -T]`. Force equilibrium therefore requires
`T = mg`.

## Environmental Wind and Quadratic Drag

The environmental model uses constant wind in the NED world frame and lumped anisotropic
quadratic coefficients along the FRD body axes.

| Name | Meaning | Frame | Shape | Units |
| --- | --- | --- | --- | --- |
| `wind_velocity_W` | Velocity of the air relative to the world | `W` (NED) | `(3,)` | m/s |
| `velocity_air_W` | Vehicle velocity relative to the air | `W` (NED) | `(3,)` | m/s |
| `velocity_air_B` | Vehicle velocity relative to the air, resolved on body axes | `B` (FRD) | `(3,)` | m/s |
| `quadratic_drag_coefficient_B` | Nonnegative lumped quadratic coefficient per body axis | `B` (FRD axes) | `(3,)` | kg/m |
| `force_drag_B` | Aerodynamic drag force applied at the modelled centre of mass | `B` (FRD) | `(3,)` | N |

The exact equations are

$$
velocity\_air_W = velocity_W - wind\_velocity_W,
$$

$$
velocity\_air_B = R_{WB}^{T} velocity\_air_W,
$$

and, element by element,

$$
force\_drag_B = -quadratic\_drag\_coefficient_B
\odot |velocity\_air_B| \odot velocity\_air_B.
$$

Here $\odot$ denotes elementwise multiplication. Because every coefficient is nonnegative,
the force cannot add power relative to the air:

$$
force\_drag_B^{T} velocity\_air_B
= -\sum_i c_i |v_i|^3 \le 0.
$$

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
`Log(R_estimate_WB.T @ R_reference_WB)`, expressed in the estimate's local body
coordinates. Thus `R_reference_WB = R_estimate_WB @ Exp(skew(delta_theta_B))`.
This is the same right-local convention used by injection and covariance reset.

The evaluation reference pairs artifact truth/bias row `j` with replay row `j-1`,
for completed rows `j=1..N`. It compares the final post-update estimate and its
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
