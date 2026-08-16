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

## Boundaries and Naming

ROS ENU/FLU conversions will be isolated in adapter functions when integration
is introduced. ENU/FLU conventions must never leak into the mathematical core.

Public rotation and quaternion names must include their source and destination
frames. Vector names must include their expression frame. Ambiguous public names
such as `q`, `R`, `velocity`, `force`, or `omega` are prohibited.
