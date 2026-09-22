import numpy as np
from numpy.typing import NDArray

from quadrotor_math.actuation import force_and_moment_body_from_rotor_speeds
from quadrotor_math.rotations import (
    quaternion_derivative_body_to_world,
    rotation_matrix_body_to_world,
)


def quadratic_drag_force_body(
    velocity_W: NDArray[np.float64],
    R_WB: NDArray[np.float64],
    wind_velocity_W: NDArray[np.float64],
    quadratic_drag_coefficient_B: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return anisotropic quadratic aerodynamic force in the FRD body frame.

    Vehicle and constant wind velocities are NED vectors in m/s. ``R_WB``
    maps FRD body vectors into NED world coordinates. The per-body-axis
    coefficients have units kg/m, and the returned force has units N.
    The force opposes air-relative motion and is applied at the centre of mass.

    Inputs are checked in shape, finiteness, then coefficient-domain order.
    Finite inputs that overflow the calculation raise ``ValueError``.
    """
    if velocity_W.shape != (3,):
        raise ValueError("velocity_W must have shape (3,)")
    if R_WB.shape != (3, 3):
        raise ValueError("R_WB must have shape (3, 3)")
    if wind_velocity_W.shape != (3,):
        raise ValueError("wind_velocity_W must have shape (3,)")
    if quadratic_drag_coefficient_B.shape != (3,):
        raise ValueError("quadratic_drag_coefficient_B must have shape (3,)")

    if not np.all(np.isfinite(velocity_W)):
        raise ValueError("velocity_W must contain only finite values")
    if not np.all(np.isfinite(R_WB)):
        raise ValueError("R_WB must contain only finite values")
    if not np.all(np.isfinite(wind_velocity_W)):
        raise ValueError("wind_velocity_W must contain only finite values")
    if not np.all(np.isfinite(quadratic_drag_coefficient_B)):
        raise ValueError("quadratic_drag_coefficient_B must contain only finite values")
    if np.any(quadratic_drag_coefficient_B < 0.0):
        raise ValueError("quadratic_drag_coefficient_B must be nonnegative")

    force_drag_B = np.zeros(3, dtype=np.float64)
    active_axes = quadratic_drag_coefficient_B > 0.0
    if not np.any(active_axes):
        return force_drag_B

    try:
        with np.errstate(over="raise", invalid="raise"):
            velocity_air_W = velocity_W - wind_velocity_W
            velocity_air_B = R_WB.T @ velocity_air_W
            force_drag_B[active_axes] = (
                -quadratic_drag_coefficient_B[active_axes]
                * np.abs(velocity_air_B[active_axes])
                * velocity_air_B[active_axes]
            )
    except FloatingPointError:
        raise ValueError("quadratic drag calculation must remain finite") from None

    if not np.all(np.isfinite(velocity_air_W)) or not np.all(np.isfinite(velocity_air_B)):
        raise ValueError("quadratic drag calculation must remain finite")
    if not np.all(np.isfinite(force_drag_B)):
        raise ValueError("quadratic drag calculation must remain finite")
    return force_drag_B


def angular_acceleration_body_from_moment(
    moment_B: NDArray[np.float64],
    omega_B: NDArray[np.float64],
    inertia_B: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return angular acceleration expressed in the FRD body frame.

    Args:
        moment_B: Moment with shape ``(3,)``, expressed in the forward-right-down
            (FRD) body frame in N·m.
        omega_B: Angular velocity with shape ``(3,)``, expressed in the
            forward-right-down (FRD) body frame in rad/s.
        inertia_B: Inertia tensor with shape ``(3, 3)``, expressed about the
            centre of mass in body coordinates in kg·m².

    Returns:
        ``angular_acceleration_B`` with shape ``(3,)``, expressed in the
        forward-right-down (FRD) body frame in rad/s².

    Raises:
        ValueError: If ``moment_B`` does not have shape ``(3,)``, ``omega_B``
            does not have shape ``(3,)``, or ``inertia_B`` does not have shape
            ``(3, 3)``, or if ``moment_B``, ``omega_B``, or ``inertia_B``
            contains a non-finite value, or if ``inertia_B`` is not symmetric
            or positive definite.

    The implemented equation is::

        I_B @ angular_acceleration_B
            = moment_B - cross(omega_B, I_B @ omega_B)
    """
    if moment_B.shape != (3,):
        raise ValueError("moment_B must have shape (3,)")

    if omega_B.shape != (3,):
        raise ValueError("omega_B must have shape (3,)")

    if inertia_B.shape != (3, 3):
        raise ValueError("inertia_B must have shape (3, 3)")

    if not np.all(np.isfinite(moment_B)):
        raise ValueError("moment_B must contain only finite values")

    if not np.all(np.isfinite(omega_B)):
        raise ValueError("omega_B must contain only finite values")

    if not np.all(np.isfinite(inertia_B)):
        raise ValueError("inertia_B must contain only finite values")

    if not np.allclose(
        inertia_B,
        inertia_B.T,
        rtol=0.0,
        atol=1e-12,
    ):
        raise ValueError("inertia_B must be symmetric")

    try:
        np.linalg.cholesky(inertia_B)
    except np.linalg.LinAlgError as error:
        raise ValueError("inertia_B must be positive definite") from error

    angular_momentum_B = inertia_B @ omega_B
    gyroscopic_moment_B = np.cross(
        omega_B,
        angular_momentum_B,
    )
    net_moment_B = moment_B - gyroscopic_moment_B
    angular_acceleration_B = np.linalg.solve(
        inertia_B,
        net_moment_B,
    )
    return np.asarray(
        angular_acceleration_B,
        dtype=np.float64,
    )


def translational_acceleration_world_from_body_force(
    force_B: NDArray[np.float64],
    R_WB: NDArray[np.float64],
    mass: float,
    gravity_acceleration: float,
) -> NDArray[np.float64]:
    """Return translational acceleration expressed in the NED world frame.

    Args:
        force_B: Force with shape ``(3,)``, expressed in the forward-right-down
            (FRD) body frame in newtons.
        R_WB: Active rotation matrix with shape ``(3, 3)`` that maps
            body-coordinate vectors into north-east-down (NED) world
            coordinates.
        mass: Vehicle mass in kilograms.
        gravity_acceleration: Positive gravity magnitude in m/s^2.

    Returns:
        ``acceleration_W`` with shape ``(3,)``, expressed in the NED world
        frame in m/s^2, where positive world z points downward.

    Raises:
        ValueError: If ``force_B`` does not have shape ``(3,)`` or ``R_WB``
            does not have shape ``(3, 3)``, or if ``force_B`` or ``R_WB``
            contains a non-finite value, or if ``mass`` is non-finite or
            non-positive, or if ``gravity_acceleration`` is non-finite or
            non-positive.
    """
    if force_B.shape != (3,):
        raise ValueError("force_B must have shape (3,)")

    if R_WB.shape != (3, 3):
        raise ValueError("R_WB must have shape (3, 3)")

    if not np.all(np.isfinite(force_B)):
        raise ValueError("force_B must contain only finite values")

    if not np.all(np.isfinite(R_WB)):
        raise ValueError("R_WB must contain only finite values")

    if not np.isfinite(mass):
        raise ValueError("mass must be finite")

    if mass <= 0.0:
        raise ValueError("mass must be positive")

    if not np.isfinite(gravity_acceleration):
        raise ValueError("gravity_acceleration must be finite")

    if gravity_acceleration <= 0.0:
        raise ValueError("gravity_acceleration must be positive")

    gravity_W = np.array(
        [0.0, 0.0, gravity_acceleration],
        dtype=np.float64,
    )
    acceleration_W = gravity_W + (R_WB @ force_B) / mass
    return np.asarray(acceleration_W, dtype=np.float64)


def rigid_body_state_derivative_from_body_wrench(
    position_W: NDArray[np.float64],
    velocity_W: NDArray[np.float64],
    q_WB: NDArray[np.float64],
    omega_B: NDArray[np.float64],
    force_B: NDArray[np.float64],
    moment_B: NDArray[np.float64],
    mass: float,
    inertia_B: NDArray[np.float64],
    gravity_acceleration: float,
) -> tuple[
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
]:
    """Return the complete rigid-body state derivative from a body-frame wrench.

    Args:
        position_W: Position with shape ``(3,)``, expressed in the
            north-east-down (NED) world frame in metres.
        velocity_W: Translational velocity with shape ``(3,)``, expressed in
            the NED world frame in m/s.
        q_WB: Hamilton scalar-first body-to-world quaternion with shape
            ``(4,)`` and components ``[w, x, y, z]``. The quaternion is
            dimensionless and maps the forward-right-down (FRD) body frame to
            the NED world frame.
        omega_B: Angular velocity with shape ``(3,)``, expressed in the FRD
            body frame in rad/s.
        force_B: Force with shape ``(3,)``, expressed in the FRD body frame in
            newtons.
        moment_B: Moment with shape ``(3,)``, expressed in the FRD body frame
            in N·m.
        mass: Vehicle mass in kilograms.
        inertia_B: Inertia tensor with shape ``(3, 3)``, expressed about the
            centre of mass in body coordinates in kg·m².
        gravity_acceleration: Positive gravity magnitude in m/s².

    Returns:
        A tuple containing, in order, ``position_derivative_W`` with shape
        ``(3,)`` in the NED world frame in m/s, ``velocity_derivative_W`` with
        shape ``(3,)`` in the NED world frame in m/s²,
        ``quaternion_derivative_WB`` with shape ``(4,)`` in s⁻¹, and
        ``angular_velocity_derivative_B`` with shape ``(3,)`` in the FRD body
        frame in rad/s².

    Raises:
        ValueError: If ``position_W`` does not have shape ``(3,)`` or
            ``velocity_W`` does not have shape ``(3,)``, or if ``position_W``
            or ``velocity_W`` contains a non-finite value.
    """
    if position_W.shape != (3,):
        raise ValueError("position_W must have shape (3,)")

    if velocity_W.shape != (3,):
        raise ValueError("velocity_W must have shape (3,)")

    if not np.all(np.isfinite(position_W)):
        raise ValueError("position_W must contain only finite values")

    if not np.all(np.isfinite(velocity_W)):
        raise ValueError("velocity_W must contain only finite values")

    position_derivative_W = np.array(
        velocity_W,
        dtype=np.float64,
        copy=True,
    )

    R_WB = rotation_matrix_body_to_world(q_WB)

    velocity_derivative_W = translational_acceleration_world_from_body_force(
        force_B,
        R_WB,
        mass,
        gravity_acceleration,
    )

    quaternion_derivative_WB = quaternion_derivative_body_to_world(
        q_WB,
        omega_B,
    )

    angular_velocity_derivative_B = angular_acceleration_body_from_moment(
        moment_B,
        omega_B,
        inertia_B,
    )

    return (
        position_derivative_W,
        velocity_derivative_W,
        quaternion_derivative_WB,
        angular_velocity_derivative_B,
    )


def rigid_body_state_derivative_from_rotor_speeds(
    position_W: NDArray[np.float64],
    velocity_W: NDArray[np.float64],
    q_WB: NDArray[np.float64],
    omega_B: NDArray[np.float64],
    rotor_omega: NDArray[np.float64],
    rotor_positions_B: NDArray[np.float64],
    rotor_spin_directions: NDArray[np.float64],
    mass: float,
    inertia_B: NDArray[np.float64],
    gravity_acceleration: float,
    thrust_coefficient: float,
    moment_coefficient: float,
    *,
    wind_velocity_W: NDArray[np.float64] | None = None,
    quadratic_drag_coefficient_B: NDArray[np.float64] | None = None,
) -> tuple[
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
]:
    """Return the rigid-body state derivative directly from rotor speeds.

    Args:
        position_W: Position with shape ``(3,)``, expressed in the
            north-east-down (NED) world frame in metres.
        velocity_W: Translational velocity with shape ``(3,)``, expressed in
            the NED world frame in m/s.
        q_WB: Dimensionless Hamilton scalar-first body-to-world quaternion with
            shape ``(4,)`` and components ``[w, x, y, z]``. It maps the
            forward-right-down (FRD) body frame to the NED world frame.
        omega_B: Angular velocity with shape ``(3,)``, expressed in the FRD
            body frame in rad/s.
        rotor_omega: Rotor angular speeds with shape ``(4,)`` in rad/s.
        rotor_positions_B: Rotor positions with shape ``(4, 3)``, expressed in
            the FRD body frame in metres.
        rotor_spin_directions: Dimensionless rotor spin directions with shape
            ``(4,)`` containing only ``-1`` or ``+1``.
        mass: Scalar vehicle mass in kilograms.
        inertia_B: Inertia tensor with shape ``(3, 3)``, expressed about the
            centre of mass in body coordinates in kg·m².
        gravity_acceleration: Scalar positive gravity magnitude in m/s².
        thrust_coefficient: Scalar thrust coefficient in N/(rad/s)^2.
        moment_coefficient: Scalar moment coefficient in N·m/(rad/s)^2.
        wind_velocity_W: Optional constant wind velocity with shape ``(3,)``,
            expressed in the NED world frame in m/s. Omission means calm air.
        quadratic_drag_coefficient_B: Optional nonnegative per-body-axis
            quadratic drag coefficients with shape ``(3,)`` in kg/m. Omission
            means exact zero drag.

    Returns:
        A tuple containing, in order, ``position_derivative_W`` with shape
        ``(3,)`` in the NED world frame in m/s, ``velocity_derivative_W`` with
        shape ``(3,)`` in the NED world frame in m/s²,
        ``quaternion_derivative_WB`` with shape ``(4,)`` in s⁻¹, and
        ``angular_velocity_derivative_B`` with shape ``(3,)`` in the FRD body
        frame in rad/s².

    Raises:
        ValueError: If an input is invalid. Validation is performed by the
            composed actuation and dynamics functions.
    """
    force_B, moment_B = force_and_moment_body_from_rotor_speeds(
        rotor_omega,
        rotor_positions_B,
        rotor_spin_directions,
        thrust_coefficient,
        moment_coefficient,
    )
    combined_force_B = force_B
    if wind_velocity_W is not None or quadratic_drag_coefficient_B is not None:
        if wind_velocity_W is None:
            wind_velocity_W = np.zeros(3, dtype=np.float64)
        if quadratic_drag_coefficient_B is None:
            quadratic_drag_coefficient_B = np.zeros(3, dtype=np.float64)
        force_drag_B = quadratic_drag_force_body(
            velocity_W,
            rotation_matrix_body_to_world(q_WB),
            wind_velocity_W,
            quadratic_drag_coefficient_B,
        )
        if np.any(force_drag_B != 0.0):
            try:
                with np.errstate(over="raise", invalid="raise"):
                    combined_force_B = force_B + force_drag_B
            except FloatingPointError:
                raise ValueError(
                    "combined rotor and quadratic drag force must remain finite"
                ) from None
            if not np.all(np.isfinite(combined_force_B)):
                raise ValueError("combined rotor and quadratic drag force must remain finite")
    return rigid_body_state_derivative_from_body_wrench(
        position_W,
        velocity_W,
        q_WB,
        omega_B,
        combined_force_B,
        moment_B,
        mass,
        inertia_B,
        gravity_acceleration,
    )
