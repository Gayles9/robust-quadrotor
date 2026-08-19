import numpy as np
from numpy.typing import NDArray

from quadrotor_math.dynamics import (
    rigid_body_state_derivative_from_rotor_speeds,
)
from quadrotor_math.rotations import normalize_quaternion_body_to_world


def rigid_body_state_euler_step_from_rotor_speeds(
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
    time_step: float,
) -> tuple[
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
]:
    """Advance the rigid-body state by one explicit-Euler integration step.

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
        time_step: Explicit-Euler integration interval in seconds.

    Returns:
        A tuple containing, in order, ``next_position_W`` with shape ``(3,)``
        in the NED world frame in metres, ``next_velocity_W`` with shape
        ``(3,)`` in the NED world frame in m/s, ``next_q_WB`` as a
        dimensionless body-to-world unit quaternion with shape ``(4,)``, and
        ``next_omega_B`` with shape ``(3,)`` in the FRD body frame in rad/s.
        The quaternion is normalized after its explicit-Euler update.

    Raises:
        ValueError: If ``time_step`` is non-finite or non-positive, or if
            another input is rejected by the composed dynamics or
            quaternion-normalization functions.
    """
    if not np.isfinite(time_step):
        raise ValueError("time_step must be finite")

    if time_step <= 0.0:
        raise ValueError("time_step must be positive")

    (
        position_derivative_W,
        velocity_derivative_W,
        quaternion_derivative_WB,
        angular_velocity_derivative_B,
    ) = rigid_body_state_derivative_from_rotor_speeds(
        position_W,
        velocity_W,
        q_WB,
        omega_B,
        rotor_omega,
        rotor_positions_B,
        rotor_spin_directions,
        mass,
        inertia_B,
        gravity_acceleration,
        thrust_coefficient,
        moment_coefficient,
    )

    next_position_W = position_W + time_step * position_derivative_W
    next_velocity_W = velocity_W + time_step * velocity_derivative_W
    unnormalized_next_q_WB = q_WB + time_step * quaternion_derivative_WB
    next_omega_B = omega_B + time_step * angular_velocity_derivative_B

    next_q_WB = normalize_quaternion_body_to_world(unnormalized_next_q_WB)

    return (
        np.asarray(next_position_W, dtype=np.float64),
        np.asarray(next_velocity_W, dtype=np.float64),
        np.asarray(next_q_WB, dtype=np.float64),
        np.asarray(next_omega_B, dtype=np.float64),
    )
