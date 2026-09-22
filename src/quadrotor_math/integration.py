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
    *,
    wind_velocity_W: NDArray[np.float64] | None = None,
    quadratic_drag_coefficient_B: NDArray[np.float64] | None = None,
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
        wind_velocity_W: Optional constant wind velocity with shape ``(3,)``
            in the NED world frame in m/s. Omission means calm air.
        quadratic_drag_coefficient_B: Optional nonnegative per-body-axis
            quadratic drag coefficients with shape ``(3,)`` in kg/m.
            Omission means exact zero drag.

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
        wind_velocity_W=wind_velocity_W,
        quadratic_drag_coefficient_B=quadratic_drag_coefficient_B,
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


def rigid_body_state_rk4_step_from_rotor_speeds(
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
    *,
    wind_velocity_W: NDArray[np.float64] | None = None,
    quadratic_drag_coefficient_B: NDArray[np.float64] | None = None,
) -> tuple[
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
]:
    """Advance the rigid-body state with fixed-step fourth-order Runge-Kutta.

    The four stages evaluate the complete state derivative at the current
    state, two successive half-step states, and a full-step state. Classical
    RK4 weighting then combines those derivatives as ``(k1 + 2*k2 + 2*k3 +
    k4) / 6``. In accessible terms, RK4 samples how the state is changing at
    the start, at two intermediate states, and at an end-like stage before
    combining those samples. Rotor inputs and physical parameters remain
    constant throughout the step. Each intermediate quaternion is normalized
    before its derivative evaluation, and the final quaternion is normalized
    after the weighted update.

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
        time_step: Fixed RK4 integration interval in seconds.
        wind_velocity_W: Optional constant wind velocity with shape ``(3,)``
            in the NED world frame in m/s. Omission means calm air.
        quadratic_drag_coefficient_B: Optional nonnegative per-body-axis
            quadratic drag coefficients with shape ``(3,)`` in kg/m.
            Omission means exact zero drag.

    Returns:
        A tuple containing, in order, ``next_position_W`` with shape ``(3,)``
        in the NED world frame in metres, ``next_velocity_W`` with shape
        ``(3,)`` in the NED world frame in m/s, ``next_q_WB`` as a
        dimensionless body-to-world unit quaternion with shape ``(4,)``, and
        ``next_omega_B`` with shape ``(3,)`` in the FRD body frame in rad/s.

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
        k1_position_W,
        k1_velocity_W,
        k1_q_WB,
        k1_omega_B,
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
        wind_velocity_W=wind_velocity_W,
        quadratic_drag_coefficient_B=quadratic_drag_coefficient_B,
    )

    position_k2_W = position_W + 0.5 * time_step * k1_position_W
    velocity_k2_W = velocity_W + 0.5 * time_step * k1_velocity_W
    q_k2_WB = normalize_quaternion_body_to_world(q_WB + 0.5 * time_step * k1_q_WB)
    omega_k2_B = omega_B + 0.5 * time_step * k1_omega_B
    (
        k2_position_W,
        k2_velocity_W,
        k2_q_WB,
        k2_omega_B,
    ) = rigid_body_state_derivative_from_rotor_speeds(
        position_k2_W,
        velocity_k2_W,
        q_k2_WB,
        omega_k2_B,
        rotor_omega,
        rotor_positions_B,
        rotor_spin_directions,
        mass,
        inertia_B,
        gravity_acceleration,
        thrust_coefficient,
        moment_coefficient,
        wind_velocity_W=wind_velocity_W,
        quadratic_drag_coefficient_B=quadratic_drag_coefficient_B,
    )

    position_k3_W = position_W + 0.5 * time_step * k2_position_W
    velocity_k3_W = velocity_W + 0.5 * time_step * k2_velocity_W
    q_k3_WB = normalize_quaternion_body_to_world(q_WB + 0.5 * time_step * k2_q_WB)
    omega_k3_B = omega_B + 0.5 * time_step * k2_omega_B
    (
        k3_position_W,
        k3_velocity_W,
        k3_q_WB,
        k3_omega_B,
    ) = rigid_body_state_derivative_from_rotor_speeds(
        position_k3_W,
        velocity_k3_W,
        q_k3_WB,
        omega_k3_B,
        rotor_omega,
        rotor_positions_B,
        rotor_spin_directions,
        mass,
        inertia_B,
        gravity_acceleration,
        thrust_coefficient,
        moment_coefficient,
        wind_velocity_W=wind_velocity_W,
        quadratic_drag_coefficient_B=quadratic_drag_coefficient_B,
    )

    position_k4_W = position_W + time_step * k3_position_W
    velocity_k4_W = velocity_W + time_step * k3_velocity_W
    q_k4_WB = normalize_quaternion_body_to_world(q_WB + time_step * k3_q_WB)
    omega_k4_B = omega_B + time_step * k3_omega_B
    (
        k4_position_W,
        k4_velocity_W,
        k4_q_WB,
        k4_omega_B,
    ) = rigid_body_state_derivative_from_rotor_speeds(
        position_k4_W,
        velocity_k4_W,
        q_k4_WB,
        omega_k4_B,
        rotor_omega,
        rotor_positions_B,
        rotor_spin_directions,
        mass,
        inertia_B,
        gravity_acceleration,
        thrust_coefficient,
        moment_coefficient,
        wind_velocity_W=wind_velocity_W,
        quadratic_drag_coefficient_B=quadratic_drag_coefficient_B,
    )

    next_position_W = position_W + (time_step / 6.0) * (
        k1_position_W + 2.0 * k2_position_W + 2.0 * k3_position_W + k4_position_W
    )
    next_velocity_W = velocity_W + (time_step / 6.0) * (
        k1_velocity_W + 2.0 * k2_velocity_W + 2.0 * k3_velocity_W + k4_velocity_W
    )
    next_q_WB = normalize_quaternion_body_to_world(
        q_WB + (time_step / 6.0) * (k1_q_WB + 2.0 * k2_q_WB + 2.0 * k3_q_WB + k4_q_WB)
    )
    next_omega_B = omega_B + (time_step / 6.0) * (
        k1_omega_B + 2.0 * k2_omega_B + 2.0 * k3_omega_B + k4_omega_B
    )

    return (
        np.asarray(next_position_W, dtype=np.float64),
        np.asarray(next_velocity_W, dtype=np.float64),
        np.asarray(next_q_WB, dtype=np.float64),
        np.asarray(next_omega_B, dtype=np.float64),
    )
