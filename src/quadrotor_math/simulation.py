import numpy as np
from numpy.typing import NDArray

from quadrotor_math.integration import (
    rigid_body_state_euler_step_from_rotor_speeds,
    rigid_body_state_rk4_step_from_rotor_speeds,
)


def simulate_rigid_body_euler_from_rotor_speeds(
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
    number_of_steps: int,
) -> tuple[
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
]:
    """Propagate a deterministic rigid-body history with explicit Euler.

    Rotor speeds, rotor geometry, and physical parameters remain constant over
    all fixed-time-step transitions. Each returned Euler state becomes the
    starting state for the next transition.

    Args:
        position_W: Initial position with shape ``(3,)``, expressed in the
            north-east-down (NED) world frame in metres.
        velocity_W: Initial translational velocity with shape ``(3,)``,
            expressed in the NED world frame in m/s.
        q_WB: Initial dimensionless Hamilton scalar-first body-to-world
            quaternion with shape ``(4,)`` and components ``[w, x, y, z]``.
            It maps the forward-right-down (FRD) body frame to the NED world
            frame.
        omega_B: Initial angular velocity with shape ``(3,)``, expressed in
            the FRD body frame in rad/s.
        rotor_omega: Constant rotor angular speeds with shape ``(4,)`` in
            rad/s.
        rotor_positions_B: Constant rotor positions with shape ``(4, 3)``,
            expressed in the FRD body frame in metres.
        rotor_spin_directions: Constant dimensionless rotor spin directions
            with shape ``(4,)`` containing only ``-1`` or ``+1``.
        mass: Constant scalar vehicle mass in kilograms.
        inertia_B: Constant inertia tensor with shape ``(3, 3)``, expressed
            about the centre of mass in body coordinates in kg·m².
        gravity_acceleration: Constant scalar positive gravity magnitude in
            m/s².
        thrust_coefficient: Constant scalar thrust coefficient in
            N/(rad/s)^2.
        moment_coefficient: Constant scalar moment coefficient in
            N·m/(rad/s)^2.
        time_step: Fixed explicit-Euler integration interval in seconds.
        number_of_steps: Positive non-Boolean integer number of explicit-Euler
            state transitions. The returned histories contain one additional
            row for the initial state.

    Returns:
        A tuple containing, in order, ``time_s`` with shape ``(N + 1,)`` in
        seconds, ``position_history_W`` with shape ``(N + 1, 3)`` in the NED
        world frame in metres, ``velocity_history_W`` with shape
        ``(N + 1, 3)`` in the NED world frame in m/s, ``q_history_WB`` with
        shape ``(N + 1, 4)`` as dimensionless body-to-world unit
        quaternions, and ``omega_history_B`` with shape ``(N + 1, 3)`` in the
        FRD body frame in rad/s, where ``N`` is ``number_of_steps``. Row zero
        contains the supplied initial state, and ``time_s[i] = i *
        time_step``.

    Raises:
        ValueError: If ``number_of_steps`` is not an integer or is not
            positive, or if another input is rejected by the composed
            explicit-Euler stepper. Validation of the time step, state, rotor
            inputs, and physical parameters is delegated to that stepper and
            its composed functions.
    """
    if not isinstance(number_of_steps, int) or isinstance(number_of_steps, bool):
        raise ValueError("number_of_steps must be an integer")

    if number_of_steps <= 0:
        raise ValueError("number_of_steps must be positive")

    (
        first_position_W,
        first_velocity_W,
        first_q_WB,
        first_omega_B,
    ) = rigid_body_state_euler_step_from_rotor_speeds(
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
        time_step,
    )

    time_s = np.arange(number_of_steps + 1, dtype=np.float64) * time_step
    position_history_W = np.empty((number_of_steps + 1, 3), dtype=np.float64)
    velocity_history_W = np.empty((number_of_steps + 1, 3), dtype=np.float64)
    q_history_WB = np.empty((number_of_steps + 1, 4), dtype=np.float64)
    omega_history_B = np.empty((number_of_steps + 1, 3), dtype=np.float64)

    position_history_W[0] = position_W
    velocity_history_W[0] = velocity_W
    q_history_WB[0] = q_WB
    omega_history_B[0] = omega_B

    position_history_W[1] = first_position_W
    velocity_history_W[1] = first_velocity_W
    q_history_WB[1] = first_q_WB
    omega_history_B[1] = first_omega_B

    for history_index in range(2, number_of_steps + 1):
        (
            next_position_W,
            next_velocity_W,
            next_q_WB,
            next_omega_B,
        ) = rigid_body_state_euler_step_from_rotor_speeds(
            position_history_W[history_index - 1],
            velocity_history_W[history_index - 1],
            q_history_WB[history_index - 1],
            omega_history_B[history_index - 1],
            rotor_omega,
            rotor_positions_B,
            rotor_spin_directions,
            mass,
            inertia_B,
            gravity_acceleration,
            thrust_coefficient,
            moment_coefficient,
            time_step,
        )
        position_history_W[history_index] = next_position_W
        velocity_history_W[history_index] = next_velocity_W
        q_history_WB[history_index] = next_q_WB
        omega_history_B[history_index] = next_omega_B

    return (
        time_s,
        position_history_W,
        velocity_history_W,
        q_history_WB,
        omega_history_B,
    )


def simulate_rigid_body_rk4_from_rotor_speeds(
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
    number_of_steps: int,
) -> tuple[
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
]:
    """Propagate a deterministic rigid-body history with fixed-step RK4.

    Rotor speeds, rotor geometry, and physical parameters remain constant over
    all RK4 transitions. Each generated state becomes the starting state for
    the next transition.

    Args:
        position_W: Initial position with shape ``(3,)``, expressed in the
            north-east-down (NED) world frame in metres.
        velocity_W: Initial translational velocity with shape ``(3,)``,
            expressed in the NED world frame in m/s.
        q_WB: Initial dimensionless Hamilton scalar-first body-to-world
            quaternion with shape ``(4,)`` and components ``[w, x, y, z]``.
            It maps the forward-right-down (FRD) body frame to the NED world
            frame.
        omega_B: Initial angular velocity with shape ``(3,)``, expressed in
            the FRD body frame in rad/s.
        rotor_omega: Constant rotor angular speeds with shape ``(4,)`` in
            rad/s.
        rotor_positions_B: Constant rotor positions with shape ``(4, 3)``,
            expressed in the FRD body frame in metres.
        rotor_spin_directions: Constant dimensionless rotor spin directions
            with shape ``(4,)`` containing only ``-1`` or ``+1``.
        mass: Constant scalar vehicle mass in kilograms.
        inertia_B: Constant inertia tensor with shape ``(3, 3)``, expressed
            about the centre of mass in body coordinates in kg·m².
        gravity_acceleration: Constant scalar positive gravity magnitude in
            m/s².
        thrust_coefficient: Constant scalar thrust coefficient in
            N/(rad/s)^2.
        moment_coefficient: Constant scalar moment coefficient in
            N·m/(rad/s)^2.
        time_step: Fixed RK4 integration interval in seconds.
        number_of_steps: Number of positive-time RK4 state transitions. The
            returned histories contain one additional row for the initial
            state.

    Returns:
        A tuple containing, in order, ``time_s`` with shape ``(N + 1,)`` in
        seconds, ``position_history_W`` with shape ``(N + 1, 3)`` in the NED
        world frame in metres, ``velocity_history_W`` with shape
        ``(N + 1, 3)`` in the NED world frame in m/s, ``q_history_WB`` with
        shape ``(N + 1, 4)`` as dimensionless body-to-world unit
        quaternions, and ``omega_history_B`` with shape ``(N + 1, 3)`` in the
        FRD body frame in rad/s, where ``N`` is ``number_of_steps``. Row zero
        contains the supplied initial state, and ``time_s[i] = i *
        time_step``.

    Raises:
        ValueError: If ``number_of_steps`` is not an integer or is not
            positive, or if another input is rejected by the composed RK4
            stepper. Validation of the time step, state, rotor inputs, and
            physical parameters is delegated to that stepper and its composed
            functions.
    """
    if not isinstance(number_of_steps, int) or isinstance(number_of_steps, bool):
        raise ValueError("number_of_steps must be an integer")

    if number_of_steps <= 0:
        raise ValueError("number_of_steps must be positive")

    (
        first_position_W,
        first_velocity_W,
        first_q_WB,
        first_omega_B,
    ) = rigid_body_state_rk4_step_from_rotor_speeds(
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
        time_step,
    )

    time_s = np.arange(number_of_steps + 1, dtype=np.float64) * time_step
    position_history_W = np.empty((number_of_steps + 1, 3), dtype=np.float64)
    velocity_history_W = np.empty((number_of_steps + 1, 3), dtype=np.float64)
    q_history_WB = np.empty((number_of_steps + 1, 4), dtype=np.float64)
    omega_history_B = np.empty((number_of_steps + 1, 3), dtype=np.float64)

    position_history_W[0] = position_W
    velocity_history_W[0] = velocity_W
    q_history_WB[0] = q_WB
    omega_history_B[0] = omega_B

    position_history_W[1] = first_position_W
    velocity_history_W[1] = first_velocity_W
    q_history_WB[1] = first_q_WB
    omega_history_B[1] = first_omega_B

    for history_index in range(2, number_of_steps + 1):
        (
            next_position_W,
            next_velocity_W,
            next_q_WB,
            next_omega_B,
        ) = rigid_body_state_rk4_step_from_rotor_speeds(
            position_history_W[history_index - 1],
            velocity_history_W[history_index - 1],
            q_history_WB[history_index - 1],
            omega_history_B[history_index - 1],
            rotor_omega,
            rotor_positions_B,
            rotor_spin_directions,
            mass,
            inertia_B,
            gravity_acceleration,
            thrust_coefficient,
            moment_coefficient,
            time_step,
        )
        position_history_W[history_index] = next_position_W
        velocity_history_W[history_index] = next_velocity_W
        q_history_WB[history_index] = next_q_WB
        omega_history_B[history_index] = next_omega_B

    return (
        time_s,
        position_history_W,
        velocity_history_W,
        q_history_WB,
        omega_history_B,
    )
