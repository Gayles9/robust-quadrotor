import numpy as np
import pytest

from quadrotor_math.integration import (
    rigid_body_state_euler_step_from_rotor_speeds,
    rigid_body_state_rk4_step_from_rotor_speeds,
)
from quadrotor_math.rotations import rotation_matrix_body_to_world
from quadrotor_math.simulation import (
    simulate_rigid_body_euler_from_rotor_speeds,
    simulate_rigid_body_rk4_from_rotor_speeds,
)


def _level_hover_simulation(simulator, *, environment: dict[str, np.ndarray] | None = None):
    if environment is None:
        environment = {}
    return simulator(
        np.zeros(3, dtype=np.float64),
        np.array([2.0, 0.0, 0.0], dtype=np.float64),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        np.full(4, 500.0, dtype=np.float64),
        np.array(
            [
                [0.5, 0.0, 0.0],
                [0.0, 0.5, 0.0],
                [-0.5, 0.0, 0.0],
                [0.0, -0.5, 0.0],
            ],
            dtype=np.float64,
        ),
        np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64),
        1.0,
        np.eye(3, dtype=np.float64),
        10.0,
        1.0e-5,
        1.0e-6,
        0.1,
        3,
        **environment,
    )


def test_simulate_rigid_body_rk4_from_rotor_speeds_returns_complete_history() -> None:
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.array([1.0, -2.0, 3.0], dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.array([0.0, 0.0, 2.0], dtype=np.float64)

    rotor_omega = np.array(
        [2.0, 0.0, 0.0, 0.0],
        dtype=np.float64,
    )
    rotor_positions_B = np.array(
        [
            [2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    mass = 1.0
    inertia_B = np.diag(
        np.array([2.0, 3.0, 4.0], dtype=np.float64),
    )
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5
    time_step = 0.1
    number_of_steps = 2

    (
        time_s,
        position_history_W,
        velocity_history_W,
        q_history_WB,
        omega_history_B,
    ) = simulate_rigid_body_rk4_from_rotor_speeds(
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
        number_of_steps,
    )

    expected_time_s = np.array(
        [0.0, 0.1, 0.2],
        dtype=np.float64,
    )
    expected_position_history_W = np.array(
        [
            [10.0, 20.0, 30.0],
            [
                10.099975092370972,
                19.799997517165757,
                30.334050062655837,
            ],
            [
                10.199609583942388,
                19.599921984169995,
                30.73620322483352,
            ],
        ],
        dtype=np.float64,
    )
    expected_velocity_history_W = np.array(
        [
            [1.0, -2.0, 3.0],
            [
                0.9990073165807823,
                -2.000123449300478,
                3.681003753240262,
            ],
            [
                0.9922919275313508,
                -2.0019342481695612,
                4.362096419257244,
            ],
        ],
        dtype=np.float64,
    )
    expected_q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [
                0.9951156964771156,
                -0.0003300038558926743,
                0.004978286004512869,
                0.09858934217641005,
            ],
            [
                0.9808490136902003,
                -0.0026043673894937066,
                0.019660144660926578,
                0.19375734392629718,
            ],
        ],
        dtype=np.float64,
    )
    expected_omega_history_B = np.array(
        [
            [0.0, 0.0, 2.0],
            [
                -0.00982278964666897,
                0.1995693389233941,
                1.9500123170009223,
            ],
            [
                -0.03850600717385667,
                0.39667081311173513,
                1.9001930333218777,
            ],
        ],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        time_s,
        expected_time_s,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        position_history_W,
        expected_position_history_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        velocity_history_W,
        expected_velocity_history_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        q_history_WB,
        expected_q_history_WB,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        omega_history_B,
        expected_omega_history_B,
        atol=1e-12,
    )


def test_simulate_rigid_body_rk4_from_rotor_speeds_matches_gravity_only_ballistic_trajectory() -> (
    None
):
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.array([1.0, -2.0, -3.0], dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)

    rotor_omega = np.zeros(4, dtype=np.float64)
    rotor_positions_B = np.array(
        [
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [-0.5, 0.0, 0.0],
            [0.0, -0.5, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    mass = 2.0
    inertia_B = np.diag(np.array([2.0, 3.0, 4.0], dtype=np.float64))
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5
    time_step = 0.25
    number_of_steps = 4

    (
        time_s,
        position_history_W,
        velocity_history_W,
        q_history_WB,
        omega_history_B,
    ) = simulate_rigid_body_rk4_from_rotor_speeds(
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
        number_of_steps,
    )

    expected_time_s = (
        np.arange(
            number_of_steps + 1,
            dtype=np.float64,
        )
        * time_step
    )
    gravity_W = np.array(
        [0.0, 0.0, gravity_acceleration],
        dtype=np.float64,
    )
    expected_position_history_W = (
        position_W
        + expected_time_s[:, None] * velocity_W
        + 0.5 * expected_time_s[:, None] ** 2 * gravity_W
    )
    expected_velocity_history_W = velocity_W + expected_time_s[:, None] * gravity_W
    expected_q_history_WB = np.tile(
        q_WB,
        (number_of_steps + 1, 1),
    )
    expected_omega_history_B = np.zeros(
        (number_of_steps + 1, 3),
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        time_s,
        expected_time_s,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        position_history_W,
        expected_position_history_W,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        velocity_history_W,
        expected_velocity_history_W,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        q_history_WB,
        expected_q_history_WB,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        omega_history_B,
        expected_omega_history_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_simulate_rigid_body_rk4_from_rotor_speeds_conserves_gravity_only_mechanical_energy() -> (
    None
):
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.array([1.0, -2.0, -3.0], dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)
    rotor_omega = np.zeros(4, dtype=np.float64)

    rotor_positions_B = np.array(
        [
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [-0.5, 0.0, 0.0],
            [0.0, -0.5, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    mass = 2.0
    inertia_B = np.diag(np.array([2.0, 3.0, 4.0], dtype=np.float64))
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5
    time_step = 0.25
    number_of_steps = 4

    (
        _time_s,
        position_history_W,
        velocity_history_W,
        _q_history_WB,
        _omega_history_B,
    ) = simulate_rigid_body_rk4_from_rotor_speeds(
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
        number_of_steps,
    )

    kinetic_energy = 0.5 * mass * np.sum(velocity_history_W**2, axis=1)
    potential_energy = -mass * gravity_acceleration * position_history_W[:, 2]
    mechanical_energy = kinetic_energy + potential_energy

    initial_mechanical_energy = -574.6
    expected_mechanical_energy = np.full(
        number_of_steps + 1,
        initial_mechanical_energy,
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        mechanical_energy,
        expected_mechanical_energy,
        rtol=0.0,
        atol=1e-12,
    )


def test_rk4_torque_free_rotation_nearly_conserves_inertial_angular_momentum() -> None:
    position_W = np.zeros(3, dtype=np.float64)
    velocity_W = np.zeros(3, dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.array([0.7, -0.4, 1.1], dtype=np.float64)
    rotor_omega = np.zeros(4, dtype=np.float64)

    rotor_positions_B = np.array(
        [
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [-0.5, 0.0, 0.0],
            [0.0, -0.5, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    mass = 2.0
    inertia_B = np.diag(
        np.array([2.0, 3.0, 4.0], dtype=np.float64),
    )
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5
    time_step = 0.05
    number_of_steps = 200

    (
        _time_s,
        _position_history_W,
        _velocity_history_W,
        q_history_WB,
        omega_history_B,
    ) = simulate_rigid_body_rk4_from_rotor_speeds(
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
        number_of_steps,
    )

    angular_momentum_history_B = (inertia_B @ omega_history_B.T).T
    angular_momentum_history_W = np.asarray(
        [
            rotation_matrix_body_to_world(q_sample_WB) @ angular_momentum_sample_B
            for q_sample_WB, angular_momentum_sample_B in zip(
                q_history_WB,
                angular_momentum_history_B,
                strict=True,
            )
        ],
        dtype=np.float64,
    )

    expected_angular_momentum_W = np.array(
        [1.4, -1.2, 4.4],
        dtype=np.float64,
    )
    expected_angular_momentum_history_W = np.tile(
        expected_angular_momentum_W,
        (number_of_steps + 1, 1),
    )
    np.testing.assert_allclose(
        angular_momentum_history_W,
        expected_angular_momentum_history_W,
        rtol=0.0,
        atol=5e-7,
    )


def test_simulate_rigid_body_euler_from_rotor_speeds_returns_complete_history() -> None:
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.array([1.0, -2.0, 3.0], dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.array([0.0, 0.0, 2.0], dtype=np.float64)

    rotor_omega = np.array([2.0, 0.0, 0.0, 0.0], dtype=np.float64)
    rotor_positions_B = np.array(
        [
            [2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    mass = 1.0
    inertia_B = np.diag(
        np.array([2.0, 3.0, 4.0], dtype=np.float64),
    )
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5
    time_step = 0.1
    number_of_steps = 2

    (
        time_s,
        position_history_W,
        velocity_history_W,
        q_history_WB,
        omega_history_B,
    ) = simulate_rigid_body_euler_from_rotor_speeds(
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
        number_of_steps,
    )

    expected_time_s = np.array(
        [0.0, 0.1, 0.2],
        dtype=np.float64,
    )
    expected_position_history_W = np.array(
        [
            [10.0, 20.0, 30.0],
            [10.1, 19.8, 30.3],
            [10.2, 19.6, 30.6681],
        ],
        dtype=np.float64,
    )
    expected_velocity_history_W = np.array(
        [
            [1.0, -2.0, 3.0],
            [1.0, -2.0, 3.681],
            [1.0, -2.0, 4.362],
        ],
        dtype=np.float64,
    )
    expected_q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [
                0.9950371902099893,
                0.0,
                0.0,
                0.09950371902099893,
            ],
            [
                0.9806367145280214,
                -0.0009902920621338263,
                0.009902920621338263,
                0.19558268227143066,
            ],
        ],
        dtype=np.float64,
    )
    expected_omega_history_B = np.array(
        [
            [0.0, 0.0, 2.0],
            [0.0, 0.2, 1.95],
            [-0.0195, 0.4, 1.9],
        ],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        time_s,
        expected_time_s,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        position_history_W,
        expected_position_history_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        velocity_history_W,
        expected_velocity_history_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        q_history_WB,
        expected_q_history_WB,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        omega_history_B,
        expected_omega_history_B,
        atol=1e-12,
    )


def test_simulate_rigid_body_euler_from_rotor_speeds_matches_discrete_gravity_only_trajectory() -> (
    None
):
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.array([1.0, -2.0, -3.0], dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)
    rotor_omega = np.zeros(4, dtype=np.float64)

    rotor_positions_B = np.array(
        [
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [-0.5, 0.0, 0.0],
            [0.0, -0.5, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    mass = 2.0
    inertia_B = np.diag(np.array([2.0, 3.0, 4.0], dtype=np.float64))
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5
    time_step = 0.25
    number_of_steps = 4

    (
        time_s,
        position_history_W,
        velocity_history_W,
        q_history_WB,
        omega_history_B,
    ) = simulate_rigid_body_euler_from_rotor_speeds(
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
        number_of_steps,
    )

    expected_time_s = (
        np.arange(
            number_of_steps + 1,
            dtype=np.float64,
        )
        * time_step
    )
    gravity_W = np.array(
        [0.0, 0.0, gravity_acceleration],
        dtype=np.float64,
    )
    expected_velocity_history_W = velocity_W + expected_time_s[:, None] * gravity_W
    expected_position_history_W = (
        position_W
        + expected_time_s[:, None] * velocity_W
        + 0.5 * (expected_time_s[:, None] ** 2 - expected_time_s[:, None] * time_step) * gravity_W
    )
    expected_q_history_WB = np.tile(
        q_WB,
        (number_of_steps + 1, 1),
    )
    expected_omega_history_B = np.zeros(
        (number_of_steps + 1, 3),
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        time_s,
        expected_time_s,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        position_history_W,
        expected_position_history_W,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        velocity_history_W,
        expected_velocity_history_W,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        q_history_WB,
        expected_q_history_WB,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        omega_history_B,
        expected_omega_history_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_simulate_rigid_body_euler_from_rotor_speeds_has_predicted_gravity_only_energy_drift() -> (
    None
):
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.array([1.0, -2.0, -3.0], dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)
    rotor_omega = np.zeros(4, dtype=np.float64)

    rotor_positions_B = np.array(
        [
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [-0.5, 0.0, 0.0],
            [0.0, -0.5, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    mass = 2.0
    inertia_B = np.diag(np.array([2.0, 3.0, 4.0], dtype=np.float64))
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5
    time_step = 0.25
    number_of_steps = 4

    (
        time_s,
        position_history_W,
        velocity_history_W,
        _q_history_WB,
        _omega_history_B,
    ) = simulate_rigid_body_euler_from_rotor_speeds(
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
        number_of_steps,
    )

    kinetic_energy = 0.5 * mass * np.sum(velocity_history_W**2, axis=1)
    potential_energy = -mass * gravity_acceleration * position_history_W[:, 2]
    mechanical_energy = kinetic_energy + potential_energy

    initial_mechanical_energy = -574.6
    expected_mechanical_energy = (
        initial_mechanical_energy + 0.5 * mass * gravity_acceleration**2 * time_s * time_step
    )

    np.testing.assert_allclose(
        mechanical_energy,
        expected_mechanical_energy,
        rtol=0.0,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    "simulate_rigid_body_from_rotor_speeds",
    [
        simulate_rigid_body_euler_from_rotor_speeds,
        simulate_rigid_body_rk4_from_rotor_speeds,
    ],
)
def test_simulators_preserve_balanced_hover_equilibrium(
    simulate_rigid_body_from_rotor_speeds,
) -> None:
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.zeros(3, dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)

    mass = 1.0
    inertia_B = np.diag(np.array([2.0, 3.0, 4.0], dtype=np.float64))
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5

    rotor_positions_B = np.array(
        [
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [-0.5, 0.0, 0.0],
            [0.0, -0.5, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    hover_rotor_speed = np.sqrt(mass * gravity_acceleration / (4.0 * thrust_coefficient))
    rotor_omega = np.full(4, hover_rotor_speed, dtype=np.float64)

    time_step = 0.05
    number_of_steps = 20

    (
        time_s,
        position_history_W,
        velocity_history_W,
        q_history_WB,
        omega_history_B,
    ) = simulate_rigid_body_from_rotor_speeds(
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
        number_of_steps,
    )

    number_of_samples = number_of_steps + 1
    expected_time_s = np.arange(number_of_samples, dtype=np.float64) * time_step
    expected_position_history_W = np.tile(position_W, (number_of_samples, 1))
    expected_velocity_history_W = np.zeros((number_of_samples, 3), dtype=np.float64)
    expected_q_history_WB = np.tile(q_WB, (number_of_samples, 1))
    expected_omega_history_B = np.zeros((number_of_samples, 3), dtype=np.float64)

    np.testing.assert_allclose(
        time_s,
        expected_time_s,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        position_history_W,
        expected_position_history_W,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        velocity_history_W,
        expected_velocity_history_W,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        q_history_WB,
        expected_q_history_WB,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        omega_history_B,
        expected_omega_history_B,
        rtol=0.0,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    ("simulator", "stepper"),
    [
        (
            simulate_rigid_body_euler_from_rotor_speeds,
            rigid_body_state_euler_step_from_rotor_speeds,
        ),
        (
            simulate_rigid_body_rk4_from_rotor_speeds,
            rigid_body_state_rk4_step_from_rotor_speeds,
        ),
    ],
)
def test_environmental_history_matches_repeated_public_steps(simulator, stepper) -> None:
    wind_velocity_W = np.array([1.0, 0.0, 0.0])
    coefficient_B = np.array([0.5, 0.0, 0.0])
    environment = {
        "wind_velocity_W": wind_velocity_W,
        "quadratic_drag_coefficient_B": coefficient_B,
    }
    actual = _level_hover_simulation(simulator, environment=environment)

    state = (
        np.zeros(3, dtype=np.float64),
        np.array([2.0, 0.0, 0.0], dtype=np.float64),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        np.zeros(3, dtype=np.float64),
    )
    expected_states = [[value.copy()] for value in state]
    for _ in range(3):
        state = stepper(
            *state,
            np.full(4, 500.0, dtype=np.float64),
            np.array(
                [
                    [0.5, 0.0, 0.0],
                    [0.0, 0.5, 0.0],
                    [-0.5, 0.0, 0.0],
                    [0.0, -0.5, 0.0],
                ]
            ),
            np.array([1.0, -1.0, 1.0, -1.0]),
            1.0,
            np.eye(3),
            10.0,
            1.0e-5,
            1.0e-6,
            0.1,
            **environment,
        )
        for expected_component, state_component in zip(expected_states, state, strict=True):
            expected_component.append(state_component.copy())

    np.testing.assert_array_equal(actual[0], np.arange(4, dtype=np.float64) * 0.1)
    for actual_history, expected_history in zip(actual[1:], expected_states, strict=True):
        np.testing.assert_array_equal(actual_history, np.asarray(expected_history))


@pytest.mark.parametrize(
    "simulator",
    [
        simulate_rigid_body_euler_from_rotor_speeds,
        simulate_rigid_body_rk4_from_rotor_speeds,
    ],
)
def test_environmental_history_preserves_zero_regression_and_array_contracts(simulator) -> None:
    omitted = _level_hover_simulation(simulator)
    explicit_zero = _level_hover_simulation(
        simulator,
        environment={
            "wind_velocity_W": np.zeros(3),
            "quadratic_drag_coefficient_B": np.zeros(3),
        },
    )
    with_drag = _level_hover_simulation(
        simulator,
        environment={
            "wind_velocity_W": np.zeros(3),
            "quadratic_drag_coefficient_B": np.array([0.5, 0.0, 0.0]),
        },
    )

    expected_shapes = ((4,), (4, 3), (4, 3), (4, 4), (4, 3))
    for omitted_array, explicit_array, shape in zip(
        omitted, explicit_zero, expected_shapes, strict=True
    ):
        np.testing.assert_array_equal(explicit_array, omitted_array)
        assert omitted_array.dtype == np.float64
        assert omitted_array.shape == shape
        assert omitted_array.flags.owndata
        assert omitted_array.flags.writeable
    assert not np.array_equal(with_drag[2], omitted[2])


@pytest.mark.parametrize(
    ("number_of_steps", "expected_message"),
    [
        (2.0, "number_of_steps must be an integer"),
        (True, "number_of_steps must be an integer"),
        (False, "number_of_steps must be an integer"),
        (0, "number_of_steps must be positive"),
        (-1, "number_of_steps must be positive"),
    ],
)
def test_simulate_rigid_body_euler_rejects_invalid_number_of_steps(
    number_of_steps: object,
    expected_message: str,
) -> None:
    position_W = np.zeros(3, dtype=np.float64)
    velocity_W = np.zeros(3, dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)
    rotor_omega = np.zeros(4, dtype=np.float64)
    rotor_positions_B = np.zeros((4, 3), dtype=np.float64)
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )
    mass = 1.0
    inertia_B = np.eye(3, dtype=np.float64)
    gravity_acceleration = 9.81
    thrust_coefficient = 1.0
    moment_coefficient = 1.0
    time_step = 0.1

    with pytest.raises(ValueError, match=expected_message):
        simulate_rigid_body_euler_from_rotor_speeds(
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
            number_of_steps,  # type: ignore[arg-type]
        )


@pytest.mark.parametrize(
    ("number_of_steps", "expected_message"),
    [
        (2.0, "number_of_steps must be an integer"),
        (True, "number_of_steps must be an integer"),
        (False, "number_of_steps must be an integer"),
        (0, "number_of_steps must be positive"),
        (-1, "number_of_steps must be positive"),
    ],
)
def test_simulate_rigid_body_rk4_rejects_invalid_number_of_steps(
    number_of_steps: object,
    expected_message: str,
) -> None:
    position_W = np.zeros(3, dtype=np.float64)
    velocity_W = np.zeros(3, dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)
    rotor_omega = np.zeros(4, dtype=np.float64)
    rotor_positions_B = np.zeros((4, 3), dtype=np.float64)
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )
    mass = 1.0
    inertia_B = np.eye(3, dtype=np.float64)
    gravity_acceleration = 9.81
    thrust_coefficient = 1.0
    moment_coefficient = 1.0
    time_step = 0.1

    with pytest.raises(ValueError, match=expected_message):
        simulate_rigid_body_rk4_from_rotor_speeds(
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
            number_of_steps,  # type: ignore[arg-type]
        )
