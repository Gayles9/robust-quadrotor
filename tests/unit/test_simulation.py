import numpy as np
import pytest

from quadrotor_math.simulation import (
    simulate_rigid_body_euler_from_rotor_speeds,
    simulate_rigid_body_rk4_from_rotor_speeds,
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
