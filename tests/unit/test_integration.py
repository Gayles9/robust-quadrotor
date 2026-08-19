import numpy as np
import pytest

from quadrotor_math.integration import (
    rigid_body_state_euler_step_from_rotor_speeds,
)


def test_rigid_body_state_euler_step_from_rotor_speeds_advances_complete_state() -> None:
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

    (
        next_position_W,
        next_velocity_W,
        next_q_WB,
        next_omega_B,
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

    expected_position_W = np.array(
        [10.1, 19.8, 30.3],
        dtype=np.float64,
    )
    expected_velocity_W = np.array(
        [1.0, -2.0, 3.681],
        dtype=np.float64,
    )
    quaternion_norm = np.sqrt(1.01)
    expected_q_WB = np.array(
        [
            1.0 / quaternion_norm,
            0.0,
            0.0,
            0.1 / quaternion_norm,
        ],
        dtype=np.float64,
    )
    expected_omega_B = np.array(
        [0.0, 0.2, 1.95],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        next_position_W,
        expected_position_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        next_velocity_W,
        expected_velocity_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        next_q_WB,
        expected_q_WB,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        next_omega_B,
        expected_omega_B,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    ("time_step", "expected_message"),
    [
        (np.nan, "time_step must be finite"),
        (np.inf, "time_step must be finite"),
        (0.0, "time_step must be positive"),
        (-0.1, "time_step must be positive"),
    ],
)
def test_rigid_body_state_euler_step_rejects_invalid_time_step(
    time_step: float,
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

    with pytest.raises(ValueError, match=expected_message):
        rigid_body_state_euler_step_from_rotor_speeds(
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
