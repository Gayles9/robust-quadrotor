import numpy as np
import pytest

from quadrotor_math.imu import (
    ideal_accelerometer_specific_force_body,
    ideal_gyroscope_angular_velocity_body,
)


def test_ideal_gyroscope_angular_velocity_body_returns_body_angular_velocity() -> None:
    omega_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )

    angular_velocity_B = ideal_gyroscope_angular_velocity_body(omega_B)

    expected_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    np.testing.assert_array_equal(
        angular_velocity_B,
        expected_angular_velocity_B,
    )


def test_ideal_gyroscope_angular_velocity_body_returns_independent_measurement() -> None:
    omega_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )

    angular_velocity_B = ideal_gyroscope_angular_velocity_body(omega_B)

    assert not np.shares_memory(angular_velocity_B, omega_B)


def test_ideal_gyroscope_angular_velocity_body_rejects_invalid_shape() -> None:
    omega_B = np.zeros((3, 1), dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"omega_B must have shape \(3,\)",
    ):
        ideal_gyroscope_angular_velocity_body(omega_B)


@pytest.mark.parametrize(
    "invalid_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_ideal_gyroscope_angular_velocity_body_rejects_nonfinite_values(
    invalid_value: float,
) -> None:
    omega_B = np.array(
        [0.7, invalid_value, 1.1],
        dtype=np.float64,
    )

    with pytest.raises(
        ValueError,
        match="omega_B must contain only finite values",
    ):
        ideal_gyroscope_angular_velocity_body(omega_B)


def test_ideal_accelerometer_specific_force_body_returns_tilted_thrust_in_frd() -> None:
    translational_acceleration_W = np.array(
        [-4.0, 0.0, 9.81],
        dtype=np.float64,
    )
    R_WB = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    gravity_acceleration = 9.81

    specific_force_B = ideal_accelerometer_specific_force_body(
        translational_acceleration_W,
        R_WB,
        gravity_acceleration,
    )

    expected_specific_force_B = np.array(
        [0.0, 0.0, -4.0],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        specific_force_B,
        expected_specific_force_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_ideal_accelerometer_specific_force_body_returns_upward_specific_force_at_level_rest() -> (
    None
):
    translational_acceleration_W = np.zeros(3, dtype=np.float64)
    R_WB = np.eye(3, dtype=np.float64)
    gravity_acceleration = 9.81

    specific_force_B = ideal_accelerometer_specific_force_body(
        translational_acceleration_W,
        R_WB,
        gravity_acceleration,
    )

    expected_specific_force_B = np.array(
        [0.0, 0.0, -9.81],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        specific_force_B,
        expected_specific_force_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_ideal_accelerometer_specific_force_body_returns_zero_in_free_fall() -> None:
    gravity_acceleration = 9.81
    translational_acceleration_W = np.array(
        [0.0, 0.0, gravity_acceleration],
        dtype=np.float64,
    )
    R_WB = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )

    specific_force_B = ideal_accelerometer_specific_force_body(
        translational_acceleration_W,
        R_WB,
        gravity_acceleration,
    )

    np.testing.assert_allclose(
        specific_force_B,
        np.zeros(3, dtype=np.float64),
        rtol=0.0,
        atol=1e-12,
    )


def test_ideal_accelerometer_specific_force_body_rejects_invalid_acceleration_shape() -> None:
    translational_acceleration_W = np.zeros((3, 1), dtype=np.float64)
    R_WB = np.eye(3, dtype=np.float64)
    gravity_acceleration = 9.81

    with pytest.raises(
        ValueError,
        match=r"translational_acceleration_W must have shape \(3,\)",
    ):
        ideal_accelerometer_specific_force_body(
            translational_acceleration_W,
            R_WB,
            gravity_acceleration,
        )


def test_ideal_accelerometer_specific_force_body_rejects_invalid_rotation_shape() -> None:
    translational_acceleration_W = np.zeros(3, dtype=np.float64)
    R_WB = np.zeros((3, 1), dtype=np.float64)
    gravity_acceleration = 9.81

    with pytest.raises(
        ValueError,
        match=r"R_WB must have shape \(3, 3\)",
    ):
        ideal_accelerometer_specific_force_body(
            translational_acceleration_W,
            R_WB,
            gravity_acceleration,
        )


def test_ideal_accelerometer_specific_force_body_rejects_nonfinite_acceleration() -> None:
    translational_acceleration_W = np.array(
        [0.0, np.nan, 0.0],
        dtype=np.float64,
    )
    R_WB = np.eye(3, dtype=np.float64)
    gravity_acceleration = 9.81

    with pytest.raises(
        ValueError,
        match="translational_acceleration_W must contain only finite values",
    ):
        ideal_accelerometer_specific_force_body(
            translational_acceleration_W,
            R_WB,
            gravity_acceleration,
        )


def test_ideal_accelerometer_specific_force_body_rejects_nonfinite_rotation() -> None:
    translational_acceleration_W = np.zeros(3, dtype=np.float64)
    R_WB = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.nan, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    gravity_acceleration = 9.81

    with pytest.raises(
        ValueError,
        match="R_WB must contain only finite values",
    ):
        ideal_accelerometer_specific_force_body(
            translational_acceleration_W,
            R_WB,
            gravity_acceleration,
        )


@pytest.mark.parametrize(
    "gravity_acceleration",
    [
        np.nan,
        np.inf,
        -np.inf,
        -1.0,
    ],
)
def test_ideal_accelerometer_specific_force_body_rejects_invalid_gravity(
    gravity_acceleration: float,
) -> None:
    translational_acceleration_W = np.zeros(3, dtype=np.float64)
    R_WB = np.eye(3, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="gravity_acceleration must be finite and nonnegative",
    ):
        ideal_accelerometer_specific_force_body(
            translational_acceleration_W,
            R_WB,
            gravity_acceleration,
        )


def test_ideal_accelerometer_specific_force_body_accepts_zero_gravity() -> None:
    translational_acceleration_W = np.array(
        [1.0, -2.0, 3.0],
        dtype=np.float64,
    )
    R_WB = np.eye(3, dtype=np.float64)
    gravity_acceleration = 0.0

    specific_force_B = ideal_accelerometer_specific_force_body(
        translational_acceleration_W,
        R_WB,
        gravity_acceleration,
    )

    expected_specific_force_B = np.array(
        [1.0, -2.0, 3.0],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        specific_force_B,
        expected_specific_force_B,
        rtol=0.0,
        atol=1e-12,
    )
