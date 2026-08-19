import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.dynamics import translational_acceleration_world_from_body_force


def test_translational_acceleration_world_rotates_body_force_and_adds_gravity() -> None:
    force_B = np.array([0.0, 0.0, -2.0], dtype=np.float64)
    R_WB = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    mass = 2.0
    gravity_acceleration = 9.81

    acceleration_W = translational_acceleration_world_from_body_force(
        force_B,
        R_WB,
        mass,
        gravity_acceleration,
    )

    expected_acceleration_W = np.array([0.0, 1.0, 9.81], dtype=np.float64)
    np.testing.assert_allclose(
        acceleration_W,
        expected_acceleration_W,
        atol=1e-12,
    )


def test_translational_acceleration_world_rejects_invalid_force_shape() -> None:
    force_B = np.zeros(2, dtype=np.float64)
    R_WB = np.eye(3, dtype=np.float64)

    with pytest.raises(ValueError, match=r"force_B must have shape \(3,\)"):
        translational_acceleration_world_from_body_force(
            force_B,
            R_WB,
            mass=1.0,
            gravity_acceleration=9.81,
        )


def test_translational_acceleration_world_rejects_invalid_rotation_shape() -> None:
    force_B = np.zeros(3, dtype=np.float64)
    R_WB = np.zeros((2, 2), dtype=np.float64)

    with pytest.raises(ValueError, match=r"R_WB must have shape \(3, 3\)"):
        translational_acceleration_world_from_body_force(
            force_B,
            R_WB,
            mass=1.0,
            gravity_acceleration=9.81,
        )


@pytest.mark.parametrize(
    ("force_B", "R_WB", "expected_message"),
    [
        (
            np.array([np.nan, 0.0, 0.0], dtype=np.float64),
            np.eye(3, dtype=np.float64),
            "force_B must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, np.inf, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            ),
            "R_WB must contain only finite values",
        ),
    ],
)
def test_translational_acceleration_world_rejects_nonfinite_array_inputs(
    force_B: NDArray[np.float64],
    R_WB: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        translational_acceleration_world_from_body_force(
            force_B,
            R_WB,
            mass=1.0,
            gravity_acceleration=9.81,
        )


@pytest.mark.parametrize(
    ("mass", "expected_message"),
    [
        (np.nan, "mass must be finite"),
        (np.inf, "mass must be finite"),
        (0.0, "mass must be positive"),
        (-1.0, "mass must be positive"),
    ],
)
def test_translational_acceleration_world_rejects_invalid_mass(
    mass: float,
    expected_message: str,
) -> None:
    force_B = np.zeros(3, dtype=np.float64)
    R_WB = np.eye(3, dtype=np.float64)

    with pytest.raises(ValueError, match=expected_message):
        translational_acceleration_world_from_body_force(
            force_B,
            R_WB,
            mass=mass,
            gravity_acceleration=9.81,
        )


@pytest.mark.parametrize(
    ("gravity_acceleration", "expected_message"),
    [
        (np.nan, "gravity_acceleration must be finite"),
        (np.inf, "gravity_acceleration must be finite"),
        (0.0, "gravity_acceleration must be positive"),
        (-9.81, "gravity_acceleration must be positive"),
    ],
)
def test_translational_acceleration_world_rejects_invalid_gravity(
    gravity_acceleration: float,
    expected_message: str,
) -> None:
    force_B = np.zeros(3, dtype=np.float64)
    R_WB = np.eye(3, dtype=np.float64)

    with pytest.raises(ValueError, match=expected_message):
        translational_acceleration_world_from_body_force(
            force_B,
            R_WB,
            mass=1.0,
            gravity_acceleration=gravity_acceleration,
        )
