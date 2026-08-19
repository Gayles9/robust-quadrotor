import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.dynamics import (
    angular_acceleration_body_from_moment,
    translational_acceleration_world_from_body_force,
)


def test_angular_acceleration_body_includes_gyroscopic_coupling() -> None:
    moment_B = np.array([4.0, 5.0, 6.0], dtype=np.float64)
    omega_B = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    inertia_B = np.diag(
        np.array([2.0, 3.0, 4.0], dtype=np.float64),
    )

    angular_acceleration_B = angular_acceleration_body_from_moment(
        moment_B,
        omega_B,
        inertia_B,
    )

    expected_angular_acceleration_B = np.array(
        [-1.0, 11.0 / 3.0, 1.0],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        angular_acceleration_B,
        expected_angular_acceleration_B,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    ("moment_B", "omega_B", "inertia_B", "expected_message"),
    [
        (
            np.zeros(2, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"moment_B must have shape \(3,\)",
        ),
        (
            np.zeros((3, 1), dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"moment_B must have shape \(3,\)",
        ),
        (
            np.zeros(4, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"moment_B must have shape \(3,\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(2, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"omega_B must have shape \(3,\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"omega_B must have shape \(3,\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(4, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"omega_B must have shape \(3,\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.zeros((2, 2), dtype=np.float64),
            r"inertia_B must have shape \(3, 3\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.zeros((3, 2), dtype=np.float64),
            r"inertia_B must have shape \(3, 3\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.zeros((3, 3, 1), dtype=np.float64),
            r"inertia_B must have shape \(3, 3\)",
        ),
    ],
)
def test_angular_acceleration_body_rejects_invalid_shapes(
    moment_B: NDArray[np.float64],
    omega_B: NDArray[np.float64],
    inertia_B: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        angular_acceleration_body_from_moment(
            moment_B,
            omega_B,
            inertia_B,
        )


@pytest.mark.parametrize(
    ("moment_B", "omega_B", "inertia_B", "expected_message"),
    [
        (
            np.array([np.nan, 0.0, 0.0], dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            "moment_B must contain only finite values",
        ),
        (
            np.array([np.inf, 0.0, 0.0], dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            "moment_B must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.array([np.nan, 0.0, 0.0], dtype=np.float64),
            np.eye(3, dtype=np.float64),
            "omega_B must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.array([np.inf, 0.0, 0.0], dtype=np.float64),
            np.eye(3, dtype=np.float64),
            "omega_B must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.array(
                [
                    [np.nan, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            ),
            "inertia_B must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.array(
                [
                    [np.inf, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            ),
            "inertia_B must contain only finite values",
        ),
    ],
)
def test_angular_acceleration_body_rejects_nonfinite_inputs(
    moment_B: NDArray[np.float64],
    omega_B: NDArray[np.float64],
    inertia_B: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        angular_acceleration_body_from_moment(
            moment_B,
            omega_B,
            inertia_B,
        )


def test_angular_acceleration_body_rejects_nonsymmetric_inertia() -> None:
    moment_B = np.zeros(3, dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)
    inertia_B = np.array(
        [
            [1.0, 0.1, 0.0],
            [0.0, 2.0, 0.0],
            [0.0, 0.0, 3.0],
        ],
        dtype=np.float64,
    )

    with pytest.raises(ValueError, match="inertia_B must be symmetric"):
        angular_acceleration_body_from_moment(
            moment_B,
            omega_B,
            inertia_B,
        )


@pytest.mark.parametrize(
    "inertia_B",
    [
        np.zeros((3, 3), dtype=np.float64),
        np.diag(
            np.array([1.0, 1.0, 0.0], dtype=np.float64),
        ),
        np.diag(
            np.array([1.0, -1.0, 1.0], dtype=np.float64),
        ),
    ],
)
def test_angular_acceleration_body_rejects_non_positive_definite_inertia(
    inertia_B: NDArray[np.float64],
) -> None:
    moment_B = np.zeros(3, dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="inertia_B must be positive definite",
    ):
        angular_acceleration_body_from_moment(
            moment_B,
            omega_B,
            inertia_B,
        )


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
