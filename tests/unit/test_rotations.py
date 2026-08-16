import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.rotations import (
    normalize_quaternion_body_to_world,
    rotation_matrix_body_to_world,
    skew_symmetric,
)


def test_skew_symmetric_represents_cross_product() -> None:
    vector_a = np.array([1.0, -2.0, 3.0], dtype=np.float64)

    skew_matrix = skew_symmetric(vector_a)

    expected = np.array(
        [
            [0.0, -3.0, -2.0],
            [3.0, 0.0, -1.0],
            [2.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    np.testing.assert_allclose(skew_matrix, expected)

    vector_b = np.array([4.0, 5.0, -6.0], dtype=np.float64)

    np.testing.assert_allclose(skew_matrix @ vector_b, np.cross(vector_a, vector_b))


def test_normalize_quaternion_body_to_world_returns_unit_quaternion() -> None:
    q_WB = np.array([2.0, -2.0, 1.0, -1.0], dtype=np.float64)

    normalized_q_WB = normalize_quaternion_body_to_world(q_WB)

    expected_q_WB = q_WB / np.sqrt(10.0)
    np.testing.assert_allclose(normalized_q_WB, expected_q_WB, atol=1e-12)
    assert np.linalg.norm(normalized_q_WB) == pytest.approx(1.0, abs=1e-12)
    np.testing.assert_array_equal(
        q_WB,
        np.array([2.0, -2.0, 1.0, -1.0], dtype=np.float64),
    )


@pytest.mark.parametrize(
    "q_WB",
    [
        np.zeros(3, dtype=np.float64),
        np.zeros((4, 1), dtype=np.float64),
        np.zeros(5, dtype=np.float64),
    ],
)
def test_normalize_quaternion_body_to_world_rejects_invalid_shape(
    q_WB: NDArray[np.float64],
) -> None:
    with pytest.raises(ValueError, match=r"q_WB must have shape \(4,\)"):
        normalize_quaternion_body_to_world(q_WB)


def test_normalize_quaternion_body_to_world_rejects_zero_norm() -> None:
    q_WB = np.zeros(4, dtype=np.float64)

    with pytest.raises(ValueError, match="q_WB must have nonzero norm"):
        normalize_quaternion_body_to_world(q_WB)


@pytest.mark.parametrize(
    "q_WB",
    [
        np.array([np.nan, 0.0, 0.0, 0.0], dtype=np.float64),
        np.array([np.inf, 0.0, 0.0, 0.0], dtype=np.float64),
        np.array([-np.inf, 0.0, 0.0, 0.0], dtype=np.float64),
    ],
)
def test_normalize_quaternion_body_to_world_rejects_non_finite_values(
    q_WB: NDArray[np.float64],
) -> None:
    with pytest.raises(ValueError, match="q_WB must contain only finite values"):
        normalize_quaternion_body_to_world(q_WB)


def test_rotation_matrix_body_to_world_maps_forward_east_for_positive_quarter_turn_yaw() -> None:
    half_sqrt_two = np.sqrt(0.5)
    q_WB = np.array(
        [half_sqrt_two, 0.0, 0.0, half_sqrt_two],
        dtype=np.float64,
    )

    R_WB = rotation_matrix_body_to_world(q_WB)

    expected_R_WB = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    np.testing.assert_allclose(R_WB, expected_R_WB, atol=1e-12)

    forward_B = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    expected_forward_W = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    np.testing.assert_allclose(R_WB @ forward_B, expected_forward_W, atol=1e-12)


def test_rotation_matrix_body_to_world_satisfies_general_rotation_invariants() -> None:
    q_WB = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float64)

    R_WB = rotation_matrix_body_to_world(q_WB)

    expected_R_WB = np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    np.testing.assert_allclose(R_WB, expected_R_WB, atol=1e-12)
    np.testing.assert_allclose(
        R_WB.T @ R_WB,
        np.eye(3, dtype=np.float64),
        atol=1e-12,
    )
    assert np.linalg.det(R_WB) == pytest.approx(1.0, abs=1e-12)
    np.testing.assert_allclose(
        rotation_matrix_body_to_world(-q_WB),
        R_WB,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    "q_WB",
    [
        np.zeros(3, dtype=np.float64),
        np.zeros((4, 1), dtype=np.float64),
        np.zeros(5, dtype=np.float64),
    ],
)
def test_rotation_matrix_body_to_world_rejects_invalid_quaternion_shape(
    q_WB: NDArray[np.float64],
) -> None:
    with pytest.raises(ValueError, match=r"q_WB must have shape \(4,\)"):
        rotation_matrix_body_to_world(q_WB)


@pytest.mark.parametrize(
    "q_WB",
    [
        np.zeros(4, dtype=np.float64),
        np.array([2.0, 0.0, 0.0, 0.0], dtype=np.float64),
    ],
)
def test_rotation_matrix_body_to_world_rejects_non_unit_quaternion(
    q_WB: NDArray[np.float64],
) -> None:
    with pytest.raises(ValueError, match="q_WB must have unit norm"):
        rotation_matrix_body_to_world(q_WB)


@pytest.mark.parametrize(
    "vector",
    [
        np.zeros(2, dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        np.zeros(4, dtype=np.float64),
    ],
)
def test_skew_symmetric_rejects_invalid_shape(vector: NDArray[np.float64]) -> None:
    with pytest.raises(ValueError, match=r"vector must have shape \(3,\)"):
        skew_symmetric(vector)
