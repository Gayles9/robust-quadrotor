import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.metrics import (
    euclidean_vector_trajectory_errors,
    quaternion_attitude_trajectory_errors_body_to_world,
)


def test_euclidean_vector_trajectory_errors_returns_rowwise_norms() -> None:
    vector_history = np.array(
        [
            [1.0, 2.0, 3.0],
            [4.0, 6.0, 8.0],
        ],
        dtype=np.float64,
    )
    reference_vector_history = np.array(
        [
            [1.0, 2.0, 3.0],
            [1.0, 2.0, 8.0],
        ],
        dtype=np.float64,
    )

    trajectory_errors = euclidean_vector_trajectory_errors(
        vector_history,
        reference_vector_history,
    )

    expected_trajectory_errors = np.array(
        [0.0, 5.0],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        trajectory_errors,
        expected_trajectory_errors,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    (
        "vector_history",
        "reference_vector_history",
        "expected_message",
    ),
    [
        (
            np.zeros(3, dtype=np.float64),
            np.zeros((1, 3), dtype=np.float64),
            "vector_history must be two-dimensional",
        ),
        (
            np.zeros((1, 3), dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            "reference_vector_history must be two-dimensional",
        ),
        (
            np.zeros((1, 3), dtype=np.float64),
            np.zeros((2, 3), dtype=np.float64),
            "vector_history and reference_vector_history must have matching shapes",
        ),
        (
            np.zeros((0, 3), dtype=np.float64),
            np.zeros((0, 3), dtype=np.float64),
            "vector histories must contain at least one sample",
        ),
        (
            np.zeros((1, 0), dtype=np.float64),
            np.zeros((1, 0), dtype=np.float64),
            "vector histories must contain at least one component",
        ),
        (
            np.array([[np.nan, 0.0, 0.0]], dtype=np.float64),
            np.zeros((1, 3), dtype=np.float64),
            "vector_history must contain only finite values",
        ),
        (
            np.zeros((1, 3), dtype=np.float64),
            np.array([[np.inf, 0.0, 0.0]], dtype=np.float64),
            "reference_vector_history must contain only finite values",
        ),
    ],
)
def test_euclidean_vector_trajectory_errors_rejects_invalid_inputs(
    vector_history: NDArray[np.float64],
    reference_vector_history: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        euclidean_vector_trajectory_errors(
            vector_history,
            reference_vector_history,
        )


def test_quaternion_attitude_trajectory_errors_body_to_world_are_sign_invariant() -> None:
    half_sqrt_two = np.sqrt(0.5)
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [-half_sqrt_two, 0.0, 0.0, -half_sqrt_two],
        ],
        dtype=np.float64,
    )
    reference_q_history_WB = np.array(
        [
            [-1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )

    attitude_errors = quaternion_attitude_trajectory_errors_body_to_world(
        q_history_WB,
        reference_q_history_WB,
    )

    expected_attitude_errors = np.array(
        [0.0, np.pi / 2.0],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        attitude_errors,
        expected_attitude_errors,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    (
        "q_history_WB",
        "reference_q_history_WB",
        "expected_message",
    ),
    [
        (
            np.zeros(4, dtype=np.float64),
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
            r"q_history_WB must have shape \(N, 4\), got \(4,\)",
        ),
        (
            np.zeros((2, 3), dtype=np.float64),
            np.array(
                [[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
                dtype=np.float64,
            ),
            r"q_history_WB must have shape \(N, 4\), got \(2, 3\)",
        ),
        (
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
            np.zeros(4, dtype=np.float64),
            r"reference_q_history_WB must have shape \(N, 4\), got \(4,\)",
        ),
        (
            np.array(
                [[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
                dtype=np.float64,
            ),
            np.zeros((2, 3), dtype=np.float64),
            r"reference_q_history_WB must have shape \(N, 4\), got \(2, 3\)",
        ),
        (
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
            np.array(
                [[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
                dtype=np.float64,
            ),
            "q_history_WB and reference_q_history_WB must have matching shapes",
        ),
        (
            np.zeros((0, 4), dtype=np.float64),
            np.zeros((0, 4), dtype=np.float64),
            "quaternion histories must contain at least one sample",
        ),
    ],
)
def test_quaternion_attitude_trajectory_errors_rejects_invalid_shapes(
    q_history_WB: NDArray[np.float64],
    reference_q_history_WB: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        quaternion_attitude_trajectory_errors_body_to_world(
            q_history_WB,
            reference_q_history_WB,
        )


@pytest.mark.parametrize(
    (
        "q_history_WB",
        "reference_q_history_WB",
        "expected_message",
    ),
    [
        (
            np.array([[np.nan, 0.0, 0.0, 0.0]], dtype=np.float64),
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
            "q_history_WB must contain only finite values",
        ),
        (
            np.array([[np.inf, 0.0, 0.0, 0.0]], dtype=np.float64),
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
            "q_history_WB must contain only finite values",
        ),
        (
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
            np.array([[np.nan, 0.0, 0.0, 0.0]], dtype=np.float64),
            "reference_q_history_WB must contain only finite values",
        ),
        (
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
            np.array([[np.inf, 0.0, 0.0, 0.0]], dtype=np.float64),
            "reference_q_history_WB must contain only finite values",
        ),
    ],
)
def test_quaternion_attitude_trajectory_errors_rejects_nonfinite_values(
    q_history_WB: NDArray[np.float64],
    reference_q_history_WB: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        quaternion_attitude_trajectory_errors_body_to_world(
            q_history_WB,
            reference_q_history_WB,
        )


@pytest.mark.parametrize(
    (
        "q_history_WB",
        "reference_q_history_WB",
        "expected_message",
    ),
    [
        (
            np.array(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [2.0, 0.0, 0.0, 0.0],
                ],
                dtype=np.float64,
            ),
            np.array(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                ],
                dtype=np.float64,
            ),
            "q_history_WB must contain only unit quaternions",
        ),
        (
            np.array(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                ],
                dtype=np.float64,
            ),
            np.array(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                ],
                dtype=np.float64,
            ),
            "reference_q_history_WB must contain only unit quaternions",
        ),
    ],
)
def test_quaternion_attitude_trajectory_errors_rejects_nonunit_quaternions(
    q_history_WB: NDArray[np.float64],
    reference_q_history_WB: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        quaternion_attitude_trajectory_errors_body_to_world(
            q_history_WB,
            reference_q_history_WB,
        )
