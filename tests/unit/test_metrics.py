import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.metrics import (
    euclidean_vector_trajectory_errors,
    observed_convergence_orders,
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


def test_observed_convergence_orders_returns_known_power_law_order() -> None:
    time_steps = np.array(
        [0.1, 0.05, 0.025],
        dtype=np.float64,
    )
    errors = np.array(
        [0.04, 0.01, 0.0025],
        dtype=np.float64,
    )

    orders = observed_convergence_orders(
        time_steps,
        errors,
        error_floor=0.0,
    )

    expected_orders = np.array(
        [2.0, 2.0],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        orders,
        expected_orders,
        atol=1e-12,
    )


def test_observed_convergence_orders_handles_error_floor_and_negative_orders() -> None:
    time_steps = np.array(
        [0.2, 0.1, 0.05, 0.025, 0.0125],
        dtype=np.float64,
    )
    errors = np.array(
        [0.04, 0.01, 1e-14, 0.02, 0.04],
        dtype=np.float64,
    )

    orders = observed_convergence_orders(
        time_steps,
        errors,
        error_floor=1e-12,
    )

    expected_orders = np.array(
        [2.0, np.nan, np.nan, -1.0],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        orders,
        expected_orders,
        atol=1e-12,
        equal_nan=True,
    )


def test_observed_convergence_orders_handles_extreme_finite_scales() -> None:
    largest_float = np.finfo(np.float64).max
    smallest_positive_float = np.nextafter(
        np.float64(0.0),
        np.float64(1.0),
    )

    time_steps = np.array(
        [largest_float, smallest_positive_float],
        dtype=np.float64,
    )
    errors = np.array(
        [largest_float, smallest_positive_float],
        dtype=np.float64,
    )

    expected_orders = np.array(
        [1.0],
        dtype=np.float64,
    )

    with np.errstate(
        over="raise",
        divide="raise",
        invalid="raise",
    ):
        orders = observed_convergence_orders(
            time_steps,
            errors,
            error_floor=0.0,
        )

    np.testing.assert_allclose(
        orders,
        expected_orders,
        atol=1e-12,
    )


def test_observed_convergence_orders_handles_adjacent_large_time_steps() -> None:
    largest_float = np.finfo(np.float64).max
    next_smaller_float = np.nextafter(
        largest_float,
        np.float64(0.0),
    )
    assert largest_float > next_smaller_float

    time_steps = np.array(
        [largest_float, next_smaller_float],
        dtype=np.float64,
    )
    errors = np.array(
        [2.0, 1.0],
        dtype=np.float64,
    )

    relative_time_step_change = (largest_float - next_smaller_float) / next_smaller_float
    expected_orders = np.array(
        [
            np.log(2.0) / np.log1p(relative_time_step_change),
        ],
        dtype=np.float64,
    )

    with np.errstate(
        over="raise",
        divide="raise",
        invalid="raise",
    ):
        orders = observed_convergence_orders(
            time_steps,
            errors,
            error_floor=0.0,
        )

    np.testing.assert_allclose(
        orders,
        expected_orders,
        rtol=1e-12,
        atol=0.0,
    )


@pytest.mark.parametrize(
    (
        "time_steps",
        "errors",
        "error_floor",
        "expected_message",
    ),
    [
        (
            np.zeros((2, 1), dtype=np.float64),
            np.array([1.0, 0.5], dtype=np.float64),
            0.0,
            "time_steps must be one-dimensional",
        ),
        (
            np.array([0.1, 0.05], dtype=np.float64),
            np.zeros((2, 1), dtype=np.float64),
            0.0,
            "errors must be one-dimensional",
        ),
        (
            np.array([0.1, 0.05], dtype=np.float64),
            np.array([1.0, 0.5, 0.25], dtype=np.float64),
            0.0,
            "time_steps and errors must have matching shapes",
        ),
        (
            np.array([0.1], dtype=np.float64),
            np.array([1.0], dtype=np.float64),
            0.0,
            "at least two time steps and errors are required",
        ),
        (
            np.array([np.nan, 0.05], dtype=np.float64),
            np.array([1.0, 0.5], dtype=np.float64),
            0.0,
            "time_steps must contain only finite values",
        ),
        (
            np.array([0.1, 0.0], dtype=np.float64),
            np.array([1.0, 0.5], dtype=np.float64),
            0.0,
            "time_steps must be positive",
        ),
        (
            np.array([0.1, 0.1], dtype=np.float64),
            np.array([1.0, 0.5], dtype=np.float64),
            0.0,
            "time_steps must be strictly decreasing",
        ),
        (
            np.array([0.1, 0.05], dtype=np.float64),
            np.array([1.0, np.nan], dtype=np.float64),
            0.0,
            "errors must contain only finite values",
        ),
        (
            np.array([0.1, 0.05], dtype=np.float64),
            np.array([1.0, -0.5], dtype=np.float64),
            0.0,
            "errors must be nonnegative",
        ),
        (
            np.array([0.1, 0.05], dtype=np.float64),
            np.array([1.0, 0.5], dtype=np.float64),
            np.nan,
            "error_floor must be finite",
        ),
        (
            np.array([0.1, 0.05], dtype=np.float64),
            np.array([1.0, 0.5], dtype=np.float64),
            -0.1,
            "error_floor must be nonnegative",
        ),
    ],
)
def test_observed_convergence_orders_rejects_invalid_inputs(
    time_steps: NDArray[np.float64],
    errors: NDArray[np.float64],
    error_floor: float,
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        observed_convergence_orders(
            time_steps,
            errors,
            error_floor=error_floor,
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
