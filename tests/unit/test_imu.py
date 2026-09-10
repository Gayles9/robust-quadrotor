from unittest.mock import Mock

import numpy as np
import pytest

from quadrotor_math.imu import (
    accelerometer_bias_random_walk_step_body,
    accelerometer_specific_force_measurement_body,
    accelerometer_specific_force_with_bias_body,
    gyroscope_angular_velocity_measurement_body,
    gyroscope_angular_velocity_with_bias_body,
    gyroscope_bias_random_walk_step_body,
    ideal_accelerometer_specific_force_body,
    ideal_gyroscope_angular_velocity_body,
)
from quadrotor_math.randomness import create_rng


def test_accelerometer_bias_random_walk_step_body_matches_seeded_update_equation() -> None:
    current_accelerometer_bias_B = np.array(
        [0.1, -0.2, 0.3],
        dtype=np.float64,
    )
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, 0.04, 0.06],
        dtype=np.float64,
    )
    time_step = 0.25
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    standard_normal_sample_B = reference_rng.standard_normal(3)
    expected_next_accelerometer_bias_B = (
        current_accelerometer_bias_B
        + accelerometer_bias_random_walk_density_B * np.sqrt(time_step) * standard_normal_sample_B
    )

    next_accelerometer_bias_B = accelerometer_bias_random_walk_step_body(
        current_accelerometer_bias_B,
        accelerometer_bias_random_walk_density_B,
        time_step,
        measurement_rng,
    )

    np.testing.assert_allclose(
        next_accelerometer_bias_B,
        expected_next_accelerometer_bias_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_accelerometer_bias_random_walk_step_body_draws_one_standard_normal_vector() -> None:
    current_accelerometer_bias_B = np.array(
        [0.1, -0.2, 0.3],
        dtype=np.float64,
    )
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, 0.04, 0.06],
        dtype=np.float64,
    )
    rng = Mock(spec=np.random.Generator)
    rng.standard_normal.return_value = np.array(
        [0.5, -1.0, 1.5],
        dtype=np.float64,
    )

    accelerometer_bias_random_walk_step_body(
        current_accelerometer_bias_B,
        accelerometer_bias_random_walk_density_B,
        0.25,
        rng,
    )

    rng.standard_normal.assert_called_once_with(3)


def test_accelerometer_bias_random_walk_step_body_zero_density_still_consumes_random_draw() -> None:
    current_accelerometer_bias_B = np.array(
        [0.1, -0.2, 0.3],
        dtype=np.float64,
    )
    accelerometer_bias_random_walk_density_B = np.zeros(3, dtype=np.float64)
    seed = 12345
    bias_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    next_accelerometer_bias_B = accelerometer_bias_random_walk_step_body(
        current_accelerometer_bias_B,
        accelerometer_bias_random_walk_density_B,
        0.25,
        bias_rng,
    )

    np.testing.assert_array_equal(
        next_accelerometer_bias_B,
        current_accelerometer_bias_B,
    )

    reference_rng.standard_normal(3)
    expected_next_standard_normal_sample_B = reference_rng.standard_normal(3)
    actual_next_standard_normal_sample_B = bias_rng.standard_normal(3)
    np.testing.assert_array_equal(
        actual_next_standard_normal_sample_B,
        expected_next_standard_normal_sample_B,
    )


def test_accelerometer_bias_random_walk_step_body_replays_same_seed_trajectory() -> None:
    initial_accelerometer_bias_B = np.array(
        [0.1, -0.2, 0.3],
        dtype=np.float64,
    )
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, 0.04, 0.06],
        dtype=np.float64,
    )
    time_step = 0.25
    seed = 12345
    first_rng = create_rng(seed)
    second_rng = create_rng(seed)
    first_trajectory_B = np.empty((2, 3), dtype=np.float64)
    second_trajectory_B = np.empty((2, 3), dtype=np.float64)
    first_bias_B = initial_accelerometer_bias_B.copy()
    second_bias_B = initial_accelerometer_bias_B.copy()

    for step_index in range(2):
        first_bias_B = accelerometer_bias_random_walk_step_body(
            first_bias_B,
            accelerometer_bias_random_walk_density_B,
            time_step,
            first_rng,
        )
        second_bias_B = accelerometer_bias_random_walk_step_body(
            second_bias_B,
            accelerometer_bias_random_walk_density_B,
            time_step,
            second_rng,
        )
        first_trajectory_B[step_index] = first_bias_B
        second_trajectory_B[step_index] = second_bias_B

    np.testing.assert_array_equal(
        first_trajectory_B,
        second_trajectory_B,
    )


def test_accelerometer_bias_random_walk_step_body_advances_rng_between_steps() -> None:
    initial_accelerometer_bias_B = np.array(
        [0.1, -0.2, 0.3],
        dtype=np.float64,
    )
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, 0.04, 0.06],
        dtype=np.float64,
    )
    time_step = 0.25
    seed = 12345
    bias_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    first_accelerometer_bias_B = accelerometer_bias_random_walk_step_body(
        initial_accelerometer_bias_B,
        accelerometer_bias_random_walk_density_B,
        time_step,
        bias_rng,
    )
    second_accelerometer_bias_B = accelerometer_bias_random_walk_step_body(
        first_accelerometer_bias_B,
        accelerometer_bias_random_walk_density_B,
        time_step,
        bias_rng,
    )

    first_standard_normal_sample_B = reference_rng.standard_normal(3)
    second_standard_normal_sample_B = reference_rng.standard_normal(3)
    increment_standard_deviation_B = accelerometer_bias_random_walk_density_B * np.sqrt(time_step)
    expected_first_accelerometer_bias_B = (
        initial_accelerometer_bias_B
        + increment_standard_deviation_B * first_standard_normal_sample_B
    )
    expected_second_accelerometer_bias_B = (
        expected_first_accelerometer_bias_B
        + increment_standard_deviation_B * second_standard_normal_sample_B
    )

    np.testing.assert_allclose(
        first_accelerometer_bias_B,
        expected_first_accelerometer_bias_B,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        second_accelerometer_bias_B,
        expected_second_accelerometer_bias_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_accelerometer_bias_random_walk_step_body_rejected_input_does_not_advance_rng() -> None:
    current_accelerometer_bias_B = np.zeros(3, dtype=np.float64)
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, 0.04, 0.06],
        dtype=np.float64,
    )
    seed = 12345
    bias_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    with pytest.raises(
        ValueError,
        match="time_step must be positive",
    ):
        accelerometer_bias_random_walk_step_body(
            current_accelerometer_bias_B,
            accelerometer_bias_random_walk_density_B,
            0.0,
            bias_rng,
        )

    next_bias_sample_B = bias_rng.standard_normal(3)
    first_reference_sample_B = reference_rng.standard_normal(3)
    np.testing.assert_array_equal(
        next_bias_sample_B,
        first_reference_sample_B,
    )


def test_accelerometer_bias_random_walk_step_body_scales_with_square_root_time() -> None:
    current_accelerometer_bias_B = np.array(
        [0.1, -0.2, 0.3],
        dtype=np.float64,
    )
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, 0.04, 0.06],
        dtype=np.float64,
    )
    seed = 12345
    one_second_rng = create_rng(seed)
    four_second_rng = create_rng(seed)

    one_second_bias_B = accelerometer_bias_random_walk_step_body(
        current_accelerometer_bias_B,
        accelerometer_bias_random_walk_density_B,
        1.0,
        one_second_rng,
    )
    four_second_bias_B = accelerometer_bias_random_walk_step_body(
        current_accelerometer_bias_B,
        accelerometer_bias_random_walk_density_B,
        4.0,
        four_second_rng,
    )

    one_second_increment_B = one_second_bias_B - current_accelerometer_bias_B
    four_second_increment_B = four_second_bias_B - current_accelerometer_bias_B
    expected_four_second_increment_B = 2.0 * one_second_increment_B
    np.testing.assert_allclose(
        four_second_increment_B,
        expected_four_second_increment_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_accelerometer_bias_random_walk_step_body_matches_increment_statistics() -> None:
    current_accelerometer_bias_B = np.array(
        [0.1, -0.2, 0.3],
        dtype=np.float64,
    )
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, 0.04, 0.06],
        dtype=np.float64,
    )
    time_step = 0.25
    sample_count = 20_000
    rng = create_rng(12345)

    next_biases_B = np.array(
        [
            accelerometer_bias_random_walk_step_body(
                current_accelerometer_bias_B,
                accelerometer_bias_random_walk_density_B,
                time_step,
                rng,
            )
            for _ in range(sample_count)
        ],
        dtype=np.float64,
    )

    increments_B = next_biases_B - current_accelerometer_bias_B
    empirical_mean_B = np.mean(increments_B, axis=0)
    empirical_standard_deviation_B = np.std(
        increments_B,
        axis=0,
        ddof=1,
    )
    expected_standard_deviation_B = accelerometer_bias_random_walk_density_B * np.sqrt(time_step)
    mean_error_in_standard_errors_B = (
        np.abs(empirical_mean_B) * np.sqrt(sample_count) / expected_standard_deviation_B
    )
    standard_deviation_error_in_standard_errors_B = (
        np.abs(empirical_standard_deviation_B - expected_standard_deviation_B)
        * np.sqrt(2.0 * (sample_count - 1))
        / expected_standard_deviation_B
    )
    five_standard_errors_B = np.full(
        3,
        5.0,
        dtype=np.float64,
    )

    np.testing.assert_array_less(
        mean_error_in_standard_errors_B,
        five_standard_errors_B,
    )
    np.testing.assert_array_less(
        standard_deviation_error_in_standard_errors_B,
        five_standard_errors_B,
    )


def test_accelerometer_bias_random_walk_step_body_returns_independent_state() -> None:
    current_accelerometer_bias_B = np.array(
        [0.1, -0.2, 0.3],
        dtype=np.float64,
    )
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, 0.04, 0.06],
        dtype=np.float64,
    )
    original_accelerometer_bias_B = current_accelerometer_bias_B.copy()
    original_accelerometer_bias_random_walk_density_B = (
        accelerometer_bias_random_walk_density_B.copy()
    )
    rng = create_rng(12345)

    next_accelerometer_bias_B = accelerometer_bias_random_walk_step_body(
        current_accelerometer_bias_B,
        accelerometer_bias_random_walk_density_B,
        0.25,
        rng,
    )

    assert not np.shares_memory(
        next_accelerometer_bias_B,
        current_accelerometer_bias_B,
    )
    assert not np.shares_memory(
        next_accelerometer_bias_B,
        accelerometer_bias_random_walk_density_B,
    )
    np.testing.assert_array_equal(
        current_accelerometer_bias_B,
        original_accelerometer_bias_B,
    )
    np.testing.assert_array_equal(
        accelerometer_bias_random_walk_density_B,
        original_accelerometer_bias_random_walk_density_B,
    )


def test_accelerometer_bias_random_walk_step_body_rejects_invalid_current_bias_shape() -> None:
    current_accelerometer_bias_B = np.zeros((3, 1), dtype=np.float64)
    accelerometer_bias_random_walk_density_B = np.zeros(3, dtype=np.float64)
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match=r"current_accelerometer_bias_B must have shape \(3,\)",
    ):
        accelerometer_bias_random_walk_step_body(
            current_accelerometer_bias_B,
            accelerometer_bias_random_walk_density_B,
            0.25,
            rng,
        )


def test_accelerometer_bias_random_walk_step_body_rejects_invalid_density_shape() -> None:
    current_accelerometer_bias_B = np.zeros(3, dtype=np.float64)
    accelerometer_bias_random_walk_density_B = np.zeros(
        (3, 1),
        dtype=np.float64,
    )
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match=r"accelerometer_bias_random_walk_density_B must have shape \(3,\)",
    ):
        accelerometer_bias_random_walk_step_body(
            current_accelerometer_bias_B,
            accelerometer_bias_random_walk_density_B,
            0.25,
            rng,
        )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_accelerometer_bias_random_walk_step_body_rejects_non_finite_current_bias(
    non_finite_value: float,
) -> None:
    current_accelerometer_bias_B = np.array(
        [0.1, non_finite_value, 0.3],
        dtype=np.float64,
    )
    accelerometer_bias_random_walk_density_B = np.zeros(3, dtype=np.float64)
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="current_accelerometer_bias_B must contain only finite values",
    ):
        accelerometer_bias_random_walk_step_body(
            current_accelerometer_bias_B,
            accelerometer_bias_random_walk_density_B,
            0.25,
            rng,
        )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_accelerometer_bias_random_walk_step_body_rejects_non_finite_density(
    non_finite_value: float,
) -> None:
    current_accelerometer_bias_B = np.zeros(3, dtype=np.float64)
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, non_finite_value, 0.06],
        dtype=np.float64,
    )
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="accelerometer_bias_random_walk_density_B must contain only finite values",
    ):
        accelerometer_bias_random_walk_step_body(
            current_accelerometer_bias_B,
            accelerometer_bias_random_walk_density_B,
            0.25,
            rng,
        )


def test_accelerometer_bias_random_walk_step_body_rejects_negative_density() -> None:
    current_accelerometer_bias_B = np.zeros(3, dtype=np.float64)
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, -0.04, 0.06],
        dtype=np.float64,
    )
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="accelerometer_bias_random_walk_density_B must be nonnegative",
    ):
        accelerometer_bias_random_walk_step_body(
            current_accelerometer_bias_B,
            accelerometer_bias_random_walk_density_B,
            0.25,
            rng,
        )


@pytest.mark.parametrize(
    "non_finite_time_step",
    [np.nan, np.inf, -np.inf],
)
def test_accelerometer_bias_random_walk_step_body_rejects_non_finite_time_step(
    non_finite_time_step: float,
) -> None:
    current_accelerometer_bias_B = np.zeros(3, dtype=np.float64)
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, 0.04, 0.06],
        dtype=np.float64,
    )
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="time_step must be finite",
    ):
        accelerometer_bias_random_walk_step_body(
            current_accelerometer_bias_B,
            accelerometer_bias_random_walk_density_B,
            non_finite_time_step,
            rng,
        )


@pytest.mark.parametrize(
    "non_positive_time_step",
    [0.0, -0.25],
)
def test_accelerometer_bias_random_walk_step_body_rejects_non_positive_time_step(
    non_positive_time_step: float,
) -> None:
    current_accelerometer_bias_B = np.zeros(3, dtype=np.float64)
    accelerometer_bias_random_walk_density_B = np.array(
        [0.02, 0.04, 0.06],
        dtype=np.float64,
    )
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="time_step must be positive",
    ):
        accelerometer_bias_random_walk_step_body(
            current_accelerometer_bias_B,
            accelerometer_bias_random_walk_density_B,
            non_positive_time_step,
            rng,
        )


def test_accelerometer_specific_force_measurement_body_adds_seeded_white_noise() -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, -0.02, 0.1],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, 0.02, 0.03],
        dtype=np.float64,
    )
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    specific_force_measurement_B = accelerometer_specific_force_measurement_body(
        ideal_specific_force_B,
        accelerometer_bias_B,
        noise_standard_deviation_B,
        measurement_rng,
    )

    standard_normal_sample_B = reference_rng.standard_normal(3)
    expected_specific_force_measurement_B = (
        ideal_specific_force_B
        + accelerometer_bias_B
        + noise_standard_deviation_B * standard_normal_sample_B
    )
    np.testing.assert_allclose(
        specific_force_measurement_B,
        expected_specific_force_measurement_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_accelerometer_measurement_body_zero_noise_still_consumes_random_draw() -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, -0.02, 0.1],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.zeros(3, dtype=np.float64)
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    specific_force_measurement_B = accelerometer_specific_force_measurement_body(
        ideal_specific_force_B,
        accelerometer_bias_B,
        noise_standard_deviation_B,
        measurement_rng,
    )

    expected_specific_force_measurement_B = ideal_specific_force_B + accelerometer_bias_B
    np.testing.assert_allclose(
        specific_force_measurement_B,
        expected_specific_force_measurement_B,
        rtol=0.0,
        atol=1e-12,
    )

    reference_rng.standard_normal(3)
    expected_next_standard_normal_sample_B = reference_rng.standard_normal(3)
    actual_next_standard_normal_sample_B = measurement_rng.standard_normal(3)
    np.testing.assert_array_equal(
        actual_next_standard_normal_sample_B,
        expected_next_standard_normal_sample_B,
    )


def test_accelerometer_measurement_body_replays_same_seed_sequence() -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, -0.02, 0.1],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, 0.02, 0.03],
        dtype=np.float64,
    )
    seed = 12345
    first_rng = create_rng(seed)
    second_rng = create_rng(seed)

    first_sequence_B = np.array(
        [
            accelerometer_specific_force_measurement_body(
                ideal_specific_force_B,
                accelerometer_bias_B,
                noise_standard_deviation_B,
                first_rng,
            )
            for _ in range(2)
        ],
        dtype=np.float64,
    )
    second_sequence_B = np.array(
        [
            accelerometer_specific_force_measurement_body(
                ideal_specific_force_B,
                accelerometer_bias_B,
                noise_standard_deviation_B,
                second_rng,
            )
            for _ in range(2)
        ],
        dtype=np.float64,
    )

    np.testing.assert_array_equal(
        first_sequence_B,
        second_sequence_B,
    )


def test_accelerometer_measurement_body_advances_rng_between_calls() -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, -0.02, 0.1],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, 0.02, 0.03],
        dtype=np.float64,
    )
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    first_measurement_B = accelerometer_specific_force_measurement_body(
        ideal_specific_force_B,
        accelerometer_bias_B,
        noise_standard_deviation_B,
        measurement_rng,
    )
    second_measurement_B = accelerometer_specific_force_measurement_body(
        ideal_specific_force_B,
        accelerometer_bias_B,
        noise_standard_deviation_B,
        measurement_rng,
    )

    first_standard_normal_B = reference_rng.standard_normal(3)
    second_standard_normal_B = reference_rng.standard_normal(3)
    expected_first_measurement_B = (
        ideal_specific_force_B
        + accelerometer_bias_B
        + noise_standard_deviation_B * first_standard_normal_B
    )
    expected_second_measurement_B = (
        ideal_specific_force_B
        + accelerometer_bias_B
        + noise_standard_deviation_B * second_standard_normal_B
    )
    np.testing.assert_allclose(
        first_measurement_B,
        expected_first_measurement_B,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        second_measurement_B,
        expected_second_measurement_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_accelerometer_measurement_body_matches_configured_noise_statistics() -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, -0.02, 0.1],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, 0.02, 0.03],
        dtype=np.float64,
    )
    sample_count = 20_000
    rng = create_rng(12345)

    measurements_B = np.array(
        [
            accelerometer_specific_force_measurement_body(
                ideal_specific_force_B,
                accelerometer_bias_B,
                noise_standard_deviation_B,
                rng,
            )
            for _ in range(sample_count)
        ],
        dtype=np.float64,
    )

    empirical_mean_B = np.mean(measurements_B, axis=0)
    empirical_standard_deviation_B = np.std(
        measurements_B,
        axis=0,
        ddof=1,
    )
    expected_mean_B = ideal_specific_force_B + accelerometer_bias_B
    mean_error_in_standard_errors_B = (
        np.abs(empirical_mean_B - expected_mean_B)
        * np.sqrt(sample_count)
        / noise_standard_deviation_B
    )
    standard_deviation_error_in_standard_errors_B = (
        np.abs(empirical_standard_deviation_B - noise_standard_deviation_B)
        * np.sqrt(2.0 * (sample_count - 1))
        / noise_standard_deviation_B
    )
    five_standard_errors_B = np.full(
        3,
        5.0,
        dtype=np.float64,
    )

    np.testing.assert_array_less(
        mean_error_in_standard_errors_B,
        five_standard_errors_B,
    )
    np.testing.assert_array_less(
        standard_deviation_error_in_standard_errors_B,
        five_standard_errors_B,
    )


def test_accelerometer_specific_force_measurement_body_returns_independent_measurement() -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, -0.02, 0.1],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, 0.02, 0.03],
        dtype=np.float64,
    )
    measurement_rng = create_rng(12345)

    specific_force_measurement_B = accelerometer_specific_force_measurement_body(
        ideal_specific_force_B,
        accelerometer_bias_B,
        noise_standard_deviation_B,
        measurement_rng,
    )

    assert not np.shares_memory(
        specific_force_measurement_B,
        ideal_specific_force_B,
    )
    assert not np.shares_memory(
        specific_force_measurement_B,
        accelerometer_bias_B,
    )
    assert not np.shares_memory(
        specific_force_measurement_B,
        noise_standard_deviation_B,
    )


def test_accelerometer_specific_force_measurement_body_rejects_invalid_ideal_shape() -> None:
    ideal_specific_force_B = np.zeros((3, 1), dtype=np.float64)
    accelerometer_bias_B = np.zeros(3, dtype=np.float64)
    noise_standard_deviation_B = np.zeros(3, dtype=np.float64)
    measurement_rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match=r"ideal_specific_force_B must have shape \(3,\)",
    ):
        accelerometer_specific_force_measurement_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
            noise_standard_deviation_B,
            measurement_rng,
        )


def test_accelerometer_specific_force_measurement_body_rejects_invalid_bias_shape() -> None:
    ideal_specific_force_B = np.zeros(3, dtype=np.float64)
    accelerometer_bias_B = np.zeros((3, 1), dtype=np.float64)
    noise_standard_deviation_B = np.zeros(3, dtype=np.float64)
    measurement_rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match=r"accelerometer_bias_B must have shape \(3,\)",
    ):
        accelerometer_specific_force_measurement_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
            noise_standard_deviation_B,
            measurement_rng,
        )


def test_accelerometer_specific_force_measurement_body_rejects_invalid_noise_shape() -> None:
    ideal_specific_force_B = np.zeros(3, dtype=np.float64)
    accelerometer_bias_B = np.zeros(3, dtype=np.float64)
    noise_standard_deviation_B = np.zeros((3, 1), dtype=np.float64)
    measurement_rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match=r"noise_standard_deviation_B must have shape \(3,\)",
    ):
        accelerometer_specific_force_measurement_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
            noise_standard_deviation_B,
            measurement_rng,
        )


@pytest.mark.parametrize(
    "invalid_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_accelerometer_specific_force_measurement_body_rejects_nonfinite_ideal_values(
    invalid_value: float,
) -> None:
    ideal_specific_force_B = np.array(
        [1.2, invalid_value, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.zeros(3, dtype=np.float64)
    noise_standard_deviation_B = np.zeros(3, dtype=np.float64)
    measurement_rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="ideal_specific_force_B must contain only finite values",
    ):
        accelerometer_specific_force_measurement_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
            noise_standard_deviation_B,
            measurement_rng,
        )


@pytest.mark.parametrize(
    "invalid_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_accelerometer_specific_force_measurement_body_rejects_nonfinite_bias_values(
    invalid_value: float,
) -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, invalid_value, 0.1],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.zeros(3, dtype=np.float64)
    measurement_rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="accelerometer_bias_B must contain only finite values",
    ):
        accelerometer_specific_force_measurement_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
            noise_standard_deviation_B,
            measurement_rng,
        )


@pytest.mark.parametrize(
    "invalid_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_accelerometer_specific_force_measurement_body_rejects_nonfinite_noise_values(
    invalid_value: float,
) -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, -0.02, 0.1],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, invalid_value, 0.03],
        dtype=np.float64,
    )
    measurement_rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="noise_standard_deviation_B must contain only finite values",
    ):
        accelerometer_specific_force_measurement_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
            noise_standard_deviation_B,
            measurement_rng,
        )


def test_accelerometer_measurement_body_rejects_negative_noise_standard_deviation() -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, -0.02, 0.1],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, -0.02, 0.03],
        dtype=np.float64,
    )
    measurement_rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="noise_standard_deviation_B must be nonnegative",
    ):
        accelerometer_specific_force_measurement_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
            noise_standard_deviation_B,
            measurement_rng,
        )


def test_accelerometer_measurement_body_rejected_input_does_not_advance_rng() -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, -0.02, 0.1],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, -0.02, 0.03],
        dtype=np.float64,
    )
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    with pytest.raises(
        ValueError,
        match="noise_standard_deviation_B must be nonnegative",
    ):
        accelerometer_specific_force_measurement_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
            noise_standard_deviation_B,
            measurement_rng,
        )

    next_measurement_sample_B = measurement_rng.standard_normal(3)
    first_reference_sample_B = reference_rng.standard_normal(3)
    np.testing.assert_array_equal(
        next_measurement_sample_B,
        first_reference_sample_B,
    )


def test_accelerometer_specific_force_with_bias_body_adds_constant_bias() -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, -0.02, 0.1],
        dtype=np.float64,
    )

    specific_force_with_bias_B = accelerometer_specific_force_with_bias_body(
        ideal_specific_force_B,
        accelerometer_bias_B,
    )

    expected_specific_force_with_bias_B = np.array(
        [1.25, -0.82, -9.71],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        specific_force_with_bias_B,
        expected_specific_force_with_bias_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_accelerometer_zero_bias_returns_ideal_specific_force() -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.zeros(3, dtype=np.float64)

    specific_force_with_bias_B = accelerometer_specific_force_with_bias_body(
        ideal_specific_force_B,
        accelerometer_bias_B,
    )

    np.testing.assert_array_equal(
        specific_force_with_bias_B,
        ideal_specific_force_B,
    )


def test_accelerometer_specific_force_with_bias_body_returns_independent_measurement() -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, -0.02, 0.1],
        dtype=np.float64,
    )

    specific_force_with_bias_B = accelerometer_specific_force_with_bias_body(
        ideal_specific_force_B,
        accelerometer_bias_B,
    )

    assert not np.shares_memory(
        specific_force_with_bias_B,
        ideal_specific_force_B,
    )
    assert not np.shares_memory(
        specific_force_with_bias_B,
        accelerometer_bias_B,
    )


def test_accelerometer_specific_force_with_bias_body_rejects_invalid_ideal_shape() -> None:
    ideal_specific_force_B = np.zeros((3, 1), dtype=np.float64)
    accelerometer_bias_B = np.zeros(3, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"ideal_specific_force_B must have shape \(3,\)",
    ):
        accelerometer_specific_force_with_bias_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
        )


def test_accelerometer_specific_force_with_bias_body_rejects_invalid_bias_shape() -> None:
    ideal_specific_force_B = np.zeros(3, dtype=np.float64)
    accelerometer_bias_B = np.zeros((3, 1), dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"accelerometer_bias_B must have shape \(3,\)",
    ):
        accelerometer_specific_force_with_bias_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
        )


@pytest.mark.parametrize(
    "invalid_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_accelerometer_specific_force_with_bias_body_rejects_nonfinite_ideal_values(
    invalid_value: float,
) -> None:
    ideal_specific_force_B = np.array(
        [1.2, invalid_value, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.zeros(3, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="ideal_specific_force_B must contain only finite values",
    ):
        accelerometer_specific_force_with_bias_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
        )


@pytest.mark.parametrize(
    "invalid_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_accelerometer_specific_force_with_bias_body_rejects_nonfinite_bias_values(
    invalid_value: float,
) -> None:
    ideal_specific_force_B = np.array(
        [1.2, -0.8, -9.81],
        dtype=np.float64,
    )
    accelerometer_bias_B = np.array(
        [0.05, invalid_value, 0.1],
        dtype=np.float64,
    )

    with pytest.raises(
        ValueError,
        match="accelerometer_bias_B must contain only finite values",
    ):
        accelerometer_specific_force_with_bias_body(
            ideal_specific_force_B,
            accelerometer_bias_B,
        )


def test_gyroscope_bias_random_walk_step_body_matches_seeded_update_equation() -> None:
    current_gyroscope_bias_B = np.array(
        [0.01, -0.02, 0.03],
        dtype=np.float64,
    )
    gyroscope_bias_random_walk_density_B = np.array(
        [0.002, 0.004, 0.006],
        dtype=np.float64,
    )
    time_step = 0.25
    seed = 12345
    bias_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    standard_normal_sample_B = reference_rng.standard_normal(3)
    expected_next_gyroscope_bias_B = (
        current_gyroscope_bias_B
        + gyroscope_bias_random_walk_density_B * np.sqrt(time_step) * standard_normal_sample_B
    )

    next_gyroscope_bias_B = gyroscope_bias_random_walk_step_body(
        current_gyroscope_bias_B,
        gyroscope_bias_random_walk_density_B,
        time_step,
        bias_rng,
    )

    np.testing.assert_allclose(
        next_gyroscope_bias_B,
        expected_next_gyroscope_bias_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_gyroscope_bias_random_walk_step_body_rejects_invalid_current_bias_shape() -> None:
    current_gyroscope_bias_B = np.zeros((3, 1), dtype=np.float64)
    gyroscope_bias_random_walk_density_B = np.zeros(3, dtype=np.float64)
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match=r"current_gyroscope_bias_B must have shape \(3,\)",
    ):
        gyroscope_bias_random_walk_step_body(
            current_gyroscope_bias_B,
            gyroscope_bias_random_walk_density_B,
            0.25,
            rng,
        )


def test_gyroscope_bias_random_walk_step_body_rejects_invalid_density_shape() -> None:
    current_gyroscope_bias_B = np.zeros(3, dtype=np.float64)
    gyroscope_bias_random_walk_density_B = np.zeros(
        (3, 1),
        dtype=np.float64,
    )
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match=r"gyroscope_bias_random_walk_density_B must have shape \(3,\)",
    ):
        gyroscope_bias_random_walk_step_body(
            current_gyroscope_bias_B,
            gyroscope_bias_random_walk_density_B,
            0.25,
            rng,
        )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_gyroscope_bias_random_walk_step_body_rejects_non_finite_current_bias(
    non_finite_value: float,
) -> None:
    current_gyroscope_bias_B = np.array(
        [0.01, non_finite_value, 0.03],
        dtype=np.float64,
    )
    gyroscope_bias_random_walk_density_B = np.zeros(3, dtype=np.float64)
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="current_gyroscope_bias_B must contain only finite values",
    ):
        gyroscope_bias_random_walk_step_body(
            current_gyroscope_bias_B,
            gyroscope_bias_random_walk_density_B,
            0.25,
            rng,
        )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_gyroscope_bias_random_walk_step_body_rejects_non_finite_density(
    non_finite_value: float,
) -> None:
    current_gyroscope_bias_B = np.zeros(3, dtype=np.float64)
    gyroscope_bias_random_walk_density_B = np.array(
        [0.002, non_finite_value, 0.006],
        dtype=np.float64,
    )
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="gyroscope_bias_random_walk_density_B must contain only finite values",
    ):
        gyroscope_bias_random_walk_step_body(
            current_gyroscope_bias_B,
            gyroscope_bias_random_walk_density_B,
            0.25,
            rng,
        )


def test_gyroscope_bias_random_walk_step_body_rejects_negative_density() -> None:
    current_gyroscope_bias_B = np.zeros(3, dtype=np.float64)
    gyroscope_bias_random_walk_density_B = np.array(
        [0.002, -0.004, 0.006],
        dtype=np.float64,
    )
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="gyroscope_bias_random_walk_density_B must be nonnegative",
    ):
        gyroscope_bias_random_walk_step_body(
            current_gyroscope_bias_B,
            gyroscope_bias_random_walk_density_B,
            0.25,
            rng,
        )


@pytest.mark.parametrize(
    ("invalid_time_step", "expected_message"),
    [
        (np.nan, "time_step must be finite"),
        (np.inf, "time_step must be finite"),
        (-np.inf, "time_step must be finite"),
        (0.0, "time_step must be positive"),
        (-0.25, "time_step must be positive"),
    ],
)
def test_gyroscope_bias_random_walk_step_body_rejects_invalid_time_step(
    invalid_time_step: float,
    expected_message: str,
) -> None:
    current_gyroscope_bias_B = np.zeros(3, dtype=np.float64)
    gyroscope_bias_random_walk_density_B = np.array(
        [0.002, 0.004, 0.006],
        dtype=np.float64,
    )
    rng = create_rng(12345)

    with np.errstate(invalid="ignore"):
        with pytest.raises(
            ValueError,
            match=expected_message,
        ):
            gyroscope_bias_random_walk_step_body(
                current_gyroscope_bias_B,
                gyroscope_bias_random_walk_density_B,
                invalid_time_step,
                rng,
            )


def test_gyroscope_bias_random_walk_rejected_time_preserves_rng() -> None:
    current_gyroscope_bias_B = np.zeros(3, dtype=np.float64)
    gyroscope_bias_random_walk_density_B = np.array(
        [0.002, 0.004, 0.006],
        dtype=np.float64,
    )
    seed = 12345
    bias_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    with pytest.raises(
        ValueError,
        match="time_step must be positive",
    ):
        gyroscope_bias_random_walk_step_body(
            current_gyroscope_bias_B,
            gyroscope_bias_random_walk_density_B,
            0.0,
            bias_rng,
        )

    next_bias_sample_B = bias_rng.standard_normal(3)
    first_reference_sample_B = reference_rng.standard_normal(3)
    np.testing.assert_array_equal(
        next_bias_sample_B,
        first_reference_sample_B,
    )


def test_gyroscope_angular_velocity_measurement_body_adds_seeded_white_noise() -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, -0.03, 0.04],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, 0.02, 0.03],
        dtype=np.float64,
    )
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    angular_velocity_measurement_B = gyroscope_angular_velocity_measurement_body(
        ideal_angular_velocity_B,
        gyroscope_bias_B,
        noise_standard_deviation_B,
        measurement_rng,
    )

    standard_normal_sample_B = reference_rng.standard_normal(3)
    expected_angular_velocity_measurement_B = (
        ideal_angular_velocity_B
        + gyroscope_bias_B
        + noise_standard_deviation_B * standard_normal_sample_B
    )
    np.testing.assert_allclose(
        angular_velocity_measurement_B,
        expected_angular_velocity_measurement_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_gyroscope_measurement_body_zero_noise_still_consumes_random_draw() -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, -0.03, 0.04],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.zeros(3, dtype=np.float64)
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    angular_velocity_measurement_B = gyroscope_angular_velocity_measurement_body(
        ideal_angular_velocity_B,
        gyroscope_bias_B,
        noise_standard_deviation_B,
        measurement_rng,
    )

    expected_angular_velocity_measurement_B = ideal_angular_velocity_B + gyroscope_bias_B
    np.testing.assert_allclose(
        angular_velocity_measurement_B,
        expected_angular_velocity_measurement_B,
        rtol=0.0,
        atol=1e-12,
    )

    reference_rng.standard_normal(3)
    next_measurement_sample_B = measurement_rng.standard_normal(3)
    next_reference_sample_B = reference_rng.standard_normal(3)
    np.testing.assert_array_equal(
        next_measurement_sample_B,
        next_reference_sample_B,
    )


def test_gyroscope_measurement_body_replays_same_seed_sequence() -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, -0.03, 0.04],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, 0.02, 0.03],
        dtype=np.float64,
    )
    seed = 12345
    first_rng = create_rng(seed)
    second_rng = create_rng(seed)

    first_sequence_B = np.array(
        [
            gyroscope_angular_velocity_measurement_body(
                ideal_angular_velocity_B,
                gyroscope_bias_B,
                noise_standard_deviation_B,
                first_rng,
            )
            for _ in range(2)
        ],
        dtype=np.float64,
    )
    second_sequence_B = np.array(
        [
            gyroscope_angular_velocity_measurement_body(
                ideal_angular_velocity_B,
                gyroscope_bias_B,
                noise_standard_deviation_B,
                second_rng,
            )
            for _ in range(2)
        ],
        dtype=np.float64,
    )

    np.testing.assert_array_equal(
        first_sequence_B,
        second_sequence_B,
    )


def test_gyroscope_measurement_body_advances_rng_between_calls() -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, -0.03, 0.04],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, 0.02, 0.03],
        dtype=np.float64,
    )
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    first_measurement_B = gyroscope_angular_velocity_measurement_body(
        ideal_angular_velocity_B,
        gyroscope_bias_B,
        noise_standard_deviation_B,
        measurement_rng,
    )
    second_measurement_B = gyroscope_angular_velocity_measurement_body(
        ideal_angular_velocity_B,
        gyroscope_bias_B,
        noise_standard_deviation_B,
        measurement_rng,
    )

    first_standard_normal_B = reference_rng.standard_normal(3)
    second_standard_normal_B = reference_rng.standard_normal(3)
    expected_first_measurement_B = (
        ideal_angular_velocity_B
        + gyroscope_bias_B
        + noise_standard_deviation_B * first_standard_normal_B
    )
    expected_second_measurement_B = (
        ideal_angular_velocity_B
        + gyroscope_bias_B
        + noise_standard_deviation_B * second_standard_normal_B
    )
    np.testing.assert_allclose(
        first_measurement_B,
        expected_first_measurement_B,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        second_measurement_B,
        expected_second_measurement_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_gyroscope_measurement_body_matches_configured_noise_statistics() -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, -0.03, 0.04],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, 0.02, 0.03],
        dtype=np.float64,
    )
    sample_count = 20_000
    rng = create_rng(12345)

    measurements_B = np.array(
        [
            gyroscope_angular_velocity_measurement_body(
                ideal_angular_velocity_B,
                gyroscope_bias_B,
                noise_standard_deviation_B,
                rng,
            )
            for _ in range(sample_count)
        ],
        dtype=np.float64,
    )

    empirical_mean_B = np.mean(measurements_B, axis=0)
    empirical_standard_deviation_B = np.std(
        measurements_B,
        axis=0,
        ddof=1,
    )
    expected_mean_B = ideal_angular_velocity_B + gyroscope_bias_B
    mean_error_in_standard_errors_B = (
        np.abs(empirical_mean_B - expected_mean_B)
        * np.sqrt(sample_count)
        / noise_standard_deviation_B
    )
    standard_deviation_error_in_standard_errors_B = (
        np.abs(empirical_standard_deviation_B - noise_standard_deviation_B)
        * np.sqrt(2.0 * (sample_count - 1))
        / noise_standard_deviation_B
    )
    five_standard_errors_B = np.full(
        3,
        5.0,
        dtype=np.float64,
    )

    np.testing.assert_array_less(
        mean_error_in_standard_errors_B,
        five_standard_errors_B,
    )
    np.testing.assert_array_less(
        standard_deviation_error_in_standard_errors_B,
        five_standard_errors_B,
    )


def test_gyroscope_angular_velocity_measurement_body_returns_independent_measurement() -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, -0.03, 0.04],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, 0.02, 0.03],
        dtype=np.float64,
    )
    rng = create_rng(12345)

    angular_velocity_measurement_B = gyroscope_angular_velocity_measurement_body(
        ideal_angular_velocity_B,
        gyroscope_bias_B,
        noise_standard_deviation_B,
        rng,
    )

    assert not np.shares_memory(
        angular_velocity_measurement_B,
        ideal_angular_velocity_B,
    )
    assert not np.shares_memory(
        angular_velocity_measurement_B,
        gyroscope_bias_B,
    )
    assert not np.shares_memory(
        angular_velocity_measurement_B,
        noise_standard_deviation_B,
    )


def test_gyroscope_angular_velocity_measurement_body_rejects_invalid_ideal_shape() -> None:
    ideal_angular_velocity_B = np.zeros((3, 1), dtype=np.float64)
    gyroscope_bias_B = np.zeros(3, dtype=np.float64)
    noise_standard_deviation_B = np.ones(3, dtype=np.float64)
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match=r"ideal_angular_velocity_B must have shape \(3,\)",
    ):
        gyroscope_angular_velocity_measurement_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
            noise_standard_deviation_B,
            rng,
        )


def test_gyroscope_angular_velocity_measurement_body_rejects_invalid_bias_shape() -> None:
    ideal_angular_velocity_B = np.zeros(3, dtype=np.float64)
    gyroscope_bias_B = np.zeros((3, 1), dtype=np.float64)
    noise_standard_deviation_B = np.ones(3, dtype=np.float64)
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match=r"gyroscope_bias_B must have shape \(3,\)",
    ):
        gyroscope_angular_velocity_measurement_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
            noise_standard_deviation_B,
            rng,
        )


def test_gyroscope_angular_velocity_measurement_body_rejects_invalid_noise_shape() -> None:
    ideal_angular_velocity_B = np.zeros(3, dtype=np.float64)
    gyroscope_bias_B = np.zeros(3, dtype=np.float64)
    noise_standard_deviation_B = np.ones((3, 1), dtype=np.float64)
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match=r"noise_standard_deviation_B must have shape \(3,\)",
    ):
        gyroscope_angular_velocity_measurement_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
            noise_standard_deviation_B,
            rng,
        )


@pytest.mark.parametrize(
    "invalid_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_gyroscope_angular_velocity_measurement_body_rejects_nonfinite_ideal_values(
    invalid_value: float,
) -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, invalid_value, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.zeros(3, dtype=np.float64)
    noise_standard_deviation_B = np.ones(3, dtype=np.float64)
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="ideal_angular_velocity_B must contain only finite values",
    ):
        gyroscope_angular_velocity_measurement_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
            noise_standard_deviation_B,
            rng,
        )


@pytest.mark.parametrize(
    "invalid_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_gyroscope_angular_velocity_measurement_body_rejects_nonfinite_bias_values(
    invalid_value: float,
) -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, invalid_value, 0.04],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.ones(3, dtype=np.float64)
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="gyroscope_bias_B must contain only finite values",
    ):
        gyroscope_angular_velocity_measurement_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
            noise_standard_deviation_B,
            rng,
        )


@pytest.mark.parametrize(
    "invalid_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_gyroscope_angular_velocity_measurement_body_rejects_nonfinite_noise_standard_deviations(
    invalid_value: float,
) -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, -0.03, 0.04],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, invalid_value, 0.03],
        dtype=np.float64,
    )
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="noise_standard_deviation_B must contain only finite values",
    ):
        gyroscope_angular_velocity_measurement_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
            noise_standard_deviation_B,
            rng,
        )


def test_gyroscope_measurement_body_rejects_negative_noise_standard_deviation() -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, -0.03, 0.04],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, -0.02, 0.03],
        dtype=np.float64,
    )
    rng = create_rng(12345)

    with pytest.raises(
        ValueError,
        match="noise_standard_deviation_B must be nonnegative",
    ):
        gyroscope_angular_velocity_measurement_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
            noise_standard_deviation_B,
            rng,
        )


def test_gyroscope_measurement_body_rejected_input_does_not_advance_rng() -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, -0.03, 0.04],
        dtype=np.float64,
    )
    noise_standard_deviation_B = np.array(
        [0.01, -0.02, 0.03],
        dtype=np.float64,
    )
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    with pytest.raises(
        ValueError,
        match="noise_standard_deviation_B must be nonnegative",
    ):
        gyroscope_angular_velocity_measurement_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
            noise_standard_deviation_B,
            measurement_rng,
        )

    next_measurement_sample_B = measurement_rng.standard_normal(3)
    first_reference_sample_B = reference_rng.standard_normal(3)
    np.testing.assert_array_equal(
        next_measurement_sample_B,
        first_reference_sample_B,
    )


def test_gyroscope_angular_velocity_with_bias_body_adds_constant_bias() -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, -0.03, 0.04],
        dtype=np.float64,
    )

    angular_velocity_with_bias_B = gyroscope_angular_velocity_with_bias_body(
        ideal_angular_velocity_B,
        gyroscope_bias_B,
    )

    expected_angular_velocity_with_bias_B = np.array(
        [0.72, -0.43, 1.14],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        angular_velocity_with_bias_B,
        expected_angular_velocity_with_bias_B,
        rtol=0.0,
        atol=1e-12,
    )


def test_gyroscope_angular_velocity_with_bias_body_returns_independent_measurement() -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, -0.03, 0.04],
        dtype=np.float64,
    )

    angular_velocity_with_bias_B = gyroscope_angular_velocity_with_bias_body(
        ideal_angular_velocity_B,
        gyroscope_bias_B,
    )

    assert not np.shares_memory(
        angular_velocity_with_bias_B,
        ideal_angular_velocity_B,
    )
    assert not np.shares_memory(
        angular_velocity_with_bias_B,
        gyroscope_bias_B,
    )


def test_gyroscope_angular_velocity_with_bias_body_rejects_invalid_ideal_shape() -> None:
    ideal_angular_velocity_B = np.zeros((3, 1), dtype=np.float64)
    gyroscope_bias_B = np.zeros(3, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"ideal_angular_velocity_B must have shape \(3,\)",
    ):
        gyroscope_angular_velocity_with_bias_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
        )


def test_gyroscope_angular_velocity_with_bias_body_rejects_invalid_bias_shape() -> None:
    ideal_angular_velocity_B = np.zeros(3, dtype=np.float64)
    gyroscope_bias_B = np.zeros((3, 1), dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"gyroscope_bias_B must have shape \(3,\)",
    ):
        gyroscope_angular_velocity_with_bias_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
        )


@pytest.mark.parametrize(
    "invalid_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_gyroscope_angular_velocity_with_bias_body_rejects_nonfinite_ideal_values(
    invalid_value: float,
) -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, invalid_value, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.zeros(3, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="ideal_angular_velocity_B must contain only finite values",
    ):
        gyroscope_angular_velocity_with_bias_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
        )


@pytest.mark.parametrize(
    "invalid_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_gyroscope_angular_velocity_with_bias_body_rejects_nonfinite_bias_values(
    invalid_value: float,
) -> None:
    ideal_angular_velocity_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    gyroscope_bias_B = np.array(
        [0.02, invalid_value, 0.04],
        dtype=np.float64,
    )

    with pytest.raises(
        ValueError,
        match="gyroscope_bias_B must contain only finite values",
    ):
        gyroscope_angular_velocity_with_bias_body(
            ideal_angular_velocity_B,
            gyroscope_bias_B,
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
