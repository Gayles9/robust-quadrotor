import numpy as np
import pytest

from quadrotor_math.position_sensors import (
    barometric_altitude_measurement,
    ideal_barometric_altitude_from_position_world,
    ideal_position_measurement_world,
    position_measurement_world,
)
from quadrotor_math.randomness import create_rng


def test_ideal_position_measurement_world_returns_ned_position() -> None:
    position_W = np.array([12.5, -7.25, 3.0], dtype=np.float64)

    measured_position_W = ideal_position_measurement_world(position_W)

    np.testing.assert_array_equal(measured_position_W, position_W)


def test_ideal_position_measurement_world_owns_float64_output() -> None:
    position_W = np.array([12.5, -7.25, 3.0], dtype=np.float64)
    original_position_W = position_W.copy()

    measured_position_W = ideal_position_measurement_world(position_W)

    assert measured_position_W.shape == (3,)
    assert measured_position_W.dtype == np.float64
    assert not np.shares_memory(measured_position_W, position_W)
    np.testing.assert_array_equal(position_W, original_position_W)


def test_ideal_position_measurement_world_rejects_invalid_shape() -> None:
    position_W = np.zeros((3, 1), dtype=np.float64)

    with pytest.raises(ValueError, match=r"position_W must have shape \(3,\)"):
        ideal_position_measurement_world(position_W)


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_ideal_position_measurement_world_rejects_non_finite_values(
    non_finite_value: float,
) -> None:
    position_W = np.array([12.5, non_finite_value, 3.0], dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="position_W must contain only finite values",
    ):
        ideal_position_measurement_world(position_W)


def test_position_measurement_world_adds_seeded_bias_and_white_noise() -> None:
    ideal_position_W = np.array([12.5, -7.25, 3.0], dtype=np.float64)
    position_bias_W = np.array([0.5, -0.25, 1.0], dtype=np.float64)
    noise_standard_deviation_W = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    standard_normal_sample_W = reference_rng.standard_normal(3)
    expected_position_measurement_W = (
        ideal_position_W + position_bias_W + noise_standard_deviation_W * standard_normal_sample_W
    )

    measured_position_W = position_measurement_world(
        ideal_position_W,
        position_bias_W,
        noise_standard_deviation_W,
        measurement_rng,
    )

    np.testing.assert_allclose(
        measured_position_W,
        expected_position_measurement_W,
        rtol=0.0,
        atol=1e-12,
    )


def test_position_measurement_world_owns_float64_output() -> None:
    ideal_position_W = np.array([12.5, -7.25, 3.0], dtype=np.float64)
    position_bias_W = np.array([0.5, -0.25, 1.0], dtype=np.float64)
    noise_standard_deviation_W = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    original_ideal_position_W = ideal_position_W.copy()
    original_position_bias_W = position_bias_W.copy()
    original_noise_standard_deviation_W = noise_standard_deviation_W.copy()

    measured_position_W = position_measurement_world(
        ideal_position_W,
        position_bias_W,
        noise_standard_deviation_W,
        create_rng(12345),
    )

    assert measured_position_W.shape == (3,)
    assert measured_position_W.dtype == np.float64
    assert not np.shares_memory(measured_position_W, ideal_position_W)
    assert not np.shares_memory(measured_position_W, position_bias_W)
    assert not np.shares_memory(
        measured_position_W,
        noise_standard_deviation_W,
    )
    np.testing.assert_array_equal(
        ideal_position_W,
        original_ideal_position_W,
    )
    np.testing.assert_array_equal(
        position_bias_W,
        original_position_bias_W,
    )
    np.testing.assert_array_equal(
        noise_standard_deviation_W,
        original_noise_standard_deviation_W,
    )


def test_position_measurement_world_zero_noise_still_consumes_random_draw() -> None:
    ideal_position_W = np.array([12.5, -7.25, 3.0], dtype=np.float64)
    position_bias_W = np.array([0.5, -0.25, 1.0], dtype=np.float64)
    noise_standard_deviation_W = np.zeros(3, dtype=np.float64)
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    first_reference_sample_W = reference_rng.standard_normal(3)
    expected_position_measurement_W = (
        ideal_position_W + position_bias_W + noise_standard_deviation_W * first_reference_sample_W
    )
    expected_next_sample_W = reference_rng.standard_normal(3)

    measured_position_W = position_measurement_world(
        ideal_position_W,
        position_bias_W,
        noise_standard_deviation_W,
        measurement_rng,
    )
    next_measurement_sample_W = measurement_rng.standard_normal(3)

    np.testing.assert_array_equal(
        measured_position_W,
        expected_position_measurement_W,
    )
    np.testing.assert_array_equal(
        next_measurement_sample_W,
        expected_next_sample_W,
    )


def test_position_measurement_world_replays_same_seed_sequence() -> None:
    ideal_position_W = np.array([12.5, -7.25, 3.0], dtype=np.float64)
    position_bias_W = np.array([0.5, -0.25, 1.0], dtype=np.float64)
    noise_standard_deviation_W = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    seed = 12345
    first_rng = create_rng(seed)
    second_rng = create_rng(seed)

    first_sequence_W = np.array(
        [
            position_measurement_world(
                ideal_position_W,
                position_bias_W,
                noise_standard_deviation_W,
                first_rng,
            )
            for _ in range(2)
        ],
        dtype=np.float64,
    )
    second_sequence_W = np.array(
        [
            position_measurement_world(
                ideal_position_W,
                position_bias_W,
                noise_standard_deviation_W,
                second_rng,
            )
            for _ in range(2)
        ],
        dtype=np.float64,
    )

    np.testing.assert_array_equal(first_sequence_W, second_sequence_W)


def test_position_measurement_world_uses_successive_random_draws() -> None:
    ideal_position_W = np.array([12.5, -7.25, 3.0], dtype=np.float64)
    position_bias_W = np.array([0.5, -0.25, 1.0], dtype=np.float64)
    noise_standard_deviation_W = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    first_reference_sample_W = reference_rng.standard_normal(3)
    second_reference_sample_W = reference_rng.standard_normal(3)
    expected_first_measurement_W = (
        ideal_position_W + position_bias_W + noise_standard_deviation_W * first_reference_sample_W
    )
    expected_second_measurement_W = (
        ideal_position_W + position_bias_W + noise_standard_deviation_W * second_reference_sample_W
    )

    first_measurement_W = position_measurement_world(
        ideal_position_W,
        position_bias_W,
        noise_standard_deviation_W,
        measurement_rng,
    )
    second_measurement_W = position_measurement_world(
        ideal_position_W,
        position_bias_W,
        noise_standard_deviation_W,
        measurement_rng,
    )

    np.testing.assert_allclose(
        first_measurement_W,
        expected_first_measurement_W,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        second_measurement_W,
        expected_second_measurement_W,
        rtol=0.0,
        atol=1e-12,
    )


def test_position_measurement_world_rejects_invalid_ideal_position_shape() -> None:
    ideal_position_W = np.zeros((3, 1), dtype=np.float64)
    position_bias_W = np.zeros(3, dtype=np.float64)
    noise_standard_deviation_W = np.zeros(3, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"ideal_position_W must have shape \(3,\)",
    ):
        position_measurement_world(
            ideal_position_W,
            position_bias_W,
            noise_standard_deviation_W,
            create_rng(12345),
        )


def test_position_measurement_world_rejects_invalid_bias_shape() -> None:
    ideal_position_W = np.zeros(3, dtype=np.float64)
    position_bias_W = np.zeros((3, 1), dtype=np.float64)
    noise_standard_deviation_W = np.zeros(3, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"position_bias_W must have shape \(3,\)",
    ):
        position_measurement_world(
            ideal_position_W,
            position_bias_W,
            noise_standard_deviation_W,
            create_rng(12345),
        )


def test_position_measurement_world_rejects_invalid_noise_shape() -> None:
    ideal_position_W = np.zeros(3, dtype=np.float64)
    position_bias_W = np.zeros(3, dtype=np.float64)
    noise_standard_deviation_W = np.zeros((3, 1), dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"noise_standard_deviation_W must have shape \(3,\)",
    ):
        position_measurement_world(
            ideal_position_W,
            position_bias_W,
            noise_standard_deviation_W,
            create_rng(12345),
        )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_position_measurement_world_rejects_non_finite_ideal_position(
    non_finite_value: float,
) -> None:
    ideal_position_W = np.array(
        [1.0, non_finite_value, 3.0],
        dtype=np.float64,
    )
    position_bias_W = np.zeros(3, dtype=np.float64)
    noise_standard_deviation_W = np.zeros(3, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="ideal_position_W must contain only finite values",
    ):
        position_measurement_world(
            ideal_position_W,
            position_bias_W,
            noise_standard_deviation_W,
            create_rng(12345),
        )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_position_measurement_world_rejects_non_finite_bias(
    non_finite_value: float,
) -> None:
    ideal_position_W = np.zeros(3, dtype=np.float64)
    position_bias_W = np.array(
        [0.5, non_finite_value, 1.0],
        dtype=np.float64,
    )
    noise_standard_deviation_W = np.zeros(3, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="position_bias_W must contain only finite values",
    ):
        position_measurement_world(
            ideal_position_W,
            position_bias_W,
            noise_standard_deviation_W,
            create_rng(12345),
        )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_position_measurement_world_rejects_non_finite_noise(
    non_finite_value: float,
) -> None:
    ideal_position_W = np.zeros(3, dtype=np.float64)
    position_bias_W = np.zeros(3, dtype=np.float64)
    noise_standard_deviation_W = np.array(
        [0.1, non_finite_value, 0.3],
        dtype=np.float64,
    )

    with pytest.raises(
        ValueError,
        match="noise_standard_deviation_W must contain only finite values",
    ):
        position_measurement_world(
            ideal_position_W,
            position_bias_W,
            noise_standard_deviation_W,
            create_rng(12345),
        )


def test_position_measurement_world_rejects_negative_noise() -> None:
    ideal_position_W = np.zeros(3, dtype=np.float64)
    position_bias_W = np.zeros(3, dtype=np.float64)
    noise_standard_deviation_W = np.array(
        [0.1, -0.2, 0.3],
        dtype=np.float64,
    )

    with pytest.raises(
        ValueError,
        match="noise_standard_deviation_W must be nonnegative",
    ):
        position_measurement_world(
            ideal_position_W,
            position_bias_W,
            noise_standard_deviation_W,
            create_rng(12345),
        )


def test_position_measurement_world_rejected_input_does_not_advance_rng() -> None:
    ideal_position_W = np.array([12.5, -7.25, 3.0], dtype=np.float64)
    position_bias_W = np.array([0.5, -0.25, 1.0], dtype=np.float64)
    noise_standard_deviation_W = np.array(
        [0.1, -0.2, 0.3],
        dtype=np.float64,
    )
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    with pytest.raises(
        ValueError,
        match="noise_standard_deviation_W must be nonnegative",
    ):
        position_measurement_world(
            ideal_position_W,
            position_bias_W,
            noise_standard_deviation_W,
            measurement_rng,
        )

    next_measurement_sample_W = measurement_rng.standard_normal(3)
    first_reference_sample_W = reference_rng.standard_normal(3)
    np.testing.assert_array_equal(
        next_measurement_sample_W,
        first_reference_sample_W,
    )


def test_position_measurement_world_matches_configured_noise_statistics() -> None:
    ideal_position_W = np.array([12.5, -7.25, 3.0], dtype=np.float64)
    position_bias_W = np.array([0.5, -0.25, 1.0], dtype=np.float64)
    noise_standard_deviation_W = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    sample_count = 20_000
    rng = create_rng(12345)

    measurements_W = np.array(
        [
            position_measurement_world(
                ideal_position_W,
                position_bias_W,
                noise_standard_deviation_W,
                rng,
            )
            for _ in range(sample_count)
        ],
        dtype=np.float64,
    )

    empirical_mean_W = np.mean(measurements_W, axis=0)
    empirical_standard_deviation_W = np.std(
        measurements_W,
        axis=0,
        ddof=1,
    )
    expected_mean_W = ideal_position_W + position_bias_W
    mean_error_in_standard_errors_W = (
        np.abs(empirical_mean_W - expected_mean_W)
        * np.sqrt(sample_count)
        / noise_standard_deviation_W
    )
    standard_deviation_error_in_standard_errors_W = (
        np.abs(empirical_standard_deviation_W - noise_standard_deviation_W)
        * np.sqrt(2.0 * (sample_count - 1))
        / noise_standard_deviation_W
    )
    five_standard_errors_W = np.full(3, 5.0, dtype=np.float64)

    np.testing.assert_array_less(
        mean_error_in_standard_errors_W,
        five_standard_errors_W,
    )
    np.testing.assert_array_less(
        standard_deviation_error_in_standard_errors_W,
        five_standard_errors_W,
    )


@pytest.mark.parametrize(
    ("down_position", "expected_altitude"),
    [
        (15.0, 110.0),
        (-8.0, 133.0),
        (0.0, 125.0),
    ],
)
def test_ideal_barometric_altitude_from_position_world_uses_ned_down_sign(
    down_position: float,
    expected_altitude: float,
) -> None:
    position_W = np.array([12.5, -7.25, down_position], dtype=np.float64)

    altitude = ideal_barometric_altitude_from_position_world(
        position_W,
        125.0,
    )

    assert altitude == expected_altitude


def test_ideal_barometric_altitude_from_position_world_returns_python_float() -> None:
    position_W = np.array([12.5, -7.25, 15.0], dtype=np.float64)

    altitude = ideal_barometric_altitude_from_position_world(
        position_W,
        125.0,
    )

    assert isinstance(altitude, float)


def test_ideal_barometric_altitude_from_position_world_rejects_invalid_shape() -> None:
    position_W = np.zeros(4, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"position_W must have shape \(3,\)",
    ):
        ideal_barometric_altitude_from_position_world(
            position_W,
            125.0,
        )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_ideal_barometric_altitude_from_position_world_rejects_non_finite_position(
    non_finite_value: float,
) -> None:
    position_W = np.array(
        [non_finite_value, -7.25, 15.0],
        dtype=np.float64,
    )

    with pytest.raises(
        ValueError,
        match="position_W must contain only finite values",
    ):
        ideal_barometric_altitude_from_position_world(
            position_W,
            125.0,
        )


@pytest.mark.parametrize(
    "non_finite_reference_altitude",
    [np.nan, np.inf, -np.inf],
)
def test_ideal_barometric_altitude_from_position_world_rejects_non_finite_reference(
    non_finite_reference_altitude: float,
) -> None:
    position_W = np.array([12.5, -7.25, 15.0], dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="reference_altitude must be finite",
    ):
        ideal_barometric_altitude_from_position_world(
            position_W,
            non_finite_reference_altitude,
        )


def test_barometric_altitude_measurement_adds_seeded_bias_and_white_noise() -> None:
    ideal_altitude = 125.0
    barometric_altitude_bias = 1.5
    noise_standard_deviation = 0.75
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    standard_normal_sample = reference_rng.standard_normal()
    expected_altitude = (
        ideal_altitude
        + barometric_altitude_bias
        + noise_standard_deviation * standard_normal_sample
    )

    measured_altitude = barometric_altitude_measurement(
        ideal_altitude,
        barometric_altitude_bias,
        noise_standard_deviation,
        measurement_rng,
    )

    np.testing.assert_allclose(
        measured_altitude,
        expected_altitude,
        rtol=0.0,
        atol=1e-12,
    )


def test_barometric_altitude_measurement_returns_python_float() -> None:
    measured_altitude = barometric_altitude_measurement(
        125.0,
        1.5,
        0.75,
        create_rng(12345),
    )

    assert isinstance(measured_altitude, float)


def test_barometric_altitude_measurement_zero_noise_still_consumes_draw() -> None:
    ideal_altitude = 125.0
    barometric_altitude_bias = 1.5
    noise_standard_deviation = 0.0
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    first_reference_sample = reference_rng.standard_normal()
    expected_altitude = (
        ideal_altitude
        + barometric_altitude_bias
        + noise_standard_deviation * first_reference_sample
    )
    expected_next_sample = reference_rng.standard_normal()

    measured_altitude = barometric_altitude_measurement(
        ideal_altitude,
        barometric_altitude_bias,
        noise_standard_deviation,
        measurement_rng,
    )
    next_measurement_sample = measurement_rng.standard_normal()

    np.testing.assert_allclose(
        measured_altitude,
        expected_altitude,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_array_equal(
        next_measurement_sample,
        expected_next_sample,
    )


def test_barometric_altitude_measurement_replays_same_seed_sequence() -> None:
    seed = 12345
    first_rng = create_rng(seed)
    second_rng = create_rng(seed)

    first_sequence = np.array(
        [
            barometric_altitude_measurement(
                125.0,
                1.5,
                0.75,
                first_rng,
            )
            for _ in range(2)
        ],
        dtype=np.float64,
    )
    second_sequence = np.array(
        [
            barometric_altitude_measurement(
                125.0,
                1.5,
                0.75,
                second_rng,
            )
            for _ in range(2)
        ],
        dtype=np.float64,
    )

    np.testing.assert_array_equal(first_sequence, second_sequence)


def test_barometric_altitude_measurement_uses_successive_random_draws() -> None:
    ideal_altitude = 125.0
    barometric_altitude_bias = 1.5
    noise_standard_deviation = 0.75
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    first_reference_sample = reference_rng.standard_normal()
    second_reference_sample = reference_rng.standard_normal()
    expected_first_altitude = (
        ideal_altitude
        + barometric_altitude_bias
        + noise_standard_deviation * first_reference_sample
    )
    expected_second_altitude = (
        ideal_altitude
        + barometric_altitude_bias
        + noise_standard_deviation * second_reference_sample
    )

    first_altitude = barometric_altitude_measurement(
        ideal_altitude,
        barometric_altitude_bias,
        noise_standard_deviation,
        measurement_rng,
    )
    second_altitude = barometric_altitude_measurement(
        ideal_altitude,
        barometric_altitude_bias,
        noise_standard_deviation,
        measurement_rng,
    )

    np.testing.assert_allclose(
        first_altitude,
        expected_first_altitude,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        second_altitude,
        expected_second_altitude,
        rtol=0.0,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_barometric_altitude_measurement_rejects_non_finite_ideal_altitude(
    non_finite_value: float,
) -> None:
    with pytest.raises(
        ValueError,
        match="ideal_altitude must be finite",
    ):
        barometric_altitude_measurement(
            non_finite_value,
            1.5,
            0.75,
            create_rng(12345),
        )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_barometric_altitude_measurement_rejects_non_finite_bias(
    non_finite_value: float,
) -> None:
    with pytest.raises(
        ValueError,
        match="barometric_altitude_bias must be finite",
    ):
        barometric_altitude_measurement(
            125.0,
            non_finite_value,
            0.75,
            create_rng(12345),
        )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_barometric_altitude_measurement_rejects_non_finite_noise(
    non_finite_value: float,
) -> None:
    with pytest.raises(
        ValueError,
        match="noise_standard_deviation must be finite",
    ):
        barometric_altitude_measurement(
            125.0,
            1.5,
            non_finite_value,
            create_rng(12345),
        )


def test_barometric_altitude_measurement_rejects_negative_noise() -> None:
    with pytest.raises(
        ValueError,
        match="noise_standard_deviation must be nonnegative",
    ):
        barometric_altitude_measurement(
            125.0,
            1.5,
            -0.75,
            create_rng(12345),
        )


def test_barometric_altitude_rejected_input_does_not_advance_rng() -> None:
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)

    with pytest.raises(
        ValueError,
        match="noise_standard_deviation must be nonnegative",
    ):
        barometric_altitude_measurement(
            125.0,
            1.5,
            -0.75,
            measurement_rng,
        )

    next_measurement_sample = measurement_rng.standard_normal()
    first_reference_sample = reference_rng.standard_normal()
    np.testing.assert_array_equal(
        next_measurement_sample,
        first_reference_sample,
    )


def test_barometric_altitude_matches_configured_noise_statistics() -> None:
    ideal_altitude = 125.0
    barometric_altitude_bias = 1.5
    noise_standard_deviation = 0.75
    sample_count = 20_000
    rng = create_rng(12345)

    measurements = np.array(
        [
            barometric_altitude_measurement(
                ideal_altitude,
                barometric_altitude_bias,
                noise_standard_deviation,
                rng,
            )
            for _ in range(sample_count)
        ],
        dtype=np.float64,
    )

    empirical_mean = float(np.mean(measurements))
    empirical_standard_deviation = float(np.std(measurements, ddof=1))
    expected_mean = ideal_altitude + barometric_altitude_bias
    mean_error_in_standard_errors = (
        abs(empirical_mean - expected_mean) * np.sqrt(sample_count) / noise_standard_deviation
    )
    standard_deviation_error_in_standard_errors = (
        abs(empirical_standard_deviation - noise_standard_deviation)
        * np.sqrt(2.0 * (sample_count - 1))
        / noise_standard_deviation
    )

    assert mean_error_in_standard_errors < 5.0
    assert standard_deviation_error_in_standard_errors < 5.0
