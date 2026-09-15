import numpy as np
import pytest

from quadrotor_math.randomness import (
    RunRandomStream,
    RunRandomStreams,
    create_run_random_streams,
    create_run_rng,
)


def test_run_random_stream_names_are_manifest_stable() -> None:
    assert {stream.name: stream.value for stream in RunRandomStream} == {
        "ACCELEROMETER_MEASUREMENT_NOISE": "accelerometer.measurement_noise",
        "ACCELEROMETER_BIAS_RANDOM_WALK": "accelerometer.bias_random_walk",
        "GYROSCOPE_MEASUREMENT_NOISE": "gyroscope.measurement_noise",
        "GYROSCOPE_BIAS_RANDOM_WALK": "gyroscope.bias_random_walk",
        "LOCAL_POSITION_MEASUREMENT_NOISE": "local_position.measurement_noise",
        "BAROMETRIC_ALTITUDE_MEASUREMENT_NOISE": "barometric_altitude.measurement_noise",
    }


def test_create_run_rng_replays_named_stream_from_same_root_seed() -> None:
    root_seed = 0x0123456789ABCDEF0123456789ABCDEF
    first_rng = create_run_rng(
        root_seed,
        RunRandomStream.ACCELEROMETER_MEASUREMENT_NOISE,
    )
    replay_rng = create_run_rng(
        root_seed,
        RunRandomStream.ACCELEROMETER_MEASUREMENT_NOISE,
    )

    np.testing.assert_array_equal(
        first_rng.standard_normal(16),
        replay_rng.standard_normal(16),
    )


def test_create_run_rng_keeps_named_stream_consumption_independent() -> None:
    root_seed = 0x0123456789ABCDEF0123456789ABCDEF
    accelerometer_rng = create_run_rng(
        root_seed,
        RunRandomStream.ACCELEROMETER_MEASUREMENT_NOISE,
    )
    position_rng = create_run_rng(
        root_seed,
        RunRandomStream.LOCAL_POSITION_MEASUREMENT_NOISE,
    )
    position_reference_rng = create_run_rng(
        root_seed,
        RunRandomStream.LOCAL_POSITION_MEASUREMENT_NOISE,
    )

    accelerometer_rng.standard_normal(100)

    np.testing.assert_array_equal(
        position_rng.standard_normal(16),
        position_reference_rng.standard_normal(16),
    )


@pytest.mark.parametrize(
    ("stream", "stream_id"),
    [
        (RunRandomStream.ACCELEROMETER_MEASUREMENT_NOISE, 1),
        (RunRandomStream.ACCELEROMETER_BIAS_RANDOM_WALK, 2),
        (RunRandomStream.GYROSCOPE_MEASUREMENT_NOISE, 3),
        (RunRandomStream.GYROSCOPE_BIAS_RANDOM_WALK, 4),
        (RunRandomStream.LOCAL_POSITION_MEASUREMENT_NOISE, 5),
        (
            RunRandomStream.BAROMETRIC_ALTITUDE_MEASUREMENT_NOISE,
            6,
        ),
    ],
)
def test_create_run_rng_matches_versioned_stream_derivation(
    stream: RunRandomStream,
    stream_id: int,
) -> None:
    root_seed = 0x0123456789ABCDEF0123456789ABCDEF
    seed_sequence = np.random.SeedSequence(
        [1, stream_id, root_seed],
        pool_size=4,
    )
    reference_rng = np.random.Generator(np.random.PCG64(seed_sequence))
    actual_rng = create_run_rng(root_seed, stream)

    np.testing.assert_array_equal(
        actual_rng.bit_generator.random_raw(8),
        reference_rng.bit_generator.random_raw(8),
    )


@pytest.mark.parametrize(
    "invalid_root_seed",
    [True, 1.5],
)
def test_create_run_rng_rejects_non_integer_root_seed(
    invalid_root_seed: object,
) -> None:
    with pytest.raises(
        TypeError,
        match="root_seed must be a non-Boolean integer",
    ):
        create_run_rng(
            invalid_root_seed,
            RunRandomStream.ACCELEROMETER_MEASUREMENT_NOISE,
        )


@pytest.mark.parametrize(
    "out_of_range_root_seed",
    [-1, 2**128],
)
def test_create_run_rng_rejects_out_of_range_root_seed(
    out_of_range_root_seed: int,
) -> None:
    with pytest.raises(
        ValueError,
        match=r"root_seed must be in \[0, 2\*\*128\)",
    ):
        create_run_rng(
            out_of_range_root_seed,
            RunRandomStream.ACCELEROMETER_MEASUREMENT_NOISE,
        )


def test_create_run_rng_rejects_non_enum_stream() -> None:
    with pytest.raises(
        TypeError,
        match="stream must be a RunRandomStream",
    ):
        create_run_rng(
            12345,
            "accelerometer.measurement_noise",
        )


def test_create_run_random_streams_matches_individual_named_factories() -> None:
    root_seed = 0x0123456789ABCDEF0123456789ABCDEF
    streams = create_run_random_streams(root_seed)

    stream_pairs = (
        (
            streams.accelerometer_measurement_noise,
            RunRandomStream.ACCELEROMETER_MEASUREMENT_NOISE,
        ),
        (
            streams.accelerometer_bias_random_walk,
            RunRandomStream.ACCELEROMETER_BIAS_RANDOM_WALK,
        ),
        (
            streams.gyroscope_measurement_noise,
            RunRandomStream.GYROSCOPE_MEASUREMENT_NOISE,
        ),
        (
            streams.gyroscope_bias_random_walk,
            RunRandomStream.GYROSCOPE_BIAS_RANDOM_WALK,
        ),
        (
            streams.local_position_measurement_noise,
            RunRandomStream.LOCAL_POSITION_MEASUREMENT_NOISE,
        ),
        (
            streams.barometric_altitude_measurement_noise,
            RunRandomStream.BAROMETRIC_ALTITUDE_MEASUREMENT_NOISE,
        ),
    )

    assert type(streams) is RunRandomStreams
    for bundled_rng, stream in stream_pairs:
        reference_rng = create_run_rng(root_seed, stream)
        np.testing.assert_array_equal(
            bundled_rng.standard_normal(16),
            reference_rng.standard_normal(16),
        )


def test_create_run_random_streams_returns_six_distinct_generators() -> None:
    streams = create_run_random_streams(0x0123456789ABCDEF0123456789ABCDEF)
    generators = (
        streams.accelerometer_measurement_noise,
        streams.accelerometer_bias_random_walk,
        streams.gyroscope_measurement_noise,
        streams.gyroscope_bias_random_walk,
        streams.local_position_measurement_noise,
        streams.barometric_altitude_measurement_noise,
    )

    assert len({id(generator) for generator in generators}) == 6
