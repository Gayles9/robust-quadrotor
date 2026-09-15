from dataclasses import dataclass
from enum import StrEnum
from typing import Final

import numpy as np
from numpy.random import PCG64, Generator, SeedSequence


class RunRandomStream(StrEnum):
    """Stable manifest names for independent run-level random streams."""

    ACCELEROMETER_MEASUREMENT_NOISE = "accelerometer.measurement_noise"
    ACCELEROMETER_BIAS_RANDOM_WALK = "accelerometer.bias_random_walk"
    GYROSCOPE_MEASUREMENT_NOISE = "gyroscope.measurement_noise"
    GYROSCOPE_BIAS_RANDOM_WALK = "gyroscope.bias_random_walk"
    LOCAL_POSITION_MEASUREMENT_NOISE = "local_position.measurement_noise"
    BAROMETRIC_ALTITUDE_MEASUREMENT_NOISE = "barometric_altitude.measurement_noise"


@dataclass(frozen=True, slots=True, eq=False)
class RunRandomStreams:
    """Hold persistent independent generators for one reproducible run."""

    accelerometer_measurement_noise: Generator
    accelerometer_bias_random_walk: Generator
    gyroscope_measurement_noise: Generator
    gyroscope_bias_random_walk: Generator
    local_position_measurement_noise: Generator
    barometric_altitude_measurement_noise: Generator


_RUN_RANDOM_STREAM_DERIVATION_VERSION: Final = 1

_RUN_RANDOM_STREAM_IDS: Final[dict[RunRandomStream, int]] = {
    RunRandomStream.ACCELEROMETER_MEASUREMENT_NOISE: 1,
    RunRandomStream.ACCELEROMETER_BIAS_RANDOM_WALK: 2,
    RunRandomStream.GYROSCOPE_MEASUREMENT_NOISE: 3,
    RunRandomStream.GYROSCOPE_BIAS_RANDOM_WALK: 4,
    RunRandomStream.LOCAL_POSITION_MEASUREMENT_NOISE: 5,
    RunRandomStream.BAROMETRIC_ALTITUDE_MEASUREMENT_NOISE: 6,
}


def create_rng(seed: int) -> Generator:
    """Create a reproducible, independent NumPy generator from a seed."""
    return np.random.default_rng(seed)


def create_run_rng(
    root_seed: int,
    stream: RunRandomStream,
) -> Generator:
    """Create a deterministic generator for one stable run-level stream."""
    if isinstance(root_seed, bool) or not isinstance(root_seed, int):
        raise TypeError("root_seed must be a non-Boolean integer")

    if not 0 <= root_seed < 2**128:
        raise ValueError("root_seed must be in [0, 2**128)")

    if not isinstance(stream, RunRandomStream):
        raise TypeError("stream must be a RunRandomStream")

    seed_sequence = SeedSequence(
        [
            _RUN_RANDOM_STREAM_DERIVATION_VERSION,
            _RUN_RANDOM_STREAM_IDS[stream],
            root_seed,
        ],
        pool_size=4,
    )
    return Generator(PCG64(seed_sequence))


def create_run_random_streams(root_seed: int) -> RunRandomStreams:
    """Create all persistent independent named streams for one run."""
    return RunRandomStreams(
        accelerometer_measurement_noise=create_run_rng(
            root_seed,
            RunRandomStream.ACCELEROMETER_MEASUREMENT_NOISE,
        ),
        accelerometer_bias_random_walk=create_run_rng(
            root_seed,
            RunRandomStream.ACCELEROMETER_BIAS_RANDOM_WALK,
        ),
        gyroscope_measurement_noise=create_run_rng(
            root_seed,
            RunRandomStream.GYROSCOPE_MEASUREMENT_NOISE,
        ),
        gyroscope_bias_random_walk=create_run_rng(
            root_seed,
            RunRandomStream.GYROSCOPE_BIAS_RANDOM_WALK,
        ),
        local_position_measurement_noise=create_run_rng(
            root_seed,
            RunRandomStream.LOCAL_POSITION_MEASUREMENT_NOISE,
        ),
        barometric_altitude_measurement_noise=create_run_rng(
            root_seed,
            RunRandomStream.BAROMETRIC_ALTITUDE_MEASUREMENT_NOISE,
        ),
    )
