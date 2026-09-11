import numpy as np
from numpy.random import Generator
from numpy.typing import NDArray


def ideal_position_measurement_world(
    position_W: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return ideal local position in the NED world frame."""
    if position_W.shape != (3,):
        raise ValueError("position_W must have shape (3,)")

    if not np.all(np.isfinite(position_W)):
        raise ValueError("position_W must contain only finite values")

    return position_W.copy()


def ideal_barometric_altitude_from_position_world(
    position_W: NDArray[np.float64],
    reference_altitude: float,
) -> float:
    """Return positive-up altitude from NED world position."""
    if position_W.shape != (3,):
        raise ValueError("position_W must have shape (3,)")

    if not np.all(np.isfinite(position_W)):
        raise ValueError("position_W must contain only finite values")

    if not np.isfinite(reference_altitude):
        raise ValueError("reference_altitude must be finite")

    return float(reference_altitude - position_W[2])


def barometric_altitude_measurement(
    ideal_altitude: float,
    barometric_altitude_bias: float,
    noise_standard_deviation: float,
    rng: Generator,
) -> float:
    """Return a biased noisy positive-up barometric-altitude measurement."""
    if not np.isfinite(ideal_altitude):
        raise ValueError("ideal_altitude must be finite")

    if not np.isfinite(barometric_altitude_bias):
        raise ValueError("barometric_altitude_bias must be finite")

    if not np.isfinite(noise_standard_deviation):
        raise ValueError("noise_standard_deviation must be finite")

    if noise_standard_deviation < 0.0:
        raise ValueError("noise_standard_deviation must be nonnegative")

    standard_normal_sample = rng.standard_normal()
    return float(
        ideal_altitude
        + barometric_altitude_bias
        + noise_standard_deviation * standard_normal_sample
    )


def position_measurement_world(
    ideal_position_W: NDArray[np.float64],
    position_bias_W: NDArray[np.float64],
    noise_standard_deviation_W: NDArray[np.float64],
    rng: Generator,
) -> NDArray[np.float64]:
    """Return a biased noisy local position measurement in the NED world frame."""
    if ideal_position_W.shape != (3,):
        raise ValueError("ideal_position_W must have shape (3,)")

    if position_bias_W.shape != (3,):
        raise ValueError("position_bias_W must have shape (3,)")

    if noise_standard_deviation_W.shape != (3,):
        raise ValueError("noise_standard_deviation_W must have shape (3,)")

    if not np.all(np.isfinite(ideal_position_W)):
        raise ValueError("ideal_position_W must contain only finite values")

    if not np.all(np.isfinite(position_bias_W)):
        raise ValueError("position_bias_W must contain only finite values")

    if not np.all(np.isfinite(noise_standard_deviation_W)):
        raise ValueError("noise_standard_deviation_W must contain only finite values")

    if np.any(noise_standard_deviation_W < 0.0):
        raise ValueError("noise_standard_deviation_W must be nonnegative")

    standard_normal_sample_W = rng.standard_normal(3)
    return np.asarray(
        ideal_position_W + position_bias_W + noise_standard_deviation_W * standard_normal_sample_W,
        dtype=np.float64,
    )
