"""Explicit adapters from recorded sensor data and nominal assumptions to ESKF.

Only the artifact clock and sensor/delivery fields are read. Truth trajectories,
true bias histories, actuator commands and truth parameters never enter replay.
"""

import numpy as np
from numpy.typing import NDArray

from .eskf import EskfNominalState
from .eskf_endpoint import EskfSampledImuNoise
from .eskf_innovation import EskfInnovationPolicy
from .eskf_replay import (
    EskfObservationKind,
    EskfReplayConfiguration,
    EskfReplayInput,
    EskfReplayObservation,
    _owned_array,
)
from .run_artifact import RunArtifactData
from .run_configuration import NominalConfiguration
from .sensor_scheduling import _time_comparison_tolerance_s


def _indices(name: str, values: NDArray[np.int64], size: int) -> NDArray[np.int64]:
    if (
        not isinstance(values, np.ndarray)
        or values.shape != (size,)
        or values.dtype != np.dtype("<i8")
    ):
        raise ValueError(f"{name} must be an int64 vector of length {size}")
    return np.array(values, dtype=np.int64, copy=True)


def _first_delivery_index(times: NDArray[np.float64], scheduled: float) -> int:
    """Binary-search the scheduler's exact due predicate on a resolved clock."""
    lower, upper = 0, times.size
    while lower < upper:
        middle = (lower + upper) // 2
        current = float(times[middle])
        if scheduled <= current + _time_comparison_tolerance_s(scheduled, current):
            upper = middle
        else:
            lower = middle + 1
    return -1 if lower == times.size else lower


def eskf_replay_input_from_run_artifact(data: RunArtifactData) -> EskfReplayInput:
    """Extract a validated, owned measurement-only view of a generated/loaded run.

    Requires the canonical zero-origin fixed clock, resolvable time spacing,
    both IMU streams at every completed row, and exactly zero IMU delivery delay.
    Position/altitude acquisitions must exactly match clock rows; their arrivals
    must match the scheduler tolerance and the global delivery table. These are
    intentionally stricter execution requirements than generic artifact storage.

    Replay row zero is artifact truth-clock row one. No truth payload, truth
    initialization, dynamics, RNG or nominal-parameter inference is performed.
    """
    raw_time = data.truth_time_s
    if not isinstance(raw_time, np.ndarray) or raw_time.ndim != 1 or raw_time.size < 2:
        raise ValueError("truth_time_s must contain the origin and at least one completed row")
    clock = _owned_array("truth_time_s", raw_time, (raw_time.size,))
    dt = float(clock[1])
    if clock[0] != 0.0 or dt <= 0.0 or np.any(clock[1:] <= clock[:-1]):
        raise ValueError("truth_time_s must be a strictly increasing zero-origin clock")
    with np.errstate(over="ignore", invalid="ignore"):
        canonical = np.arange(clock.size, dtype=np.float64) * dt
    if not np.array_equal(clock, canonical):
        raise ValueError("truth_time_s must equal the canonical fixed grid")
    if dt <= 2.0 * _time_comparison_tolerance_s(float(clock[-1]), float(clock[-1])):
        raise ValueError("truth_time_s spacing must resolve scheduler comparison tolerance")
    times = clock[1:]
    n = times.size
    expected_deliveries: list[tuple[int, int, int]] = []
    observations: list[EskfReplayObservation] = []
    imu: dict[str, NDArray[np.float64]] = {}
    for sensor_id, (name, measurement_name, width) in enumerate(
        (
            ("accelerometer", "accelerometer_measurements_B", 3),
            ("gyroscope", "gyroscope_measurements_B", 3),
            ("local_position", "local_position_measurements_W", 3),
            ("barometric_altitude", "barometric_altitude_measurements", 1),
        ),
        start=1,
    ):
        raw_acquired = getattr(data, f"{name}_acquisition_time_s")
        if not isinstance(raw_acquired, np.ndarray) or raw_acquired.ndim != 1:
            raise ValueError(f"{name}_acquisition_time_s must be a vector")
        count = raw_acquired.size
        acquired = _owned_array(f"{name}_acquisition_time_s", raw_acquired, (count,))
        scheduled = _owned_array(
            f"{name}_delivery_time_s", getattr(data, f"{name}_delivery_time_s"), (count,)
        )
        arrived = _indices(
            f"{name}_delivered_at_truth_index",
            getattr(data, f"{name}_delivered_at_truth_index"),
            count,
        )
        for suffix in ("sequence_index", "observation_index"):
            indices = _indices(f"{name}_{suffix}", getattr(data, f"{name}_{suffix}"), count)
            if not np.array_equal(indices, np.arange(count)):
                raise ValueError(f"{name}_{suffix} must be consecutive from zero")
        shape = (count,) if sensor_id == 4 else (count, width)
        measurements = _owned_array(measurement_name, getattr(data, measurement_name), shape)
        if sensor_id <= 2:
            if (
                not np.array_equal(acquired, times)
                or not np.array_equal(scheduled, times)
                or not np.array_equal(arrived, np.arange(1, n + 1))
            ):
                raise ValueError(f"{name} must be full-rate, paired and zero-delay")
            imu[name] = measurements
        if np.any(acquired[1:] <= acquired[:-1]) or np.any(scheduled < acquired):
            raise ValueError(f"{name} must have increasing acquisitions and causal delivery times")
        acquired_indices = np.searchsorted(times, acquired)
        if np.any(acquired_indices >= n) or not np.array_equal(times[acquired_indices], acquired):
            raise ValueError(f"{name} acquisitions must exactly match replay clock rows")
        for row in range(count):
            delivery_index = _first_delivery_index(times, float(scheduled[row]))
            expected_truth_index = -1 if delivery_index == -1 else delivery_index + 1
            if int(arrived[row]) != expected_truth_index:
                raise ValueError(f"{name} arrival metadata disagrees with scheduled delivery")
            if delivery_index != -1:
                expected_deliveries.append((delivery_index, sensor_id, row))
            if sensor_id >= 3:
                value = measurements[row] if sensor_id == 3 else np.array([measurements[row]])
                observations.append(
                    EskfReplayObservation(
                        EskfObservationKind(sensor_id),
                        row,
                        int(acquired_indices[row]),
                        delivery_index,
                        value,
                    )
                )
    expected_deliveries.sort()
    count = len(expected_deliveries)
    for name, column in (
        ("delivery_sensor_id", 1),
        ("delivery_sequence_index", 2),
        ("delivery_observation_index", 2),
    ):
        actual = _indices(name, getattr(data, name), count)
        expected = np.array([row[column] for row in expected_deliveries], dtype=np.int64)
        if not np.array_equal(actual, expected):
            raise ValueError(f"{name} must match canonical sensor deliveries")
    return EskfReplayInput(times, imu["accelerometer"], imu["gyroscope"], tuple(observations))


def eskf_replay_configuration_from_nominal(
    nominal: NominalConfiguration,
    *,
    initial_time_s: float,
    initial_state: EskfNominalState,
    initial_covariance: NDArray[np.float64],
    continuous_noise_covariance: NDArray[np.float64],
    fuse_local_position: bool = True,
    fuse_barometric_altitude: bool = True,
    innovation_policy: EskfInnovationPolicy | None = None,
    sampled_imu_noise: EskfSampledImuNoise | None = None,
) -> EskfReplayConfiguration:
    """Use nominal world/position parameters, preserving explicit prior and Q_c.

    Square discrete position/altitude noise standard deviations to obtain R.
    Do not turn per-sample IMU standard deviations into continuous densities
    implicitly. Caller-supplied IMU bias estimates are not replaced by nominal
    or true initial biases. There is no access to RunConfiguration.truth.
    An optional innovation policy is caller-supplied, never inferred or tuned
    from nominal noise or observed data. None retains the existing unscored path.
    Explicit sampled_imu_noise selects endpoint prediction; Q_c must then be zero.
    Neither noise contract is inferred from truth or silently converted.
    """
    if not isinstance(nominal, NominalConfiguration):
        raise TypeError("nominal must be a NominalConfiguration")
    sensors = nominal.position_sensors
    try:
        with np.errstate(over="raise", invalid="raise", under="ignore"):
            position_variances = np.square(sensors.local_position_noise_standard_deviation_W)
            altitude_variance = float(
                np.square(sensors.barometric_altitude_noise_standard_deviation)
            )
    except FloatingPointError:
        raise ValueError("nominal measurement variances must remain finite") from None
    if np.any(
        (sensors.local_position_noise_standard_deviation_W > 0.0) & (position_variances == 0.0)
    ) or (sensors.barometric_altitude_noise_standard_deviation > 0.0 and altitude_variance == 0.0):
        raise ValueError("positive nominal measurement variances must be representable")
    return EskfReplayConfiguration(
        initial_time_s=initial_time_s,
        initial_state=initial_state,
        initial_covariance=initial_covariance,
        gravity_acceleration=nominal.world.gravity_acceleration,
        continuous_noise_covariance=continuous_noise_covariance,
        local_position_bias_W=sensors.local_position_bias_W,
        local_position_noise_covariance_W=np.diag(position_variances),
        barometric_reference_altitude=sensors.barometric_reference_altitude,
        barometric_altitude_bias=sensors.barometric_altitude_bias,
        barometric_altitude_noise_variance=altitude_variance,
        fuse_local_position=fuse_local_position,
        fuse_barometric_altitude=fuse_barometric_altitude,
        innovation_policy=innovation_policy,
        sampled_imu_noise=sampled_imu_noise,
    )
