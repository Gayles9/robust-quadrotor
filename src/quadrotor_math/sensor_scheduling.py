from collections import deque
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True, eq=False)
class SensorMeasurement[MeasurementT: (float, NDArray[np.float64])]:
    """Store one sensor value with acquisition and fixed-delay delivery time.

    Times are elapsed simulation time in seconds. ``sequence_index`` is the
    zero-based acquisition order for one scheduler.
    """

    sequence_index: int
    acquisition_time_s: float
    delivery_time_s: float
    measurement: MeasurementT


class FixedRateSensorScheduler[MeasurementT: (float, NDArray[np.float64])]:
    """Acquire measurements on a fixed truth-grid stride and deliver them in order."""

    def __init__(
        self,
        *,
        sample_period_s: float,
        truth_time_step_s: float,
        delivery_delay_s: float = 0.0,
    ) -> None:
        """Initialize an elapsed-time sensor schedule.

        Args:
            sample_period_s: Positive acquisition period in seconds. It must
                align with a positive integer number of truth time steps.
            truth_time_step_s: Positive fixed truth-history interval in
                seconds.
            delivery_delay_s: Finite nonnegative fixed delivery delay in
                seconds.

        Raises:
            ValueError: If a scalar is non-finite, a period or truth time step
                is non-positive, the sample period is not aligned with the
                truth grid, or the delivery delay is negative.
        """
        if not np.isfinite(sample_period_s):
            raise ValueError("sample_period_s must be finite")

        if sample_period_s <= 0.0:
            raise ValueError("sample_period_s must be positive")

        if not np.isfinite(truth_time_step_s):
            raise ValueError("truth_time_step_s must be finite")

        if truth_time_step_s <= 0.0:
            raise ValueError("truth_time_step_s must be positive")

        sample_stride_ratio = sample_period_s / truth_time_step_s
        if not np.isfinite(sample_stride_ratio):
            raise ValueError("sample_period_s must be an integer multiple of truth_time_step_s")

        sample_stride = int(round(sample_stride_ratio))
        effective_sample_period_s = sample_stride * truth_time_step_s
        if sample_stride <= 0 or not np.isclose(
            sample_period_s,
            effective_sample_period_s,
            rtol=1e-12,
            atol=0.0,
        ):
            raise ValueError("sample_period_s must be an integer multiple of truth_time_step_s")

        if not np.isfinite(delivery_delay_s):
            raise ValueError("delivery_delay_s must be finite")

        if delivery_delay_s < 0.0:
            raise ValueError("delivery_delay_s must be nonnegative")

        self._truth_time_step_s = float(truth_time_step_s)
        self._sample_stride = sample_stride
        self._sample_period_s = float(effective_sample_period_s)
        self._delivery_delay_s = float(delivery_delay_s)
        self._next_sequence_index = 0
        self._last_update_time_s = 0.0
        self._pending_measurements: deque[SensorMeasurement[MeasurementT]] = deque()
        self._latest_delivered_measurement: SensorMeasurement[MeasurementT] | None = None

    @property
    def latest_delivered_measurement(self) -> SensorMeasurement[MeasurementT] | None:
        """Return an owned snapshot of the latest delivery, or ``None`` before delivery."""
        if self._latest_delivered_measurement is None:
            return None

        return _copy_sensor_measurement_record(self._latest_delivered_measurement)

    def update(
        self,
        current_time_s: float,
        acquire_measurement: Callable[[int, float], MeasurementT],
    ) -> tuple[SensorMeasurement[MeasurementT], ...]:
        """Acquire every due truth row and return newly delivered records.

        ``acquire_measurement`` receives the authoritative truth-history index
        and acquisition time in seconds. All scheduler-owned validation occurs
        before the first producer call.

        Args:
            current_time_s: Finite nonnegative elapsed simulation time in
                seconds. Calls must be monotonically nondecreasing within the
                scheduler time-comparison tolerance.
            acquire_measurement: Caller-owned acquisition function accepting a
                truth-history index and acquisition timestamp.

        Returns:
            Newly delivered measurements in ascending sequence order.

        Raises:
            ValueError: If the current time is non-finite, negative, or
                materially earlier than the last processed time, or if a due
                scheduled timestamp is non-finite.
        """
        if not np.isfinite(current_time_s):
            raise ValueError("current_time_s must be finite")

        if current_time_s < 0.0:
            raise ValueError("current_time_s must be nonnegative")

        monotonic_tolerance_s = _time_comparison_tolerance_s(
            current_time_s,
            self._last_update_time_s,
        )
        if current_time_s < self._last_update_time_s - monotonic_tolerance_s:
            raise ValueError("current_time_s must be monotonically nondecreasing")

        authoritative_current_time_s = max(current_time_s, self._last_update_time_s)
        due_timestamps: list[tuple[int, int, float, float]] = []
        sequence_index = self._next_sequence_index

        while True:
            truth_index = (sequence_index + 1) * self._sample_stride
            acquisition_time_s = truth_index * self._truth_time_step_s
            if not np.isfinite(acquisition_time_s):
                raise ValueError("scheduled acquisition_time_s must be finite")

            acquisition_tolerance_s = _time_comparison_tolerance_s(
                acquisition_time_s,
                authoritative_current_time_s,
            )
            if acquisition_time_s > authoritative_current_time_s + acquisition_tolerance_s:
                break

            delivery_time_s = acquisition_time_s + self._delivery_delay_s
            if not np.isfinite(delivery_time_s):
                raise ValueError("scheduled delivery_time_s must be finite")

            due_timestamps.append(
                (
                    sequence_index,
                    truth_index,
                    float(acquisition_time_s),
                    float(delivery_time_s),
                )
            )
            sequence_index += 1

        for (
            sequence_index,
            truth_index,
            acquisition_time_s,
            delivery_time_s,
        ) in due_timestamps:
            measurement = acquire_measurement(truth_index, acquisition_time_s)
            self._pending_measurements.append(
                SensorMeasurement(
                    sequence_index=sequence_index,
                    acquisition_time_s=acquisition_time_s,
                    delivery_time_s=delivery_time_s,
                    measurement=_copy_measurement_value(measurement),
                )
            )
            self._next_sequence_index = sequence_index + 1

        delivered_measurements: list[SensorMeasurement[MeasurementT]] = []
        while self._pending_measurements:
            pending_measurement = self._pending_measurements[0]
            delivery_tolerance_s = _time_comparison_tolerance_s(
                pending_measurement.delivery_time_s,
                authoritative_current_time_s,
            )
            if (
                pending_measurement.delivery_time_s
                > authoritative_current_time_s + delivery_tolerance_s
            ):
                break

            delivered_measurement = self._pending_measurements.popleft()
            self._latest_delivered_measurement = delivered_measurement
            delivered_measurements.append(_copy_sensor_measurement_record(delivered_measurement))

        self._last_update_time_s = authoritative_current_time_s
        return tuple(delivered_measurements)


def _time_comparison_tolerance_s(first_time_s: float, second_time_s: float) -> float:
    """Return the scale-aware float64 comparison tolerance for two times."""
    return float(16.0 * np.finfo(np.float64).eps * max(1.0, abs(first_time_s), abs(second_time_s)))


def _copy_measurement_value[
    MeasurementT: (float, NDArray[np.float64]),
](measurement: MeasurementT) -> MeasurementT:
    """Copy a vector measurement while preserving immutable scalar values."""
    if isinstance(measurement, np.ndarray):
        return measurement.copy()

    return measurement


def _copy_sensor_measurement_record[
    MeasurementT: (float, NDArray[np.float64]),
](record: SensorMeasurement[MeasurementT]) -> SensorMeasurement[MeasurementT]:
    """Return a new record whose measurement does not alias internal vector state."""
    return SensorMeasurement(
        sequence_index=record.sequence_index,
        acquisition_time_s=record.acquisition_time_s,
        delivery_time_s=record.delivery_time_s,
        measurement=_copy_measurement_value(record.measurement),
    )
