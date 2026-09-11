from dataclasses import FrozenInstanceError

import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.randomness import create_rng
from quadrotor_math.sensor_scheduling import (
    FixedRateSensorScheduler,
    SensorMeasurement,
)


@pytest.mark.parametrize("sample_period_s", [np.nan, np.inf, -np.inf])
def test_scheduler_rejects_non_finite_sample_period(sample_period_s: float) -> None:
    with pytest.raises(ValueError, match="sample_period_s must be finite"):
        FixedRateSensorScheduler[float](
            sample_period_s=sample_period_s,
            truth_time_step_s=0.1,
        )


@pytest.mark.parametrize("sample_period_s", [0.0, -0.1])
def test_scheduler_rejects_non_positive_sample_period(sample_period_s: float) -> None:
    with pytest.raises(ValueError, match="sample_period_s must be positive"):
        FixedRateSensorScheduler[float](
            sample_period_s=sample_period_s,
            truth_time_step_s=0.1,
        )


@pytest.mark.parametrize("truth_time_step_s", [np.nan, np.inf, -np.inf])
def test_scheduler_rejects_non_finite_truth_time_step(truth_time_step_s: float) -> None:
    with pytest.raises(ValueError, match="truth_time_step_s must be finite"):
        FixedRateSensorScheduler[float](
            sample_period_s=0.5,
            truth_time_step_s=truth_time_step_s,
        )


@pytest.mark.parametrize("truth_time_step_s", [0.0, -0.1])
def test_scheduler_rejects_non_positive_truth_time_step(truth_time_step_s: float) -> None:
    with pytest.raises(ValueError, match="truth_time_step_s must be positive"):
        FixedRateSensorScheduler[float](
            sample_period_s=0.5,
            truth_time_step_s=truth_time_step_s,
        )


def test_scheduler_rejects_sample_period_not_aligned_to_truth_grid() -> None:
    with pytest.raises(
        ValueError,
        match="sample_period_s must be an integer multiple of truth_time_step_s",
    ):
        FixedRateSensorScheduler[float](
            sample_period_s=0.25,
            truth_time_step_s=0.1,
        )


@pytest.mark.parametrize("delivery_delay_s", [np.nan, np.inf, -np.inf])
def test_scheduler_rejects_non_finite_delivery_delay(delivery_delay_s: float) -> None:
    with pytest.raises(ValueError, match="delivery_delay_s must be finite"):
        FixedRateSensorScheduler[float](
            sample_period_s=0.5,
            truth_time_step_s=0.1,
            delivery_delay_s=delivery_delay_s,
        )


def test_scheduler_rejects_negative_delivery_delay() -> None:
    with pytest.raises(ValueError, match="delivery_delay_s must be nonnegative"):
        FixedRateSensorScheduler[float](
            sample_period_s=0.5,
            truth_time_step_s=0.1,
            delivery_delay_s=-0.1,
        )


@pytest.mark.parametrize(
    ("sample_period_s", "truth_time_step_s", "delivery_delay_s", "expected_message"),
    [
        (np.nan, np.nan, np.nan, "sample_period_s must be finite"),
        (0.0, np.nan, np.nan, "sample_period_s must be positive"),
        (0.5, np.nan, np.nan, "truth_time_step_s must be finite"),
        (0.5, 0.0, np.nan, "truth_time_step_s must be positive"),
        (
            0.25,
            0.1,
            np.nan,
            "sample_period_s must be an integer multiple of truth_time_step_s",
        ),
        (0.5, 0.1, np.nan, "delivery_delay_s must be finite"),
        (0.5, 0.1, -0.1, "delivery_delay_s must be nonnegative"),
    ],
)
def test_scheduler_constructor_uses_declared_validation_order(
    sample_period_s: float,
    truth_time_step_s: float,
    delivery_delay_s: float,
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        FixedRateSensorScheduler[float](
            sample_period_s=sample_period_s,
            truth_time_step_s=truth_time_step_s,
            delivery_delay_s=delivery_delay_s,
        )


def test_sensor_measurement_is_frozen_slotted_and_uses_identity_equality() -> None:
    measurement = SensorMeasurement[float](
        sequence_index=0,
        acquisition_time_s=0.5,
        delivery_time_s=0.5,
        measurement=5.0,
    )
    equal_fields = SensorMeasurement[float](
        sequence_index=0,
        acquisition_time_s=0.5,
        delivery_time_s=0.5,
        measurement=5.0,
    )

    assert not hasattr(measurement, "__dict__")
    assert measurement != equal_fields
    with pytest.raises(FrozenInstanceError):
        measurement.sequence_index = 1  # type: ignore[misc]


def test_scheduler_initially_has_no_latest_delivered_measurement() -> None:
    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )

    assert scheduler.latest_delivered_measurement is None


def test_scheduler_does_not_acquire_at_time_zero() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return float(truth_index)

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )

    delivered = scheduler.update(0.0, acquire_measurement)

    assert delivered == ()
    assert producer_calls == []
    assert scheduler.latest_delivered_measurement is None


def test_scheduler_does_not_acquire_clearly_before_first_boundary() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return float(truth_index)

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )

    delivered = scheduler.update(0.49, acquire_measurement)

    assert delivered == ()
    assert producer_calls == []
    assert scheduler.latest_delivered_measurement is None


def test_scheduler_acquires_and_delivers_sequence_zero_at_first_period() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return 12.5

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )

    delivered = scheduler.update(0.5, acquire_measurement)

    assert producer_calls == [(5, 0.5)]
    assert len(delivered) == 1
    assert delivered[0].sequence_index == 0
    assert delivered[0].acquisition_time_s == 0.5
    assert delivered[0].delivery_time_s == 0.5
    assert delivered[0].measurement == 12.5
    latest = scheduler.latest_delivered_measurement
    assert latest is not None
    assert latest is not delivered[0]
    assert latest.sequence_index == delivered[0].sequence_index
    assert latest.acquisition_time_s == delivered[0].acquisition_time_s
    assert latest.delivery_time_s == delivered[0].delivery_time_s
    assert latest.measurement == delivered[0].measurement


def test_scheduler_canonicalizes_aligned_period_to_integer_truth_stride() -> None:
    configured_period_s = 0.3 * (1.0 + 0.5e-12)
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return acquisition_time_s

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=configured_period_s,
        truth_time_step_s=0.1,
    )

    delivered = scheduler.update(0.3, acquire_measurement)

    expected_acquisition_time_s = 3 * 0.1
    assert producer_calls == [(3, expected_acquisition_time_s)]
    assert delivered[0].acquisition_time_s == expected_acquisition_time_s


def test_scheduler_repeated_equal_time_update_is_idempotent() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return float(truth_index)

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )

    first_delivered = scheduler.update(0.5, acquire_measurement)
    repeated_delivered = scheduler.update(0.5, acquire_measurement)

    assert len(first_delivered) == 1
    assert repeated_delivered == ()
    assert producer_calls == [(5, 0.5)]
    latest = scheduler.latest_delivered_measurement
    assert latest is not None
    assert latest is not first_delivered[0]
    assert latest.sequence_index == first_delivered[0].sequence_index
    assert latest.acquisition_time_s == first_delivered[0].acquisition_time_s
    assert latest.delivery_time_s == first_delivered[0].delivery_time_s
    assert latest.measurement == first_delivered[0].measurement


def test_scheduler_catches_up_every_crossed_truth_row_in_sequence_order() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return 100.0 + truth_index

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.2,
        truth_time_step_s=0.1,
    )

    delivered = scheduler.update(0.6, acquire_measurement)

    expected_truth_indices = [2, 4, 6]
    expected_acquisition_times_s = [truth_index * 0.1 for truth_index in expected_truth_indices]
    assert producer_calls == list(
        zip(expected_truth_indices, expected_acquisition_times_s, strict=True)
    )
    assert [record.sequence_index for record in delivered] == [0, 1, 2]
    assert [record.acquisition_time_s for record in delivered] == expected_acquisition_times_s
    assert [record.delivery_time_s for record in delivered] == expected_acquisition_times_s
    assert [record.measurement for record in delivered] == [102.0, 104.0, 106.0]
    latest = scheduler.latest_delivered_measurement
    assert latest is not None
    assert latest is not delivered[-1]
    assert latest.sequence_index == delivered[-1].sequence_index
    assert latest.acquisition_time_s == delivered[-1].acquisition_time_s
    assert latest.delivery_time_s == delivered[-1].delivery_time_s
    assert latest.measurement == delivered[-1].measurement


def test_scheduler_treats_time_immediately_below_boundary_within_tolerance_as_due() -> None:
    boundary_time_s = 0.5
    tolerance_s = (
        16.0 * np.finfo(np.float64).eps * max(1.0, abs(boundary_time_s), abs(boundary_time_s))
    )
    current_time_s = boundary_time_s - 0.5 * tolerance_s
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return acquisition_time_s

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )

    delivered = scheduler.update(current_time_s, acquire_measurement)

    assert producer_calls == [(5, 0.5)]
    assert len(delivered) == 1
    assert delivered[0].acquisition_time_s == 0.5


@pytest.mark.parametrize(
    ("current_time_s", "expected_message"),
    [
        (np.nan, "current_time_s must be finite"),
        (np.inf, "current_time_s must be finite"),
        (-np.inf, "current_time_s must be finite"),
        (-0.1, "current_time_s must be nonnegative"),
    ],
)
def test_scheduler_rejects_invalid_update_time_before_calling_producer(
    current_time_s: float,
    expected_message: str,
) -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return float(truth_index)

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )

    with pytest.raises(ValueError, match=expected_message):
        scheduler.update(current_time_s, acquire_measurement)

    assert producer_calls == []
    assert scheduler.latest_delivered_measurement is None


def test_scheduler_rejects_materially_backward_time_before_calling_producer() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return float(truth_index)

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )
    first_delivered = scheduler.update(0.5, acquire_measurement)

    with pytest.raises(
        ValueError,
        match="current_time_s must be monotonically nondecreasing",
    ):
        scheduler.update(0.25, acquire_measurement)

    assert producer_calls == [(5, 0.5)]
    latest = scheduler.latest_delivered_measurement
    assert latest is not None
    assert latest is not first_delivered[0]
    assert latest.sequence_index == first_delivered[0].sequence_index
    assert latest.acquisition_time_s == first_delivered[0].acquisition_time_s
    assert latest.delivery_time_s == first_delivered[0].delivery_time_s
    assert latest.measurement == first_delivered[0].measurement


def test_scheduler_tiny_backward_update_does_not_decrease_authoritative_time() -> None:
    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=2.0,
        truth_time_step_s=0.1,
    )
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return float(truth_index)

    last_time_s = 1.0
    tolerance_s = 16.0 * np.finfo(np.float64).eps
    scheduler.update(last_time_s, acquire_measurement)

    accepted_time_s = last_time_s - 0.5 * tolerance_s
    assert scheduler.update(accepted_time_s, acquire_measurement) == ()

    rejected_time_s = last_time_s - 1.25 * tolerance_s
    with pytest.raises(
        ValueError,
        match="current_time_s must be monotonically nondecreasing",
    ):
        scheduler.update(rejected_time_s, acquire_measurement)

    assert producer_calls == []


def test_scheduler_rejects_non_finite_delivery_timestamp_before_producer() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return float(truth_index)

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=1e308,
        truth_time_step_s=1e308,
        delivery_delay_s=1e308,
    )

    with pytest.raises(ValueError, match="scheduled delivery_time_s must be finite"):
        scheduler.update(1e308, acquire_measurement)

    assert producer_calls == []
    assert scheduler.latest_delivered_measurement is None


def test_invalid_update_preserves_observable_scheduler_state() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return float(truth_index)

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )
    first_delivered = scheduler.update(0.5, acquire_measurement)

    with pytest.raises(
        ValueError,
        match="current_time_s must be monotonically nondecreasing",
    ):
        scheduler.update(0.25, acquire_measurement)

    delivered_after_rejection = scheduler.update(1.5, acquire_measurement)

    assert producer_calls == [(5, 0.5), (10, 1.0), (15, 1.5)]
    assert first_delivered[0].sequence_index == 0
    assert [record.sequence_index for record in delivered_after_rejection] == [1, 2]
    assert [record.measurement for record in delivered_after_rejection] == [10.0, 15.0]
    latest = scheduler.latest_delivered_measurement
    assert latest is not None
    assert latest is not delivered_after_rejection[-1]
    assert latest.sequence_index == 2
    assert latest.acquisition_time_s == delivered_after_rejection[-1].acquisition_time_s
    assert latest.delivery_time_s == delivered_after_rejection[-1].delivery_time_s
    assert latest.measurement == delivered_after_rejection[-1].measurement


def test_positive_delay_acquires_without_immediate_delivery() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return 12.5

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
        delivery_delay_s=0.25,
    )

    delivered = scheduler.update(0.5, acquire_measurement)

    assert delivered == ()
    assert producer_calls == [(5, 0.5)]
    assert scheduler.latest_delivered_measurement is None


def test_positive_delay_delivers_float_at_exact_nominal_time() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return 12.5

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
        delivery_delay_s=0.25,
    )
    assert scheduler.update(0.5, acquire_measurement) == ()

    delivered = scheduler.update(0.75, acquire_measurement)

    assert producer_calls == [(5, 0.5)]
    assert len(delivered) == 1
    assert delivered[0].sequence_index == 0
    assert delivered[0].acquisition_time_s == 0.5
    assert delivered[0].delivery_time_s == 0.75
    assert isinstance(delivered[0].measurement, float)
    assert delivered[0].measurement == 12.5
    latest = scheduler.latest_delivered_measurement
    assert latest is not None
    assert latest is not delivered[0]
    assert latest.sequence_index == delivered[0].sequence_index
    assert latest.acquisition_time_s == delivered[0].acquisition_time_s
    assert latest.delivery_time_s == delivered[0].delivery_time_s
    assert latest.measurement == delivered[0].measurement


def test_positive_delay_delivers_immediately_below_boundary_within_tolerance() -> None:
    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
        delivery_delay_s=0.25,
    )
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return 12.5

    assert scheduler.update(0.5, acquire_measurement) == ()
    delivery_time_s = 0.75
    tolerance_s = 16.0 * np.finfo(np.float64).eps

    delivered = scheduler.update(delivery_time_s - 0.5 * tolerance_s, acquire_measurement)

    assert producer_calls == [(5, 0.5)]
    assert len(delivered) == 1
    assert delivered[0].delivery_time_s == delivery_time_s


def test_positive_delay_does_not_deliver_clearly_before_boundary() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return 12.5

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
        delivery_delay_s=0.25,
    )
    assert scheduler.update(0.5, acquire_measurement) == ()

    delivered = scheduler.update(0.74, acquire_measurement)

    assert delivered == ()
    assert producer_calls == [(5, 0.5)]
    assert scheduler.latest_delivered_measurement is None


def test_update_releases_multiple_delayed_records_in_sequence_order() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return float(truth_index)

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.2,
        truth_time_step_s=0.1,
        delivery_delay_s=0.5,
    )
    assert scheduler.update(0.6, acquire_measurement) == ()

    delivered = scheduler.update(1.1, acquire_measurement)

    expected_truth_indices = [2, 4, 6, 8, 10]
    expected_acquisition_times_s = [truth_index * 0.1 for truth_index in expected_truth_indices]
    assert producer_calls == list(
        zip(expected_truth_indices, expected_acquisition_times_s, strict=True)
    )
    assert [record.sequence_index for record in delivered] == [0, 1, 2]
    assert [record.acquisition_time_s for record in delivered] == expected_acquisition_times_s[:3]
    assert [record.delivery_time_s for record in delivered] == [0.7, 0.9, 1.1]
    assert [record.measurement for record in delivered] == [2.0, 4.0, 6.0]
    latest = scheduler.latest_delivered_measurement
    assert latest is not None
    assert latest.sequence_index == 2


def test_update_completes_acquisition_catch_up_before_releasing_deliveries() -> None:
    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.25,
        truth_time_step_s=0.05,
        delivery_delay_s=0.5,
    )
    latest_seen_during_acquisition: list[SensorMeasurement[float] | None] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        latest_seen_during_acquisition.append(scheduler.latest_delivered_measurement)
        return float(truth_index)

    assert scheduler.update(0.25, acquire_measurement) == ()

    delivered = scheduler.update(0.75, acquire_measurement)

    assert latest_seen_during_acquisition == [None, None, None]
    assert [record.sequence_index for record in delivered] == [0]
    latest = scheduler.latest_delivered_measurement
    assert latest is not None
    assert latest.sequence_index == 0


def test_latest_changes_only_when_a_newer_measurement_is_delivered() -> None:
    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
        delivery_delay_s=0.25,
    )

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        return float(truth_index)

    assert scheduler.update(0.5, acquire_measurement) == ()
    first_delivery = scheduler.update(0.75, acquire_measurement)
    assert first_delivery[0].sequence_index == 0

    assert scheduler.update(1.0, acquire_measurement) == ()
    latest_after_pending_acquisition = scheduler.latest_delivered_measurement
    assert latest_after_pending_acquisition is not None
    assert latest_after_pending_acquisition.sequence_index == 0

    assert scheduler.update(1.1, acquire_measurement) == ()
    latest_after_idle_update = scheduler.latest_delivered_measurement
    assert latest_after_idle_update is not None
    assert latest_after_idle_update.sequence_index == 0

    second_delivery = scheduler.update(1.25, acquire_measurement)
    assert [record.sequence_index for record in second_delivery] == [1]
    latest_after_second_delivery = scheduler.latest_delivered_measurement
    assert latest_after_second_delivery is not None
    assert latest_after_second_delivery.sequence_index == 1
    assert latest_after_second_delivery.measurement == 10.0


def test_delivery_only_update_does_not_call_producer() -> None:
    producer_calls: list[tuple[int, float]] = []

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        producer_calls.append((truth_index, acquisition_time_s))
        return 12.5

    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=1.0,
        truth_time_step_s=0.1,
        delivery_delay_s=0.25,
    )
    assert scheduler.update(1.0, acquire_measurement) == ()

    delivered = scheduler.update(1.25, acquire_measurement)

    assert len(delivered) == 1
    assert producer_calls == [(10, 1.0)]


def test_pending_ndarray_storage_is_independent_of_producer_array() -> None:
    producer_array = np.array([1.0, -2.0, 3.0], dtype=np.float64)
    expected_measurement = producer_array.copy()
    scheduler = FixedRateSensorScheduler[NDArray[np.float64]](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
        delivery_delay_s=0.25,
    )

    def acquire_measurement(
        truth_index: int,
        acquisition_time_s: float,
    ) -> NDArray[np.float64]:
        return producer_array

    assert scheduler.update(0.5, acquire_measurement) == ()
    producer_array[:] = 99.0

    delivered = scheduler.update(0.75, acquire_measurement)

    np.testing.assert_array_equal(delivered[0].measurement, expected_measurement)
    assert not np.shares_memory(delivered[0].measurement, producer_array)


def test_returned_ndarray_delivery_is_independent_of_producer_and_held_state() -> None:
    producer_array = np.array([1.0, -2.0, 3.0], dtype=np.float64)
    expected_measurement = producer_array.copy()
    scheduler = FixedRateSensorScheduler[NDArray[np.float64]](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
        delivery_delay_s=0.25,
    )

    def acquire_measurement(
        truth_index: int,
        acquisition_time_s: float,
    ) -> NDArray[np.float64]:
        return producer_array

    assert scheduler.update(0.5, acquire_measurement) == ()
    delivered = scheduler.update(0.75, acquire_measurement)
    latest = scheduler.latest_delivered_measurement

    assert latest is not None
    assert delivered[0] is not latest
    assert not np.shares_memory(delivered[0].measurement, producer_array)
    assert not np.shares_memory(delivered[0].measurement, latest.measurement)
    assert delivered[0].measurement.shape == (3,)
    assert delivered[0].measurement.dtype == np.float64
    np.testing.assert_array_equal(delivered[0].measurement, expected_measurement)
    np.testing.assert_array_equal(latest.measurement, expected_measurement)


def test_mutating_returned_ndarray_delivery_does_not_change_later_latest_snapshot() -> None:
    expected_measurement = np.array([1.0, -2.0, 3.0], dtype=np.float64)
    scheduler = FixedRateSensorScheduler[NDArray[np.float64]](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )

    def acquire_measurement(
        truth_index: int,
        acquisition_time_s: float,
    ) -> NDArray[np.float64]:
        return expected_measurement

    delivered = scheduler.update(0.5, acquire_measurement)
    delivered[0].measurement[:] = 99.0

    latest = scheduler.latest_delivered_measurement

    assert latest is not None
    np.testing.assert_array_equal(latest.measurement, np.array([1.0, -2.0, 3.0]))


def test_repeated_latest_ndarray_snapshots_have_independent_storage() -> None:
    expected_measurement = np.array([1.0, -2.0, 3.0], dtype=np.float64)
    scheduler = FixedRateSensorScheduler[NDArray[np.float64]](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )

    def acquire_measurement(
        truth_index: int,
        acquisition_time_s: float,
    ) -> NDArray[np.float64]:
        return expected_measurement

    scheduler.update(0.5, acquire_measurement)
    first_latest = scheduler.latest_delivered_measurement
    second_latest = scheduler.latest_delivered_measurement

    assert first_latest is not None
    assert second_latest is not None
    assert first_latest is not second_latest
    assert not np.shares_memory(first_latest.measurement, second_latest.measurement)
    first_latest.measurement[:] = 99.0
    np.testing.assert_array_equal(second_latest.measurement, expected_measurement)
    third_latest = scheduler.latest_delivered_measurement
    assert third_latest is not None
    np.testing.assert_array_equal(third_latest.measurement, expected_measurement)


def test_rng_draw_occurs_when_measurement_is_acquired() -> None:
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)
    expected_acquired_value = float(reference_rng.standard_normal())
    expected_next_value = float(reference_rng.standard_normal())
    acquired_values: list[float] = []
    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
        delivery_delay_s=0.25,
    )

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        measurement = float(measurement_rng.standard_normal())
        acquired_values.append(measurement)
        return measurement

    assert scheduler.update(0.5, acquire_measurement) == ()

    assert acquired_values == [expected_acquired_value]
    assert float(measurement_rng.standard_normal()) == expected_next_value


def test_delivery_only_update_does_not_advance_rng() -> None:
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)
    expected_acquired_value = float(reference_rng.standard_normal())
    expected_next_value = float(reference_rng.standard_normal())
    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=1.0,
        truth_time_step_s=0.1,
        delivery_delay_s=0.25,
    )

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        return float(measurement_rng.standard_normal())

    assert scheduler.update(1.0, acquire_measurement) == ()
    delivered = scheduler.update(1.25, acquire_measurement)

    assert delivered[0].measurement == expected_acquired_value
    assert float(measurement_rng.standard_normal()) == expected_next_value


def test_rejected_scheduler_update_preserves_rng_state() -> None:
    seed = 12345
    measurement_rng = create_rng(seed)
    reference_rng = create_rng(seed)
    expected_first_value = float(reference_rng.standard_normal())
    producer_call_count = 0
    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        nonlocal producer_call_count
        producer_call_count += 1
        return float(measurement_rng.standard_normal())

    with pytest.raises(ValueError, match="current_time_s must be nonnegative"):
        scheduler.update(-0.1, acquire_measurement)

    delivered = scheduler.update(0.5, acquire_measurement)

    assert producer_call_count == 1
    assert delivered[0].measurement == expected_first_value


def test_equal_seed_schedulers_replay_values_and_timestamp_records() -> None:
    def run_schedule(seed: int) -> tuple[list[float], list[tuple[int, float, float, float]]]:
        measurement_rng = create_rng(seed)
        acquired_values: list[float] = []
        delivered_records: list[tuple[int, float, float, float]] = []
        scheduler = FixedRateSensorScheduler[float](
            sample_period_s=0.5,
            truth_time_step_s=0.1,
            delivery_delay_s=0.25,
        )

        def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
            measurement = float(measurement_rng.standard_normal())
            acquired_values.append(measurement)
            return measurement

        for current_time_s in [0.0, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75]:
            for record in scheduler.update(current_time_s, acquire_measurement):
                delivered_records.append(
                    (
                        record.sequence_index,
                        record.acquisition_time_s,
                        record.delivery_time_s,
                        record.measurement,
                    )
                )

        return acquired_values, delivered_records

    first_values, first_records = run_schedule(12345)
    second_values, second_records = run_schedule(12345)

    assert first_values == second_values
    assert first_records == second_records


def test_different_fixed_delays_preserve_acquisition_rng_sequence() -> None:
    zero_delay_rng = create_rng(12345)
    positive_delay_rng = create_rng(12345)
    zero_delay_acquisitions: list[tuple[int, float]] = []
    positive_delay_acquisitions: list[tuple[int, float]] = []
    zero_delay_scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
    )
    positive_delay_scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
        delivery_delay_s=1.0,
    )

    def acquire_zero_delay(truth_index: int, acquisition_time_s: float) -> float:
        measurement = float(zero_delay_rng.standard_normal())
        zero_delay_acquisitions.append((truth_index, measurement))
        return measurement

    def acquire_positive_delay(truth_index: int, acquisition_time_s: float) -> float:
        measurement = float(positive_delay_rng.standard_normal())
        positive_delay_acquisitions.append((truth_index, measurement))
        return measurement

    for current_time_s in [0.5, 1.0, 1.5]:
        zero_delay_scheduler.update(current_time_s, acquire_zero_delay)
        positive_delay_scheduler.update(current_time_s, acquire_positive_delay)

    assert zero_delay_acquisitions == positive_delay_acquisitions
    assert len(zero_delay_acquisitions) == 3


def test_incremental_and_catch_up_updates_preserve_acquisition_sequence() -> None:
    incremental_rng = create_rng(12345)
    catch_up_rng = create_rng(12345)
    incremental_acquisitions: list[tuple[int, float]] = []
    catch_up_acquisitions: list[tuple[int, float]] = []
    incremental_batches: list[tuple[SensorMeasurement[float], ...]] = []
    incremental_scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.2,
        truth_time_step_s=0.1,
    )
    catch_up_scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.2,
        truth_time_step_s=0.1,
    )

    def acquire_incrementally(truth_index: int, acquisition_time_s: float) -> float:
        measurement = float(incremental_rng.standard_normal())
        incremental_acquisitions.append((truth_index, measurement))
        return measurement

    def acquire_during_catch_up(truth_index: int, acquisition_time_s: float) -> float:
        measurement = float(catch_up_rng.standard_normal())
        catch_up_acquisitions.append((truth_index, measurement))
        return measurement

    for current_time_s in [0.2, 0.4, 0.6]:
        incremental_batches.append(
            incremental_scheduler.update(current_time_s, acquire_incrementally)
        )
    catch_up_batch = catch_up_scheduler.update(0.6, acquire_during_catch_up)

    assert incremental_acquisitions == catch_up_acquisitions
    assert len(incremental_acquisitions) == 3
    assert [len(batch) for batch in incremental_batches] == [1, 1, 1]
    assert len(catch_up_batch) == 3
    assert [batch[0].sequence_index for batch in incremental_batches] == [0, 1, 2]
    assert [record.sequence_index for record in catch_up_batch] == [0, 1, 2]
    assert [batch[0].measurement for batch in incremental_batches] == [
        record.measurement for record in catch_up_batch
    ]


def test_pending_measurement_is_not_implicitly_delivered_at_simulation_end() -> None:
    producer_call_count = 0
    scheduler = FixedRateSensorScheduler[float](
        sample_period_s=0.5,
        truth_time_step_s=0.1,
        delivery_delay_s=1.0,
    )

    def acquire_measurement(truth_index: int, acquisition_time_s: float) -> float:
        nonlocal producer_call_count
        producer_call_count += 1
        return 12.5

    assert scheduler.update(0.5, acquire_measurement) == ()

    delivered_at_simulation_end = scheduler.update(0.75, acquire_measurement)

    assert delivered_at_simulation_end == ()
    assert producer_call_count == 1
    assert scheduler.latest_delivered_measurement is None
