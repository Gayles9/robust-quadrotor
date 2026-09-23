"""Fault labels, causal timing, independent storage and unchanged estimator contracts."""

from dataclasses import replace

import numpy as np
import pytest

from quadrotor_math.eskf_faults import (
    EskfObservationFault,
    inject_eskf_observation_faults,
)
from quadrotor_math.eskf_innovation import chi_square_99_percent_eskf_innovation_policy
from quadrotor_math.eskf_replay import EskfObservationKind as Kind
from quadrotor_math.eskf_replay import EskfReplayStatus, replay_eskf
from quadrotor_math.eskf_synthetic import make_eskf_synthetic_case


@pytest.fixture
def case():
    return make_eskf_synthetic_case(10, family="stationary", number_of_steps=40, stochastic=False)


def test_empty_fault_plan_preserves_every_measurement_and_independent_memory(case):
    result = inject_eskf_observation_faults(case.measurements, ())
    assert len(result.records) == len(case.measurements.observations)
    for actual, original in zip(
        result.measurements.observations, case.measurements.observations, strict=True
    ):
        assert (
            actual.kind,
            actual.observation_index,
            actual.acquisition_index,
            actual.delivery_index,
        ) == (
            original.kind,
            original.observation_index,
            original.acquisition_index,
            original.delivery_index,
        )
        np.testing.assert_array_equal(actual.measurement, original.measurement)
        assert not np.shares_memory(actual.measurement, original.measurement)
        assert not actual.measurement.flags.writeable
    assert not np.shares_memory(
        result.measurements.specific_force_measurements_B,
        case.measurements.specific_force_measurements_B,
    )


def test_combined_faults_reindex_sources_and_deliver_in_causal_order(case):
    faults = (
        EskfObservationFault(Kind.LOCAL_POSITION, 0, dropout=True),
        EskfObservationFault(
            Kind.LOCAL_POSITION, 1, delay_steps=15, offset=np.array([1.0, 2.0, 3.0])
        ),
        EskfObservationFault(Kind.LOCAL_POSITION, 4, delay_steps=1),
    )
    injected = inject_eskf_observation_faults(case.measurements, faults)
    pos = [r for r in injected.records if r.source.kind is Kind.LOCAL_POSITION]
    assert pos[0].output is None
    assert [r.output.observation_index for r in pos[1:]] == [0, 1, 2, 3]
    assert pos[1].output.acquisition_index == 10
    assert pos[1].output.delivery_index == 25
    assert pos[-1].output.delivery_index == -1
    np.testing.assert_array_equal(pos[1].output.measurement, [1, 2, 3])
    result = replay_eskf(injected.measurements, case.configuration)
    statuses = [
        (e.observation.kind, e.observation.observation_index, e.status) for e in result.events
    ]
    assert (Kind.LOCAL_POSITION, 0, EskfReplayStatus.STALE) in statuses
    assert (Kind.LOCAL_POSITION, 3, EskfReplayStatus.PENDING) in statuses
    assert len(result.events) == len(case.measurements.observations) - 1


def test_delays_are_additive_and_pending_does_not_become_available(case):
    first = inject_eskf_observation_faults(
        case.measurements,
        (
            EskfObservationFault(Kind.LOCAL_POSITION, 0, delay_steps=3),
            EskfObservationFault(Kind.LOCAL_POSITION, 4, delay_steps=3),
        ),
    )
    second = inject_eskf_observation_faults(
        first.measurements,
        (
            EskfObservationFault(Kind.LOCAL_POSITION, 0, delay_steps=2),
            EskfObservationFault(Kind.LOCAL_POSITION, 4, delay_steps=2),
        ),
    )
    outputs = [r.output for r in second.records if r.source.kind is Kind.LOCAL_POSITION]
    assert outputs[0].delivery_index == 5
    assert outputs[-1].delivery_index == -1


@pytest.mark.parametrize("kind,width", [(Kind.LOCAL_POSITION, 3), (Kind.BAROMETRIC_ALTITUDE, 1)])
def test_large_outlier_is_rejected_without_truth_or_fault_labels(case, kind, width):
    injected = inject_eskf_observation_faults(
        case.measurements, (EskfObservationFault(kind, 1, offset=np.full(width, 100.0)),)
    )
    result = replay_eskf(
        injected.measurements,
        replace(
            case.configuration, innovation_policy=chi_square_99_percent_eskf_innovation_policy()
        ),
    )
    fault_event = next(
        e
        for e in result.events
        if e.observation.kind is kind and e.observation.observation_index == 1
    )
    assert fault_event.status is EskfReplayStatus.REJECTED
    assert fault_event.innovation.normalized_innovation_squared > fault_event.nis_threshold


@pytest.mark.parametrize(
    "change",
    [
        dict(kind=3),
        dict(observation_index=True),
        dict(observation_index=-1),
        dict(delay_steps=True),
        dict(delay_steps=-1),
        dict(delay_steps=1.5),
        dict(dropout=1),
        dict(dropout=True, delay_steps=1),
        dict(dropout=True, offset=np.zeros(3)),
        dict(offset=np.zeros(2)),
        dict(offset=np.full(3, np.nan)),
        dict(offset=np.full(3, np.inf)),
        dict(offset=np.ones(3, dtype=complex)),
        dict(offset=[1, 2, 3]),
    ],
)
def test_invalid_faults_fail_at_boundary(change):
    kwargs = dict(kind=Kind.LOCAL_POSITION, observation_index=0)
    kwargs.update(change)
    with pytest.raises((TypeError, ValueError)):
        EskfObservationFault(**kwargs)


def test_unknown_duplicate_and_wrong_fault_types_fail_atomically(case):
    fault = EskfObservationFault(Kind.LOCAL_POSITION, 0)
    before = case.measurements.specific_force_measurements_B.copy()
    for faults in ((fault, fault), (EskfObservationFault(Kind.LOCAL_POSITION, 99),), (object(),)):
        with pytest.raises((TypeError, ValueError)):
            inject_eskf_observation_faults(case.measurements, faults)
    np.testing.assert_array_equal(case.measurements.specific_force_measurements_B, before)


def test_offset_ownership_and_arithmetic_overflow(case):
    offset = np.ones(3)
    fault = EskfObservationFault(Kind.LOCAL_POSITION, 0, offset=offset)
    offset[:] = 12
    np.testing.assert_array_equal(fault.offset, np.ones(3))
    assert not fault.offset.flags.writeable
    observation = replace(case.measurements.observations[0], measurement=np.full(3, 1.7e308))
    data = replace(
        case.measurements, observations=(observation, *case.measurements.observations[1:])
    )
    with pytest.raises(ValueError, match="finite"):
        inject_eskf_observation_faults(data, (replace(fault, offset=np.full(3, 1.7e308)),))
    np.testing.assert_array_equal(observation.measurement, np.full(3, 1.7e308))


def test_fault_ledger_rejects_forged_missing_or_duplicate_records(case):
    injection = inject_eskf_observation_faults(case.measurements, ())
    first = injection.records[0]
    with pytest.raises(ValueError):
        replace(injection, records=injection.records[1:])
    with pytest.raises(ValueError):
        replace(injection, records=(first, *injection.records))
    with pytest.raises(ValueError):
        replace(
            injection,
            records=(
                replace(
                    first, output=replace(first.output, measurement=first.output.measurement + 1)
                ),
                *injection.records[1:],
            ),
        )
    with pytest.raises(ValueError):
        replace(
            injection,
            records=(
                replace(first, fault=replace(first.fault, delay_steps=1)),
                *injection.records[1:],
            ),
        )
    with pytest.raises(TypeError):
        replace(injection, records=(object(),))
    original = list(injection.records)
    owned = replace(injection, records=original)
    original.clear()
    assert len(owned.records) == len(injection.records)
