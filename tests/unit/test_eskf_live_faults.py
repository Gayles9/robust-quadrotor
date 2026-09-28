"""Live fault causality, offline reconciliation and unchanged public mission data."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.estimated_feedback_evidence import pack_history
from experiments.estimated_feedback_validation import make_configuration
from quadrotor_math.eskf_faults import EskfObservationFault, inject_eskf_observation_faults
from quadrotor_math.eskf_live_faults import EskfLiveObservationFaults
from quadrotor_math.eskf_replay import (
    EskfObservationKind as Kind,
)
from quadrotor_math.eskf_replay import EskfReplayInput, EskfReplayObservation, replay_eskf
from quadrotor_math.estimated_mission import simulate_estimated_mission
from quadrotor_math.run_configuration import SensorSchedule


def source():
    times = np.arange(13) * 0.01
    observations = tuple(
        EskfReplayObservation(kind, i, k, -1 if k == 12 else k, np.full(width, k * 0.01))
        for kind, stride, width in ((Kind.LOCAL_POSITION, 2, 3), (Kind.BAROMETRIC_ALTITUDE, 3, 1))
        for i, k in enumerate(range(stride, 13, stride))
    )
    return EskfReplayInput(times, np.tile([0, 0, -9.81], (13, 1)), np.zeros((13, 3)), observations)


def drive(channel, data):
    deliveries = []
    for k, time in enumerate(data.time_s):
        deliveries.append(
            channel.step(float(time), tuple(o for o in data.observations if o.delivery_index == k))
        )
    return deliveries


def assert_observations(left, right):
    assert len(left) == len(right)
    for a, b in zip(left, right, strict=True):
        assert (a.kind, a.observation_index, a.acquisition_index, a.delivery_index) == (
            b.kind,
            b.observation_index,
            b.acquisition_index,
            b.delivery_index,
        )
        np.testing.assert_array_equal(a.measurement, b.measurement)


@pytest.mark.parametrize("kind", list(Kind))
@pytest.mark.parametrize("mode", ["empty", "dropout", "delay", "offset", "pending", "unreached"])
def test_live_delivery_and_exhaustive_ledger_match_pure_offline_injection(kind, mode):
    index = (
        50
        if mode == "unreached"
        else (5 if kind is Kind.LOCAL_POSITION else 3)
        if mode == "pending"
        else 1
    )
    mutation = {
        "dropout": {"dropout": True},
        "delay": {"delay_steps": 4},
        "offset": {"offset": np.full(3 if kind is Kind.LOCAL_POSITION else 1, 5)},
        "pending": {"delay_steps": 10},
        "unreached": {"dropout": True},
        "empty": {},
    }[mode]
    faults = () if mode == "empty" else (EskfObservationFault(kind, index, **mutation),)
    data = source()
    channel = EskfLiveObservationFaults(faults)
    emitted = drive(channel, data)
    actual = channel.finish(data)
    expected = inject_eskf_observation_faults(data, () if mode == "unreached" else faults)
    assert_observations(actual.measurements.observations, expected.measurements.observations)
    assert len(actual.records) == len(data.observations)
    assert channel.injection is actual
    assert_observations(channel.delivered, tuple(o for rows in emitted for o in rows))
    for k, rows in enumerate(emitted):
        assert all(o.delivery_index == k and o.acquisition_index <= k for o in rows)
    with pytest.raises(ValueError, match="finished"):
        channel.step(0.13)
    with pytest.raises(ValueError, match="finished"):
        channel.finish(data)
    channel.reset()
    repeated = drive(channel, data)
    for a, b in zip(emitted, repeated, strict=True):
        assert_observations(a, b)


def test_delayed_arrival_keeps_acquisition_identity_after_newer_delivery():
    data = source()
    channel = EskfLiveObservationFaults(
        (EskfObservationFault(Kind.LOCAL_POSITION, 0, delay_steps=5),)
    )
    output = drive(channel, data)
    assert [o.observation_index for o in output[4] if o.kind is Kind.LOCAL_POSITION] == [1]
    assert [(o.observation_index, o.acquisition_index) for o in output[7]] == [(0, 2)]
    channel.finish(data)


def test_future_fault_cannot_change_emitted_prefix_or_rng_input():
    data = source()
    nominal = drive(EskfLiveObservationFaults(), data)
    faulted = drive(
        EskfLiveObservationFaults((EskfObservationFault(Kind.LOCAL_POSITION, 3, dropout=True),)),
        data,
    )
    for a, b in zip(nominal[:8], faulted[:8], strict=True):
        assert_observations(a, b)
    assert any(o.kind is Kind.LOCAL_POSITION for o in nominal[8])
    assert not any(o.kind is Kind.LOCAL_POSITION for o in faulted[8])


@pytest.mark.parametrize("bad", [True, -1, np.nan, np.inf, "0"])
def test_invalid_clocks_are_atomic(bad):
    channel = EskfLiveObservationFaults()
    with pytest.raises((ValueError, TypeError)):
        channel.step(bad)
    assert channel.epoch_count == 0
    assert channel.step(0) == ()
    with pytest.raises(ValueError):
        channel.step(0)
    assert channel.epoch_count == 1


@pytest.mark.parametrize("bad", ["pending", "future", "skipped_id", "reverse", "list", "type"])
def test_invalid_arrivals_consume_no_identity_or_clock(bad):
    channel = EskfLiveObservationFaults()
    p = EskfReplayObservation(Kind.LOCAL_POSITION, 0, 0, 0, np.zeros(3))
    a = EskfReplayObservation(Kind.BAROMETRIC_ALTITUDE, 0, 0, 0, np.zeros(1))
    arrivals = {
        "pending": (replace(p, delivery_index=-1),),
        "future": (replace(p, delivery_index=1),),
        "skipped_id": (replace(p, observation_index=1),),
        "reverse": (a, p),
        "list": [p],
        "type": (True,),
    }[bad]
    with pytest.raises((ValueError, TypeError)):
        channel.step(0, arrivals)
    assert channel.epoch_count == 0 and channel.delivered == ()
    assert len(channel.step(0, (p, a))) == 2


def test_overflow_rolls_back_already_prepared_arrivals():
    channel = EskfLiveObservationFaults(
        (EskfObservationFault(Kind.BAROMETRIC_ALTITUDE, 0, offset=np.array([1e308])),)
    )
    p = EskfReplayObservation(Kind.LOCAL_POSITION, 0, 0, 0, np.zeros(3))
    a = EskfReplayObservation(Kind.BAROMETRIC_ALTITUDE, 0, 0, 0, np.array([1e308]))
    with pytest.raises(ValueError, match="finite"):
        channel.step(0, (p, a))
    assert channel.epoch_count == 0 and channel.delivered == ()
    assert len(channel.step(0, (p, replace(a, measurement=np.zeros(1))))) == 2


def test_finish_rejects_altered_source_and_allows_atomic_retry():
    data = source()
    channel = EskfLiveObservationFaults()
    drive(channel, data)
    bad = replace(
        data,
        observations=(
            replace(data.observations[0], measurement=np.ones(3)),
            *data.observations[1:],
        ),
    )
    with pytest.raises(ValueError, match="source ledger"):
        channel.finish(bad)
    assert channel.injection is None
    channel.finish(data)


@pytest.mark.parametrize(
    "faults", [[True], (True,), (EskfObservationFault(Kind.LOCAL_POSITION, 0),) * 2]
)
def test_invalid_fault_plan_rejected(faults):
    with pytest.raises((ValueError, TypeError)):
        EskfLiveObservationFaults(faults)


def inputs():
    args = make_configuration({"case": "smoke", "seed": 31, "noiseless": False})
    args["plan"] = replace(
        args["plan"], segments=tuple(replace(s, duration_s=0.1) for s in args["plan"].segments)
    )
    args["sensors"] = replace(
        args["sensors"],
        local_position_schedule=SensorSchedule(0.02, 0.003),
        barometric_altitude_schedule=SensorSchedule(0.01, 0),
    )
    return args


def test_empty_live_channel_preserves_every_existing_payload_byte():
    args = inputs()
    baseline = simulate_estimated_mission(**args)
    channel = EskfLiveObservationFaults()
    result = simulate_estimated_mission(**args, observation_faults=channel)
    before, after = pack_history(baseline, baseline.mission), pack_history(result, result.mission)
    assert before.keys() == after.keys()
    assert all(before[k].tobytes() == after[k].tobytes() for k in before)


def test_real_mission_dropout_delay_offset_and_pending_replay_exactly():
    args = inputs()
    faults = (
        EskfObservationFault(Kind.LOCAL_POSITION, 0, dropout=True),
        EskfObservationFault(Kind.LOCAL_POSITION, 1, delay_steps=10),
        EskfObservationFault(Kind.BAROMETRIC_ALTITUDE, 5, offset=np.array([5.0])),
        EskfObservationFault(Kind.LOCAL_POSITION, 5, delay_steps=1000),
    )
    channel = EskfLiveObservationFaults(faults)
    result = simulate_estimated_mission(**args, observation_faults=channel)
    replay = replay_eskf(result.measurements, args["estimator_configuration"])
    np.testing.assert_array_equal(replay.covariances, result.estimates.covariances)
    assert_observations(
        tuple(e.observation for e in replay.events), result.measurements.observations
    )
    assert [e.status for e in replay.events] == [e.status for e in result.estimates.events]
    assert len(channel.injection.records) == len(result.measurements.observations) + 1
    assert any(r.output is None for r in channel.injection.records)
    assert any(
        r.output is not None and r.output.delivery_index == -1 for r in channel.injection.records
    )


@pytest.mark.parametrize("bad", ["type", "used", "start", "overflow"])
def test_mission_fault_preflight_finishes_before_rng(bad, monkeypatch):
    import quadrotor_math.estimated_mission as module

    channel = EskfLiveObservationFaults(initial_time_s=1 if bad == "start" else 0)
    if bad == "used":
        channel.step(0)
    if bad == "type":
        channel = True
    if bad == "overflow":
        channel = EskfLiveObservationFaults(
            (EskfObservationFault(Kind.LOCAL_POSITION, 0, delay_steps=10**400),)
        )
    monkeypatch.setattr(module, "create_run_random_streams", lambda *_: pytest.fail("RNG reached"))
    with pytest.raises((TypeError, ValueError)):
        simulate_estimated_mission(**inputs(), observation_faults=channel)
