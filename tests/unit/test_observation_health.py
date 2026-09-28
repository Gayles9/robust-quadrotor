"""Independent availability timelines and real ESKF event contract checks."""

from dataclasses import FrozenInstanceError, replace
from functools import lru_cache

import numpy as np
import pytest

from experiments.estimated_feedback_validation import make_configuration
from quadrotor_math.eskf_innovation import EskfInnovationPolicy
from quadrotor_math.eskf_online import EskfOnlineEstimator
from quadrotor_math.eskf_replay import (
    EskfObservationKind as Kind,
)
from quadrotor_math.eskf_replay import (
    EskfReplayEvent,
    EskfReplayObservation,
)
from quadrotor_math.eskf_replay import (
    EskfReplayStatus as Status,
)
from quadrotor_math.observation_health import (
    ObservationHealthConfiguration,
    ObservationHealthMonitor,
    ObservationHealthPolicy,
)
from quadrotor_math.observation_health import (
    ObservationHealthReason as Reason,
)
from quadrotor_math.observation_health import (
    ObservationHealthState as State,
)


def policy(**changes):
    return replace(ObservationHealthPolicy(1.0, 0.0, 0.25, 2, 4, 3, 2, 4), **changes)


def monitor(**changes):
    p = policy(**changes)
    return ObservationHealthMonitor(ObservationHealthConfiguration(p, p))


@lru_cache
def template(kind, status):
    config = make_configuration({"case": "smoke", "seed": 31, "noiseless": False})[
        "estimator_configuration"
    ]
    config = replace(
        config,
        innovation_policy=EskfInnovationPolicy(1.0, 1.0),
        fuse_local_position=status is not Status.DISABLED,
        fuse_barometric_altitude=status is not Status.DISABLED,
    )
    width = 3 if kind is Kind.LOCAL_POSITION else 1
    measurement = np.full(width, 100.0 if status is Status.REJECTED else 0.0)
    obs = EskfReplayObservation(kind, 0, 0, 0, measurement)
    event = (
        EskfOnlineEstimator(config)
        .step(0, np.array([0.0, 0, -9.81]), np.zeros(3), (obs,))
        .events[0]
    )
    assert event.status is status
    return event


def event(index, status=Status.FUSED, *, kind=Kind.LOCAL_POSITION, identity=None, acquired=None):
    identity = index if identity is None else identity
    acquired = index if acquired is None else acquired
    width = 3 if kind is Kind.LOCAL_POSITION else 1
    if status in (Status.STALE, Status.PENDING):
        obs = EskfReplayObservation(
            kind, identity, acquired, -1 if status is Status.PENDING else index, np.zeros(width)
        )
        return EskfReplayEvent(obs, status)
    base = template(kind, status)
    return replace(
        base,
        observation=replace(
            base.observation,
            observation_index=identity,
            acquisition_index=acquired,
            delivery_index=index,
        ),
    )


def test_nominal_delivery_and_isolated_outlier_do_not_create_loss():
    m = monitor()
    first = m.step(0)
    assert [x.current for x in first.transitions] == [State.WAITING, State.WAITING]
    assert all(x.previous is None for x in first.transitions)
    for k in range(1, 7):
        status = Status.REJECTED if k == 2 else Status.FUSED
        row = m.step(k * 0.25, (event(k, status), event(k, kind=Kind.BAROMETRIC_ALTITUDE)))
        assert row.local_position.state is State.HEALTHY
        assert row.barometric_altitude.state is State.HEALTHY
        assert len(row.transitions) == (2 if k == 1 else 0)
        if k == 2:
            assert row.local_position.accepted_age_s == 0.25
            assert row.local_position.recent_rejected_count == 1
    assert row.local_position.recent_statuses == (Status.FUSED,) * 4
    assert row.local_position.recent_accepted_count == 4
    assert first.local_position.last_delivery_time_s is None


def test_rejection_limit_and_two_acceptance_recovery_with_interruption():
    m = monitor()
    m.step(0, (event(0),))
    sequence = [Status.REJECTED] * 3 + [Status.FUSED, Status.REJECTED, Status.FUSED, Status.FUSED]
    expected = [
        State.HEALTHY,
        State.HEALTHY,
        State.DEGRADED,
        State.RECOVERING,
        State.DEGRADED,
        State.RECOVERING,
        State.HEALTHY,
    ]
    for k, (status, state) in enumerate(zip(sequence, expected, strict=True), 1):
        row = m.step(k * 0.125, (event(k, status),))
        assert row.local_position.state is state
        if k == 3:
            assert row.local_position.reason is Reason.REJECTION_RUN
            assert row.local_position.consecutive_rejections == 3
        if k == 5:
            assert row.local_position.consecutive_acceptances == 0
            assert row.local_position.reason is Reason.RECOVERY
    assert row.local_position.accepted_age_s == 0
    assert row.local_position.consecutive_acceptances == 2


@pytest.mark.parametrize("accepted", [False, True])
@pytest.mark.parametrize("which", ["warning", "lost"])
def test_exact_age_boundary_and_one_representable_step_past_it(accepted, which):
    m = monitor()
    m.step(0, (event(0),) if accepted else ())
    boundary = 2.25 if which == "warning" else 4.25
    before = m.step(np.nextafter(boundary, 0))
    at = m.step(boundary)
    beyond = m.step(np.nextafter(boundary, np.inf))
    inside_state = (
        (State.HEALTHY if accepted else State.WAITING) if which == "warning" else State.DEGRADED
    )
    assert before.local_position.state is at.local_position.state is inside_state
    assert beyond.local_position.state is (State.DEGRADED if which == "warning" else State.LOST)
    assert beyond.local_position.recent_rejected_count == 0
    assert beyond.local_position.accepted_age_s == (beyond.time_s if accepted else None)
    assert beyond.transitions[0].time_s == beyond.time_s


def test_sensor_loss_is_independent_and_long_gap_resets_old_acceptance_streak():
    m = monitor()
    for k in range(4):
        m.step(k * 0.25, (event(k), event(k, kind=Kind.BAROMETRIC_ALTITUDE)))
    lost = m.step(5.25, (event(4, kind=Kind.BAROMETRIC_ALTITUDE),))
    assert lost.local_position.state is State.LOST
    assert lost.local_position.consecutive_acceptances == 0
    assert lost.barometric_altitude.state is State.RECOVERING
    recovered = m.step(5.5, (event(5), event(5, kind=Kind.BAROMETRIC_ALTITUDE)))
    assert recovered.local_position.state is State.RECOVERING
    assert recovered.barometric_altitude.state is State.HEALTHY
    assert recovered.local_position.last_accepted_acquisition_time_s == 5.5
    assert recovered.local_position.last_accepted_delivery_time_s == 5.5
    assert m.step(5.75, (event(6),)).local_position.state is State.HEALTHY


def test_baro_can_stay_healthy_while_position_is_lost():
    m = monitor()
    for k in range(21):
        observations = ((event(0),) if k == 0 else ()) + (event(k, kind=Kind.BAROMETRIC_ALTITUDE),)
        row = m.step(k * 0.25, observations)
    assert row.local_position.state is State.LOST
    assert row.barometric_altitude.state is State.HEALTHY


def test_delayed_out_of_order_stale_observations_never_refresh_accepted_age():
    m = monitor(delivery_delay_s=0.5)
    for time in (0.0, 0.25, 0.5):
        m.step(time)
    a = m.step(1, (event(3, Status.STALE, acquired=1, identity=1),))
    b = m.step(1.25, (event(4, Status.STALE, acquired=0, identity=0),))
    assert a.local_position.last_acquisition_time_s == 0.25
    assert b.local_position.last_acquisition_time_s == 0
    assert b.local_position.last_delivery_time_s == 1.25
    assert b.local_position.accepted_age_s is None
    assert b.local_position.recent_statuses == (Status.STALE, Status.STALE)
    assert m.step(5).local_position.state is State.LOST


def test_disabled_stream_retains_evidence_without_timeout_or_recovery():
    m = monitor(enabled=False)
    m.step(0)
    row = m.step(100, (event(1, Status.DISABLED),))
    assert row.local_position.state is State.DISABLED
    assert row.local_position.last_delivery_time_s == 100
    assert row.local_position.last_accepted_delivery_time_s is None
    row = m.step(101, (event(2, Status.STALE, acquired=0, identity=9),))
    assert row.local_position.state is State.DISABLED
    assert row.local_position.recent_statuses == (Status.DISABLED, Status.STALE)


def test_thresholds_follow_period_delay_and_clock_without_assuming_integer_seconds():
    p = policy(sample_period_s=0.125, delivery_delay_s=0.0625, check_interval_s=0.03125)
    assert p.warning_age_s == 0.34375
    assert p.lost_age_s == 0.59375
    m = ObservationHealthMonitor(ObservationHealthConfiguration(p, p), initial_time_s=2)
    m.step(2)
    assert m.step(2.34375).local_position.state is State.WAITING
    assert m.step(2.375).local_position.state is State.DEGRADED


@pytest.mark.parametrize(
    "change",
    [
        {"sample_period_s": 0},
        {"sample_period_s": True},
        {"sample_period_s": "1"},
        {"sample_period_s": np.inf},
        {"delivery_delay_s": -1},
        {"delivery_delay_s": np.nan},
        {"check_interval_s": 0},
        {"check_interval_s": -1},
        {"warning_periods": 0},
        {"warning_periods": True},
        {"lost_periods": 2},
        {"rejection_limit": 0},
        {"recovery_acceptances": 1},
        {"recovery_acceptances": 2.0},
        {"evidence_window_size": 2},
        {"enabled": 1},
        {"sample_period_s": 1e308},
        {"warning_periods": 10**400, "lost_periods": 10**400 + 1},
        {"delivery_delay_s": 1e100},
    ],
)
def test_bad_policy_rejected(change):
    with pytest.raises((ValueError, TypeError)):
        policy(**change)


@pytest.mark.parametrize("bad", [True, np.nan, np.inf, -1, "0", 1e308])
def test_bad_reset_is_atomic(bad):
    m = monitor()
    first = m.step(0, (event(0),))
    with pytest.raises((ValueError, TypeError)):
        m.reset(initial_time_s=bad)
    assert m.latest is first and m.history == (first,)


@pytest.mark.parametrize(
    "bad", ["clock", "nan", "future", "pending", "duplicate", "order", "type", "disabled"]
)
def test_invalid_epoch_is_atomic_and_retry_matches_fresh_monitor(bad):
    m, control = monitor(), monitor()
    m.step(0)
    control.step(0)
    time, events = 0.25, (event(1), event(1, kind=Kind.BAROMETRIC_ALTITUDE))
    if bad == "clock":
        time = 0
    elif bad == "nan":
        time = np.nan
    elif bad == "future":
        events = (event(2),)
    elif bad == "pending":
        events = (event(1, Status.PENDING),)
    elif bad == "duplicate":
        events = (event(1), event(1))
    elif bad == "order":
        events = tuple(reversed(events))
    elif bad == "type":
        events = list(events)
    else:
        events = (event(1, Status.DISABLED),)
    with pytest.raises((ValueError, TypeError)):
        m.step(time, events)
    assert m.history == control.history
    valid = (event(1), event(1, kind=Kind.BAROMETRIC_ALTITUDE))
    assert m.step(0.25, valid) == control.step(0.25, valid)


@pytest.mark.parametrize("reuse", ["identity", "acquisition"])
def test_cross_epoch_duplicate_rejected(reuse):
    m = monitor()
    m.step(0)
    m.step(0.25, (event(1, Status.STALE, acquired=0, identity=10),))
    duplicate = event(
        2,
        Status.STALE,
        acquired=1 if reuse == "identity" else 0,
        identity=10 if reuse == "identity" else 11,
    )
    with pytest.raises(ValueError, match="duplicate"):
        m.step(0.5, (duplicate,))
    assert m.latest.epoch_index == 1
    m.step(0.5, (event(2),))


def test_disabled_policy_rejects_fusion_and_rejection():
    m = monitor(enabled=False)
    for status in (Status.FUSED, Status.REJECTED):
        with pytest.raises(ValueError, match="fusion flag"):
            m.step(0, (event(0, status),))
    assert m.latest is None


def test_reset_reuses_identifiers_and_outputs_cannot_change_continuation():
    m = monitor()
    first = m.step(0, (event(0),))
    with pytest.raises(FrozenInstanceError):
        first.local_position.state = State.LOST
    with pytest.raises(FrozenInstanceError):
        m.configuration.local_position.enabled = False
    old_history = m.history
    m.reset(initial_time_s=10)
    assert m.latest is None and m.history == ()
    restarted = m.step(10, (event(0),))
    assert restarted.local_position.accepted_age_s == 0
    assert old_history == (first,)
    assert restarted.transitions[0].previous is None


def test_invalid_configuration_and_initial_epoch():
    with pytest.raises(TypeError):
        ObservationHealthConfiguration(policy(), True)
    with pytest.raises(TypeError):
        ObservationHealthMonitor(True)
    with pytest.raises(ValueError):
        monitor().step(0.25)
