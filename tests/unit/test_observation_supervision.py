"""Real ESKF fault delivery, independent budgets, causal ordering and reset."""

from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest

from experiments.estimated_feedback_validation import make_configuration
from quadrotor_math.eskf_faults import EskfObservationFault, inject_eskf_observation_faults
from quadrotor_math.eskf_online import EskfOnlineEstimator
from quadrotor_math.eskf_replay import EskfObservationKind as Kind
from quadrotor_math.eskf_replay import EskfReplayInput, EskfReplayObservation
from quadrotor_math.eskf_replay import EskfReplayStatus as Status
from quadrotor_math.observation_health import (
    ObservationHealthConfiguration,
    ObservationHealthMonitor,
    ObservationHealthPolicy,
)
from quadrotor_math.observation_health import ObservationHealthState as Health
from quadrotor_math.observation_supervision import (
    ObservationSupervisionPolicy,
    ObservationSupervisor,
)

H = 1 / 64


def monitor(*, position_enabled=True, altitude_enabled=True, initial_time_s=0):
    return ObservationHealthMonitor(
        ObservationHealthConfiguration(
            ObservationHealthPolicy(4 * H, 0, H, 2, 4, 3, 2, 4, position_enabled),
            ObservationHealthPolicy(2 * H, 0, H, 2, 4, 3, 2, 4, altitude_enabled),
        ),
        initial_time_s=initial_time_s,
    )


def source():
    times = np.arange(65) * H
    observations = tuple(
        EskfReplayObservation(kind, i, k, k, np.zeros(width))
        for kind, stride, width in ((Kind.LOCAL_POSITION, 4, 3), (Kind.BAROMETRIC_ALTITUDE, 2, 1))
        for i, k in enumerate(range(stride, len(times), stride))
    )
    return EskfReplayInput(
        times, np.tile([0.0, 0, -9.81], (len(times), 1)), np.zeros((len(times), 3)), observations
    )


def run(faults=(), *, timeout=0.25, altitude_timeout=0.25):
    # Fault identities/labels remain outside all three causal components.
    injection = inject_eskf_observation_faults(source(), faults)
    data = injection.measurements
    estimator = EskfOnlineEstimator(
        make_configuration({"case": "smoke", "seed": 31, "noiseless": True})[
            "estimator_configuration"
        ]
    )
    health = monitor()
    supervisor = ObservationSupervisor(
        health, ObservationSupervisionPolicy(timeout, altitude_timeout)
    )
    events = []
    for k, time in enumerate(data.time_s):
        estimate = estimator.step(
            float(time),
            data.specific_force_measurements_B[k],
            data.angular_velocity_measurements_B[k],
            tuple(o for o in data.observations if o.delivery_index == k),
        )
        events.extend(estimate.events)
        health.step(float(time), estimate.events)
        supervisor.step()
    return health, supervisor, events


def burst(kind, first, last, **fault):
    return tuple(EskfObservationFault(kind, i, **fault) for i in range(first, last))


def first_abort(supervisor):
    return next((d for d in supervisor.history if d.abort_reason is not None), None)


@pytest.mark.parametrize("kind", list(Kind))
def test_nominal_and_one_real_rejected_observation_remain_healthy(kind):
    width = 3 if kind is Kind.LOCAL_POSITION else 1
    health, supervisor, events = run((EskfObservationFault(kind, 3, offset=np.full(width, 100)),))
    assert sum(e.status is Status.REJECTED for e in events) == 1
    assert first_abort(supervisor) is None
    assert health.latest.local_position.state is Health.HEALTHY
    assert health.latest.barometric_altitude.state is Health.HEALTHY
    assert supervisor.latest.local_position_unhealthy_since_s is None
    assert supervisor.latest.barometric_altitude_unhealthy_since_s is None


@pytest.mark.parametrize("kind", list(Kind))
@pytest.mark.parametrize("fault", ["dropout", "rejected", "delayed"])
def test_persistent_independent_loss_uses_elapsed_budget_and_preserves_other_stream(kind, fault):
    width = 3 if kind is Kind.LOCAL_POSITION else 1
    count = 16 if kind is Kind.LOCAL_POSITION else 32
    mutation = {
        "dropout": {"dropout": True},
        "rejected": {"offset": np.full(width, 100)},
        "delayed": {"delay_steps": 1},
    }[fault]
    health, supervisor, events = run(burst(kind, 3, count, **mutation))
    label = "local_position" if kind is Kind.LOCAL_POSITION else "barometric_altitude"
    other = "barometric_altitude" if kind is Kind.LOCAL_POSITION else "local_position"
    onset = next(s.time_s for s in health.history if getattr(s, label).state is Health.DEGRADED)
    decision = first_abort(supervisor)
    assert decision.abort_reason == f"observation_{label}_timeout"
    assert decision.time_s == onset + 0.25
    assert all(getattr(s, other).state is Health.HEALTHY for s in health.history[4:])
    if fault != "dropout":
        expected = Status.REJECTED if fault == "rejected" else Status.STALE
        assert any(e.observation.kind is kind and e.status is expected for e in events)


def test_optional_altitude_loss_never_vetoes_position_but_position_loss_still_does():
    _, optional, _ = run(
        burst(Kind.BAROMETRIC_ALTITUDE, 0, 32, dropout=True), altitude_timeout=None
    )
    assert first_abort(optional) is None
    assert all(d.barometric_altitude_unhealthy_since_s is None for d in optional.history)
    _, required, _ = run(burst(Kind.LOCAL_POSITION, 0, 16, dropout=True), altitude_timeout=None)
    assert first_abort(required).abort_reason == "observation_local_position_timeout"
    assert first_abort(required).time_s == 0.25


@pytest.mark.parametrize("timeout,aborted", [(15 * H, False), (14 * H, True), (13 * H, True)])
def test_complete_recovery_before_deadline_clears_timer_but_at_or_after_does_not(timeout, aborted):
    # Last acceptance k=8, warning first exceeded k=18; next accepts k=28,32.
    # Full recovery at k=32. Initial silence clears at k=4, well within budget.
    health, supervisor, _ = run(burst(Kind.LOCAL_POSITION, 2, 6, dropout=True), timeout=timeout)
    assert health.history[28].local_position.state is Health.RECOVERING
    assert health.history[32].local_position.state is Health.HEALTHY
    assert supervisor.history[28].local_position_unhealthy_since_s == 18 * H
    assert (first_abort(supervisor) is not None) is aborted
    if aborted:
        assert first_abort(supervisor).time_s == 18 * H + timeout
        assert supervisor.latest.abort_reason == "observation_local_position_timeout"
    else:
        assert supervisor.latest.local_position_unhealthy_since_s is None


def test_partial_recovery_rejection_and_fresh_altitude_do_not_restart_position_timer():
    faults = burst(Kind.LOCAL_POSITION, 2, 6, dropout=True) + (
        EskfObservationFault(Kind.LOCAL_POSITION, 7, offset=np.full(3, 100)),
    )
    health, supervisor, _ = run(faults)
    assert health.history[28].local_position.state is Health.RECOVERING
    assert health.history[32].local_position.state is Health.DEGRADED
    assert supervisor.history[32].local_position_unhealthy_since_s == 18 * H
    assert first_abort(supervisor).time_s == 34 * H


def test_future_fault_labels_cannot_change_prior_health_or_decisions():
    normal_health, normal, _ = run()
    fault_health, faulted, _ = run(burst(Kind.LOCAL_POSITION, 8, 16, dropout=True))
    assert normal_health.history[:36] == fault_health.history[:36]
    assert normal.history[:36] == faulted.history[:36]
    assert first_abort(normal) is None
    assert first_abort(faulted) is not None


def test_initial_silence_exact_deadline_tie_and_latch():
    health = monitor(initial_time_s=2)
    supervisor = ObservationSupervisor(health, ObservationSupervisionPolicy(0.5, 0.5))
    for time in (2.0, np.nextafter(2.5, 0), 2.5):
        health.step(time)
        supervisor.step()
    assert supervisor.history[1].abort_reason is None
    assert supervisor.latest.abort_reason == "observation_local_position_timeout"
    assert supervisor.latest.local_position_unhealthy_since_s == 2
    assert supervisor.latest.barometric_altitude_unhealthy_since_s == 2


def test_long_clock_gap_cannot_clear_an_expired_waiting_budget_with_fresh_acceptance():
    data = source()
    estimator = EskfOnlineEstimator(
        make_configuration({"case": "smoke", "seed": 31, "noiseless": True})[
            "estimator_configuration"
        ]
    )
    health = monitor()
    supervisor = ObservationSupervisor(health, ObservationSupervisionPolicy(0.125, None))
    health.step(0)
    supervisor.step()
    obs = EskfReplayObservation(Kind.LOCAL_POSITION, 0, 1, 1, np.zeros(3))
    estimator.step(0, data.specific_force_measurements_B[0], np.zeros(3))
    event = estimator.step(0.125, data.specific_force_measurements_B[0], np.zeros(3), (obs,)).events
    assert health.step(0.125, event).local_position.state is Health.HEALTHY
    assert supervisor.step().abort_reason == "observation_local_position_timeout"


@pytest.mark.parametrize("field", ["local_position_timeout_s", "barometric_altitude_timeout_s"])
@pytest.mark.parametrize("bad", [0, -1, True, False, np.nan, np.inf, "1", [1]])
def test_invalid_timeout_rejected(field, bad):
    with pytest.raises((TypeError, ValueError)):
        replace(ObservationSupervisionPolicy(1, 1), **{field: bad})


def test_position_cannot_be_optional():
    with pytest.raises((TypeError, ValueError)):
        ObservationSupervisionPolicy(None, None)


@pytest.mark.parametrize("position,altitude,timeout", [(False, True, None), (True, False, 1)])
def test_disabled_required_sensor_rejected(position, altitude, timeout):
    with pytest.raises(ValueError, match="required"):
        ObservationSupervisor(
            monitor(position_enabled=position, altitude_enabled=altitude),
            ObservationSupervisionPolicy(1, timeout),
        )


def test_disabled_optional_altitude_is_accepted():
    health = monitor(altitude_enabled=False)
    supervisor = ObservationSupervisor(health, ObservationSupervisionPolicy(1, None))
    health.step(0)
    assert supervisor.step().barometric_altitude_unhealthy_since_s is None


@pytest.mark.parametrize("bad_monitor,bad_policy", [(True, False), (False, True)])
def test_invalid_object_types_rejected(bad_monitor, bad_policy):
    with pytest.raises(TypeError):
        ObservationSupervisor(
            True if bad_monitor else monitor(),
            True if bad_policy else ObservationSupervisionPolicy(1, 1),
        )


def test_epoch_checks_are_atomic_and_reset_restores_identical_trace():
    health = monitor()
    supervisor = ObservationSupervisor(health, ObservationSupervisionPolicy(1, 1))
    with pytest.raises(ValueError, match="exactly once"):
        supervisor.step()
    assert supervisor.latest is None
    health.step(0)
    first = supervisor.step()
    with pytest.raises(ValueError, match="exactly once"):
        supervisor.step()
    assert supervisor.history == (first,)
    with pytest.raises(ValueError, match="fresh"):
        supervisor.reset()
    assert supervisor.history == (first,)
    health.step(H)
    health.step(2 * H)
    with pytest.raises(ValueError, match="exactly once"):
        supervisor.step()
    assert supervisor.history == (first,)
    health.reset()
    supervisor.reset()
    health.step(0)
    assert supervisor.step() == first
    with pytest.raises(FrozenInstanceError):
        first.abort_reason = "changed"
    with pytest.raises(FrozenInstanceError):
        supervisor.policy.local_position_timeout_s = 2
    with pytest.raises(AttributeError):
        supervisor.monitor = monitor()


def test_construction_requires_fresh_monitor():
    health = monitor()
    health.step(0)
    with pytest.raises(ValueError, match="fresh"):
        ObservationSupervisor(health, ObservationSupervisionPolicy(1, 1))
