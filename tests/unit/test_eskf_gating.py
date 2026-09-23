"""Pre-correction gate ordering, event invariants and legacy replay compatibility."""

from dataclasses import fields, replace

import numpy as np
import pytest

import quadrotor_math.eskf_replay as replay_module
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_innovation import EskfInnovation, EskfInnovationPolicy
from quadrotor_math.eskf_replay import (
    EskfObservationKind,
    EskfReplayConfiguration,
    EskfReplayEvent,
    EskfReplayInput,
    EskfReplayObservation,
    EskfReplayStatus,
    replay_eskf,
)


def _config(policy=None, **changes):
    configuration = EskfReplayConfiguration(
        0.1,
        EskfNominalState(
            np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
        ),
        np.eye(15) * 0.5,
        9.81,
        np.zeros((12, 12)),
        np.zeros(3),
        np.eye(3) * 0.5,
        100.0,
        0.0,
        0.5,
        innovation_policy=policy,
    )
    return replace(configuration, **changes)


def _observation(
    kind=EskfObservationKind.LOCAL_POSITION, value=2.0, acquired=0, delivered=0, index=0
):
    measured = (
        np.array([value, 0.0, 0.0])
        if kind is EskfObservationKind.LOCAL_POSITION
        else np.array([100 + value])
    )
    return EskfReplayObservation(kind, index, acquired, delivered, measured)


def _data(observations=(), n=1):
    return EskfReplayInput(
        np.arange(1, n + 1) * 0.1,
        np.tile([0.0, 0.0, -9.81], (n, 1)),
        np.zeros((n, 3)),
        observations,
    )


def _state_bytes(state):
    return tuple(getattr(state, field.name).tobytes() for field in fields(state))


@pytest.mark.parametrize("kind", list(EskfObservationKind))
@pytest.mark.parametrize(
    "threshold,status",
    [
        (np.nextafter(4.0, 0.0), "REJECTED"),
        (4.0, "FUSED"),
        (np.nextafter(4.0, np.inf), "FUSED"),
        (None, "FUSED"),
    ],
)
def test_strict_threshold_boundary_and_diagnostics(kind, threshold, status) -> None:
    configuration = _config(EskfInnovationPolicy(threshold, threshold))
    event = replay_eskf(_data((_observation(kind),)), configuration).events[0]
    assert event.status is EskfReplayStatus[status]
    assert event.innovation.normalized_innovation_squared == 4.0
    assert event.nis_threshold == threshold
    if status == "REJECTED":
        assert event.update is None
    else:
        assert event.update is not None
        assert event.update.innovation.tobytes() == event.innovation.innovation.tobytes()
        assert (
            event.update.innovation_covariance.tobytes()
            == event.innovation.innovation_covariance.tobytes()
        )


@pytest.mark.parametrize("kind", list(EskfObservationKind))
def test_rejection_precedes_correction_and_preserves_prior_exactly(monkeypatch, kind) -> None:
    def forbidden(*args, **kwargs):
        pytest.fail("Rejected observation reached the correction core")

    monkeypatch.setattr(replay_module, "update_eskf_local_position", forbidden)
    monkeypatch.setattr(replay_module, "update_eskf_barometric_altitude", forbidden)
    configuration = _config(EskfInnovationPolicy(1, 1))
    result = replay_eskf(_data((_observation(kind, 1e150),)), configuration)
    assert result.events[0].status is EskfReplayStatus.REJECTED
    assert _state_bytes(result.states[0]) == _state_bytes(configuration.initial_state)
    assert result.covariances[0].tobytes() == configuration.initial_covariance.tobytes()


def test_none_policy_bypasses_diagnostic_arithmetic(monkeypatch) -> None:
    def forbidden(*args, **kwargs):
        pytest.fail("Legacy replay attempted innovation scoring")

    monkeypatch.setattr(replay_module, "compute_eskf_linear_innovation", forbidden)
    result = replay_eskf(_data((_observation(),)), _config())
    assert result.events[0].status is EskfReplayStatus.FUSED
    assert result.events[0].innovation is None
    assert result.events[0].nis_threshold is None


def test_opt_in_scoring_does_not_shrink_legacy_finite_domain() -> None:
    # Zero gain makes the legacy correction valid even when the squared residual overflows.
    configuration = _config(initial_covariance=np.zeros((15, 15)))
    data = _data((_observation(value=1e200),))
    assert replay_eskf(data, configuration).events[0].status is EskfReplayStatus.FUSED
    with pytest.raises(ValueError, match="NIS"):
        replay_eskf(data, replace(configuration, innovation_policy=EskfInnovationPolicy()))


def test_diagnostics_only_preserves_legacy_state_covariance_and_update_bytes() -> None:
    data = _data((_observation(), _observation(EskfObservationKind.BAROMETRIC_ALTITUDE)), n=5)
    configuration = _config()
    legacy = replay_eskf(data, configuration)
    scored = replay_eskf(data, replace(configuration, innovation_policy=EskfInnovationPolicy()))
    assert legacy.covariances.tobytes() == scored.covariances.tobytes()
    assert [_state_bytes(s) for s in legacy.states] == [_state_bytes(s) for s in scored.states]
    for a, b in zip(legacy.events, scored.events, strict=True):
        assert a.status is b.status is EskfReplayStatus.FUSED
        for field in fields(a.update):
            if field.name != "nominal_state":
                assert (
                    getattr(a.update, field.name).tobytes()
                    == getattr(b.update, field.name).tobytes()
                )


@pytest.mark.parametrize("position_rejected", [False, True])
def test_later_same_epoch_score_uses_actual_prior_after_first_gate(position_rejected) -> None:
    # Position z_down=2 gives p_down=1 and variance=.25 if accepted.
    # Altitude z=99 then has zero residual, S=.75. If position rejects,
    # altitude residual=-1 and S=1. Its NIS=1 then rejects against .5.
    position = replace(_observation(), measurement=np.array([0.0, 0.0, 2.0]))
    altitude = _observation(EskfObservationKind.BAROMETRIC_ALTITUDE, -1)
    policy = EskfInnovationPolicy(1 if position_rejected else 4, 0.5)
    result = replay_eskf(_data((altitude, position)), _config(policy))
    first, second = result.events
    assert first.observation.kind is EskfObservationKind.LOCAL_POSITION
    if position_rejected:
        assert first.status is second.status is EskfReplayStatus.REJECTED
        assert second.innovation.normalized_innovation_squared == 1.0
        assert result.states[0].position_W[2] == 0
    else:
        assert first.status is second.status is EskfReplayStatus.FUSED
        assert second.innovation.normalized_innovation_squared == 0.0
        assert second.innovation.innovation_covariance[0, 0] == 0.75
        assert result.states[0].position_W[2] == 1.0


@pytest.mark.parametrize("position_gate", [True, False])
def test_sensor_thresholds_are_independent(position_gate) -> None:
    policy = EskfInnovationPolicy(1 if position_gate else None, None if position_gate else 1)
    observations = (_observation(), _observation(EskfObservationKind.BAROMETRIC_ALTITUDE))
    result = replay_eskf(_data(observations), _config(policy))
    expected = [EskfReplayStatus.REJECTED, EskfReplayStatus.FUSED]
    assert [e.status for e in result.events] == (expected if position_gate else expected[::-1])
    assert all(e.innovation is not None for e in result.events)


@pytest.mark.parametrize("kind", list(EskfObservationKind))
@pytest.mark.parametrize(
    "status,acquired,delivered", [("STALE", 0, 1), ("PENDING", 0, -1), ("DISABLED", 0, 0)]
)
def test_unavailable_or_disabled_values_are_not_scored(
    monkeypatch, kind, status, acquired, delivered
) -> None:
    def forbidden(*args, **kwargs):
        pytest.fail("Ineligible observation was scored")

    monkeypatch.setattr(replay_module, "compute_eskf_linear_innovation", forbidden)
    configuration = _config(
        EskfInnovationPolicy(1, 1), fuse_local_position=False, fuse_barometric_altitude=False
    )
    event = replay_eskf(
        _data((_observation(kind, 1e200, acquired, delivered),), n=2), configuration
    ).events[0]
    assert event.status is EskfReplayStatus[status]
    assert event.innovation is event.nis_threshold is event.update is None


@pytest.mark.parametrize("kind", list(EskfObservationKind))
@pytest.mark.parametrize("defect", ["singular", "nis_overflow"])
def test_model_or_numerical_failure_is_not_statistical_rejection(kind, defect) -> None:
    configuration = _config(EskfInnovationPolicy(1, 1))
    value = 1e200
    if defect == "singular":
        configuration = replace(
            configuration,
            initial_covariance=np.zeros((15, 15)),
            local_position_noise_covariance_W=np.zeros((3, 3)),
            barometric_altitude_noise_variance=0,
        )
        value = 1e100
    before = configuration.initial_covariance.tobytes(), _state_bytes(configuration.initial_state)
    with pytest.raises(ValueError):
        replay_eskf(_data((_observation(kind, value),)), configuration)
    assert before == (
        configuration.initial_covariance.tobytes(),
        _state_bytes(configuration.initial_state),
    )


def test_nonzero_biases_and_positive_up_altitude_enter_pre_update_models() -> None:
    configuration = _config(
        EskfInnovationPolicy(1, 1),
        local_position_bias_W=np.array([2.0, 0, 0]),
        barometric_altitude_bias=2,
    )
    result = replay_eskf(
        _data((_observation(), _observation(EskfObservationKind.BAROMETRIC_ALTITUDE))),
        configuration,
    )
    assert all(e.status is EskfReplayStatus.FUSED for e in result.events)
    assert all(e.innovation.normalized_innovation_squared == 0 for e in result.events)


@pytest.mark.parametrize("policy", [True, {}, (1, 1), 4, "99%"])
def test_configuration_rejects_wrong_policy_type(policy) -> None:
    with pytest.raises(TypeError, match="innovation_policy"):
        _config(policy)


@pytest.mark.parametrize("threshold", [1, 4])
def test_policy_and_nested_event_diagnostics_are_owned_and_immutable(threshold) -> None:
    policy = EskfInnovationPolicy(threshold, threshold)
    configuration = _config(policy)
    assert configuration.innovation_policy is not policy
    result = replay_eskf(_data((_observation(),)), configuration)
    event = result.events[0]
    duplicate = replace(event)
    for name in ("innovation", "innovation_covariance", "whitened_innovation"):
        a, b = getattr(event.innovation, name), getattr(duplicate.innovation, name)
        assert not np.shares_memory(a, b)
        assert not a.flags.writeable
        with pytest.raises(ValueError):
            a.flat[0] = 0
    assert not np.shares_memory(event.observation.measurement, event.innovation.innovation)
    assert not np.shares_memory(result.covariances, event.innovation.innovation_covariance)


@pytest.mark.parametrize("status", ["STALE", "DISABLED", "PENDING"])
@pytest.mark.parametrize("field", ["innovation", "nis_threshold"])
def test_non_scored_event_cannot_claim_gate_diagnostics(status, field) -> None:
    observation = _observation(delivered={"STALE": 1, "DISABLED": 0, "PENDING": -1}[status])
    value = EskfInnovation(np.ones(3), np.eye(3)) if field == "innovation" else 1
    with pytest.raises(ValueError):
        EskfReplayEvent(observation, EskfReplayStatus[status], **{field: value})


@pytest.mark.parametrize(
    "defect", ["no_diagnostics", "no_threshold", "below", "equal", "dimension", "update"]
)
def test_rejected_event_requires_consistent_above_threshold_diagnostics(defect) -> None:
    event = replay_eskf(_data((_observation(),)), _config(EskfInnovationPolicy(1, 1))).events[0]
    changes = {
        "no_diagnostics": {"innovation": None},
        "no_threshold": {"nis_threshold": None},
        "below": {"nis_threshold": 5},
        "equal": {"nis_threshold": 4},
        "dimension": {"innovation": EskfInnovation(np.array([2.0]), np.eye(1))},
        "update": {"update": replay_eskf(_data((_observation(),)), _config()).events[0].update},
    }[defect]
    with pytest.raises(ValueError):
        replace(event, **changes)


@pytest.mark.parametrize(
    "defect", ["above", "no_diagnostics", "residual_mismatch", "covariance_mismatch", "dimension"]
)
def test_fused_event_requires_matching_update_and_passing_gate(defect) -> None:
    event = replay_eskf(_data((_observation(),)), _config(EskfInnovationPolicy(4, 4))).events[0]
    changes = {
        "above": {"nis_threshold": 1},
        "no_diagnostics": {"innovation": None},
        "residual_mismatch": {"innovation": EskfInnovation(np.array([1.0, 0, 0]), np.eye(3))},
        "covariance_mismatch": {"innovation": EskfInnovation(np.array([2.0, 0, 0]), np.eye(3) * 2)},
        "dimension": {"innovation": EskfInnovation(np.array([2.0]), np.eye(1))},
    }[defect]
    with pytest.raises(ValueError):
        replace(event, **changes)


@pytest.mark.parametrize("bad", [True, 0, -1, np.inf, np.nan, "4"])
def test_event_threshold_is_validated(bad) -> None:
    event = replay_eskf(_data((_observation(),)), _config(EskfInnovationPolicy(4, 4))).events[0]
    with pytest.raises(ValueError):
        replace(event, nis_threshold=bad)


def test_event_diagnostics_type_is_checked() -> None:
    event = replay_eskf(_data((_observation(),)), _config()).events[0]
    with pytest.raises(TypeError, match="innovation"):
        replace(event, innovation={})


def test_scoring_failure_after_a_successful_update_is_atomic(monkeypatch) -> None:
    observations = (_observation(), _observation(EskfObservationKind.BAROMETRIC_ALTITUDE, 1e200))
    data = _data(observations)
    configuration = _config(EskfInnovationPolicy(4, 4))
    original_update = replay_module.update_eskf_local_position
    calls = []

    def recorded(*args, **kwargs):
        calls.append(1)
        return original_update(*args, **kwargs)

    monkeypatch.setattr(replay_module, "update_eskf_local_position", recorded)
    prior = _state_bytes(configuration.initial_state), configuration.initial_covariance.tobytes()
    measured = tuple(o.measurement.tobytes() for o in data.observations)
    with pytest.raises(ValueError, match="NIS"):
        replay_eskf(data, configuration)
    assert calls == [1]
    assert prior == (
        _state_bytes(configuration.initial_state),
        configuration.initial_covariance.tobytes(),
    )
    assert measured == tuple(o.measurement.tobytes() for o in data.observations)
