"""Timing, ownership and composition contracts for measurement-only ESKF replay."""

from dataclasses import FrozenInstanceError, fields, replace

import numpy as np
import pytest

from quadrotor_math.eskf import (
    EskfNominalState,
    predict_eskf,
    update_eskf_barometric_altitude,
    update_eskf_local_position,
)
from quadrotor_math.eskf_replay import (
    EskfObservationKind,
    EskfReplayConfiguration,
    EskfReplayEvent,
    EskfReplayInput,
    EskfReplayObservation,
    EskfReplayStatus,
    replay_eskf,
)


def _state() -> EskfNominalState:
    return EskfNominalState(
        np.zeros(3), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3), np.zeros(3)
    )


def _configuration(**changes: object) -> EskfReplayConfiguration:
    config = EskfReplayConfiguration(
        initial_time_s=0.1,
        initial_state=_state(),
        initial_covariance=np.eye(15),
        gravity_acceleration=9.81,
        continuous_noise_covariance=np.zeros((12, 12)),
        local_position_bias_W=np.zeros(3),
        local_position_noise_covariance_W=np.eye(3),
        barometric_reference_altitude=100.0,
        barometric_altitude_bias=0.0,
        barometric_altitude_noise_variance=1.0,
    )
    return replace(config, **changes)


def _input(n: int = 4, observations: tuple = ()) -> EskfReplayInput:
    return EskfReplayInput(
        time_s=np.arange(1, n + 1) * 0.1,
        specific_force_measurements_B=np.tile([0.0, 0.0, -9.81], (n, 1)),
        angular_velocity_measurements_B=np.zeros((n, 3)),
        observations=observations,
    )


def _position(index: int = 0, acquired: int = 0, delivered: int = 0) -> EskfReplayObservation:
    return EskfReplayObservation(
        EskfObservationKind.LOCAL_POSITION, index, acquired, delivered, np.array([1.0, 2.0, 3.0])
    )


def _altitude(index: int = 0, acquired: int = 0, delivered: int = 0) -> EskfReplayObservation:
    return EskfReplayObservation(
        EskfObservationKind.BAROMETRIC_ALTITUDE, index, acquired, delivered, np.array([102.0])
    )


def test_first_epoch_fuses_without_prediction_and_uses_ned_altitude_sign() -> None:
    data = _input(1, (_altitude(),))
    result = replay_eskf(data, _configuration())
    np.testing.assert_array_equal(result.time_s, [0.1])
    # The scaled Cholesky solves introduce bounded machine roundoff.
    np.testing.assert_allclose(
        result.states[0].position_W, [0.0, 0.0, -1.0], rtol=0.0, atol=4 * np.finfo(float).eps
    )
    assert result.covariances[0, 2, 2] == pytest.approx(0.5, abs=4 * np.finfo(float).eps)
    assert result.events[0].status is EskfReplayStatus.FUSED
    assert result.events[0].update is not None
    np.testing.assert_array_equal(result.events[0].update.innovation, [2.0])


def test_left_held_imu_no_backfill_no_final_sample_lookahead() -> None:
    data = _input(4)
    force = data.specific_force_measurements_B.copy()
    force[:, 0] = [1.0, 2.0, 3.0, 999.0]
    data = replace(data, specific_force_measurements_B=force)
    result = replay_eskf(data, _configuration())
    np.testing.assert_allclose([s.position_W[0] for s in result.states], [0, 0.005, 0.025, 0.07])
    np.testing.assert_allclose([s.velocity_W[0] for s in result.states], [0, 0.1, 0.3, 0.6])


def test_replay_matches_explicit_prediction_then_position_then_altitude() -> None:
    observations = (_altitude(0, 2, 2), _position(1, 2, 2), _position())
    data = _input(observations=observations)
    config = _configuration()
    result = replay_eskf(data, config)
    state, covariance = config.initial_state, config.initial_covariance
    expected_events = []
    for index in range(len(data.time_s)):
        if index:
            state, covariance = predict_eskf(
                state,
                covariance,
                data.specific_force_measurements_B[index - 1],
                data.angular_velocity_measurements_B[index - 1],
                config.gravity_acceleration,
                config.continuous_noise_covariance,
                float(data.time_s[index] - data.time_s[index - 1]),
            )
        for observation in sorted(observations, key=lambda o: (o.kind.value, o.observation_index)):
            if observation.delivery_index != index:
                continue
            if observation.kind is EskfObservationKind.LOCAL_POSITION:
                update = update_eskf_local_position(
                    state,
                    covariance,
                    observation.measurement,
                    config.local_position_noise_covariance_W,
                    config.local_position_bias_W,
                )
            else:
                update = update_eskf_barometric_altitude(
                    state,
                    covariance,
                    float(observation.measurement[0]),
                    config.barometric_altitude_noise_variance,
                    config.barometric_reference_altitude,
                    config.barometric_altitude_bias,
                )
            state, covariance = update.nominal_state, update.covariance
            expected_events.append((observation.kind, observation.observation_index))
        for field in fields(state):
            np.testing.assert_array_equal(
                getattr(result.states[index], field.name), getattr(state, field.name)
            )
        np.testing.assert_array_equal(result.covariances[index], covariance)
    assert [
        (e.observation.kind, e.observation.observation_index) for e in result.events
    ] == expected_events


def test_stale_pending_and_disabled_never_change_filter() -> None:
    observations = (_position(0, 0, 2), _position(1, 1, -1), _altitude(0, 3, 3))
    data = _input(observations=observations)
    config = _configuration(fuse_barometric_altitude=False)
    result = replay_eskf(data, config)
    reference = replay_eskf(_input(), config)
    np.testing.assert_array_equal(result.covariances, reference.covariances)
    for actual, expected in zip(result.states, reference.states, strict=True):
        for field in fields(actual):
            np.testing.assert_array_equal(
                getattr(actual, field.name), getattr(expected, field.name)
            )
    assert [event.status for event in result.events] == [
        EskfReplayStatus.STALE,
        EskfReplayStatus.DISABLED,
        EskfReplayStatus.PENDING,
    ]
    assert all(event.update is None for event in result.events)


def test_initial_epoch_must_equal_first_measurement_epoch() -> None:
    with pytest.raises(ValueError, match="initial_time_s"):
        replay_eskf(_input(), _configuration(initial_time_s=0.0))


def test_owned_frozen_input_configuration_and_result() -> None:
    data = _input(observations=(_position(),))
    config = _configuration()
    result = replay_eskf(data, config)
    for array in (
        data.time_s,
        data.specific_force_measurements_B,
        data.observations[0].measurement,
        config.initial_covariance,
        config.continuous_noise_covariance,
        result.time_s,
        result.covariances,
        result.states[0].position_W,
    ):
        assert array.flags.c_contiguous and array.flags.owndata and not array.flags.writeable
    assert not np.shares_memory(result.time_s, data.time_s)
    assert not np.shares_memory(result.states[0].q_WB, config.initial_state.q_WB)
    with pytest.raises(FrozenInstanceError):
        data.time_s = np.zeros(4)


@pytest.mark.parametrize(
    "times",
    [
        np.array([]),
        np.array([0.1, 0.1]),
        np.array([0.2, 0.1]),
        np.array([-0.1, 0.1]),
        np.array([0.1, np.inf]),
    ],
)
def test_invalid_epochs_reject(times: np.ndarray) -> None:
    with pytest.raises(ValueError, match="time_s"):
        replace(_input(2), time_s=times)


@pytest.mark.parametrize(
    "changes",
    [
        {"observation_index": -1},
        {"observation_index": True},
        {"acquisition_index": -1},
        {"delivery_index": -2},
        {"acquisition_index": 2, "delivery_index": 1},
        {"kind": 3},
        {"measurement": np.ones(1)},
        {"measurement": np.array([0.0, np.nan, 0.0])},
    ],
)
def test_invalid_observations_reject(changes: dict) -> None:
    with pytest.raises((TypeError, ValueError)):
        replace(_position(), **changes)


@pytest.mark.parametrize(
    "observations",
    [
        (_position(), _position()),
        (_position(0, 4, 4),),
        (_position(0, 0, 4),),
        (_position(1, 0, 0),),
    ],
)
def test_input_rejects_duplicate_missing_or_out_of_range_observation_indices(
    observations: tuple,
) -> None:
    with pytest.raises(ValueError):
        _input(observations=observations)


def test_singular_innovation_is_error_not_silent_skip() -> None:
    data = _input(observations=(_position(),))
    config = _configuration(
        initial_covariance=np.zeros((15, 15)), local_position_noise_covariance_W=np.zeros((3, 3))
    )
    with pytest.raises(ValueError, match="positive definite"):
        replay_eskf(data, config)
    np.testing.assert_array_equal(config.initial_covariance, np.zeros((15, 15)))


@pytest.mark.parametrize(
    "field_name,shape",
    [
        ("initial_covariance", (15, 15)),
        ("continuous_noise_covariance", (12, 12)),
        ("local_position_noise_covariance_W", (3, 3)),
        ("local_position_bias_W", (3,)),
    ],
)
@pytest.mark.parametrize("bad", ["shape", "nan", "inf", "complex", "object"])
def test_configuration_array_domains(field_name: str, shape: tuple, bad: str) -> None:
    values = np.zeros(shape)
    if bad == "shape":
        values = values.reshape(-1) if len(shape) == 2 else values[:, None]
    elif bad in ("nan", "inf"):
        values.flat[0] = np.nan if bad == "nan" else np.inf
    else:
        values = values.astype(complex if bad == "complex" else object)
    with pytest.raises(ValueError, match=field_name):
        _configuration(**{field_name: values})


@pytest.mark.parametrize(
    "name,size",
    [
        ("initial_covariance", 15),
        ("continuous_noise_covariance", 12),
        ("local_position_noise_covariance_W", 3),
    ],
)
@pytest.mark.parametrize("bad", ["asymmetric", "indefinite", "negative", "zero_coupling"])
def test_configuration_covariance_domains(name: str, size: int, bad: str) -> None:
    values = np.eye(size)
    if bad == "asymmetric":
        values[0, 1] = 0.1
    elif bad == "indefinite":
        values[:2, :2] = [[1, 2], [2, 1]]
    elif bad == "negative":
        values[0, 0] = -np.nextafter(0.0, 1.0)
    else:
        values[0, 0] = 0
        values[0, 1] = values[1, 0] = 0.1
    with pytest.raises(ValueError, match=name):
        _configuration(**{name: values})


@pytest.mark.parametrize(
    "name",
    [
        "initial_time_s",
        "gravity_acceleration",
        "barometric_reference_altitude",
        "barometric_altitude_bias",
        "barometric_altitude_noise_variance",
    ],
)
@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf, True, "1", 1 + 2j])
def test_configuration_scalar_domains(name: str, bad: object) -> None:
    with pytest.raises(ValueError, match=name):
        _configuration(**{name: bad})


@pytest.mark.parametrize(
    "name,bad",
    [
        ("initial_time_s", -1.0),
        ("gravity_acceleration", -1.0),
        ("gravity_acceleration", 0.0),
        ("barometric_altitude_noise_variance", -1.0),
        ("fuse_local_position", 1),
        ("fuse_barometric_altitude", np.bool_(True)),
        ("initial_state", None),
    ],
)
def test_configuration_other_domains(name: str, bad: object) -> None:
    with pytest.raises((ValueError, TypeError), match=name):
        _configuration(**{name: bad})


@pytest.mark.parametrize(
    "name", ["specific_force_measurements_B", "angular_velocity_measurements_B"]
)
@pytest.mark.parametrize(
    "bad",
    [
        np.ones((4, 2)),
        np.ones((3, 3)),
        np.full((4, 3), np.inf),
        np.full((4, 3), np.nan),
        np.ones((4, 3), dtype=complex),
    ],
)
def test_input_imu_domain(name: str, bad: np.ndarray) -> None:
    with pytest.raises(ValueError, match=name):
        replace(_input(), **{name: bad})


def test_future_samples_and_observations_do_not_change_prior_history() -> None:
    data = _input(8, (_position(0, 4, 4),))
    force = data.specific_force_measurements_B.copy()
    force[3:, 0] = 7.0
    changed = replace(
        data,
        specific_force_measurements_B=force,
        observations=(replace(data.observations[0], measurement=np.full(3, 100.0)),),
    )
    first, second = replay_eskf(data, _configuration()), replay_eskf(changed, _configuration())
    np.testing.assert_array_equal(first.covariances[:4], second.covariances[:4])
    for a, b in zip(first.states[:4], second.states[:4], strict=True):
        for field in fields(a):
            np.testing.assert_array_equal(getattr(a, field.name), getattr(b, field.name))
    assert second.states[4].position_W[0] != first.states[4].position_W[0]


def test_dropout_prediction_and_recovery_covariance() -> None:
    observations = tuple(
        replace(_position(i, epoch, epoch), measurement=np.zeros(3))
        for i, epoch in enumerate([0, 1, 15, 16])
    )
    result = replay_eskf(_input(18, observations), _configuration())
    assert result.covariances[14, 2, 2] > result.covariances[1, 2, 2]
    assert result.covariances[15, 2, 2] < result.covariances[14, 2, 2]
    assert len(result.events) == 4


@pytest.mark.parametrize("sign", [-1.0, 1.0])
def test_constant_yaw_rate_has_analytic_orientation_and_zero_translation(sign: float) -> None:
    data = _input(101)
    gyro = np.tile([0.0, 0.0, 0.2], (101, 1))
    config = _configuration(initial_state=replace(_state(), q_WB=np.array([sign, 0, 0, 0])))
    result = replay_eskf(replace(data, angular_velocity_measurements_B=gyro), config)
    for index, state in enumerate(result.states):
        angle = 0.2 * (data.time_s[index] - data.time_s[0])
        expected = sign * np.array([np.cos(angle / 2), 0, 0, np.sin(angle / 2)])
        np.testing.assert_allclose(state.q_WB, expected, rtol=0, atol=3e-14)
        np.testing.assert_array_equal(state.position_W, np.zeros(3))
    np.testing.assert_array_equal(result.covariances, result.covariances.transpose(0, 2, 1))


def test_nonuniform_direct_input_uses_actual_intervals() -> None:
    times = np.array([0.1, 0.13, 0.4, 0.41])
    data = replace(
        _input(), time_s=times, specific_force_measurements_B=np.tile([2.0, 0.0, -9.81], (4, 1))
    )
    result = replay_eskf(data, _configuration())
    np.testing.assert_allclose(
        [s.position_W[0] for s in result.states], (times - times[0]) ** 2, atol=1e-16
    )


def test_shuffled_observations_have_byte_identical_replay() -> None:
    observations = (_position(), _position(1, 1, 3), _altitude(0, 0, 3), _position(2, 2, -1))
    config = _configuration()
    first = replay_eskf(_input(observations=observations), config)
    second = replay_eskf(_input(observations=observations[::-1]), config)
    assert first.covariances.tobytes() == second.covariances.tobytes()
    assert [e.status for e in first.events] == [e.status for e in second.events]
    assert [e.observation.kind for e in first.events] == [e.observation.kind for e in second.events]


@pytest.mark.parametrize(
    "status", [EskfReplayStatus.FUSED, EskfReplayStatus.STALE, EskfReplayStatus.PENDING]
)
def test_event_rejects_inconsistent_status(status: EskfReplayStatus) -> None:
    with pytest.raises(ValueError):
        EskfReplayEvent(_position(), status)


def test_event_and_result_reject_invalid_diagnostics() -> None:
    result = replay_eskf(_input(2, (_position(), _altitude(0, 1, 1))), _configuration())
    with pytest.raises(ValueError, match="dimension"):
        replace(result.events[0], update=result.events[1].update)
    with pytest.raises(ValueError):
        replace(result.events[0], status=EskfReplayStatus.DISABLED)
    for changes in (
        {"events": result.events[::-1]},
        {"states": ()},
        {"covariances": np.zeros((1, 15, 15))},
        {"covariances": np.full((2, 15, 15), np.nan)},
        {"events": (result.events[0], result.events[0])},
    ):
        with pytest.raises(ValueError):
            replace(result, **changes)


def test_nested_result_storage_is_independent_and_read_only() -> None:
    data, config = _input(2, (_position(),)), _configuration()
    result = replay_eskf(data, config)
    event = result.events[0]
    assert event.update is not None
    assert not np.shares_memory(event.observation.measurement, data.observations[0].measurement)
    assert not np.shares_memory(event.update.nominal_state.position_W, result.states[0].position_W)
    assert not np.shares_memory(event.update.covariance, result.covariances[0])
    for name in ("innovation", "innovation_covariance", "kalman_gain", "error_state_correction"):
        assert not getattr(event.update, name).flags.writeable
    raw = np.arange(12.0).reshape(4, 3)
    independent = replace(data, specific_force_measurements_B=raw[:2])
    raw[:] = -123
    np.testing.assert_array_equal(
        independent.specific_force_measurements_B, np.arange(6.0).reshape(2, 3)
    )


def test_failure_after_successful_update_is_atomic_for_inputs() -> None:
    data = _input(3, (_position(),))
    force = data.specific_force_measurements_B.copy()
    force[1, 0] = np.finfo(float).max
    data = replace(data, specific_force_measurements_B=force)
    config = _configuration()
    before = data.specific_force_measurements_B.tobytes(), config.initial_covariance.tobytes()
    with pytest.raises(ValueError, match="finite"):
        replay_eskf(data, config)
    assert before == (
        data.specific_force_measurements_B.tobytes(),
        config.initial_covariance.tobytes(),
    )


def test_rotating_body_specific_force_and_biases_match_world_kinematics() -> None:
    data = _input(61)
    elapsed = data.time_s - data.time_s[0]
    axis = np.array([1.0, 2.0, -1.0]) / np.sqrt(6.0)
    rate = 0.3
    acceleration_W = np.array([0.2, -0.1, 0.05])
    bias_a, bias_g = np.array([0.01, -0.02, 0.03]), np.array([0.002, 0.001, -0.003])
    # Independent Rodrigues construction, not the production rotation utility.
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    force = []
    for time in elapsed:
        angle = rate * time
        rotation = np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)
        force.append(rotation.T @ (acceleration_W - [0, 0, 9.81]) + bias_a)
    data = replace(
        data,
        specific_force_measurements_B=np.array(force),
        angular_velocity_measurements_B=np.tile(rate * axis + bias_g, (61, 1)),
    )
    config = _configuration(
        initial_state=replace(_state(), accelerometer_bias_B=bias_a, gyroscope_bias_B=bias_g)
    )
    result = replay_eskf(data, config)
    np.testing.assert_allclose(
        [s.position_W for s in result.states],
        0.5 * elapsed[:, None] ** 2 * acceleration_W,
        rtol=0,
        atol=2e-12,
    )
    np.testing.assert_allclose(
        [s.velocity_W for s in result.states], elapsed[:, None] * acceleration_W, rtol=0, atol=2e-12
    )
    expected_q_WB = np.column_stack(
        (np.cos(rate * elapsed / 2), np.sin(rate * elapsed[:, None] / 2) * axis)
    )
    np.testing.assert_allclose([s.q_WB for s in result.states], expected_q_WB, rtol=0, atol=3e-14)


def test_second_same_epoch_singular_update_does_not_publish_partial_result() -> None:
    data = _input(1, (_position(), _altitude()))
    config = _configuration(
        local_position_noise_covariance_W=np.zeros((3, 3)), barometric_altitude_noise_variance=0.0
    )
    with pytest.raises(ValueError, match="positive definite"):
        replay_eskf(data, config)
    np.testing.assert_array_equal(config.initial_covariance, np.eye(15))
    np.testing.assert_array_equal(config.initial_state.position_W, np.zeros(3))
